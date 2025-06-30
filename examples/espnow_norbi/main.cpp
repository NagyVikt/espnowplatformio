#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "driver/i2c.h"
#include <cstring>

static const char* TAG = "NORBI";

// ======== Constants ========
static constexpr i2c_port_t I2C_MASTER_NUM      = I2C_NUM_0;
static constexpr gpio_num_t I2C_MASTER_SDA_IO   = GPIO_NUM_21;
static constexpr gpio_num_t I2C_MASTER_SCL_IO   = GPIO_NUM_22;
static constexpr uint32_t    I2C_MASTER_FREQ_HZ  = 100000;
static constexpr int         MAX_CHANNEL         = 40;
static constexpr size_t      MAX_MSG_LEN         = 128;
static constexpr uint8_t     ESPNOW_CHANNEL      = 0;
static constexpr uint8_t     MCP_I2C_ADDRS[]     = {0x20,0x21,0x22,0x23,0x24};

// ======== Globals ========
static uint8_t  lastSenderMac[6];
static bool     haveLastSender = false;
static bool     monitored[MAX_CHANNEL];
static bool     blinkState     = false;

enum class State {
    SELF_CHECK,
    WAIT_FOR_TARGET,
    MONITORING,
    FINAL_CHECK,
};
static State state = State::SELF_CHECK;

struct ChannelPins { uint8_t mcp_idx, led_pin, sw_pin; };
static ChannelPins channelPins[MAX_CHANNEL];

// ======== MCP23X17 I²C Helpers ========
static esp_err_t mcp_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr<<1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t res = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return res;
}

static esp_err_t mcp_read_reg(uint8_t addr, uint8_t reg, uint8_t &val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr<<1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr<<1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &val, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t res = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return res;
}

// Build mapping from channel → (MCP index, LED pin, SW pin)
static void build_channel_pins() {
    auto remap = [](uint8_t pin)->uint8_t { return (pin < 8) ? (pin + 8) : (pin - 8); };
    for (int ch = 0; ch < MAX_CHANNEL; ++ch) {
        uint16_t base = ch * 2;
        uint8_t idx    = base / 16;
        uint8_t rawLed = base % 16;
        uint8_t rawSw  = (base + 1) % 16;
        channelPins[ch] = {
            idx,
            remap(rawLed),
            remap(rawSw)
        };
    }
}

static void set_led(int ch, bool on) {
    const auto &p = channelPins[ch];
    uint8_t reg = 0x14 + ((p.led_pin >> 3) * 2);
    uint8_t bit = uint8_t(1 << (p.led_pin & 7));
    mcp_write_reg(MCP_I2C_ADDRS[p.mcp_idx], reg, on ? bit : 0);
}

static bool read_switch(int ch) {
    const auto &p = channelPins[ch];
    uint8_t reg = 0x12 + ((p.sw_pin >> 3) * 2);
    uint8_t val = 0;
    mcp_read_reg(MCP_I2C_ADDRS[p.mcp_idx], reg, val);
    return (val & uint8_t(1 << (p.sw_pin & 7))) != 0;
}

static bool send_command(const char *cmd) {
    if (!haveLastSender) return false;
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, lastSenderMac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    esp_err_t res = esp_now_send(lastSenderMac, (const uint8_t*)cmd, strlen(cmd)+1);
    esp_now_del_peer(lastSenderMac);
    return (res == ESP_OK);
}

static bool check_all() {
    bool ok = true;
    for (int ch = 0; ch < MAX_CHANNEL; ++ch) {
        bool sw = read_switch(ch);
        if (monitored[ch]) {
            set_led(ch, sw);
            if (sw) ok = false;
        } else {
            set_led(ch, false);
            if (!sw) {
                set_led(ch, blinkState);
                ok = false;
            }
        }
    }
    return ok;
}

// ======== ESP-NOW Callback ========
static void on_data_recv(const esp_now_recv_info_t *info,
                         const uint8_t *data,
                         int data_len)
{
    memcpy(lastSenderMac, info->src_addr, 6);
    haveLastSender = true;

    ESP_LOGI(TAG, "Recv %d bytes from %02X:%02X:%02X:%02X:%02X:%02X",
             data_len,
             info->src_addr[0],info->src_addr[1],info->src_addr[2],
             info->src_addr[3],info->src_addr[4],info->src_addr[5]);

    if (state == State::WAIT_FOR_TARGET) {
        const char prefix[] = "MONITOR ";
        if (data_len > int(sizeof(prefix)-1) &&
            strncmp((const char*)data, prefix, sizeof(prefix)-1) == 0)
        {
            memset(monitored, 0, sizeof(monitored));
            for (int i = 0; i < MAX_CHANNEL; ++i) set_led(i, false);

            char buf[MAX_MSG_LEN];
            int len = (data_len < int(MAX_MSG_LEN-1)) ? data_len : int(MAX_MSG_LEN-1);
            memcpy(buf, data, len);
            buf[len] = '\0';

            for (char *tok = strtok(buf + (sizeof(prefix)-1), ","); tok; tok = strtok(nullptr, ",")) {
                int ch = atoi(tok) - 1;
                if (ch >= 0 && ch < MAX_CHANNEL) {
                    monitored[ch] = true;
                    set_led(ch, true);
                }
            }
            state = State::MONITORING;
            ESP_LOGI(TAG, ">> MONITORING");
        }
    }
    else if (state == State::MONITORING) {
        const char check_cmd[] = "CHECK";
        if (data_len >= int(sizeof(check_cmd)-1) &&
            strncmp((const char*)data, check_cmd, sizeof(check_cmd)-1) == 0)
        {
            state = State::FINAL_CHECK;
            ESP_LOGI(TAG, ">> FINAL_CHECK");
        }
    }
}

// ======== Initialization ========
static void init_espnow() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
}

// ======== App Entry Point ========
extern "C" void app_main() {
    // I2C Master Setup
    i2c_config_t i2c_conf;
    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.sda_io_num = I2C_MASTER_SDA_IO;
    i2c_conf.scl_io_num = I2C_MASTER_SCL_IO;
    i2c_conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));

    build_channel_pins();
    // Initialize MCP23X17: A bank outputs, B inputs
    for (uint8_t a : MCP_I2C_ADDRS) {
        ESP_ERROR_CHECK(mcp_write_reg(a, 0x00, 0x00)); // IODIR A
        ESP_ERROR_CHECK(mcp_write_reg(a, 0x01, 0xFF)); // IODIR B
    }

    init_espnow();

    while (true) {
        blinkState = !blinkState;
        switch (state) {
            case State::SELF_CHECK: {
                bool any = false;
                for (int ch = 0; ch < MAX_CHANNEL; ++ch) {
                    bool pressed = !read_switch(ch);
                    set_led(ch, blinkState && pressed);
                    any |= pressed;
                }
                if (!any) {
                    state = State::WAIT_FOR_TARGET;
                    ESP_LOGI(TAG, ">> SELF_CHECK OK");
                }
                break;
            }
            case State::MONITORING:
                check_all();
                break;
            case State::FINAL_CHECK: {
                int good = 0;
                for (int i = 0; i < 5; ++i) {
                    if (check_all()) good++;
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
                if (good >= 5) {
                    ESP_LOGI(TAG, ">> FINAL_PASS");
                    send_command("SUCCESS");
                    state = State::SELF_CHECK;
                } else {
                    ESP_LOGI(TAG, ">> FINAL_FAIL");
                    send_command("FAILURE");
                    state = State::MONITORING;
                }
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
