#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include <cstring>
#include <string>
#include <algorithm>

extern "C" {
    #include "nvs_flash.h"
    #include "esp_netif.h"
    #include "esp_event.h"
    #include "esp_wifi.h"
    #include "esp_now.h"
}

static const char* TAG = "NORBI_SLAVE";

// ======== Constants ========
static constexpr i2c_port_t  I2C_MASTER_NUM     = I2C_NUM_0;
static constexpr gpio_num_t  I2C_MASTER_SDA_IO  = GPIO_NUM_21;
static constexpr gpio_num_t  I2C_MASTER_SCL_IO  = GPIO_NUM_22;
static constexpr uint32_t    I2C_MASTER_FREQ_HZ = 100000;
static constexpr int         MAX_CHANNEL        = 40;
static constexpr size_t      MAX_MSG_LEN        = 128;
static constexpr uint8_t     ESPNOW_CHANNEL     = 0;
static constexpr uint8_t     MCP_I2C_ADDRS[]    = {0x20,0x21,0x22,0x23,0x24};

// special 4-byte hub command and ACK
static const uint8_t CMD_BYTES[4] = {0xFF,0xFF,0xFF,0xFF};
static const uint8_t ACK_BYTES[4] = {0xEE,0xEE,0xEE,0xEE};

// ======== Globals ========
static uint8_t     lastHubMac[6];
static bool        haveLastHub = false;
static bool        monitored[MAX_CHANNEL];
static bool        blinkState  = false;
static std::string missing, extra;

// plain enum so switch(state) compiles
enum State {
    SELF_CHECK,
    WAIT_FOR_TARGET,
    MONITORING,
    FINAL_CHECK
};
static State state = SELF_CHECK;

struct ChannelPins { uint8_t mcp_idx, led_pin, sw_pin; };
static ChannelPins channelPins[MAX_CHANNEL];

// ======== I2C Helpers ========
static esp_err_t mcp_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr<<1)|I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t mcp_read_reg(uint8_t addr, uint8_t reg, uint8_t &val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr<<1)|I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr<<1)|I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &val, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

// ======== Channel mapping ========
static void build_channel_pins() {
    auto remap = [](uint8_t pin)->uint8_t { return (pin < 8) ? (pin + 8) : (pin - 8); };
    for (int ch = 0; ch < MAX_CHANNEL; ++ch) {
        uint16_t base  = ch * 2;
        uint8_t idx    = base / 16;
        uint8_t rawLed = base % 16;
        uint8_t rawSw  = (base + 1) % 16;
        channelPins[ch] = { idx, remap(rawLed), remap(rawSw) };
    }
}

// Set or clear an LED
static void set_led(int ch, bool on) {
    const auto &p = channelPins[ch];
    uint8_t reg = 0x14 + ((p.led_pin >> 3) * 2);
    uint8_t bit = (1 << (p.led_pin & 7));
    mcp_write_reg(MCP_I2C_ADDRS[p.mcp_idx], reg, on ? bit : 0);
}

// Read a switch (HIGH when not pressed)
static bool read_switch(int ch) {
    const auto &p = channelPins[ch];
    uint8_t reg = 0x12 + ((p.sw_pin >> 3) * 2);
    uint8_t val = 0;
    mcp_read_reg(MCP_I2C_ADDRS[p.mcp_idx], reg, val);
    return (val & (1 << (p.sw_pin & 7))) != 0;
}

// ======== ESP-NOW send helpers ========
static bool send_raw(const uint8_t *data, size_t len) {
    if (!haveLastHub) return false;
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, lastHubMac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    esp_err_t res = esp_now_send(lastHubMac, data, len);
    esp_now_del_peer(lastHubMac);
    return (res == ESP_OK);
}

static bool send_result(const char *txt) {
    return send_raw(reinterpret_cast<const uint8_t*>(txt), strlen(txt) + 1);
}

// ======== Check all channels ========
static bool check_all() {
    bool ok = true;
    missing.clear();
    extra.clear();

    for (int ch = 0; ch < MAX_CHANNEL; ++ch) {
        bool sw = read_switch(ch);
        if (monitored[ch]) {
            set_led(ch, sw);
            if (sw) {
                ok = false;
                missing += std::to_string(ch + 1) + ",";
            }
        } else {
            set_led(ch, false);
            if (!sw) {
                set_led(ch, blinkState);
                ok = false;
                extra += std::to_string(ch + 1) + ",";
            }
        }
    }
    return ok;
}

// ======== ESP-NOW receive callback ========
static void on_data_recv(const esp_now_recv_info_t *info,
                         const uint8_t *data, int len)
{
    // stash hub MAC
    memcpy(lastHubMac, info->src_addr, 6);
    haveLastHub = true;

    // 1) Hub handshake
    if (len == 4 && memcmp(data, CMD_BYTES, 4) == 0) {
        ESP_LOGI(TAG, "Got HUB CMD → sending ACK");
        send_raw(ACK_BYTES, 4);
        return;
    }

    // 2) MONITOR / CHECK
    ESP_LOGI(TAG, "Recv %d bytes from HUB", len);
    if (state == WAIT_FOR_TARGET) {
        const char prefix[] = "MONITOR ";
        if (len > int(strlen(prefix)) &&
            strncmp(reinterpret_cast<const char*>(data), prefix, strlen(prefix)) == 0)
        {
            memset(monitored, 0, sizeof(monitored));
            for (int i = 0; i < MAX_CHANNEL; ++i) set_led(i, false);

            char buf[MAX_MSG_LEN];
            int l = std::min(len, int(MAX_MSG_LEN - 1));
            memcpy(buf, data, l);
            buf[l] = '\0';

            for (char *tok = strtok(buf + strlen(prefix), ","); tok; tok = strtok(nullptr, ",")) {
                int c = atoi(tok) - 1;
                if (c >= 0 && c < MAX_CHANNEL) {
                    monitored[c] = true;
                    set_led(c, true);
                }
            }
            state = MONITORING;
            ESP_LOGI(TAG, ">> ENTERING MONITORING");
        }
    }
    else if (state == MONITORING) {
        const char check_cmd[] = "CHECK";
        if (len >= int(strlen(check_cmd)) &&
            strncmp(reinterpret_cast<const char*>(data), check_cmd, strlen(check_cmd)) == 0)
        {
            state = FINAL_CHECK;
            ESP_LOGI(TAG, ">> FINAL_CHECK");
        }
    }
}

// ======== Initialization ========
static void init_i2c_and_mcp() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));

    build_channel_pins();
    for (uint8_t a : MCP_I2C_ADDRS) {
        // Port A = outputs
        ESP_ERROR_CHECK(mcp_write_reg(a, 0x00, 0x00));
        // Port B = inputs
        ESP_ERROR_CHECK(mcp_write_reg(a, 0x01, 0xFF));
        // Port B pull-ups ON (like Arduino INPUT_PULLUP)
        ESP_ERROR_CHECK(mcp_write_reg(a, 0x0D, 0xFF));
    }
}

static void init_espnow_slave() {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
}

// ======== App Main ========
extern "C" void app_main() {
    ESP_LOGI(TAG, "Slave starting...");

    init_i2c_and_mcp();
    init_espnow_slave();

    while (true) {
        blinkState = !blinkState;
        switch (state) {
            case SELF_CHECK: {
                bool any = false;
                for (int ch = 0; ch < MAX_CHANNEL; ++ch) {
                    bool pressed = !read_switch(ch);
                    set_led(ch, blinkState && pressed);
                    any |= pressed;
                }
                if (!any) {
                    state = WAIT_FOR_TARGET;
                    ESP_LOGI(TAG, ">> SELF_CHECK passed, waiting for MONITOR");
                }
                break;
            }

            case WAIT_FOR_TARGET: {
                for (int ch = 0; ch < MAX_CHANNEL; ++ch) {
                    set_led(ch, false);
                }
                break;
            }

            case MONITORING:
                check_all();
                break;

            case FINAL_CHECK: {
                int good = 0;
                for (int i = 0; i < 5; ++i) {
                    if (check_all()) good++;
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
                if (good >= 5) {
                    ESP_LOGI(TAG, ">> FINAL_PASS");
                    send_result("SUCCESS");
                    state = SELF_CHECK;
                } else {
                    ESP_LOGI(TAG, ">> FINAL_FAIL");
                    std::string msg = "FAILURE";
                    if (!missing.empty()) {
                        msg += " MISSING ";
                        msg += missing;
                    }
                    if (!extra.empty()) {
                        if (missing.empty()) {
                            msg += " EXTRA ";
                        } else {
                            msg += ";EXTRA ";
                        }
                        msg += extra;
                    }
                    msg += ',';
                    send_result(msg.c_str());
                    state = MONITORING;
                }
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
