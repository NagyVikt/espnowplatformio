// app_main.cpp

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_console.h"
#include "esp_console_repl.h"
#include "esp_console_dev_uart.h"
#include "argtable3/argtable3.h"
#include "esp_now.h"
#include <string.h>
#include <string>

static const char *TAG = "MASTER";
static constexpr char *NVS_NS  = "storage";
static constexpr char *NVS_KEY = "stations";
static constexpr int   MAX_PEERS = 10;

static int      peer_count = 0;
static uint8_t  peers[MAX_PEERS][6];

// argtable for console commands
static struct { struct arg_str *mac; struct arg_end *end;} add_args, del_args;

// parse "AA:BB:CC:DD:EE:FF" → 6 bytes
static bool parse_mac(const char *s, uint8_t m[6]) {
    int v[6];
    if (sscanf(s, "%x:%x:%x:%x:%x:%x",
               &v[0],&v[1],&v[2],
               &v[3],&v[4],&v[5]) != 6) {
        return false;
    }
    for(int i=0;i<6;i++) m[i] = (uint8_t)v[i];
    return true;
}

static void load_peers() {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t len=0;
    if (nvs_get_str(h, NVS_KEY, nullptr, &len)!=ESP_OK || len==0) {
        nvs_close(h);
        return;
    }
    char *buf = (char*)malloc(len);
    nvs_get_str(h, NVS_KEY, buf, &len);
    nvs_close(h);

    peer_count = 0;
    for(char *tok=strtok(buf,";"); tok && peer_count<MAX_PEERS; tok=strtok(nullptr,";")) {
        uint8_t mac[6];
        if(parse_mac(tok,mac)) {
            memcpy(peers[peer_count++], mac, 6);
        }
    }
    free(buf);
}

static void save_peers() {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h)!=ESP_OK) return;
    std::string csv;
    char tmp[18];
    for(int i=0;i<peer_count;i++) {
        snprintf(tmp,sizeof(tmp),
                 "%02X:%02X:%02X:%02X:%02X:%02X",
                 peers[i][0],peers[i][1],peers[i][2],
                 peers[i][3],peers[i][4],peers[i][5]);
        csv += tmp;
        if (i+1<peer_count) csv += ';';
    }
    nvs_set_str(h, NVS_KEY, csv.c_str());
    nvs_commit(h);
    nvs_close(h);
}

static void register_peers() {
    esp_now_peer_info_t pi = {};
    pi.channel = 0; pi.encrypt = false;
    for(int i=0;i<peer_count;i++){
        memcpy(pi.peer_addr, peers[i], 6);
        esp_now_add_peer(&pi);
        ESP_LOGI(TAG, "Peer added %02X:%02X:%02X:%02X:%02X:%02X",
                 peers[i][0],peers[i][1],peers[i][2],
                 peers[i][3],peers[i][4],peers[i][5]);
    }
}

static int cmd_add(int argc, char **argv) {
    if(arg_parse(argc,argv,(void**)&add_args)){
        arg_print_errors(stderr,add_args.end,argv[0]);
        return 1;
    }
    if(peer_count>=MAX_PEERS){ printf("List full\n"); return 1; }
    uint8_t mac[6];
    if(!parse_mac(add_args.mac->sval[0],mac)){
        printf("Bad MAC\n"); return 1;
    }
    for(int i=0;i<peer_count;i++){
        if(!memcmp(peers[i],mac,6)){ printf("Exists\n"); return 0; }
    }
    memcpy(peers[peer_count++], mac, 6);
    save_peers();
    esp_now_peer_info_t pi = {}; pi.channel=0; pi.encrypt=false;
    memcpy(pi.peer_addr, mac, 6);
    esp_now_add_peer(&pi);
    printf("Added %s\n", add_args.mac->sval[0]);
    return 0;
}

static int cmd_del(int argc, char **argv) {
    if(arg_parse(argc,argv,(void**)&del_args)){
        arg_print_errors(stderr,del_args.end,argv[0]);
        return 1;
    }
    uint8_t mac[6];
    if(!parse_mac(del_args.mac->sval[0],mac)){
        printf("Bad MAC\n"); return 1;
    }
    int idx=-1;
    for(int i=0;i<peer_count;i++){
        if(!memcmp(peers[i],mac,6)){ idx=i; break; }
    }
    if(idx<0){ printf("Not found\n"); return 1; }
    for(int i=idx;i+1<peer_count;i++){
        memcpy(peers[i], peers[i+1], 6);
    }
    peer_count--;
    save_peers();
    esp_now_del_peer(mac);
    printf("Deleted %s\n", del_args.mac->sval[0]);
    return 0;
}

extern "C" void app_main() {
    // 1) NVS
    ESP_ERROR_CHECK(nvs_flash_init());

    // 2) Wi-Fi driver only (no TCP/IP)
    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_log_level_set("wifi", ESP_LOG_WARN);  // silence wifi INFO logs

    // 3) ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());

    // 4) Console REPL on UART0
    esp_console_repl_t *repl = nullptr;
    esp_console_repl_config_t crepl = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    crepl.prompt = "MASTER> ";
    esp_console_dev_uart_config_t ucfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&ucfg, &crepl, &repl));

    // 5) load & register
    load_peers();
    register_peers();

    // 6) register commands
    add_args.mac = arg_str1(nullptr,"mac","<MAC>","Station MAC");
    add_args.end = arg_end(1);
    esp_console_cmd_register(&(esp_console_cmd_t){
        .command  = "add_station",
        .help     = "Add slave MAC",
        .hint     = "--mac AA:BB:CC:DD:EE:FF",
        .func     = &cmd_add,
        .argtable = &add_args
    });
    del_args.mac = arg_str1(nullptr,"mac","<MAC>","Station MAC");
    del_args.end = arg_end(1);
    esp_console_cmd_register(&(esp_console_cmd_t){
        .command  = "del_station",
        .help     = "Remove slave MAC",
        .hint     = "--mac AA:BB:CC:DD:EE:FF",
        .func     = &cmd_del,
        .argtable = &del_args
    });

    ESP_LOGI(TAG, "Console ready. Type commands now.");
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    // (then your master_task can be started if you have one)
}
