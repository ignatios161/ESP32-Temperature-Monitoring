#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/inet.h"
#include <inttypes.h>
#include "string.h"
#include "mqtt_client.h"
#include "esp_timer.h"



#define LMT86_V0_MV 1777.3f
#define LMT86_TC_MV_PER_C 10.888f
#define LMT86_REF_TEMP_C 30.0f
#define CONNECTED_BIT BIT0
#define IP4_ADDR_STRLEN 16
#define MAX_RETRY_COUNT 5


char addr_str[IP4_ADDR_STRLEN];
static int retry_count = 0;
static EventGroupHandle_t s_wifi_event_group;
static esp_mqtt_client_handle_t mqtt_client = NULL;
static const char *TAG = "MQTT_CLIENT";
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle = NULL;
static unsigned long start_time = 0; 
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void start_temperature_measurements(int count, int interval);
float read_temperature();

void wait_for_ip() {
    while (!(xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, false, true, pdMS_TO_TICKS(10000)))) {
        ESP_LOGI("wifi_event_handler", "Trying to reconnect... (%d/%d)", retry_count, MAX_RETRY_COUNT);
        if (++retry_count > MAX_RETRY_COUNT) {
            ESP_LOGE("wifi_event_handler", "Failed to connect after %d attempts.", MAX_RETRY_COUNT);
            break;
        }
        esp_wifi_connect();
    }
}
static void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    ESP_LOGI("wifi_event_handler", "Event received: Base=%s, ID=%" PRIi32, event_base == WIFI_EVENT ? "WIFI_EVENT" : "IP_EVENT", event_id);
    if (strcmp(event_base, WIFI_EVENT) == 0) {
        if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
            ESP_LOGI("wifi_event_handler", "WiFi started, trying to connect...");
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            ESP_LOGI("wifi_event_handler", "Disconnected from WiFi, trying to reconnect...");
            esp_wifi_connect();
            xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
        }
    } else if (strcmp(event_base, IP_EVENT) == 0) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            esp_ip4addr_ntoa(&event->ip_info.ip, addr_str, IP4_ADDR_STRLEN);
            ESP_LOGI("wifi_event_handler", "Got IP: %s", addr_str);
            xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
            retry_count = 0; 
        }
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data; 
    
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            esp_mqtt_client_subscribe(mqtt_client, CONFIG_MQTT_COMMAND_TOPIC, 0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED");
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA: Topic=%.*s, Data=%.*s", event->topic_len, event->topic, event->data_len, event->data);
            char* payload = event->data;
            int measurements, interval;
            if (sscanf(payload, "measure:%d,%d", &measurements, &interval) == 2) {
                ESP_LOGI(TAG, "Received measurement command: %d measurements, %d ms interval", measurements, interval);
                start_temperature_measurements(measurements, interval);
            } else {
                ESP_LOGE(TAG, "Received malformed command");
            }
            break;
    }
}

static void start_temperature_measurements(int count, int interval) {
    if (count == 3) { 
        start_time = (unsigned long)(esp_timer_get_time() / 1000); 
    }
    ESP_LOGI(TAG, "Measurement count: %d", count);
    float temperature = read_temperature();
    unsigned long expected_uptime = start_time + (3 - count) * interval; 
    char response[100];
    snprintf(response, sizeof(response), "%d,%.2f,%lu", count - 1, temperature, expected_uptime);
    ESP_LOGI(TAG, "Publishing temperature: %s", response);
    esp_mqtt_client_publish(mqtt_client, CONFIG_MQTT_RESPONSE_TOPIC, response, 0, 1, 0);
    vTaskDelay(pdMS_TO_TICKS(interval));
    if (count == 1) {
        ESP_LOGI(TAG, "No more measurements needed.");
        vTaskDelay(pdMS_TO_TICKS(5000));
        return;
    }
    start_temperature_measurements(count - 1, interval);
    
}
float read_temperature() {
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_6, &raw));
    int voltage = 0;
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, raw, &voltage));
    float temperature_c = (LMT86_V0_MV - voltage) / LMT86_TC_MV_PER_C + LMT86_REF_TEMP_C;
    return temperature_c;
}
void configure_adc() {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_6, &chan_cfg));
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_cfg, &cali_handle));
}
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    wifi_init_sta();
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID, &wifi_event_handler,NULL,NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));                            
    wait_for_ip(); 

    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = CONFIG_MQTT_BROKER_URI
            }
        } 
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    configure_adc();
   
}

