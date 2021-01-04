// Copyright 2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mdf_common.h"
#include "mesh_mqtt_handle.h"
#include "mwifi.h"
#include "mupgrade.h"
#include "mlink.h"
#include "driver/uart.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "string.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <sys/fcntl.h>
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "math.h"
#include <stdlib.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_event.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"

#define MEMORY_DEBUG
#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WEB_SERVER         CONFIG_WEB_SERVER
#define WEB_URL            CONFIG_WEB_URL
#define WEB_PORT           CONFIG_WEB_PORT
// for DC motor
// #define GPIO_RIGHT 26
// #define GPIO_LEFT 27
//for LED testing
#define GPIO_VALV 25
#define GPIO_MESH 26
#define GPIO_ROOT 27
#define DEFAULT_VREF 1100
#define NO_OF_SAMPLES 64

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_4;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
esp_netif_t *sta_netif;

static const char REQUEST[512] = "POST " WEB_URL " HTTP/1.0\r\n"
    "User-Agent: esp-idf/1.0 esp32\r\n"
    "Host: "WEB_SERVER"\r\n"
    "Content-Type: application/json\r\n"
    "Content-Length: 35\r\n"
    "\r\n"
    "{\"macAddress\": %s}";

static const char *TAG = "mqtt_Node_VME_Prov";

static float pct2075d_read_temperature(int slave_address);
static uint16_t read_value(int slave_address);
static int findComplement(int n);

int findComplement(int n){
    int bit_count = floor(log2(n))+1;
    int ones = ((1 << bit_count) - 1);
    return ones ^ n;
}

static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

float pct2075d_read_temperature(int slave_address) {

	// get the raw value from the sensor

	uint16_t raw_temperature = read_value(slave_address);
    float tmp_temperature;
    float delta_temp;
    
	if ((raw_temperature & 1024) == 0){
		printf("positive temperature\n");
        tmp_temperature = (float) (floor((raw_temperature * 0.125)));
        printf("Tmp Temperature: %f\n", tmp_temperature);
        delta_temp = ((float) (raw_temperature * 0.125) - tmp_temperature);
        printf("Delta Temperature: %f\n", delta_temp);
        if (delta_temp <= 0.250){
            return tmp_temperature;
        }
        else if (delta_temp >= 0.750){
            return tmp_temperature + 1;
        }
        else{
            return tmp_temperature + 0.5;
        }
	}
	else{
		printf("negative temperature\n");
        printf("complemento ad uno:%d\n", findComplement(raw_temperature));
        float neg_temp = (((-1) * ((findComplement(raw_temperature)) + 1)) * 0.125);
        printf("Neg Temperature: %f\n", neg_temp);
        tmp_temperature = (float) (ceil(neg_temp));
        printf("Tmp Temperature: %f\n", tmp_temperature);	
        delta_temp = tmp_temperature - neg_temp;
        printf("Delta Temperature: %f\n", delta_temp);
        if(delta_temp <= 0.250){
            if (tmp_temperature == -0.000000){
                tmp_temperature = 0;
            }
            return tmp_temperature;
        }
        else if(delta_temp >= 0.750){
            return tmp_temperature - 1;
        }
        else{
            return tmp_temperature - 0.5;
        }
	}
}

void pct2075_initialization() {
    printf("i2c scanner\r\n\r\n");
	// configure the i2c controller 0 in master mode, fast speed
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = 21;
	conf.scl_io_num = 22;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	printf("- i2c controller configured\r\n");
	
	// install the driver
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
	printf("- i2c driver installed\r\n\r\n");
	
	printf("scanning the bus...\r\n\r\n");
	int devices_found = 0;
	
	for(int address = 1; address < 127; address++) {
	
		// create and execute the command link
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
		i2c_master_stop(cmd);
		if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS) == ESP_OK) {
			printf("-> found device with address 0x%02x\r\n", address);
			printf("Address Integer: %d\n", address);
			devices_found++;
		}
		i2c_cmd_link_delete(cmd);
	}
	if(devices_found == 0) printf("\r\n-> no devices found\r\n");
	printf("\r\n...scan completed!\r\n");
}

uint16_t read_value(int slave_address) {
	
	esp_err_t ret;
	// send the command
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (slave_address << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, 0x00, true);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if(ret != ESP_OK){
		printf("ret != OK\n");
		return 0;
	}
	// wait for the sensor (50ms)
	vTaskDelay(50 / portTICK_RATE_MS);
	
	// receive the answer
	uint8_t msb, lsb;
	
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (slave_address << 1) | I2C_MASTER_READ, true);
	i2c_master_read_byte(cmd, &msb, 0x00);
	i2c_master_read_byte(cmd, &lsb, 0x01);
	i2c_master_stop(cmd);

	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if(ret != ESP_OK) {
		printf("error\n");
		return 0;
	}

	uint16_t raw_value = (((uint16_t) msb << 8) | (uint16_t) lsb) ;
	raw_value = raw_value >> 5;
	return raw_value; 
	
}

// START HTTPS CLIENT CODE +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void https_get_task(void *pvParameters)
{
    
    char REQUEST_1[512];
    cJSON *json_root = NULL;
    cJSON *json_mesh_id = NULL;
    cJSON *json_mesh_pass = NULL;
    cJSON *json_gateway_essid = NULL;
    cJSON *json_gateway_pass = NULL;
    cJSON *json_mqtt_url = NULL;
    cJSON *json_firmware_url = NULL;

    uint8_t sta_mac[MWIFI_ADDR_LEN] = {0};
    char sta_mac_string[20] = {0};

    esp_efuse_mac_get_default(sta_mac);
    sprintf(sta_mac_string, "\"%x%x:%x%x:%x%x:%x%x:%x%x:%x%x\"", 
                    ((sta_mac[0]&240)>>4),(sta_mac[0]&15), ((sta_mac[1]&240)>>4),(sta_mac[1]&15), ((sta_mac[2]&240)>>4),(sta_mac[2]&15), ((sta_mac[3]&240)>>4),(sta_mac[3]&15), ((sta_mac[4]&240)>>4),(sta_mac[4]&15), ((sta_mac[5]&240)>>4),(sta_mac[5]&15));   
    printf("STA_MAC %s\n", sta_mac_string);
    sprintf(REQUEST_1, REQUEST, sta_mac_string);           
    puts(REQUEST_1);
    
    char buf[512];
    char json[512];
    int ret, len;

    while(1){
        esp_tls_cfg_t cfg = {
            .crt_bundle_attach = esp_crt_bundle_attach,
        };

        struct esp_tls *tls = esp_tls_conn_http_new(WEB_URL, &cfg);

        if(tls != NULL) {
            ESP_LOGI(TAG, "Connection established...");
        } else {
            ESP_LOGE(TAG, "Connection failed...");
            goto exit;
        }

        size_t written_bytes = 0;
        do {
            ret = esp_tls_conn_write(tls,
                                     REQUEST_1 + written_bytes,
                                     strlen(REQUEST_1) - written_bytes);
            if (ret >= 0) {
                ESP_LOGI(TAG, "%d bytes written", ret);
                written_bytes += ret;
            } else if (ret != ESP_TLS_ERR_SSL_WANT_READ  && ret != ESP_TLS_ERR_SSL_WANT_WRITE) {
                ESP_LOGE(TAG, "esp_tls_conn_write  returned 0x%x", ret);
                goto exit;
            }
        } while(written_bytes < strlen(REQUEST_1));

        ESP_LOGI(TAG, "Reading HTTP response...");

        do
        {
            len = sizeof(buf) - 1;
            bzero(buf, sizeof(buf));
            ret = esp_tls_conn_read(tls, (char *)buf, len);

            if(ret == ESP_TLS_ERR_SSL_WANT_WRITE  || ret == ESP_TLS_ERR_SSL_WANT_READ)
                continue;

            if(ret < 0)
           {
                ESP_LOGE(TAG, "esp_tls_conn_read  returned -0x%x", -ret);
                break;
            }

            if(ret == 0)
            {
                ESP_LOGI(TAG, "connection closed");
                break;
            }
    
            len = ret;
            ESP_LOGD(TAG, "%d bytes read", len);
            printf("RECEIVED:%s\n",buf);
                for(int i = 0; i < len; i++) {
                    if(buf[i] == '{'){
                        for(int j = 0; j < len - i; j++){
                            json[j] = buf[j + i];
                        }
                        json[len - i] = '\0';
                    }
                }
            printf("RECEIVED JSON:%s\n",json);
            json_root = cJSON_Parse(json);
            json_gateway_essid = cJSON_GetObjectItem(json_root,"gatewayESSID");
            json_gateway_pass = cJSON_GetObjectItem(json_root,"gatewayPassword");
            json_mesh_id = cJSON_GetObjectItem(json_root,"meshId");
            json_mesh_pass = cJSON_GetObjectItem(json_root,"meshPassword");
            json_mqtt_url = cJSON_GetObjectItem(json_root,"brokerURL");
            json_firmware_url = cJSON_GetObjectItem(json_root,"httpServerURL");
            // printf("Gateway ESSID: %s\n", json_gateway_essid->valuestring);
            // printf("Gateway password: %s\n", json_gateway_pass->valuestring);
            // printf("Mesh ID: %s\n", json_mesh_id->valuestring);
            // printf("Mesh pass: %s\n", json_mesh_pass->valuestring);
            // printf("MQTT URL: %s\n", json_mqtt_url->valuestring);
            // printf("Firmware URL: %s\n", json_firmware_url->valuestring);
                
        } while(1);

    exit:
        esp_tls_conn_delete(tls);
        putchar('\n'); // JSON output doesn't have a newline at end
        if(json_gateway_essid == NULL || json_gateway_pass == NULL || json_mesh_id == NULL || 
            json_mesh_pass == NULL || json_mqtt_url == NULL || json_firmware_url == NULL){
            printf("Some parameter has not been initialized yet\n");
            for(int countdown = 10; countdown >= 0; countdown--) {
                ESP_LOGI(TAG, "%d...", countdown);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            ESP_LOGI(TAG, "Starting again!");
        }

        else if(json_gateway_essid->valuestring == NULL || json_gateway_pass->valuestring == NULL || json_mesh_id->valuestring == NULL || 
            json_mesh_pass->valuestring == NULL || json_mqtt_url->valuestring == NULL || json_firmware_url->valuestring == NULL){
            printf("Some parameter has NULL values\n");
            for(int countdown = 10; countdown >= 0; countdown--) {
                ESP_LOGI(TAG, "%d...", countdown);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            ESP_LOGI(TAG, "Starting again!");
        }

        else{
            printf("Configuration: \n");
            printf("Gateway ESSID: %s\n", json_gateway_essid->valuestring);
            printf("Gateway password: %s\n", json_gateway_pass->valuestring);
            printf("Mesh ID: %s\n", json_mesh_id->valuestring);
            printf("Mesh pass: %s\n", json_mesh_pass->valuestring);
            printf("MQTT URL: %s\n", json_mqtt_url->valuestring);
            printf("Firmware URL: %s\n", json_firmware_url->valuestring);
            printf("Update NVS variable and reboot\n");
            printf("Opening Non-Volatile Storage (NVS) handle... ");

            esp_err_t err;
            nvs_handle_t my_handle;
            err = nvs_open("storage", NVS_READWRITE, &my_handle);
            if (err != ESP_OK) {
                printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
            } else {
                printf("Done\n");
            }
            printf("Setting the value to 1\n");
            int boot_start = 1;
            err = nvs_set_i32(my_handle,"boot_start",boot_start);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            printf("Updating Parameters\n");

            err = nvs_set_str(my_handle, "mesh_id", json_mesh_id->valuestring);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            err = nvs_set_str(my_handle, "mesh_password", json_mesh_pass->valuestring);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            err = nvs_set_str(my_handle, "Gateway_ESSID", json_gateway_essid->valuestring);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            err = nvs_set_str(my_handle, "MQTT_URL", json_mqtt_url->valuestring);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            err = nvs_set_str(my_handle, "Firmware_URL", json_firmware_url->valuestring);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            err = nvs_set_str(my_handle, "Gateway_Pass", json_gateway_pass->valuestring);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

            printf("Committing updates in NVS ... ");
            err = nvs_commit(my_handle);
            nvs_close(my_handle);

            ESP_LOGI(TAG, "Restarting in 10 seconds!");
            for(int countdown = 10; countdown >= 0; countdown--) {
                ESP_LOGI(TAG, "%d...", countdown);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            fflush(stdout);
            esp_restart(); 
        }
    }
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
                 xTaskCreate(&https_get_task, "https_get_task", 8192, NULL, 5, NULL);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

// FINISH HTTPS CLIENT CODE +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void root_write_task(void *arg)
{
    mdf_err_t ret = MDF_OK;
    char *data = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    size_t size = MWIFI_PAYLOAD_LEN;
    uint8_t src_addr[MWIFI_ADDR_LEN] = { 0 };
    mwifi_data_type_t data_type = { 0 };

    MDF_LOGI("Root write task is running");

    while (esp_mesh_is_root()) {
        size = MWIFI_PAYLOAD_LEN;
        memset(data, 0, MWIFI_PAYLOAD_LEN);
        if (!mesh_mqtt_is_connect()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        /**
         * @brief Recv data from node, and forward to mqtt server.
         */
        ret = mwifi_root_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        //MDF_ERROR_GOTO(ret != MDF_OK, MEM_FREE, "<%s> mwifi_root_read", mdf_err_to_name(ret));
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_root_recv", mdf_err_to_name(ret));
        if (data_type.upgrade){
            ret = mupgrade_root_handle(src_addr,data,size);
            MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mupgrade_root_handle", mdf_err_to_name(ret));
        } else {
            ret = mesh_mqtt_write(src_addr, data, size, MESH_MQTT_DATA_JSON);
            //MDF_ERROR_GOTO(ret != MDF_OK, MEM_FREE, "<%s> mesh_mqtt_publish", mdf_err_to_name(ret));
            MDF_LOGI("Receive [NODE] addr: " MACSTR ", size: %d, data: %s",
                     MAC2STR(src_addr), size, data);
//MEM_FREE:
        //MDF_FREE(data);
        }        
    }
        
    MDF_LOGW("Root write task is exit");
    mesh_mqtt_stop();
    MDF_FREE(data);
    vTaskDelete(NULL);
}

static void root_read_task(void *arg)
{
    mdf_err_t ret = MDF_OK;
    
    MDF_LOGI("Root read task is running");

    while (mwifi_is_connected() && esp_mesh_get_layer() == MESH_ROOT) {
        if (!mesh_mqtt_is_connect()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        mesh_mqtt_data_t *request = NULL;
        mwifi_data_type_t data_type = { 0x0 };
        
        /**
         * @brief Recv data from mqtt data queue, and forward to special device.
         */

        ret = mesh_mqtt_read(&request, pdMS_TO_TICKS(500));
        //MDF_LOGI("MESH_MQTT_READ");

        if (ret != MDF_OK) {
            continue;
        }
        uint8_t mac[MWIFI_ADDR_LEN];
        
        MDF_LOGI("Root Received data from mqtt data queue");
        if (strcmp(request->subtype, "Root_Election") == 0){
            MDF_LOGI("Election of a new Root");

            //mesh_vote_t* vote = (mesh_vote_t*)malloc(sizeof(mesh_vote_t));
            //vote->percentage = 0.8;
            //vote->is_rc_specified = false;

            ret = esp_mesh_waive_root(NULL, MESH_VOTE_REASON_ROOT_INITIATED);
            //ret = esp_mesh_waive_root(vote, MESH_VOTE_REASON_ROOT_INITIATED);
            if (ret != MDF_OK){
                printf("Error: %s\n",mdf_err_to_name(ret));
            }
            //MDF_ERROR_GOTO(ret != MDF_OK, MEM_FREE, "<%s> mwifi_root_write", mdf_err_to_name(ret));
        }


        else if (strcmp(request->subtype,"update") == 0){

            MDF_LOGI("Update");
            mdf_err_t ret = MDF_OK;
            uint8_t *data_1       = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
            char name[32]       = {0x0};
            size_t total_size   = 0;
            int start_time      = 0;
            mupgrade_result_t upgrade_result = {0};
            mwifi_data_type_t data_type_1 = {.communicate = MWIFI_COMMUNICATE_MULTICAST};
            int n = request->addrs_num;
            uint8_t dest_addr_update[n][MWIFI_ADDR_LEN];
            
            for (int i =0;i<n;i++){
                memcpy(mac,request->addrs_list + i * MWIFI_ADDR_LEN, MWIFI_ADDR_LEN);
                printf("Address: "MACSTR"\n",MAC2STR(mac));
                memcpy(dest_addr_update[i],mac,MWIFI_ADDR_LEN);
            }

            esp_http_client_config_t config = {
                .url            = CONFIG_FIRMWARE_UPGRADE_URL,
                .transport_type = HTTP_TRANSPORT_UNKNOWN,
            };
            printf("Opening Non-Volatile Storage (NVS) handle... ");
            esp_err_t err;
            nvs_handle_t my_handle_1;
            err = nvs_open("storage", NVS_READWRITE, &my_handle_1);
            if (err != ESP_OK) {
                printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
            } else {
                printf("Done\n");
            }
            size_t size_4;
            //printf("first nvs_get_blob \n");
            err = nvs_get_str(my_handle_1, "Firmware_URL", NULL, &size_4);
            if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
                printf("Error (%s): ", esp_err_to_name(err));
            }
            char* value_4 = malloc(size_4);
            //printf("second nvs_get_blob \n");
            err = nvs_get_str(my_handle_1, "Firmware_URL", value_4, &size_4);
            switch (err) {
                case ESP_OK:
                    printf("Done\n");
                    printf("Firmware URL= %s\n", value_4);
                break;
                case ESP_ERR_NVS_NOT_FOUND:
                    printf("The value is not initialized yet!\n");
                break;
                default :
                    printf("Error (%s) reading!\n", esp_err_to_name(err));
            }
            nvs_close(my_handle_1);
            strcpy((char *)config.url, value_4);
            /**
            * @brief 1. Connect to the server
            */
           
            MDF_LOGI("CONNECT TO THE SERVER");
            esp_http_client_handle_t client = esp_http_client_init(&config);
            MDF_ERROR_GOTO(!client, EXIT, "Initialise HTTP connection");

            start_time = xTaskGetTickCount();

            MDF_LOGI("Open HTTP connection: %s", value_4);

            /**
            * @brief First, the firmware is obtained from the http server and stored on the root node.
            */
           
            do {
                ret = esp_http_client_open(client, 0);

                if (ret != MDF_OK) {
                    if (!esp_mesh_is_root()) {
                        MDF_LOGI("NODE IS NOT ROOT --> Go to EXIT");
                        goto EXIT;
                    }

                    vTaskDelay(pdMS_TO_TICKS(1000));
                    MDF_LOGW("<%s> Connection service failed", mdf_err_to_name(ret));
                }
            } while (ret != MDF_OK);

            total_size = esp_http_client_fetch_headers(client);
            sscanf(value_4, "%*[^//]//%*[^/]/%[^.bin]", name);
            free(value_4);

            if (total_size <= 0) {
                MDF_LOGW("Please check the address of the server");
                ret = esp_http_client_read(client, (char *)data_1, MWIFI_PAYLOAD_LEN);
                MDF_ERROR_GOTO(ret < 0, EXIT, "<%s> Read data from http stream", mdf_err_to_name(ret));

                MDF_LOGW("Recv data: %.*s", ret, data_1);
                MDF_LOGI("TOTAL SIZE <= 0 --> Go to EXIT");
                goto EXIT;
            }

            /**
            * @brief 2. Initialize the upgrade status and erase the upgrade partition.
            */
           
            MDF_LOGI("INITIALIZE THE UPGRADE STATUS AND ERASE THE UPGRADE PARTITION");
            ret = mupgrade_firmware_init(name, total_size);
            MDF_ERROR_GOTO(ret != MDF_OK, EXIT, "<%s> Initialize the upgrade status", mdf_err_to_name(ret));

            /**
            * @brief 3. Read firmware from the server and write it to the flash of the root node
            */
           
            MDF_LOGI("READ FIRMWARE FROM THE SERVER AND WRITE IT TO THE FLASH OF THE ROOT NODE");
            for (ssize_t size = 0, recv_size = 0; recv_size < total_size; recv_size += size) {
                size = esp_http_client_read(client, (char *)data_1, MWIFI_PAYLOAD_LEN);
                MDF_ERROR_GOTO(size < 0, EXIT, "<%s> Read data from http stream", mdf_err_to_name(ret));

                if (size > 0) {
                    
                /* @brief  Write firmware to flash */
                    
                    ret = mupgrade_firmware_download(data_1, size);
                    MDF_ERROR_GOTO(ret != MDF_OK, EXIT, "<%s> Write firmware to flash, size: %d, data: %.*s",
                                   mdf_err_to_name(ret), size, size, data_1);
                } else {
                    MDF_LOGI("SIZE > 0 --> Go to EXIT");
                    MDF_LOGW("<%s> esp_http_client_read", mdf_err_to_name(ret));
                    goto EXIT;
                }
            }

            MDF_LOGI("The service download firmware is complete, Spend time: %ds",
                     (xTaskGetTickCount() - start_time) * portTICK_RATE_MS / 1000);

            start_time = xTaskGetTickCount();

            /**
            * @brief 4. The firmware will be sent to each node.
            */
           
            MDF_LOGI("THE FIRMWARE WILL BE SENT TO EACH NODE");
            ret = mupgrade_firmware_send((uint8_t *)dest_addr_update, sizeof(dest_addr_update) / MWIFI_ADDR_LEN, &upgrade_result);
            MDF_ERROR_GOTO(ret != MDF_OK, EXIT, "<%s> mupgrade_firmware_send", mdf_err_to_name(ret));
            
            if (upgrade_result.successed_num == 0) {
                MDF_LOGI("dEVICE UPGRADE FAILED --> Go to EXIT");
                MDF_LOGW("Devices upgrade failed, unfinished_num: %d", upgrade_result.unfinished_num);
                goto EXIT;
            }

            MDF_LOGI("Firmware is sent to the device to complete, Spend time: %ds",
                     (xTaskGetTickCount() - start_time) * portTICK_RATE_MS / 1000);
            MDF_LOGI("Devices upgrade completed, successed_num: %d, unfinished_num: %d", upgrade_result.successed_num, upgrade_result.unfinished_num);

            /**
            * @brief 5. the root notifies nodes to restart
            */
           
            MDF_LOGI("THE ROOT NOTIFIES NODES TO RESTART");
            const char *restart_str = "restart";
            ret = mwifi_root_write(upgrade_result.successed_addr, upgrade_result.successed_num,
                                   &data_type_1, restart_str, strlen(restart_str), true);
            MDF_ERROR_GOTO(ret != MDF_OK, EXIT, "<%s> mwifi_root_recv", mdf_err_to_name(ret));

        EXIT:
            MDF_FREE(data_1);
            mupgrade_result_free(&upgrade_result);
            printf("erasing ota_1\n");
            nvs_flash_erase_partition("ota_1");
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            //vTaskDelete(NULL);

        } else {    
            MDF_LOGI("Forward to special device");
            ret = mwifi_root_write(request->addrs_list, request->addrs_num, &data_type, request->data, request->size, true);
            MDF_ERROR_GOTO(ret != MDF_OK, MEM_FREE, "<%s> mwifi_root_write", mdf_err_to_name(ret));
        }

MEM_FREE:
        MDF_FREE(request->addrs_list);
        MDF_FREE(request->data);
        MDF_FREE(request);
    }

    MDF_LOGW("Root read task is exit");
    mesh_mqtt_stop();
    vTaskDelete(NULL);
}

static void node_read_task(void *arg)
{
    mdf_err_t ret = MDF_OK;
    char *data = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    size_t size = MWIFI_PAYLOAD_LEN;
    mwifi_data_type_t data_type = {0x0};
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0};
    double voltage = 0;
    uint8_t max_rotation_time = 0;

    MDF_LOGI("Node read task is running");

    //while (mwifi_is_connected()) {
    for(;;){
        if (!mwifi_is_connected()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        size = MWIFI_PAYLOAD_LEN;
        memset(data, 0, MWIFI_PAYLOAD_LEN);
        ret = mwifi_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_root_recv", mdf_err_to_name(ret));

        if (data_type.upgrade) { // This mesh package contains upgrade data.
            MDF_LOGI("MUPGRADE");
            ret = mupgrade_handle(src_addr, data, size);
            MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mupgrade_handle", mdf_err_to_name(ret));
        } else {
            MDF_LOGI("Receive [ROOT] addr: " MACSTR ", size: %d, data: %s",
                     MAC2STR(src_addr), size, data);

            /**
             * @brief Finally, the node receives a restart notification. Restart it yourself..
             */
            if (!strcmp(data, "restart")) {
                MDF_LOGI("Restart the version of the switching device");
                MDF_LOGW("The device will restart after 3 seconds");
                vTaskDelay(pdMS_TO_TICKS(3000));
                esp_restart();
            }
            else{
                MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_read", mdf_err_to_name(ret));
                MDF_LOGI("Node receive: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);
                printf("RECEIVED DATA:%s\n",data);
                if(strcmp(data,"\"accensione\"") == 0){
                printf("ACCENSIONE\n");
                gpio_set_level(GPIO_VALV,1);
                //     while ((voltage < 350 && max_rotation_time < 45) || max_rotation_time == 1) {
                //         gpio_set_level(GPIO_RIGHT, 1);
                //         gpio_set_level(GPIO_LEFT, 0);
                //         double adc_reading = 0;
                //         //Multisampling
                //         for (int i = 0; i < NO_OF_SAMPLES; i++) {
                //             if (unit == ADC_UNIT_1) {
                //             adc_reading += adc1_get_raw((adc1_channel_t)channel);
                //             } else {
                //             int raw;
                //             adc2_get_raw((adc2_channel_t)channel, width, &raw);
                //             adc_reading += raw;
                //             }
                //         }
                //         adc_reading /= NO_OF_SAMPLES;
                //         voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
                //         printf("Raw: %f\tVoltage: %fmV\n", adc_reading, voltage);
                //         max_rotation_time = max_rotation_time + 1;
                //         vTaskDelay(pdMS_TO_TICKS(500));
                //     }
                //     voltage = 0;
                //     max_rotation_time = 0;
                //     vTaskDelay(5000 / portTICK_PERIOD_MS);
                //     gpio_set_level(GPIO_RIGHT, 0);
                //     gpio_set_level(GPIO_LEFT, 0);
                 }
                else if(strcmp(data,"\"spegnimento\"") == 0){
                    printf("SPEGNIMENTO\n");
                    gpio_set_level(GPIO_VALV,0);
                    // gpio_set_level(GPIO_RIGHT, 0);
                    // gpio_set_level(GPIO_LEFT, 1);
                    // vTaskDelay(17000 / portTICK_PERIOD_MS);
                    // gpio_set_level(GPIO_RIGHT, 0);
                    // gpio_set_level(GPIO_LEFT, 0);
                }
            }
        }
    }

    MDF_LOGW("Node read task is exit");
    MDF_FREE(data);
    vTaskDelete(NULL);
}

static void node_write_task(void *arg)
{
    mdf_err_t ret = MDF_OK;
    size_t size = 0;
    char *data = NULL;
    mwifi_data_type_t data_type = { 0x0 };
    uint8_t sta_mac[MWIFI_ADDR_LEN] = { 0 };
    mesh_addr_t parent_mac = { 0 };

    //SENDING CHILD LIST
    char mac_addr[13];
    int table_size = 0;
    mesh_addr_t *route_table = NULL;

    pct2075_initialization();

    MDF_LOGI("Node task is running");

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);

    for (;;) {
        if (!mwifi_is_connected() || !mwifi_get_root_status()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        /**
         * @brief Send device information to mqtt server throught root node.
         */
        
        cJSON *obj = cJSON_CreateArray();
        table_size = esp_mesh_get_routing_table_size();
        route_table = MDF_CALLOC(table_size, sizeof(mesh_addr_t));
        ESP_ERROR_CHECK(esp_mesh_get_routing_table(route_table, table_size * sizeof(mesh_addr_t), &table_size));
        
        for (int i = 0; i < table_size; i++) {
            mlink_mac_hex2str(route_table[i].addr, mac_addr);
            cJSON *item = cJSON_CreateString(mac_addr);
            //MDF_ERROR_GOTO(item == NULL, _no_mem, "Create string object failed");
            cJSON_AddItemToArray(obj, item);
        }
        MDF_FREE(route_table);
        char *str = cJSON_PrintUnformatted(obj);
        esp_mesh_get_parent_bssid(&parent_mac);
        size = asprintf(&data, "{\"Temperature_Low_Sensor\":%f, \"Temperature_Medium_Sensor\":%f,\"self\": \"%02x%02x%02x%02x%02x%02x\", \"parent\":\"%02x%02x%02x%02x%02x%02x\",\"layer\":%d,\"Root\":%d,\"Rssi\":%d,\"Child_List\":%s}",
                        pct2075d_read_temperature(73), pct2075d_read_temperature(72), MAC2STR(sta_mac), MAC2STR(parent_mac.addr), esp_mesh_get_layer(), esp_mesh_is_root(), mwifi_get_parent_rssi(), str);

        MDF_LOGD("Node send, size: %d, data: %s", size, data);
        ret = mwifi_write(NULL, &data_type, data, size, true);
        MDF_FREE(data);
        cJSON_Delete(obj);
        MDF_FREE(str);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));

        vTaskDelay(3000 / portTICK_RATE_MS);
    }

    MDF_LOGW("Node task is exit");
    vTaskDelete(NULL);
}

static mdf_err_t wifi_init()
{   
    // const uint8_t protocol = WIFI_PROTOCOL_LR;
    // ESP_ERROR_CHECK( esp_wifi_set_protocol(WIFI_IF_STA, protocol) );
    mdf_err_t ret = nvs_flash_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        MDF_ERROR_ASSERT(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    MDF_ERROR_ASSERT(ret);

    MDF_ERROR_ASSERT(esp_netif_init());
    MDF_ERROR_ASSERT(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&sta_netif, NULL));
    MDF_ERROR_ASSERT(esp_wifi_init(&cfg));
    MDF_ERROR_ASSERT(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    MDF_ERROR_ASSERT(esp_wifi_set_mode(WIFI_MODE_STA));
    MDF_ERROR_ASSERT(esp_wifi_set_ps(WIFI_PS_NONE));
    MDF_ERROR_ASSERT(esp_mesh_set_6m_rate(false));
    MDF_ERROR_ASSERT(esp_wifi_start());
    //MDF_ERROR_ASSERT(esp_wifi_set_max_tx_power(78));

    return MDF_OK;
}

/**
 * @brief All module events will be sent to this task in esp-mdf
 *
 * @Note:
 *     1. Do not block or lengthy operations in the callback function.
 *     2. Do not consume a lot of memory in the callback function.
 *        The task memory of the callback function is only 4KB.
 */
static mdf_err_t event_loop_cb(mdf_event_loop_t event, void *ctx)
{
    MDF_LOGI("event_loop_cb, event: %d", event);

    switch (event) {
        case MDF_EVENT_MWIFI_STARTED:
            MDF_LOGI("MESH is started");
            break;

        case MDF_EVENT_MWIFI_PARENT_CONNECTED:
            MDF_LOGI("Parent is connected on station interface");
            gpio_set_level(GPIO_MESH, 1);
            if (esp_mesh_is_root()) {
                esp_netif_dhcpc_start(sta_netif);
            }
            break;

        case MDF_EVENT_MWIFI_PARENT_DISCONNECTED:
            MDF_LOGI("Parent is disconnected on station interface");
            gpio_set_level(GPIO_MESH, 0);
            if (esp_mesh_is_root()) {
                gpio_set_level(GPIO_ROOT, 0);
                mesh_mqtt_stop();
            }
            break;

        case MDF_EVENT_MWIFI_ROUTING_TABLE_ADD:
        
        case MDF_EVENT_MWIFI_ROUTING_TABLE_REMOVE:
            MDF_LOGI("MDF_EVENT_MWIFI_ROUTING_TABLE_REMOVE, total_num: %d", esp_mesh_get_total_node_num());
            if (esp_mesh_is_root() && mwifi_get_root_status()) {
                mdf_err_t err = mesh_mqtt_update_topo();
                if (err != MDF_OK) {
                    MDF_LOGE("Update topo failed");
                }
            }

            break;

        case MDF_EVENT_MWIFI_ROOT_GOT_IP: {
            MDF_LOGI("Root obtains the IP address. It is posted by LwIP stack automatically");
            gpio_set_level(GPIO_ROOT, 1);
            printf("Opening Non-Volatile Storage (NVS) handle... ");
            nvs_handle_t my_handle_2;
            esp_err_t err;
            err = nvs_open("storage", NVS_READWRITE, &my_handle_2);
            if (err != ESP_OK) {
                printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
            } else {
                printf("Done\n");
            }
            size_t size_5;
            //printf("first nvs_get_blob \n");
            err = nvs_get_str(my_handle_2, "MQTT_URL", NULL, &size_5);
            if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
            char* value_5 = malloc(size_5);
            //printf("second nvs_get_blob \n");
            err = nvs_get_str(my_handle_2, "MQTT_URL", value_5, &size_5);
            switch (err) {
                case ESP_OK:
                    printf("Done\n");
                    printf("MQTT URL = %s\n", value_5);
                break;
                case ESP_ERR_NVS_NOT_FOUND:
                    printf("The value is not initialized yet!\n");
                break;
                default :
                    printf("Error (%s) reading!\n", esp_err_to_name(err));
            }
            nvs_close(my_handle_2);
            
            mesh_mqtt_start(value_5); 
            free(value_5);

            xTaskCreate(root_write_task, "root_write", 4 * 1024,
                    NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);  
            xTaskCreate(root_read_task, "root_read", 4 * 1024,
                    NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);            
            break;
        }

        case MDF_EVENT_CUSTOM_MQTT_CONNECT:
            MDF_LOGI("MQTT connect");
            mdf_err_t err = mesh_mqtt_update_topo();
            if (err != MDF_OK) {
                MDF_LOGE("Update topo failed");
            }
            err = mesh_mqtt_subscribe();
            if (err != MDF_OK) {
                MDF_LOGE("Subscribe failed");
            }
            mwifi_post_root_status(true);

            break;

        case MDF_EVENT_CUSTOM_MQTT_DISCONNECT:
            MDF_LOGI("MQTT disconnected");
            mwifi_post_root_status(false);
            break;

        case MDF_EVENT_MUPGRADE_STARTED: {
            mupgrade_status_t status = {0x0};
            mupgrade_get_status(&status);
            MDF_LOGI("MDF_EVENT_MUPGRADE_STARTED, name: %s, size: %d",
                     status.name, status.total_size);
            break;
        }

        case MDF_EVENT_MUPGRADE_STATUS:
            MDF_LOGI("Upgrade progress: %d%%", (int)ctx);
            break;

        default:
            break;
    }

    return MDF_OK;
}

void app_main()
{	
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    // Open
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");
    }
    // Read
        printf("Reading boot start script from NVS ... ");
        int32_t boot_start = 0; // value will default to 0, if not set yet in NVS
        err = nvs_get_i32(my_handle, "boot_start", &boot_start);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("Boot Start Variable = %d\n", boot_start);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                printf("Setting the value to 0\n");
                err = nvs_set_i32(my_handle,"boot_start",boot_start);
                printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                printf("Committing updates in NVS ... ");
                err = nvs_commit(my_handle);
                printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
        if (boot_start == 0){
            // if the value in flash is not initialized, then start as STA
            //else if the value is initialized and equal to 1, start as VME
            nvs_close(my_handle);

            wifi_init_sta();
            
        } else{
            size_t size;
            //printf("first nvs_get_blob \n");
            err = nvs_get_str(my_handle, "mesh_id", NULL, &size);
            if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
                printf("Error: %s\n", esp_err_to_name(err));
            }
            char* value = malloc(size);
            //printf("second nvs_get_blob \n");
            err = nvs_get_str(my_handle, "mesh_id", value, &size);
            switch (err) {
                case ESP_OK:
                    printf("Done\n");
                    printf("MESH ID = %s\n", value);
                break;
                case ESP_ERR_NVS_NOT_FOUND:
                    printf("The value is not initialized yet!\n");
                break;
                default :
                    printf("Error (%s) reading!\n", esp_err_to_name(err));
            }

            size_t size_1;
            //printf("first nvs_get_blob \n");
            err = nvs_get_str(my_handle, "mesh_password", NULL, &size_1);
            if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
                printf("Error: %s\n", esp_err_to_name(err));
            }
            char* value_1 = malloc(size_1);
            //printf("second nvs_get_blob \n");
            err = nvs_get_str(my_handle, "mesh_password", value_1, &size_1);
            switch (err) {
                case ESP_OK:
                    printf("Done\n");
                    printf("MESH PASSWORD = %s\n", value_1);
                break;
                case ESP_ERR_NVS_NOT_FOUND:
                    printf("The value is not initialized yet!\n");
                break;
                default :
                    printf("Error (%s) reading!\n", esp_err_to_name(err));
            }

            size_t size_2;
            //printf("first nvs_get_blob \n");
            err = nvs_get_str(my_handle, "Gateway_ESSID", NULL, &size_2);
            if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
                printf("Error: %s\n", esp_err_to_name(err));
            }
            char* value_2 = malloc(size_2);
            //printf("second nvs_get_blob \n");
            err = nvs_get_str(my_handle, "Gateway_ESSID", value_2, &size_2);
            switch (err) {
                case ESP_OK:
                    printf("Done\n");
                    printf("Gateway ESSID = %s\n", value_2);
                break;
                case ESP_ERR_NVS_NOT_FOUND:
                    printf("The value is not initialized yet!\n");
                break;
                default :
                    printf("Error (%s) reading!\n", esp_err_to_name(err));
            }
            size_t size_3;
            //printf("first nvs_get_blob \n");
            err = nvs_get_str(my_handle, "Gateway_Pass", NULL, &size_3);
            if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
                printf("Error: %s\n", esp_err_to_name(err));
            }
            char* value_3 = malloc(size_3);
            //printf("second nvs_get_blob \n");
            err = nvs_get_str(my_handle, "Gateway_Pass", value_3, &size_3);
            switch (err) {
                case ESP_OK:
                    printf("Done\n");   
                    printf("Gateway PASSWORD = %s\n", value_3);
                break;
                case ESP_ERR_NVS_NOT_FOUND:
                    printf("The value is not initialized yet!\n");
                break;
                default :
                    printf("Error (%s) reading!\n", esp_err_to_name(err));
            }

            nvs_close(my_handle);
            check_efuse();
            //Configure ADC
            if (unit == ADC_UNIT_1) {
                adc1_config_width(width);
                adc1_config_channel_atten(channel, atten);
            } else {
                adc2_config_channel_atten((adc2_channel_t)channel, atten);
            }
            //Characterize ADC
            adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
            esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
            print_char_val_type(val_type);
            // gpio_pad_select_gpio(GPIO_RIGHT);
            gpio_pad_select_gpio(GPIO_VALV);
            // gpio_pad_select_gpio(GPIO_LEFT);
            gpio_pad_select_gpio(GPIO_MESH);
            gpio_pad_select_gpio(GPIO_ROOT);
            /* Set the GPIO as a push/pull output */
            // gpio_set_direction(GPIO_RIGHT, GPIO_MODE_OUTPUT);
            gpio_set_direction(GPIO_VALV, GPIO_MODE_OUTPUT);
            //gpio_set_direction(GPIO_LEFT, GPIO_MODE_OUTPUT);
            gpio_set_direction(GPIO_MESH, GPIO_MODE_OUTPUT);
            gpio_set_direction(GPIO_ROOT, GPIO_MODE_OUTPUT);
            mwifi_init_config_t cfg = MWIFI_INIT_CONFIG_DEFAULT();
            mwifi_config_t config = {
                .router_ssid = CONFIG_GATEWAY_ESSID,
                .router_password = CONFIG_GATEWAY_PASSWORD,
                .mesh_id = CONFIG_MESH_ID,
                .mesh_password = CONFIG_MESH_PASSWORD,
                //.mesh_type = CONFIG_DEVICE_TYPE,
            };
            strcpy((char *)config.mesh_id, value);
            strcpy((char *)config.mesh_password, value_1);
            strcpy((char *)config.router_ssid, value_2);
            strcpy((char *)config.router_password, value_3);

            free(value);
            free(value_1);
            free(value_2);
            free(value_3);

    /**
     * @brief Set the log level for serial port printing.
     */
            esp_log_level_set("*", ESP_LOG_INFO);
            esp_log_level_set(TAG, ESP_LOG_DEBUG);
            esp_log_level_set("mesh_mqtt", ESP_LOG_DEBUG);
            esp_log_level_set("mupgrade_node", ESP_LOG_DEBUG);
            esp_log_level_set("mupgrade_root", ESP_LOG_DEBUG);
    

    /**
     * @brief Initialize wifi mesh.
     */
            MDF_ERROR_ASSERT(mdf_event_loop_init(event_loop_cb));
            MDF_ERROR_ASSERT(wifi_init());
            MDF_ERROR_ASSERT(mwifi_init(&cfg));
            MDF_ERROR_ASSERT(mwifi_set_config(&config));
            MDF_ERROR_ASSERT(mwifi_start());

    /**
     * @brief Create node handler
     */
    xTaskCreate(node_write_task, "node_write_task", 4 * 1024,
                NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    xTaskCreate(node_read_task, "node_read_task", 4 * 1024,
                NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);

    //TimerHandle_t timer = xTimerCreate("print_system_info", 10000 / portTICK_RATE_MS,
    //                                   true, NULL, print_system_info_timercb);
    //xTimerStart(timer, 0);
        }
}
