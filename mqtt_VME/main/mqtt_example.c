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
#include "driver/uart.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "string.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "math.h"
#include <stdlib.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define MEMORY_DEBUG
#define GPIO_RIGHT 26
#define GPIO_LEFT 27
#define DEFAULT_VREF 1100
#define NO_OF_SAMPLES 64

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_4;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

static const char *TAG = "mqtt_VME_Node";
esp_netif_t *sta_netif;

static float pct2075d_read_temperature(int slave_address);
static uint16_t read_value(int slave_address);
static int findComplement(int n);

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

int findComplement(int n){
    int bit_count = floor(log2(n))+1;
    int ones = ((1 << bit_count) - 1);
    return ones ^ n;
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
        
        else if(strcmp(request->subtype,"update") == 0){

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
            uint8_t dest_addr_update_1[][MWIFI_ADDR_LEN] = {MWIFI_ADDR_ANY};
            for (int i =0;i<n;i++){
                memcpy(mac,request->addrs_list + i * MWIFI_ADDR_LEN, MWIFI_ADDR_LEN);
                printf("Address: "MACSTR"\n",MAC2STR(mac));
                memcpy(dest_addr_update[i],mac,MWIFI_ADDR_LEN);
            }

            esp_http_client_config_t config = {
                .url            = CONFIG_FIRMWARE_UPGRADE_URL,
                .transport_type = HTTP_TRANSPORT_UNKNOWN,
            };
            
            /**
            * @brief 1. Connect to the server
            */
           
            MDF_LOGI("CONNECT TO THE SERVER");
            esp_http_client_handle_t client = esp_http_client_init(&config);
            MDF_ERROR_GOTO(!client, EXIT, "Initialise HTTP connection");

            start_time = xTaskGetTickCount();

            MDF_LOGI("Open HTTP connection: %s", CONFIG_FIRMWARE_UPGRADE_URL);

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
            sscanf(CONFIG_FIRMWARE_UPGRADE_URL, "%*[^//]//%*[^/]/%[^.bin]", name);

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
                    while ((voltage < 350 && max_rotation_time < 45) || max_rotation_time == 1) {
                        gpio_set_level(GPIO_RIGHT, 1);
                        gpio_set_level(GPIO_LEFT, 0);
                        double adc_reading = 0;
                        //Multisampling
                        for (int i = 0; i < NO_OF_SAMPLES; i++) {
                            if (unit == ADC_UNIT_1) {
                            adc_reading += adc1_get_raw((adc1_channel_t)channel);
                            } else {
                            int raw;
                            adc2_get_raw((adc2_channel_t)channel, width, &raw);
                            adc_reading += raw;
                            }
                        }
                        adc_reading /= NO_OF_SAMPLES;
                        voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
                        printf("Raw: %f\tVoltage: %fmV\n", adc_reading, voltage);
                        max_rotation_time = max_rotation_time + 1;
                        vTaskDelay(pdMS_TO_TICKS(500));
                    }
                    voltage = 0;
                    max_rotation_time = 0;
                    vTaskDelay(5000 / portTICK_PERIOD_MS);
                    gpio_set_level(GPIO_RIGHT, 0);
                    gpio_set_level(GPIO_LEFT, 0);
                }
                else if(strcmp(data,"\"spegnimento\"") == 0){
                    printf("SPEGNIMENTO\n");
                    gpio_set_level(GPIO_RIGHT, 0);
                    gpio_set_level(GPIO_LEFT, 1);
                    vTaskDelay(17000 / portTICK_PERIOD_MS);
                    gpio_set_level(GPIO_RIGHT, 0);
                    gpio_set_level(GPIO_LEFT, 0);
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
        
        esp_mesh_get_parent_bssid(&parent_mac);
        size = asprintf(&data, "{\"Temperature_Low_Sensor\":%f, \"Temperature_Medium_Sensor\":%f,\"self\": \"%02x%02x%02x%02x%02x%02x\", \"parent\":\"%02x%02x%02x%02x%02x%02x\",\"layer\":%d,\"Root\":%d}",
                        pct2075d_read_temperature(73), pct2075d_read_temperature(72), MAC2STR(sta_mac), MAC2STR(parent_mac.addr), esp_mesh_get_layer(), esp_mesh_is_root());

        MDF_LOGD("Node send, size: %d, data: %s", size, data);
        ret = mwifi_write(NULL, &data_type, data, size, true);
        MDF_FREE(data);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));

        vTaskDelay(30000 / portTICK_RATE_MS);
    }

    MDF_LOGW("Node task is exit");
    vTaskDelete(NULL);
}

/**
 * @brief Timed printing system information
 */
static void print_system_info_timercb(void *timer)
{
    uint8_t primary = 0;
    wifi_second_chan_t second = 0;
    mesh_addr_t parent_bssid = { 0 };
    uint8_t sta_mac[MWIFI_ADDR_LEN] = { 0 };
    mesh_assoc_t mesh_assoc = { 0x0 };
    wifi_sta_list_t wifi_sta_list = { 0x0 };

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_wifi_get_channel(&primary, &second);
    esp_wifi_vnd_mesh_get(&mesh_assoc);
    esp_mesh_get_parent_bssid(&parent_bssid);

    MDF_LOGI("System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR
             ", parent rssi: %d, node num: %d, free heap: %u",
             primary,
             esp_mesh_get_layer(), MAC2STR(sta_mac), MAC2STR(parent_bssid.addr),
             mesh_assoc.rssi, esp_mesh_get_total_node_num(), esp_get_free_heap_size());

    for (int i = 0; i < wifi_sta_list.num; i++) {
        MDF_LOGI("Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
    }

#ifdef MEMORY_DEBUG

    if (!heap_caps_check_integrity_all(true)) {
        MDF_LOGE("At least one heap is corrupt");
    }

    //mdf_mem_print_heap();
    mdf_mem_print_record();
    //mdf_mem_print_task();
#endif /**< MEMORY_DEBUG */
}

static mdf_err_t wifi_init()
{
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

            if (esp_mesh_is_root()) {
                esp_netif_dhcpc_start(sta_netif);
            }
              
            break;

        case MDF_EVENT_MWIFI_PARENT_DISCONNECTED:
            MDF_LOGI("Parent is disconnected on station interface");

            if (esp_mesh_is_root()) {
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
            
            mesh_mqtt_start(CONFIG_MQTT_URL);    
            
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
    gpio_pad_select_gpio(GPIO_RIGHT);
    gpio_pad_select_gpio(GPIO_LEFT);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_RIGHT, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_LEFT, GPIO_MODE_OUTPUT);

    mwifi_init_config_t cfg = MWIFI_INIT_CONFIG_DEFAULT();
    mwifi_config_t config = {
        .router_ssid = CONFIG_ROUTER_SSID,
        .router_password = CONFIG_ROUTER_PASSWORD,
        .mesh_id = CONFIG_MESH_ID,
        .mesh_password = CONFIG_MESH_PASSWORD,
        //.mesh_type = CONFIG_DEVICE_TYPE,
    };

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
