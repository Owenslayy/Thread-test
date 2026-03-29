/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * OpenThread Command Line Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "esp_openthread.h"
#include "esp_openthread_cli.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_types.h"
#include "esp_openthread_netif_glue.h"
#include "esp_ot_config.h"
#include "esp_vfs_eventfd.h"
#include "nvs_flash.h"
#include "openthread/thread.h"
#include "openthread/thread_ftd.h"
#include "openthread/instance.h"
#include "openthread/udp.h"
#include "openthread/ip6.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
//#include "ot_examples_common.h"

#if CONFIG_OPENTHREAD_STATE_INDICATOR_ENABLE
#include "ot_led_strip.h"
#endif

#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
#include "esp_ot_cli_extension.h"
#endif // CONFIG_OPENTHREAD_CLI_ESP_EXTENSION

#define TAG "ot_esp_cli"
#define LED_GPIO 8  // ESP32-C6 built-in RGB LED (WS2812)

// UART configuration for leader
#define UART_NUM UART_NUM_1
#define UART_TX_PIN 5
#define UART_RX_PIN 4
#define UART_BUF_SIZE 1024
#define CONTROL_PIN 7  // GPIO 7 for output control

// Global UDP socket and child address
static otUdpSocket sUdpSocket;
static otIp6Address sChildAddr;
static bool sChildAddrSet = false;

// Function to check UART data and control GPIO 7
void check_uart_and_control_pin(uint8_t *data, int len)
{
    if (len > 0) {
        // Check if received data is 0x00
        if (data[0] == 0x00) {
            gpio_set_level(CONTROL_PIN, 1);  // Turn GPIO 7 ON
            ESP_LOGI(TAG, "UART received 0x00 - GPIO %d turned ON", CONTROL_PIN);
        } else {
            gpio_set_level(CONTROL_PIN, 0);  // Turn GPIO 7 OFF for other values
            ESP_LOGI(TAG, "UART received 0x%02X - GPIO %d turned OFF", data[0], CONTROL_PIN);
        }
    }
}

// Initialize UDP socket for sending to child
void init_udp_socket(otInstance *instance)
{
    otError error = otUdpOpen(instance, &sUdpSocket, NULL, NULL);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to open UDP socket: %d", error);
        return;
    }
    ESP_LOGI(TAG, "UDP socket initialized");
}

// Send data to the child device
void send_to_child(otInstance *instance, const uint8_t *data, uint16_t len)
{
    if (!sChildAddrSet) {
        ESP_LOGW(TAG, "Child address not set yet");
        return;
    }
    
    otError error;
    otMessage *message = otUdpNewMessage(instance, NULL);
    if (message == NULL) {
        ESP_LOGE(TAG, "Failed to create UDP message");
        return;
    }
    
    error = otMessageAppend(message, data, len);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to append data: %d", error);
        otMessageFree(message);
        return;
    }
    
    otMessageInfo messageInfo;
    memset(&messageInfo, 0, sizeof(otMessageInfo));
    messageInfo.mPeerAddr = sChildAddr;
    messageInfo.mPeerPort = 12345;
    
    error = otUdpSend(instance, &sUdpSocket, message, &messageInfo);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to send UDP message: %d", error);
        otMessageFree(message);
    } else {
        ESP_LOGI(TAG, "Data sent to child (%d bytes)", len);
    }
}

// Set the child's IPv6 address for sending data
void set_child_address(const otIp6Address *addr)
{
    sChildAddr = *addr;
    sChildAddrSet = true;
    ESP_LOGI(TAG, "Child address set");
}

// LED blink task - works for both router and end device using RGB LED
void led_blink_task(void *pvParameters)
{
    // Configure RGB LED strip
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    
    ESP_LOGI(TAG, "RGB LED task running on GPIO %d", LED_GPIO);
    
    while (1) {
        // Get Thread state to adjust blink pattern
        esp_openthread_lock_acquire(portMAX_DELAY);
        otInstance *instance = esp_openthread_get_instance();
        otDeviceRole role = otThreadGetDeviceRole(instance);
        esp_openthread_lock_release();
        
#ifdef CONFIG_DEVICE_TYPE_END_DEVICE
        // Child: log every 50 blinks (10 seconds) to avoid flooding CLI
        static uint32_t log_counter = 0;
        if (log_counter++ % 50 == 0) {
            ESP_LOGI(TAG, "Device role: %d (0=disabled, 1=detached, 2=child, 3=router, 4=leader)", role);
        }
#else
        // Leader: print role only once when it becomes leader
        static bool role_printed = false;
        if (!role_printed && role == OT_DEVICE_ROLE_LEADER) {
            ESP_LOGI(TAG, "Device role: %d (leader)", role);
            role_printed = true;
        }
#endif
        
        if (role == OT_DEVICE_ROLE_LEADER || role == OT_DEVICE_ROLE_ROUTER) {
            // Router/Leader ready: Fast green blink (200ms cycle)
            led_strip_set_pixel(led_strip, 0, 0, 50, 0);  // Green
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(100));
            led_strip_clear(led_strip);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else if (role == OT_DEVICE_ROLE_CHILD) {
            // End device connected: Medium blue blink (400ms cycle)
            led_strip_set_pixel(led_strip, 0, 0, 0, 50);  // Blue
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(200));
            led_strip_clear(led_strip);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            // Not connected: Slow red blink (1s cycle)
            led_strip_set_pixel(led_strip, 0, 50, 0, 0);  // Red
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(500));
            led_strip_clear(led_strip);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

// UART reading task - only for leader
void uart_read_task(void *pvParameters)
{
    uint8_t *data = (uint8_t *) malloc(UART_BUF_SIZE);
    
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, UART_BUF_SIZE, pdMS_TO_TICKS(2000));
        if (len > 0) {
            ESP_LOGI(TAG, "UART received %d bytes:", len);
            ESP_LOG_BUFFER_HEX(TAG, data, len);
            
            // Check if data is 0x00 and control GPIO 7
            check_uart_and_control_pin(data, len);
            
            // Echo back what was received (optional)
            uart_write_bytes(UART_NUM, (const char *)data, len);
        } else {
            // Print status every 2-3 seconds when no UART data
            ESP_LOGI(TAG, "UART: Waiting for data on GPIO%d...", UART_RX_PIN);
        }
    }
    
    free(data);
}

// Example task: Send test data to child every 10 seconds
void send_data_example_task(void *pvParameters)
{
    // otInstance *instance = (otInstance *)pvParameters;
    // uint8_t test_data[] = {0x48, 0x65, 0x6C, 0x6C, 0x6F};  // "Hello"
    
    vTaskDelay(pdMS_TO_TICKS(5000));  // Wait 5 seconds before first send
    
    // while (1) {
    //     esp_openthread_lock_acquire(portMAX_DELAY);
    //     send_to_child(instance, test_data, sizeof(test_data));
    //     esp_openthread_lock_release();
    //     
    //     vTaskDelay(pdMS_TO_TICKS(10000));  // Send every 10 seconds
    // }
}

void app_main(void)
{
    esp_vfs_eventfd_config_t eventfd_config = {
        .max_fds = 3,
    };

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));

#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_init();
#endif

    const esp_openthread_config_t config = {
        .netif_config = ESP_NETIF_DEFAULT_OPENTHREAD(),
        .platform_config = {
            .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
            .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
            .port_config = {
                .storage_partition_name = "nvs",
                .netif_queue_size = 10,
                .task_queue_size = 10,
            },
        },
    };

    // This starts the OT task in the background
    ESP_ERROR_CHECK(esp_openthread_start(&config));
    
    // Get instance pointer once
    otInstance *instance = esp_openthread_get_instance();

#ifdef CONFIG_DEVICE_TYPE_END_DEVICE
    /* ---------------- END DEVICE CONFIG ---------------- */
    esp_openthread_lock_acquire(portMAX_DELAY);
    
    otLinkModeConfig mode = {.mRxOnWhenIdle = true, .mDeviceType = false, .mNetworkData = false};
    otThreadSetLinkMode(instance, mode);
    otThreadSetChildTimeout(instance, 15);
    
    // Always set the dataset for child
    otOperationalDataset dataset;
    memset(&dataset, 0, sizeof(otOperationalDataset));
    dataset.mActiveTimestamp.mSeconds = 1;
    dataset.mComponents.mIsActiveTimestampPresent = true;
    strcpy(dataset.mNetworkName.m8, "OpenThread");
    dataset.mComponents.mIsNetworkNamePresent = true;
    dataset.mPanId = 0x676b;
    dataset.mComponents.mIsPanIdPresent = true;
    // Add Network Key (16 bytes)
    uint8_t networkKey[16] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
    memcpy(dataset.mNetworkKey.m8, networkKey, sizeof(networkKey));
    dataset.mComponents.mIsNetworkKeyPresent = true;
    // Add Extended PAN ID (8 bytes)
    uint8_t extPanId[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
    memcpy(dataset.mExtendedPanId.m8, extPanId, sizeof(extPanId));
    dataset.mComponents.mIsExtendedPanIdPresent = true;
    dataset.mChannel = 15;
    dataset.mComponents.mIsChannelPresent = true;
    otError error = otDatasetSetActive(instance, &dataset);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to set active dataset: %d", error);
    }
    
    error = otIp6SetEnabled(instance, true);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to enable IP6: %d", error);
    }
    error = otThreadSetEnabled(instance, true);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to enable thread: %d", error);
    } else {
        ESP_LOGI(TAG, "Child thread enabled");
    }
    
    esp_openthread_lock_release(); // RELEASE LOCK IMMEDIATELY
    
    xTaskCreate(led_blink_task, "led_blink", 4096, NULL, 5, NULL);

#else
    // 1. Setup Network (Locked)
    esp_openthread_lock_acquire(portMAX_DELAY);
    otThreadSetEnabled(instance, false);
    otInstanceErasePersistentInfo(instance); // Clear old data
    
    otOperationalDataset ds;
    memset(&ds, 0, sizeof(otOperationalDataset));
    ds.mActiveTimestamp.mSeconds = 1;
    ds.mComponents.mIsActiveTimestampPresent = true;
    strcpy(ds.mNetworkName.m8, "OpenThread");
    ds.mComponents.mIsNetworkNamePresent = true;
    ds.mPanId = 0x676b;
    ds.mComponents.mIsPanIdPresent = true;
    ds.mChannel = 15;
    ds.mComponents.mIsChannelPresent = true;
    // Add Network Key (16 bytes)
    uint8_t networkKey[16] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
    memcpy(ds.mNetworkKey.m8, networkKey, sizeof(networkKey));
    ds.mComponents.mIsNetworkKeyPresent = true;
    // Add Extended PAN ID (8 bytes)
    uint8_t extPanId[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
    memcpy(ds.mExtendedPanId.m8, extPanId, sizeof(extPanId));
    ds.mComponents.mIsExtendedPanIdPresent = true;
    
    otError error = otDatasetSetActive(instance, &ds);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to set active dataset: %d", error);
    }
    otIp6SetEnabled(instance, true);
    error = otThreadSetEnabled(instance, true);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to enable thread: %d", error);
    }
    esp_openthread_lock_release(); // RELEASE LOCK

    // 2. Wait for stack to settle (Unlocked)
    vTaskDelay(pdMS_TO_TICKS(500)); 

    // 3. Promote to Leader (Locked)
    esp_openthread_lock_acquire(portMAX_DELAY);
    error = otThreadBecomeLeader(instance);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to become leader: %d", error);
    }
    esp_openthread_lock_release(); // RELEASE LOCK

    // 4. Configure Hardware (No lock needed for local UART/GPIO)
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, -1, -1);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CONTROL_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    // 5. Start Tasks
    xTaskCreate(uart_read_task, "uart_read", 4096, NULL, 5, NULL);
    xTaskCreate(send_data_example_task, "send_example", 4096, instance, 4, NULL);
    xTaskCreate(led_blink_task, "led_blink", 4096, NULL, 5, NULL);

#endif

    // Final CLI initialization
#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
    esp_cli_custom_command_init();
#endif
}
