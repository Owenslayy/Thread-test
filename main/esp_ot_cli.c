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
#include "esp_ot_config.h"
#include "esp_vfs_eventfd.h"
#include "nvs_flash.h"
#include "openthread/thread.h"
#include "openthread/thread_ftd.h"
#include "openthread/instance.h"
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
    
    static uint32_t log_counter = 0;
    static bool role_printed = false;
    
    while (1) {
        // Get Thread state to adjust blink pattern
        esp_openthread_lock_acquire(portMAX_DELAY);
        otInstance *instance = esp_openthread_get_instance();
        otDeviceRole role = otThreadGetDeviceRole(instance);
        esp_openthread_lock_release();
        
#ifdef CONFIG_DEVICE_TYPE_END_DEVICE
        // Child: log every 50 blinks (10 seconds) to avoid flooding CLI
        if (log_counter++ % 50 == 0) {
            ESP_LOGI(TAG, "Device role: %d (0=disabled, 1=detached, 2=child, 3=router, 4=leader)", role);
        }
#else
        // Leader: print role only once when it becomes leader
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
            
            // Echo back what was received (optional)
            uart_write_bytes(UART_NUM, (const char *)data, len);
        } else {
            // Print status every 2-3 seconds when no UART data
            ESP_LOGI(TAG, "UART: Waiting for data on GPIO%d...", UART_RX_PIN);
        }
    }
    
    free(data);
}

void app_main(void)
{
    // Used eventfds:
    // * netif
    // * ot task queue
    // * radio driver
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

    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };

    ESP_ERROR_CHECK(esp_openthread_start(&config));
    
    // Configure device role
    esp_openthread_lock_acquire(portMAX_DELAY);
    otInstance *instance = esp_openthread_get_instance();
    
#ifdef CONFIG_DEVICE_TYPE_END_DEVICE
    // Configure as end device (non-sleepy for LED blinking)
    otLinkModeConfig mode;
    mode.mRxOnWhenIdle = true;   // Stay awake to blink LED
    mode.mDeviceType = false;    // End device (not router)
    mode.mNetworkData = false;   // Don't need full network data
    otThreadSetLinkMode(instance, mode);
    ESP_LOGI(TAG, "Configured as End Device (Non-sleepy)");
    
    // Set faster disconnection detection - 30 seconds timeout
    otThreadSetChildTimeout(instance, 15);  // 30 seconds timeout before detaching
    
    // Check if we have stored network credentials
    otOperationalDataset dataset;
    if (otDatasetGetActive(instance, &dataset) == OT_ERROR_NONE) {
        ESP_LOGI(TAG, "Found stored credentials - auto-joining network: %s", dataset.mNetworkName.m8);
        // Auto-start Thread interface - will use stored credentials
        otIp6SetEnabled(instance, true);
        otThreadSetEnabled(instance, true);
        ESP_LOGI(TAG, "Thread interface auto-started with stored credentials");
    } else {
        ESP_LOGI(TAG, "No stored credentials found - configuring hardcoded credentials");
        
        // Configure hardcoded network credentials
        otOperationalDataset newDataset;
        memset(&newDataset, 0, sizeof(otOperationalDataset));
        
        // Set Active Timestamp (required)
        newDataset.mActiveTimestamp.mSeconds = 1;
        newDataset.mActiveTimestamp.mTicks = 0;
        newDataset.mActiveTimestamp.mAuthoritative = false;
        newDataset.mComponents.mIsActiveTimestampPresent = true;
        
        // Set network name
        const char *networkName = "OpenThread";
        size_t length = strlen(networkName);
        memcpy(newDataset.mNetworkName.m8, networkName, length);
        newDataset.mComponents.mIsNetworkNamePresent = true;
        
        // Set PAN ID
        newDataset.mPanId = 0x676b;
        newDataset.mComponents.mIsPanIdPresent = true;
        
        // Set Extended PAN ID
        uint8_t extPanId[] = {0xde, 0xad, 0x00, 0xbe, 0xef, 0x00, 0xca, 0xfe};
        memcpy(newDataset.mExtendedPanId.m8, extPanId, sizeof(extPanId));
        newDataset.mComponents.mIsExtendedPanIdPresent = true;
        
        // Set Network Key
        uint8_t networkKey[] = {0xc7, 0x16, 0xd0, 0x75, 0x30, 0x43, 0xae, 0x2f,
                                0x5b, 0x63, 0xc7, 0x1e, 0x3e, 0x51, 0xd7, 0xd0};
        memcpy(newDataset.mNetworkKey.m8, networkKey, sizeof(networkKey));
        newDataset.mComponents.mIsNetworkKeyPresent = true;
        
        // Set Channel
        newDataset.mChannel = 11;
        newDataset.mComponents.mIsChannelPresent = true;
        
        // Set Channel Mask
        newDataset.mChannelMask = 0x07fff800;  // All channels
        newDataset.mComponents.mIsChannelMaskPresent = true;
        
        // Set Security Policy
        newDataset.mSecurityPolicy.mRotationTime = 672;
        newDataset.mSecurityPolicy.mObtainNetworkKeyEnabled = true;
        newDataset.mSecurityPolicy.mNativeCommissioningEnabled = true;
        newDataset.mSecurityPolicy.mRoutersEnabled = true;
        newDataset.mSecurityPolicy.mExternalCommissioningEnabled = true;
        newDataset.mComponents.mIsSecurityPolicyPresent = true;
        
        // Apply the dataset
        otError error = otDatasetSetActive(instance, &newDataset);
        if (error == OT_ERROR_NONE) {
            ESP_LOGI(TAG, "Hardcoded credentials configured and saved");
        } else {
            ESP_LOGE(TAG, "Failed to set dataset: %d", error);
        }
        
        // Start Thread interface
        otIp6SetEnabled(instance, true);
        otThreadSetEnabled(instance, true);
        ESP_LOGI(TAG, "Thread interface started - joining network");
    }
    
    // Start LED blink task
    xTaskCreate(led_blink_task, "led_blink", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "LED blink started: slow=disconnected, blue=connected");
#else
    // Configure as router (default) - only available with FTD
    #if OPENTHREAD_FTD
    otThreadSetRouterEligible(instance, true);
    #endif
    
    // Always configure hardcoded network credentials for router
    ESP_LOGI(TAG, "Configuring router with hardcoded network credentials");
    
    // First, clear any existing dataset in NVS to force new network formation
    otThreadSetEnabled(instance, false);
    otIp6SetEnabled(instance, false);
    otInstanceErasePersistentInfo(instance);
    ESP_LOGI(TAG, "Cleared existing network data from NVS");
    
    otOperationalDataset dataset;
    memset(&dataset, 0, sizeof(otOperationalDataset));
    
    // Set Active Timestamp
    dataset.mActiveTimestamp.mSeconds = 1;
    dataset.mActiveTimestamp.mTicks = 0;
    dataset.mActiveTimestamp.mAuthoritative = false;
        dataset.mComponents.mIsActiveTimestampPresent = true;
        
        // Set network name
        const char *networkName = "OpenThread";
        size_t length = strlen(networkName);
        memcpy(dataset.mNetworkName.m8, networkName, length);
        dataset.mComponents.mIsNetworkNamePresent = true;
        
        // Set PAN ID
        dataset.mPanId = 0x676b;
        dataset.mComponents.mIsPanIdPresent = true;
        
        // Set Extended PAN ID
        uint8_t extPanId[] = {0xde, 0xad, 0x00, 0xbe, 0xef, 0x00, 0xca, 0xfe};
        memcpy(dataset.mExtendedPanId.m8, extPanId, sizeof(extPanId));
        dataset.mComponents.mIsExtendedPanIdPresent = true;
        
        // Set Network Key
        uint8_t networkKey[] = {0xc7, 0x16, 0xd0, 0x75, 0x30, 0x43, 0xae, 0x2f,
                                0x5b, 0x63, 0xc7, 0x1e, 0x3e, 0x51, 0xd7, 0xd0};
        memcpy(dataset.mNetworkKey.m8, networkKey, sizeof(networkKey));
        dataset.mComponents.mIsNetworkKeyPresent = true;
        
        // Set Channel
        dataset.mChannel = 11;
        dataset.mComponents.mIsChannelPresent = true;
        
        // Set Channel Mask
        dataset.mChannelMask = 0x07fff800;
        dataset.mComponents.mIsChannelMaskPresent = true;
        
        // Set Security Policy
        dataset.mSecurityPolicy.mRotationTime = 672;
        dataset.mSecurityPolicy.mObtainNetworkKeyEnabled = true;
        dataset.mSecurityPolicy.mNativeCommissioningEnabled = true;
        dataset.mSecurityPolicy.mRoutersEnabled = true;
        dataset.mSecurityPolicy.mExternalCommissioningEnabled = true;
        dataset.mComponents.mIsSecurityPolicyPresent = true;
        
        // Make sure Thread is disabled before setting dataset
        otThreadSetEnabled(instance, false);
        otIp6SetEnabled(instance, false);
        
        // Apply the dataset
        otError error = otDatasetSetActive(instance, &dataset);
        if (error == OT_ERROR_NONE) {
            ESP_LOGI(TAG, "Hardcoded network credentials configured");
        } else {
            ESP_LOGE(TAG, "Failed to set dataset: %d", error);
        }
    
    // Now start Thread interface for router with new credentials
    otIp6SetEnabled(instance, true);
    otThreadSetEnabled(instance, true);
    
    // Wait a moment for Thread to initialize, then force leader role
    vTaskDelay(pdMS_TO_TICKS(500));
    otError leaderError = otThreadBecomeLeader(instance);
    if (leaderError == OT_ERROR_NONE) {
        ESP_LOGI(TAG, "Forced device to become leader");
    } else {
        ESP_LOGI(TAG, "Leader promotion result: %d (will become leader after attach attempts)", leaderError);
    }
    
    ESP_LOGI(TAG, "Configured as Router - Thread interface started with hardcoded credentials");
    
    // Wait a moment for network to form, then print credentials
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Print network credentials for child devices to join (reuse dataset variable)
    if (otDatasetGetActive(instance, &dataset) == OT_ERROR_NONE) {
        ESP_LOGI(TAG, "=== Network Credentials for Child Devices ===");
        
        // Print Network Name
        ESP_LOGI(TAG, "Network Name: %s", dataset.mNetworkName.m8);
        
        // Print PAN ID
        ESP_LOGI(TAG, "PAN ID: 0x%04x", dataset.mPanId);
        
        // Print Extended PAN ID
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, dataset.mExtendedPanId.m8, OT_EXT_PAN_ID_SIZE, ESP_LOG_INFO);
        
        // Print Network Key
        ESP_LOGI(TAG, "Network Key (use on child): ");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, dataset.mNetworkKey.m8, OT_NETWORK_KEY_SIZE, ESP_LOG_INFO);
        
        // Print Channel
        ESP_LOGI(TAG, "Channel: %d", dataset.mChannel);
        
        ESP_LOGI(TAG, "===========================================");
    }
    
    // Configure UART for leader
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "UART configured on RX:GPIO%d, TX:GPIO%d", UART_RX_PIN, UART_TX_PIN);
    
    // Start UART read task
    xTaskCreate(uart_read_task, "uart_read", 4096, NULL, 5, NULL);
    
    // Start LED blink task
    xTaskCreate(led_blink_task, "led_blink", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "LED blink: slow=not ready, fast=ready for devices to join");
#endif
    
    esp_openthread_lock_release();
    
#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
    esp_cli_custom_command_init();
#endif
#if CONFIG_OPENTHREAD_NETWORK_AUTO_START
    ot_network_auto_start();
#endif
}
