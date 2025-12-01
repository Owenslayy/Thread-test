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
        
        ESP_LOGI(TAG, "Device role: %d (0=disabled, 1=detached, 2=child, 3=router, 4=leader)", role);
        
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
    
    // Auto-start Thread interface
    otIp6SetEnabled(instance, true);
    otThreadSetEnabled(instance, true);
    ESP_LOGI(TAG, "Thread interface auto-started");
    
    // Start LED blink task
    xTaskCreate(led_blink_task, "led_blink", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "LED blink started: slow=disconnected, fast=connected");
#else
    // Configure as router (default) - only available with FTD
    #if OPENTHREAD_FTD
    otThreadSetRouterEligible(instance, true);
    #endif
    
    // Auto-start Thread interface for router
    otIp6SetEnabled(instance, true);
    otThreadSetEnabled(instance, true);
    ESP_LOGI(TAG, "Configured as Router - Thread interface auto-started");
    
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
