/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * OpenThread Command Line Example - corrected version
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

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
#include "openthread/dataset_ftd.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led_strip.h"

#if CONFIG_OPENTHREAD_STATE_INDICATOR_ENABLE
#include "ot_led_strip.h"
#endif

#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
#include "esp_ot_cli_extension.h"
#endif

#define TAG "ot_esp_cli"
#define LED_GPIO 8

#define UART_NUM        UART_NUM_1
#define UART_TX_PIN     5
#define UART_RX_PIN     4
#define UART_BUF_SIZE   1024
#define CONTROL_PIN     7

#define UDP_PORT        12345
#define CHILD_TIMEOUT_S 60
#define SEND_PERIOD_MS  5000

static otUdpSocket sUdpSocket;
static otUdpSocket sReceiveSocket;

static bool sUdpSocketOpen = false;
static bool sReceiveSocketOpen = false;

static otIp6Address sChildAddr;
static bool sChildAddrSet = false;

static uint8_t sCurrentLedColor = 0x42;  // 'B'

static void check_uart_and_control_pin(const uint8_t *data, int len)
{
    if (len <= 0) {
        return;
    }

    if (data[0] == 0x00) {
        gpio_set_level(CONTROL_PIN, 1);
        ESP_LOGI(TAG, "UART received 0x00 - GPIO %d turned ON", CONTROL_PIN);
    } else {
        gpio_set_level(CONTROL_PIN, 0);
        ESP_LOGI(TAG, "UART received 0x%02X - GPIO %d turned OFF", data[0], CONTROL_PIN);
    }
}

static void set_child_address(const otIp6Address *addr)
{
    sChildAddr = *addr;
    sChildAddrSet = true;

    char addrStr[OT_IP6_ADDRESS_STRING_SIZE];
    otIp6AddressToString(addr, addrStr, sizeof(addrStr));
    ESP_LOGI(TAG, "Child address set to %s", addrStr);
}

static void clear_child_address(void)
{
    memset(&sChildAddr, 0, sizeof(sChildAddr));
    sChildAddrSet = false;
    ESP_LOGW(TAG, "Child address cleared");
}

static bool init_udp_socket_locked(otInstance *instance)
{
    if (sUdpSocketOpen) {
        return true;
    }

    otError error = otUdpOpen(instance, &sUdpSocket, NULL, NULL);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to open UDP socket: %d", error);
        return false;
    }

    sUdpSocketOpen = true;
    ESP_LOGI(TAG, "UDP send socket initialized");
    return true;
}

static void handle_udp_receive(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    (void)aContext;
    (void)aMessageInfo;

    uint16_t length = otMessageGetLength(aMessage);

    if (length == 0 || length > 256) {
        ESP_LOGW(TAG, "Received UDP message with invalid length: %u", length);
        otMessageFree(aMessage);
        return;
    }

    uint8_t data[256] = {0};
    otError error = otMessageRead(aMessage, 0, data, length);

    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to read UDP message: %d", error);
        otMessageFree(aMessage);
        return;
    }

    ESP_LOGI(TAG, "Received UDP data: 0x%02X", data[0]);

    if (data[0] == 0x42) {
        sCurrentLedColor = 0x42;
        ESP_LOGI(TAG, "LED color changed to BLUE");
    } else if (data[0] == 0x47) {
        sCurrentLedColor = 0x47;
        ESP_LOGI(TAG, "LED color changed to GREEN");
    }

    otMessageFree(aMessage);
}

static bool init_receive_socket_locked(otInstance *instance)
{
    if (sReceiveSocketOpen) {
        return true;
    }

    otError error = otUdpOpen(instance, &sReceiveSocket, handle_udp_receive, NULL);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to open receive UDP socket: %d", error);
        return false;
    }

    otSockAddr sockaddr;
    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.mPort = UDP_PORT;

    error = otUdpBind(instance, &sReceiveSocket, &sockaddr, OT_NETIF_THREAD_INTERNAL);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to bind receive UDP socket: %d", error);
        otUdpClose(instance, &sReceiveSocket);
        return false;
    }

    sReceiveSocketOpen = true;
    ESP_LOGI(TAG, "Receive UDP socket initialized on port %d", UDP_PORT);
    return true;
}

static bool is_role_ready_to_send_locked(otInstance *instance)
{
    otDeviceRole role = otThreadGetDeviceRole(instance);
    return (role == OT_DEVICE_ROLE_LEADER || role == OT_DEVICE_ROLE_ROUTER);
}

static bool discover_first_child_address_locked(otInstance *instance, otIp6Address *outAddr)
{
    otChildInfo childInfo;
    uint16_t childIndex = 0;

    while (otThreadGetChildInfoByIndex(instance, childIndex, &childInfo) == OT_ERROR_NONE) {
        otChildIp6AddressIterator iterator = OT_CHILD_IP6_ADDRESS_ITERATOR_INIT;
        otIp6Address candidate;

        ESP_LOGI(TAG, "Found child %u with RLOC16: 0x%04x, timeout: %u s",
                 childIndex, childInfo.mRloc16, childInfo.mTimeout);

        while (otThreadGetChildNextIp6Address(instance, childIndex, &iterator, &candidate) == OT_ERROR_NONE) {
            char addrStr[OT_IP6_ADDRESS_STRING_SIZE];
            otIp6AddressToString(&candidate, addrStr, sizeof(addrStr));
            ESP_LOGI(TAG, "Child %u IPv6 address: %s", childIndex, addrStr);

            *outAddr = candidate;
            return true;
        }

        childIndex++;
    }

    return false;
}

static bool child_address_still_valid_locked(otInstance *instance, const otIp6Address *addrToCheck)
{
    otChildInfo childInfo;
    uint16_t childIndex = 0;

    while (otThreadGetChildInfoByIndex(instance, childIndex, &childInfo) == OT_ERROR_NONE) {
        otChildIp6AddressIterator iterator = OT_CHILD_IP6_ADDRESS_ITERATOR_INIT;
        otIp6Address checkAddr;

        while (otThreadGetChildNextIp6Address(instance, childIndex, &iterator, &checkAddr) == OT_ERROR_NONE) {
            if (memcmp(addrToCheck, &checkAddr, sizeof(otIp6Address)) == 0) {
                return true;
            }
        }

        childIndex++;
    }

    return false;
}

static bool ensure_child_address_locked(otInstance *instance)
{
    if (sChildAddrSet && child_address_still_valid_locked(instance, &sChildAddr)) {
        return true;
    }

    if (sChildAddrSet) {
        ESP_LOGW(TAG, "Stored child address is no longer valid");
        clear_child_address();
    }

    otIp6Address discoveredAddr;
    if (discover_first_child_address_locked(instance, &discoveredAddr)) {
        set_child_address(&discoveredAddr);
        return true;
    }

    ESP_LOGW(TAG, "No valid child address found");
    return false;
}

static bool send_to_child_locked(otInstance *instance, const uint8_t *data, uint16_t len)
{
    if (!is_role_ready_to_send_locked(instance)) {
        ESP_LOGW(TAG, "Leader/router not ready to send");
        return false;
    }

    if (!init_udp_socket_locked(instance)) {
        return false;
    }

    if (!ensure_child_address_locked(instance)) {
        return false;
    }

    char addrStr[OT_IP6_ADDRESS_STRING_SIZE];
    otIp6AddressToString(&sChildAddr, addrStr, sizeof(addrStr));
    ESP_LOGI(TAG, "Sending to child address: %s", addrStr);

    otMessage *message = otUdpNewMessage(instance, NULL);
    if (message == NULL) {
        ESP_LOGE(TAG, "Failed to create UDP message");
        return false;
    }

    otError error = otMessageAppend(message, data, len);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to append data: %d", error);
        otMessageFree(message);
        return false;
    }

    otMessageInfo messageInfo;
    memset(&messageInfo, 0, sizeof(messageInfo));
    messageInfo.mPeerAddr = sChildAddr;
    messageInfo.mPeerPort = UDP_PORT;
    messageInfo.mSockPort = 0;

    error = otUdpSend(instance, &sUdpSocket, message, &messageInfo);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to send UDP message: %d", error);
        otMessageFree(message);
        return false;
    }

    ESP_LOGI(TAG, "Data sent to child (%u bytes)", len);
    return true;
}

static void led_blink_task(void *pvParameters)
{
    (void)pvParameters;

    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
    };

    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    ESP_LOGI(TAG, "RGB LED task running on GPIO %d", LED_GPIO);

    while (1) {
        esp_openthread_lock_acquire(portMAX_DELAY);
        otInstance *instance = esp_openthread_get_instance();
        otDeviceRole role = otThreadGetDeviceRole(instance);
        esp_openthread_lock_release();

#ifdef CONFIG_DEVICE_TYPE_END_DEVICE
        static uint32_t log_counter = 0;
        if ((log_counter++ % 50) == 0) {
            ESP_LOGI(TAG, "Device role: %d (0=disabled, 1=detached, 2=child, 3=router, 4=leader)", role);
        }
#else
        static bool role_printed = false;
        if (!role_printed && role == OT_DEVICE_ROLE_LEADER) {
            ESP_LOGI(TAG, "Device role: %d (leader)", role);
            role_printed = true;
        }
#endif

        if (role == OT_DEVICE_ROLE_LEADER || role == OT_DEVICE_ROLE_ROUTER) {
            led_strip_set_pixel(led_strip, 0, 0, 50, 0);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(100));
            led_strip_clear(led_strip);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else if (role == OT_DEVICE_ROLE_CHILD) {
            if (sCurrentLedColor == 0x47) {
                led_strip_set_pixel(led_strip, 0, 0, 50, 0);
            } else {
                led_strip_set_pixel(led_strip, 0, 0, 0, 50);
            }
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(200));
            led_strip_clear(led_strip);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            led_strip_set_pixel(led_strip, 0, 50, 0, 0);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(500));
            led_strip_clear(led_strip);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

static void uart_read_task(void *pvParameters)
{
    (void)pvParameters;

    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate UART buffer");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, UART_BUF_SIZE, pdMS_TO_TICKS(2000));
        if (len > 0) {
            ESP_LOGI(TAG, "UART received %d bytes:", len);
            ESP_LOG_BUFFER_HEX(TAG, data, len);

            check_uart_and_control_pin(data, len);
            uart_write_bytes(UART_NUM, (const char *)data, len);
        } else {
            ESP_LOGI(TAG, "UART: Waiting for data on GPIO%d...", UART_RX_PIN);
        }
    }
}

static void send_data_example_task(void *pvParameters)
{
    otInstance *instance = (otInstance *)pvParameters;
    bool blue_color = true;

    vTaskDelay(pdMS_TO_TICKS(5000));

    while (1) {
        uint8_t color_command = blue_color ? 0x42 : 0x47;

        esp_openthread_lock_acquire(portMAX_DELAY);
        bool ok = send_to_child_locked(instance, &color_command, 1);
        esp_openthread_lock_release();

        if (ok) {
            ESP_LOGI(TAG, "Sent color command: %c (%s)",
                     color_command, blue_color ? "BLUE" : "GREEN");
            blue_color = !blue_color;
        } else {
            ESP_LOGW(TAG, "Send skipped or failed");
        }

        vTaskDelay(pdMS_TO_TICKS(SEND_PERIOD_MS));
    }
}

static void configure_uart_and_gpio(void)
{
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
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, -1, -1));

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CONTROL_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static void fill_dataset(otOperationalDataset *dataset)
{
    memset(dataset, 0, sizeof(*dataset));

    dataset->mActiveTimestamp.mSeconds = 1;
    dataset->mComponents.mIsActiveTimestampPresent = true;

    strcpy((char *)dataset->mNetworkName.m8, "OpenThread");
    dataset->mComponents.mIsNetworkNamePresent = true;

    dataset->mPanId = 0x676b;
    dataset->mComponents.mIsPanIdPresent = true;

    dataset->mChannel = 15;
    dataset->mComponents.mIsChannelPresent = true;

    const uint8_t networkKey[16] = {
        0x00, 0x11, 0x22, 0x33,
        0x44, 0x55, 0x66, 0x77,
        0x88, 0x99, 0xaa, 0xbb,
        0xcc, 0xdd, 0xee, 0xff
    };
    memcpy(dataset->mNetworkKey.m8, networkKey, sizeof(networkKey));
    dataset->mComponents.mIsNetworkKeyPresent = true;

    const uint8_t extPanId[8] = {
        0x11, 0x11, 0x11, 0x11,
        0x11, 0x11, 0x11, 0x11
    };
    memcpy(dataset->mExtendedPanId.m8, extPanId, sizeof(extPanId));
    dataset->mComponents.mIsExtendedPanIdPresent = true;
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

    ESP_ERROR_CHECK(esp_openthread_start(&config));

    otInstance *instance = esp_openthread_get_instance();

#ifdef CONFIG_DEVICE_TYPE_END_DEVICE

    esp_openthread_lock_acquire(portMAX_DELAY);

    otOperationalDataset dataset;
    fill_dataset(&dataset);

    otError error = otDatasetSetActive(instance, &dataset);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to set active dataset: %d", error);
    }

    otLinkModeConfig mode = {
        .mRxOnWhenIdle = true,
        .mDeviceType = false,
        .mNetworkData = false
    };
    otThreadSetLinkMode(instance, mode);
    otThreadSetChildTimeout(instance, CHILD_TIMEOUT_S);

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

    init_receive_socket_locked(instance);

    esp_openthread_lock_release();

    xTaskCreate(led_blink_task, "led_blink", 4096, NULL, 5, NULL);

#else

    esp_openthread_lock_acquire(portMAX_DELAY);

    otOperationalDataset dataset;
    fill_dataset(&dataset);

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
    }

    init_udp_socket_locked(instance);

    esp_openthread_lock_release();

    vTaskDelay(pdMS_TO_TICKS(500));

    esp_openthread_lock_acquire(portMAX_DELAY);
    error = otThreadBecomeLeader(instance);
    if (error != OT_ERROR_NONE) {
        ESP_LOGW(TAG, "Failed to become leader explicitly: %d", error);
    }
    esp_openthread_lock_release();

    configure_uart_and_gpio();

    xTaskCreate(uart_read_task, "uart_read", 4096, NULL, 5, NULL);
    xTaskCreate(send_data_example_task, "send_example", 4096, instance, 4, NULL);
    xTaskCreate(led_blink_task, "led_blink", 4096, NULL, 5, NULL);

#endif

#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
    esp_cli_custom_command_init();
#endif
}