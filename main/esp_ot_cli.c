/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * OpenThread Command Line Example — ESP32-C6 Wroom-1
 *
 * Changes vs previous version:
 *   + servo_init()        — configures LEDC for 2 servo motors
 *   + servo_set_angle()   — sets angle 0-180° on either servo
 *   + servo_rotate_180()  — sweeps a servo from 0° to 180°
 *   + UDP 0x10/0x11       — rotate servo 1 / servo 2 via Thread command
 *   + UDP 0x12/0x13       — return servo 1 / servo 2 to 0°
 *
 * Servo PWM spec (standard hobby servo):
 *   Frequency  : 50 Hz  (20 ms period)
 *   0°  pulse  : 1.0 ms → duty = 1000 / 20000 × 16383 =  819
 *   90° pulse  : 1.5 ms → duty = 1500 / 20000 × 16383 = 1228
 *   180° pulse : 2.0 ms → duty = 2000 / 20000 × 16383 = 1638
 *
 * GPIO assignment (free pins on ESP32-C6 Wroom-1):
 *   SERVO_1_GPIO  GPIO 4
 *   SERVO_2_GPIO  GPIO 5
 *   (existing pins 7,8,9,10,12,16,17,22 untouched)
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
#include "driver/ledc.h"       /* servo PWM */
 
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
 
/* ── LED strip ───────────────────────────────────────────────── */
#define LED_GPIO        5
#define LED_MAX         4
 
/* ── UART ────────────────────────────────────────────────────── */
#define UART_NUM        UART_NUM_0
#define UART_TX_PIN     16
#define UART_RX_PIN     17
#define UART_BUF_SIZE   1024
 
/* ── Control GPIOs ───────────────────────────────────────────── */
#define CONTROL_PIN_1   7
#define CONTROL_PIN_2   8
#define CONTROL_PIN_3   9
#define IN1             10
#define IN2             22

/* ── GPIO Monitor (input → envoie UDP automatiquement) ───────── */
/* Broche à surveiller — change selon ton câblage               */
#define GPIO_MONITOR_PIN    12
/* Commande envoyée à l'enfant quand le GPIO passe à HIGH        */
#define GPIO_CMD_HIGH       0x20
/* Commande envoyée à l'enfant quand le GPIO passe à LOW         */
#define GPIO_CMD_LOW        0x21
/* Intervalle de polling en ms                                   */
#define GPIO_POLL_MS        100

/* ── Toggle Buttons (leader only) ───────────────────────────── */
/*  Bouton 1 → GPIO 2 → lumière  (allume 0x11 / eteins 0x21)   */
/*  Bouton 2 → GPIO 3 → robinet  (allume 0x13 / eteins 0x23)   */
#define BTN_LUMIERE_PIN     2
#define BTN_ROBINET_PIN     3
#define BTN_DEBOUNCE_MS     50      /* anti-rebond 50 ms         */
#define BTN_POLL_MS         20      /* lecture toutes les 20 ms  */

/* Commandes lumière */
#define CMD_LUMIERE_ON      0x11    /* allume lumiere            */
#define CMD_LUMIERE_OFF     0x21    /* eteins lumiere            */

/* Commandes robinet */
#define CMD_ROBINET_ON      0x13    /* allume robinet            */
#define CMD_ROBINET_OFF     0x23    /* eteins robinet            */
 
/* ── Servo PWM (LEDC) ────────────────────────────────────────── */
#define SERVO_1_GPIO        4
#define SERVO_2_GPIO        6
 
#define SERVO_FREQ_HZ       50
#define SERVO_TIMER_RES     LEDC_TIMER_14_BIT   /* 0 .. 16383 */
#define SERVO_TIMER_MAX     16383U
#define SERVO_PERIOD_US     20000U              /* 1 / 50 Hz  */
#define SERVO_PULSE_MIN_US  1000U               /* 0°         */
#define SERVO_PULSE_MAX_US  2000U               /* 180°       */
 
#define SERVO_1_CHANNEL     LEDC_CHANNEL_0
#define SERVO_2_CHANNEL     LEDC_CHANNEL_1
#define SERVO_LEDC_TIMER    LEDC_TIMER_0
 
/* ── Thread / UDP ────────────────────────────────────────────── */
#define UDP_PORT        12345
#define CHILD_TIMEOUT_S 15
#define SEND_PERIOD_MS  5000
 
/* ══════════════════════════════════════════════════════════════
 *  SHARED STATE
 * ══════════════════════════════════════════════════════════════ */
static otUdpSocket sUdpSocket;
static otUdpSocket sReceiveSocket;
 
static bool sUdpSocketOpen     = false;
static bool sReceiveSocketOpen = false;
 
static otIp6Address sChildAddr;
static bool         sChildAddrSet = false;
 
static volatile uint8_t sCurrentLedColor = 0x42;
 
/* ══════════════════════════════════════════════════════════════
 *  SERVO — internal helper
 *
 *  Converts an angle (0–180°) to a 14-bit LEDC duty count.
 *  Linear mapping: 0° → 1000 µs pulse, 180° → 2000 µs pulse.
 * ══════════════════════════════════════════════════════════════ */
static uint32_t angle_to_duty(uint32_t angle_deg)
{
    if (angle_deg > 180) angle_deg = 180;
 
    uint32_t pulse_us = SERVO_PULSE_MIN_US +
                        (angle_deg * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US)) / 180U;
 
    return (pulse_us * SERVO_TIMER_MAX) / SERVO_PERIOD_US;
}
 
/* ══════════════════════════════════════════════════════════════
 *  servo_init
 *
 *  Configures LEDC timer 0 at 50 Hz 14-bit.
 *  Binds channel 0 → GPIO 4 (servo 1)
 *       channel 1 → GPIO 5 (servo 2)
 *  Both servos start at 0°.
 * ══════════════════════════════════════════════════════════════ */
static void servo_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = SERVO_LEDC_TIMER,
        .duty_resolution = SERVO_TIMER_RES,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));
 
    ledc_channel_config_t ch0 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = SERVO_1_CHANNEL,
        .timer_sel  = SERVO_LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = SERVO_1_GPIO,
        .duty       = angle_to_duty(0),
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch0));
 
    ledc_channel_config_t ch1 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = SERVO_2_CHANNEL,
        .timer_sel  = SERVO_LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = SERVO_2_GPIO,
        .duty       = angle_to_duty(0),
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch1));
 
    ESP_LOGI(TAG, "Servo init OK — GPIO%d ch0, GPIO%d ch1 @ %dHz",
             SERVO_1_GPIO, SERVO_2_GPIO, SERVO_FREQ_HZ);
}
 
/* ══════════════════════════════════════════════════════════════
 *  servo_set_angle
 *
 *  Moves servo on `channel` to `angle_deg` (0–180°) immediately.
 *  Use this for direct position control.
 * ══════════════════════════════════════════════════════════════ */
static void servo_set_angle(ledc_channel_t channel, uint32_t angle_deg)
{
    uint32_t duty = angle_to_duty(angle_deg);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
    ESP_LOGI(TAG, "Servo ch%d → %lu° (duty %lu)", channel, angle_deg, duty);
}
 
/* ══════════════════════════════════════════════════════════════
 *  servo_rotate_180
 *
 *  Sweeps the servo from 0° to 180° smoothly.
 *  Steps: 2° per step, 15 ms delay → 90 steps × 15 ms ≈ 1.35 s
 *
 *  NOTE: called from the UDP callback (OpenThread task context).
 *  The vTaskDelay inside will stall the OT stack for ~1.35 s.
 *  Perfectly fine for occasional valve/door actuation.
 *  For continuous/frequent use, offload to a dedicated task.
 * ══════════════════════════════════════════════════════════════ */
static void servo_rotate_180(ledc_channel_t channel)
{
    ESP_LOGI(TAG, "Servo ch%d: 0° → 180° sweep start", channel);
 
    servo_set_angle(channel, 0);
    vTaskDelay(pdMS_TO_TICKS(300));   /* reach start position */
 
    for (uint32_t angle = 0; angle <= 180; angle += 2) {
        servo_set_angle(channel, angle);
        vTaskDelay(pdMS_TO_TICKS(15));
    }
 
    ESP_LOGI(TAG, "Servo ch%d: reached 180°", channel);
}
 
/* ══════════════════════════════════════════════════════════════
 *  CHILD ADDRESS HELPERS  (unchanged)
 * ══════════════════════════════════════════════════════════════ */
static void set_child_address(const otIp6Address *addr)
{
    sChildAddr    = *addr;
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
 
/* ══════════════════════════════════════════════════════════════
 *  UDP SOCKET INIT  (unchanged)
 * ══════════════════════════════════════════════════════════════ */
static bool init_udp_socket_locked(otInstance *instance)
{
    if (sUdpSocketOpen) return true;
 
    otError error = otUdpOpen(instance, &sUdpSocket, NULL, NULL);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to open UDP socket: %d", error);
        return false;
    }
 
    otSockAddr localAddr;
    memset(&localAddr, 0, sizeof(localAddr));
    localAddr.mPort = UDP_PORT;
 
    error = otUdpBind(instance, &sUdpSocket, &localAddr,
                      OT_NETIF_THREAD_INTERNAL);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to bind send UDP socket: %d", error);
        otUdpClose(instance, &sUdpSocket);
        return false;
    }
 
    sUdpSocketOpen = true;
    ESP_LOGI(TAG, "UDP send socket initialized on port %d", UDP_PORT);
    return true;
}
 
/* ══════════════════════════════════════════════════════════════
 *  UDP RECEIVE CALLBACK
 *
 *  Servo commands added:
 *    0x10 → servo 1: sweep 0° → 180°
 *    0x11 → servo 2: sweep 0° → 180°
 *    0x12 → servo 1: return to 0°
 *    0x13 → servo 2: return to 0°
 * ══════════════════════════════════════════════════════════════ */
static void handle_udp_receive(void        *aContext,
                               otMessage   *aMessage,
                               const otMessageInfo *aMessageInfo)
{
    (void)aContext;
    (void)aMessageInfo;
 
    uint16_t length = otMessageGetLength(aMessage);
    if (length == 0 || length > 256) {
        ESP_LOGW(TAG, "Invalid UDP length: %u", length);
        return;
    }
 
    uint8_t  data[256] = {0};
    uint16_t bytesRead = otMessageRead(aMessage, 0, data, length);
    if (bytesRead != length) {
        ESP_LOGE(TAG, "Partial read: expected %u got %u", length, bytesRead);
        return;
    }
 
    ESP_LOGI(TAG, "UDP received: 0x%02X", data[0]);
 
    switch (data[0]) {
 
        /* ── GPIO commands ─────────────────────────────────── */
        case 0x00:
            gpio_set_level(CONTROL_PIN_1, 1);
            sCurrentLedColor = 0x47;
            ESP_LOGI(TAG, "0x00 -> GPIO%d HIGH + LED GREEN", CONTROL_PIN_1);
            break;
        case 0x01:
            gpio_set_level(CONTROL_PIN_1, 0);
            sCurrentLedColor = 0x42;
            ESP_LOGI(TAG, "0x01 -> GPIO%d LOW", CONTROL_PIN_1);
            break;
        case 0x02:
            gpio_set_level(CONTROL_PIN_2, 1);
            ESP_LOGI(TAG, "0x02 -> GPIO%d HIGH", CONTROL_PIN_2);
            break;
        case 0x03:
            gpio_set_level(CONTROL_PIN_2, 0);
            ESP_LOGI(TAG, "0x03 -> GPIO%d LOW", CONTROL_PIN_2);
            break;
        case 0x04:
            gpio_set_level(CONTROL_PIN_3, 1);
            ESP_LOGI(TAG, "0x04 -> GPIO%d HIGH", CONTROL_PIN_3);
            break;
        case 0x05:
            gpio_set_level(CONTROL_PIN_3, 0);
            ESP_LOGI(TAG, "0x05 -> GPIO%d LOW", CONTROL_PIN_3);
            break;
 
        /* ── Servo commands ────────────────────────────────── */
        case 0x10:
            ESP_LOGI(TAG, "0x10 -> Servo 1: 0° → 180°");
            servo_rotate_180(SERVO_1_CHANNEL);
            break;
        case 0x11:
            ESP_LOGI(TAG, "0x11 -> Servo 2: 0° → 180°");
            servo_rotate_180(SERVO_2_CHANNEL);
            break;
        case 0x12:
            ESP_LOGI(TAG, "0x12 -> Servo 1: return to 0°");
            servo_set_angle(SERVO_1_CHANNEL, 0);
            break;
        case 0x13:
            ESP_LOGI(TAG, "0x13 -> Servo 2: return to 0°");
            servo_set_angle(SERVO_2_CHANNEL, 0);
            break;
 
        /* ── LED colour commands ───────────────────────────── */
        case 0x42:
            sCurrentLedColor = 0x42;
            ESP_LOGI(TAG, "LED -> BLUE");
            break;
        case 0x47:
            sCurrentLedColor = 0x47;
            ESP_LOGI(TAG, "LED -> GREEN");
            break;
        case 0x46:
            sCurrentLedColor = 0x46;
            ESP_LOGI(TAG, "LED -> RED");
            break;
 
        default:
            ESP_LOGW(TAG, "Unknown command: 0x%02X", data[0]);
            break;
    }
}
 
static bool init_receive_socket_locked(otInstance *instance)
{
    if (sReceiveSocketOpen) return true;
 
    otError error = otUdpOpen(instance, &sReceiveSocket,
                              handle_udp_receive, NULL);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to open receive socket: %d", error);
        return false;
    }
 
    otSockAddr sockaddr;
    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.mPort = UDP_PORT;
 
    error = otUdpBind(instance, &sReceiveSocket, &sockaddr,
                      OT_NETIF_THREAD_INTERNAL);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to bind receive socket: %d", error);
        otUdpClose(instance, &sReceiveSocket);
        return false;
    }
 
    sReceiveSocketOpen = true;
    ESP_LOGI(TAG, "Receive socket initialized on port %d", UDP_PORT);
    return true;
}
 
/* ══════════════════════════════════════════════════════════════
 *  CHILD ADDRESS DISCOVERY  (unchanged)
 * ══════════════════════════════════════════════════════════════ */
static bool is_role_ready_to_send_locked(otInstance *instance)
{
    otDeviceRole role = otThreadGetDeviceRole(instance);
    return (role == OT_DEVICE_ROLE_LEADER || role == OT_DEVICE_ROLE_ROUTER);
}
 
static bool discover_first_child_address_locked(otInstance   *instance,
                                                otIp6Address *outAddr)
{
    otChildInfo childInfo;
    uint16_t    childIndex = 0;
 
    while (otThreadGetChildInfoByIndex(instance, childIndex, &childInfo)
           == OT_ERROR_NONE) {
        otChildIp6AddressIterator iterator = OT_CHILD_IP6_ADDRESS_ITERATOR_INIT;
        otIp6Address candidate;
        ESP_LOGI(TAG, "Found child %u RLOC16=0x%04x timeout=%us",
                 childIndex, childInfo.mRloc16, childInfo.mTimeout);
        while (otThreadGetChildNextIp6Address(instance, childIndex,
                                              &iterator, &candidate)
               == OT_ERROR_NONE) {
            char addrStr[OT_IP6_ADDRESS_STRING_SIZE];
            otIp6AddressToString(&candidate, addrStr, sizeof(addrStr));
            ESP_LOGI(TAG, "Child %u addr: %s", childIndex, addrStr);
            *outAddr = candidate;
            return true;
        }
        childIndex++;
    }
    return false;
}
 
static bool child_address_still_valid_locked(otInstance         *instance,
                                             const otIp6Address *addrToCheck)
{
    otChildInfo childInfo;
    uint16_t    childIndex = 0;
 
    while (otThreadGetChildInfoByIndex(instance, childIndex, &childInfo)
           == OT_ERROR_NONE) {
        otChildIp6AddressIterator iterator = OT_CHILD_IP6_ADDRESS_ITERATOR_INIT;
        otIp6Address checkAddr;
        while (otThreadGetChildNextIp6Address(instance, childIndex,
                                              &iterator, &checkAddr)
               == OT_ERROR_NONE) {
            if (memcmp(addrToCheck, &checkAddr, sizeof(otIp6Address)) == 0)
                return true;
        }
        childIndex++;
    }
    return false;
}
 
static bool ensure_child_address_locked(otInstance *instance)
{
    if (sChildAddrSet &&
        child_address_still_valid_locked(instance, &sChildAddr))
        return true;
 
    if (sChildAddrSet) {
        ESP_LOGW(TAG, "Stored child address no longer valid");
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
 
/* ══════════════════════════════════════════════════════════════
 *  UDP SEND  (unchanged)
 * ══════════════════════════════════════════════════════════════ */
static bool send_to_child_locked(otInstance    *instance,
                                 const uint8_t *data,
                                 uint16_t       len)
{
    if (!is_role_ready_to_send_locked(instance)) {
        ESP_LOGW(TAG, "Not leader/router — cannot send");
        return false;
    }
    if (!init_udp_socket_locked(instance))      return false;
    if (!ensure_child_address_locked(instance)) return false;
 
    char addrStr[OT_IP6_ADDRESS_STRING_SIZE];
    otIp6AddressToString(&sChildAddr, addrStr, sizeof(addrStr));
    ESP_LOGI(TAG, "Sending %u bytes to %s", len, addrStr);
 
    otMessage *message = otUdpNewMessage(instance, NULL);
    if (!message) {
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
    messageInfo.mSockPort = UDP_PORT;
 
    error = otUdpSend(instance, &sUdpSocket, message, &messageInfo);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to send: %d", error);
        otMessageFree(message);
        return false;
    }
 
    ESP_LOGI(TAG, "Sent %u bytes", len);
    return true;
}
 
/* ══════════════════════════════════════════════════════════════
 *  LED BLINK TASK  (unchanged)
 * ══════════════════════════════════════════════════════════════ */
static void led_blink_task(void *pvParameters)
{
    (void)pvParameters;
 
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds       = LED_MAX,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
    };
 
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config,
                                              &led_strip));
    ESP_LOGI(TAG, "LED task running — GPIO %d, %d LEDs", LED_GPIO, LED_MAX);
 
    uint8_t last_color = 0xFF;
 
    while (1) {
        esp_openthread_lock_acquire(portMAX_DELAY);
        otInstance   *instance = esp_openthread_get_instance();
        otDeviceRole  role     = otThreadGetDeviceRole(instance);
        esp_openthread_lock_release();
 
#ifdef CONFIG_DEVICE_TYPE_END_DEVICE
        static uint32_t log_counter = 0;
        if ((log_counter++ % 50) == 0)
            ESP_LOGI(TAG, "Role: %d (0=dis 1=det 2=child 3=router 4=leader)", role);
#else
        static bool role_printed = false;
        if (!role_printed && role == OT_DEVICE_ROLE_LEADER) {
            ESP_LOGI(TAG, "Role: %d (leader)", role);
            role_printed = true;
        }
#endif
 
        if (role == OT_DEVICE_ROLE_LEADER || role == OT_DEVICE_ROLE_ROUTER) {
            for (int i = 0; i < LED_MAX; i++)
                led_strip_set_pixel(led_strip, i, 0, 50, 0);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(100));
            led_strip_clear(led_strip);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(100));
 
        } else if (role == OT_DEVICE_ROLE_CHILD) {
            uint8_t color = sCurrentLedColor;
            if (color != last_color) {
                led_strip_clear(led_strip);
                led_strip_refresh(led_strip);
                last_color = color;
            }
            if (color == 0x47) {
                for (int i = 0; i < LED_MAX; i++)
                    led_strip_set_pixel(led_strip, i, 50, 30, 0);
            } else if (color == 0x46) {
                for (int i = 0; i < LED_MAX; i++)
                    led_strip_set_pixel(led_strip, i, 50, 0, 0);
            } else {
                for (int i = 0; i < LED_MAX; i++)
                    led_strip_set_pixel(led_strip, i, 0, 0, 50);
            }
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(200));
 
        } else {
            led_strip_set_pixel(led_strip, 0, 50, 0, 0);
            for (int i = 1; i < LED_MAX; i++)
                led_strip_set_pixel(led_strip, i, 0, 0, 0);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(500));
            led_strip_clear(led_strip);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}
 
/* ══════════════════════════════════════════════════════════════
 *  UART READ TASK  (unchanged)
 * ══════════════════════════════════════════════════════════════ */
static void uart_read_task(void *pvParameters)
{
    otInstance *instance = (otInstance *)pvParameters;
 
    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);
    if (!data) {
        ESP_LOGE(TAG, "Failed to allocate UART buffer");
        vTaskDelete(NULL);
        return;
    }
 
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, UART_BUF_SIZE,
                                  pdMS_TO_TICKS(2000));
        if (len > 0) {
            ESP_LOGI(TAG, "UART received %d bytes:", len);
            ESP_LOG_BUFFER_HEX(TAG, data, len);
 
            esp_openthread_lock_acquire(portMAX_DELAY);
            bool ok = send_to_child_locked(instance, data, len);
            esp_openthread_lock_release();
 
            if (ok)
                ESP_LOGI(TAG, "UDP forwarded (%d bytes)", len);
            else
                ESP_LOGW(TAG, "UDP send failed");
 
            uart_write_bytes(UART_NUM, (const char *)data, len);
        } else {
            ESP_LOGI(TAG, "UART: waiting on GPIO%d...", UART_RX_PIN);
        }
    }
}
 
/* ══════════════════════════════════════════════════════════════
 *  HEAT CONTROL TASK  (unchanged)
 * ══════════════════════════════════════════════════════════════ */
static void set_heat_control_pins(void *pvParameters)
{
    while (1) {
        ESP_LOGI(TAG, "Heat/Cool task running");
        gpio_set_level(IN1, 1);
        gpio_set_level(IN2, 0);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
 
/* ══════════════════════════════════════════════════════════════
 *  BUTTON TOGGLE TASK  (leader only)
 *
 *  Surveille 2 boutons en input avec pull-up interne.
 *  Chaque appui inverse le flag et envoie la commande UDP :
 *
 *    Bouton GPIO2 → lumière
 *      flag false → true  : envoie CMD_LUMIERE_ON  (0x11)
 *      flag true  → false : envoie CMD_LUMIERE_OFF (0x21)
 *
 *    Bouton GPIO3 → robinet
 *      flag false → true  : envoie CMD_ROBINET_ON  (0x13)
 *      flag true  → false : envoie CMD_ROBINET_OFF (0x23)
 *
 *  Anti-rebond par polling : on attend BTN_DEBOUNCE_MS ms après
 *  le front descendant avant de confirmer l'appui.
 * ══════════════════════════════════════════════════════════════ */
static void button_toggle_task(void *pvParameters)
{
    otInstance *instance = (otInstance *)pvParameters;

    /* Configure les 2 boutons en entrée avec pull-up interne
     * → bouton non appuyé = HIGH, appuyé = LOW              */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_LUMIERE_PIN) |
                        (1ULL << BTN_ROBINET_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_LOGI(TAG, "Button task started — GPIO%d (lumiere) GPIO%d (robinet)",
             BTN_LUMIERE_PIN, BTN_ROBINET_PIN);

    /* Flags d'état — false = OFF, true = ON */
    bool lumiere_on = false;
    bool robinet_on = false;

    /* Dernier état lu (pull-up → repos = 1) */
    int last_lumiere = 1;
    int last_robinet = 1;

    /* Compteurs anti-rebond en ms */
    int debounce_lumiere = 0;
    int debounce_robinet = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(BTN_POLL_MS));

        int cur_lumiere = gpio_get_level(BTN_LUMIERE_PIN);
        int cur_robinet = gpio_get_level(BTN_ROBINET_PIN);

        /* ── Bouton lumière (GPIO2) ──────────────────────── */
        if (last_lumiere == 1 && cur_lumiere == 0) {
            /* Front descendant détecté → début anti-rebond */
            debounce_lumiere = BTN_DEBOUNCE_MS;
        } else if (debounce_lumiere > 0) {
            debounce_lumiere -= BTN_POLL_MS;
            if (debounce_lumiere <= 0 && cur_lumiere == 0) {
                /* Appui confirmé → inverse le flag */
                lumiere_on = !lumiere_on;
                uint8_t cmd = lumiere_on ? CMD_LUMIERE_ON : CMD_LUMIERE_OFF;

                ESP_LOGI(TAG, "BTN lumiere → %s (0x%02X)",
                         lumiere_on ? "ON" : "OFF", cmd);

                esp_openthread_lock_acquire(portMAX_DELAY);
                bool ok = send_to_child_locked(instance, &cmd, 1);
                esp_openthread_lock_release();

                if (ok)
                    ESP_LOGI(TAG, "Lumiere cmd sent: 0x%02X", cmd);
                else
                    ESP_LOGW(TAG, "Lumiere cmd failed (no child?)");

                debounce_lumiere = 0;
            }
        }
        last_lumiere = cur_lumiere;

        /* ── Bouton robinet (GPIO3) ──────────────────────── */
        if (last_robinet == 1 && cur_robinet == 0) {
            debounce_robinet = BTN_DEBOUNCE_MS;
        } else if (debounce_robinet > 0) {
            debounce_robinet -= BTN_POLL_MS;
            if (debounce_robinet <= 0 && cur_robinet == 0) {
                robinet_on = !robinet_on;
                uint8_t cmd = robinet_on ? CMD_ROBINET_ON : CMD_ROBINET_OFF;

                ESP_LOGI(TAG, "BTN robinet → %s (0x%02X)",
                         robinet_on ? "ON" : "OFF", cmd);

                esp_openthread_lock_acquire(portMAX_DELAY);
                bool ok = send_to_child_locked(instance, &cmd, 1);
                esp_openthread_lock_release();

                if (ok)
                    ESP_LOGI(TAG, "Robinet cmd sent: 0x%02X", cmd);
                else
                    ESP_LOGW(TAG, "Robinet cmd failed (no child?)");

                debounce_robinet = 0;
            }
        }
        last_robinet = cur_robinet;
    }
}

/* ══════════════════════════════════════════════════════════════
 *  GPIO MONITOR TASK
 *
 *  Surveille GPIO_MONITOR_PIN en input avec pull-down interne.
 *  Quand l'état change (LOW→HIGH ou HIGH→LOW), envoie un byte
 *  UDP à l'enfant via Thread :
 *    GPIO HIGH → 0x20
 *    GPIO LOW  → 0x21
 *
 *  Polling toutes les GPIO_POLL_MS ms.
 *  Lancé uniquement sur le parent (bloc #else dans app_main).
 * ══════════════════════════════════════════════════════════════ */
static void gpio_monitor_task(void *pvParameters)
{
    otInstance *instance = (otInstance *)pvParameters;

    /* Configure GPIO_MONITOR_PIN en entrée avec pull-down */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_MONITOR_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_LOGI(TAG, "GPIO monitor task started on GPIO%d", GPIO_MONITOR_PIN);

    int last_level = gpio_get_level(GPIO_MONITOR_PIN);
    ESP_LOGI(TAG, "GPIO%d initial state: %d", GPIO_MONITOR_PIN, last_level);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(GPIO_POLL_MS));

        int current_level = gpio_get_level(GPIO_MONITOR_PIN);

        if (current_level != last_level) {
            last_level = current_level;

            uint8_t cmd = (current_level == 1) ? GPIO_CMD_HIGH : GPIO_CMD_LOW;

            ESP_LOGI(TAG, "GPIO%d changed → %d, sending 0x%02X to child",
                     GPIO_MONITOR_PIN, current_level, cmd);

            esp_openthread_lock_acquire(portMAX_DELAY);
            bool ok = send_to_child_locked(instance, &cmd, 1);
            esp_openthread_lock_release();

            if (ok)
                ESP_LOGI(TAG, "GPIO event sent: 0x%02X", cmd);
            else
                ESP_LOGW(TAG, "GPIO event send failed (no child?)");
        }
    }
}

/* ══════════════════════════════════════════════════════════════
 *  GPIO INIT  (unchanged)
 * ══════════════════════════════════════════════════════════════ */
static void gpio_init_pins(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IN1) | (1ULL << IN2),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 0);
}
 
/* ══════════════════════════════════════════════════════════════
 *  UART + CONTROL PINS INIT  (unchanged)
 * ══════════════════════════════════════════════════════════════ */
static void configure_uart_and_gpio(void)
{
    uart_config_t uart_config = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2,
                                        0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, -1, -1));
 
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CONTROL_PIN_1) |
                        (1ULL << CONTROL_PIN_2) |
                        (1ULL << CONTROL_PIN_3),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(CONTROL_PIN_1, 0);
    gpio_set_level(CONTROL_PIN_2, 0);
    gpio_set_level(CONTROL_PIN_3, 0);
}
 
/* ══════════════════════════════════════════════════════════════
 *  DATASET  (unchanged)
 * ══════════════════════════════════════════════════════════════ */
static void fill_dataset(otOperationalDataset *dataset)
{
    memset(dataset, 0, sizeof(*dataset));
 
    dataset->mActiveTimestamp.mSeconds             = 1;
    dataset->mComponents.mIsActiveTimestampPresent = true;
 
    strcpy((char *)dataset->mNetworkName.m8, "OpenThread");
    dataset->mComponents.mIsNetworkNamePresent     = true;
 
    dataset->mPanId                                = 0x676b;
    dataset->mComponents.mIsPanIdPresent           = true;
 
    dataset->mChannel                              = 15;
    dataset->mComponents.mIsChannelPresent         = true;
 
    const uint8_t networkKey[16] = {
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
        0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff
    };
    memcpy(dataset->mNetworkKey.m8, networkKey, sizeof(networkKey));
    dataset->mComponents.mIsNetworkKeyPresent      = true;
 
    const uint8_t extPanId[8] = {
        0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11
    };
    memcpy(dataset->mExtendedPanId.m8, extPanId, sizeof(extPanId));
    dataset->mComponents.mIsExtendedPanIdPresent   = true;
}
 
/* ══════════════════════════════════════════════════════════════
 *  APP MAIN
 *
 *  Added: servo_init() before OpenThread starts.
 * ══════════════════════════════════════════════════════════════ */
void app_main(void)
{
    esp_vfs_eventfd_config_t eventfd_config = { .max_fds = 3 };
 
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));
 
    gpio_init_pins();
    servo_init();       /* both servos start at 0° */
 
#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_init();
#endif
 
    const esp_openthread_config_t config = {
        .netif_config    = ESP_NETIF_DEFAULT_OPENTHREAD(),
        .platform_config = {
            .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
            .host_config  = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
            .port_config  = {
                .storage_partition_name = "nvs",
                .netif_queue_size       = 10,
                .task_queue_size        = 10,
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
    if (error != OT_ERROR_NONE)
        ESP_LOGE(TAG, "Failed to set dataset: %d", error);
 
    otLinkModeConfig mode = {
        .mRxOnWhenIdle = true,
        .mDeviceType   = false,
        .mNetworkData  = false
    };
    otThreadSetLinkMode(instance, mode);
    otThreadSetChildTimeout(instance, CHILD_TIMEOUT_S);
 
    error = otIp6SetEnabled(instance, true);
    if (error != OT_ERROR_NONE)
        ESP_LOGE(TAG, "Failed to enable IP6: %d", error);
 
    error = otThreadSetEnabled(instance, true);
    if (error != OT_ERROR_NONE)
        ESP_LOGE(TAG, "Failed to enable Thread: %d", error);
    else
        ESP_LOGI(TAG, "Child thread enabled");
 
    init_receive_socket_locked(instance);
    esp_openthread_lock_release();
 
    xTaskCreate(led_blink_task,        "led_blink",    4096, NULL, 5, NULL);
    xTaskCreate(set_heat_control_pins, "heat_control", 2048, NULL, 5, NULL);
 
#else
    esp_openthread_lock_acquire(portMAX_DELAY);
 
    otOperationalDataset dataset;
    fill_dataset(&dataset);
 
    otError error = otDatasetSetActive(instance, &dataset);
    if (error != OT_ERROR_NONE)
        ESP_LOGE(TAG, "Failed to set dataset: %d", error);
 
    error = otIp6SetEnabled(instance, true);
    if (error != OT_ERROR_NONE)
        ESP_LOGE(TAG, "Failed to enable IP6: %d", error);
 
    error = otThreadSetEnabled(instance, true);
    if (error != OT_ERROR_NONE)
        ESP_LOGE(TAG, "Failed to enable Thread: %d", error);
 
    init_udp_socket_locked(instance);
    esp_openthread_lock_release();
 
    vTaskDelay(pdMS_TO_TICKS(500));
 
    esp_openthread_lock_acquire(portMAX_DELAY);
    error = otThreadBecomeLeader(instance);
    if (error != OT_ERROR_NONE)
        ESP_LOGW(TAG, "Failed to become leader: %d", error);
    esp_openthread_lock_release();
 
    configure_uart_and_gpio();
 
    xTaskCreate(uart_read_task,    "uart_read",    4096, instance, 5, NULL);
    xTaskCreate(gpio_monitor_task, "gpio_monitor", 2048, instance, 5, NULL);
    xTaskCreate(button_toggle_task,"btn_toggle",   2048, instance, 5, NULL);
    xTaskCreate(led_blink_task,    "led_blink",    4096, NULL,     5, NULL);
 
#endif
 
#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
    esp_cli_custom_command_init();
#endif
}