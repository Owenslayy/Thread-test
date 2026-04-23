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
#define LED_GPIO 10

#define UART_NUM        UART_NUM_0
#define UART_TX_PIN     16
#define UART_RX_PIN     17
#define UART_BUF_SIZE   1024
#define CONTROL_PIN_1 7
#define CONTROL_PIN_2 8
#define CONTROL_PIN_3 9


#define UDP_PORT        12345
#define CHILD_TIMEOUT_S 60
#define SEND_PERIOD_MS  5000



static otUdpSocket sUdpSocket;
static otUdpSocket sReceiveSocket;

static bool sUdpSocketOpen = false;
static bool sReceiveSocketOpen = false;

static otIp6Address sChildAddr;
static bool sChildAddrSet = false;
static bool sLedCommandReceived = false;
static uint8_t sCurrentLedColor = 0x42;  // 'B'

// Tâche de test pour faire clignoter les LED en rouge, vert et bleu
static void check_uart_and_control_pin(const uint8_t *data, int len)
{
    if (len <= 0) {
        return;
    }

    /*if (data[0] == 0x00) {
        gpio_set_level(CONTROL_PIN, 1);
        ESP_LOGI(TAG, "UART received 0x00 - GPIO %d turned ON", CONTROL_PIN);
    } else {
        gpio_set_level(CONTROL_PIN, 0);
        ESP_LOGI(TAG, "UART received 0x%02X - GPIO %d turned OFF", data[0], CONTROL_PIN);
    }*/
}

static void set_child_address(const otIp6Address *addr)
{
    sChildAddr = *addr;
    sChildAddrSet = true;

    char addrStr[OT_IP6_ADDRESS_STRING_SIZE];
    otIp6AddressToString(addr, addrStr, sizeof(addrStr));
    ESP_LOGI(TAG, "Child address set to %s", addrStr);
}

// Fonction pour effacer l'adresse de l'enfant lorsque celui-ci se déconnecte ou devient invalide
static void clear_child_address(void)
{
    memset(&sChildAddr, 0, sizeof(sChildAddr));
    sChildAddrSet = false;
    ESP_LOGW(TAG, "Child address cleared");
}
// Fonction pour vérifier si l'adresse de l'enfant est toujours valide
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

    otSockAddr localAddr;
    memset(&localAddr, 0, sizeof(localAddr));
    localAddr.mPort = UDP_PORT;

    error = otUdpBind(instance, &sUdpSocket, &localAddr, OT_NETIF_THREAD_INTERNAL);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to bind send UDP socket: %d", error);
        otUdpClose(instance, &sUdpSocket);
        return false;
    }

    sUdpSocketOpen = true;
    ESP_LOGI(TAG, "UDP send socket initialized on port %d", UDP_PORT);
    return true;
}
// Fonction de rappel pour la réception de messages UDP
static void handle_udp_receive(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    (void)aContext;
    (void)aMessageInfo;

    uint16_t length = otMessageGetLength(aMessage);

    if (length == 0 || length > 256) {
        ESP_LOGW(TAG, "Received UDP message with invalid length: %u", length);
        return;
    }

    uint8_t data[256] = {0};
    uint16_t bytesRead = otMessageRead(aMessage, 0, data, length);

    if (bytesRead != length) {
        ESP_LOGE(TAG, "Partial UDP read: expected %u, got %u", length, bytesRead);
        return;
    }

    ESP_LOGI(TAG, "Received UDP data: 0x%02X", data[0]);
    if (data[0] == 0x00) {
   // gpio_set_level(CONTROL_PIN_1, 1);
    sCurrentLedColor = 0x47;
    vTaskDelay(pdMS_TO_TICKS(3000));
    sCurrentLedColor = 0x00;
    //ESP_LOGI(TAG, "0x00 -> GPIO %d HIGH", CONTROL_PIN_1);

    } else if (data[0] == 0x01) {
        gpio_set_level(CONTROL_PIN_1, 0);
        ESP_LOGI(TAG, "0x01 -> GPIO %d LOW", CONTROL_PIN_1);

    } else if (data[0] == 0x02) {
        gpio_set_level(CONTROL_PIN_2, 1);
        ESP_LOGI(TAG, "0x02 -> GPIO %d HIGH", CONTROL_PIN_2);

    } else if (data[0] == 0x03) {
        gpio_set_level(CONTROL_PIN_2, 0);
        ESP_LOGI(TAG, "0x03 -> GPIO %d LOW", CONTROL_PIN_2);

    } else if (data[0] == 0x04) {
        gpio_set_level(CONTROL_PIN_3, 1);
        ESP_LOGI(TAG, "0x04 -> GPIO %d HIGH", CONTROL_PIN_3);

    } else if (data[0] == 0x05) {
        gpio_set_level(CONTROL_PIN_3, 0);
        ESP_LOGI(TAG, "0x05 -> GPIO %d LOW", CONTROL_PIN_3);

    } 
    // 🔵 LED BLEU
    else if (data[0] == 0x42) {
        sCurrentLedColor = 0x42;
        sLedCommandReceived = true;
        ESP_LOGI(TAG, "LED color changed to BLUE");

    } 
    // 🟢 LED VERT
    else if (data[0] == 0x47) {
        sCurrentLedColor = 0x47;
        sLedCommandReceived = true;
        ESP_LOGI(TAG, "LED color changed to GREEN");

    } 
    // 🔴 LED ROUGE
    else if (data[0] == 0x46) {
        sCurrentLedColor = 0x46;
        sLedCommandReceived = true;
        ESP_LOGI(TAG, "LED color changed to RED");

    } else {
        ESP_LOGW(TAG, "Unknown command: 0x%02X", data[0]);
    }
}
// Fonction pour initialiser le socket de réception UDP
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

/**
 * @brief Vérifie si le rôle de l'appareil permet l'envoi de messages
 *
 * Cette fonction détermine si l'appareil actuel a un rôle dans le réseau Thread
 * qui lui permet d'envoyer des messages UDP aux appareils enfants.
 * Seuls les leaders et les routeurs peuvent initier des communications.
 *
 * @param instance Instance OpenThread pour vérifier le rôle
 * @return true si le rôle permet l'envoi, false sinon
 */
static bool is_role_ready_to_send_locked(otInstance *instance)
{
    otDeviceRole role = otThreadGetDeviceRole(instance);
    return (role == OT_DEVICE_ROLE_LEADER || role == OT_DEVICE_ROLE_ROUTER);
}

/**
 * @brief Découvre l'adresse IPv6 du premier appareil enfant disponible
 *
 * Cette fonction parcourt tous les appareils enfants connectés au réseau Thread
 * et retourne l'adresse IPv6 du premier enfant trouvé. Elle est utilisée pour
 * établir la communication UDP entre le parent et l'enfant.
 *
 * @param instance Instance OpenThread pour accéder aux informations des enfants
 * @param outAddr Pointeur vers la structure où stocker l'adresse découverte
 * @return true si un enfant est trouvé et son adresse extraite, false sinon
 */
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

/**
 * @brief Vérifie si une adresse d'enfant est toujours valide
 *
 * Cette fonction vérifie si l'adresse IPv6 d'un enfant est toujours présente
 * dans la liste des adresses des enfants connectés. Cela permet de détecter
 * si un enfant s'est déconnecté du réseau.
 *
 * @param instance Instance OpenThread pour vérifier les enfants
 * @param addrToCheck Adresse IPv6 à vérifier
 * @return true si l'adresse est toujours valide, false si l'enfant est parti
 */
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

/**
 * @brief S'assure qu'une adresse d'enfant valide est disponible
 *
 * Cette fonction vérifie si l'adresse d'enfant actuellement stockée est toujours
 * valide. Si ce n'est pas le cas, elle tente de découvrir automatiquement
 * l'adresse du premier enfant disponible dans le réseau.
 *
 * @param instance Instance OpenThread pour accéder aux informations réseau
 * @return true si une adresse d'enfant valide est disponible, false sinon
 */
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

/**
 * @brief Envoie des données UDP à l'appareil enfant
 *
 * Cette fonction envoie un message UDP à l'appareil enfant dont l'adresse
 * a été découverte précédemment. Elle gère l'initialisation du socket UDP,
 * la validation de l'adresse de destination et l'envoi effectif des données.
 *
 * @param instance Instance OpenThread pour l'envoi réseau
 * @param data Pointeur vers les données à envoyer
 * @param len Longueur des données en octets
 * @return true si l'envoi réussit, false en cas d'erreur
 */
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
    messageInfo.mSockPort = UDP_PORT;

    error = otUdpSend(instance, &sUdpSocket, message, &messageInfo);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to send UDP message: %d", error);
        otMessageFree(message);
        return false;
    }

    ESP_LOGI(TAG, "Data sent to child (%u bytes)", len);
    return true;
}

/**
 * @brief Tâche de contrôle de la LED RGB avec indication du rôle réseau
 *
 * Cette tâche FreeRTOS gère l'affichage visuel de l'état du réseau Thread
 * et des commandes reçues via UDP. La LED RGB indique différents états:
 *
 * Pour les appareils Leader/Router:
 * - Clignotement vert rapide (100ms on/off) pour indiquer le rôle de parent
 *
 * Pour les appareils Child (enfants):
 * - Bleu fixe: Commande 0x42 reçue (bleu)
 * - Vert fixe: Commande 0x47 reçue (vert)
 * - Clignotement bleu/vert lent (200ms) selon la dernière commande
 *
 * Pour les autres états:
 * - Clignotement rouge lent (500ms) pour indiquer un état détaché/désactivé
 *
 * La tâche s'exécute en boucle infinie avec des délais pour éviter la surcharge CPU.
 *
 * @param pvParameters Paramètres de la tâche (non utilisés)
 */
static void led_blink_task(void *pvParameters)
{
    (void)pvParameters;

    // Configuration de la bande LED
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,  // GPIO connecté à la LED
        .max_leds = 4,               // Six LED dans la bande
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,  // 10 MHz pour le contrôle RMT
    };

    // Initialisation du périphérique LED
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    ESP_LOGI(TAG, "RGB LED task running on GPIO %d", LED_GPIO);

    // Boucle principale de contrôle LED
    while (1) {
        // Acquérir le verrou OpenThread pour accéder aux informations réseau
        esp_openthread_lock_acquire(portMAX_DELAY);
        otInstance *instance = esp_openthread_get_instance();
        otDeviceRole role = otThreadGetDeviceRole(instance);
        esp_openthread_lock_release();

        // Logging périodique du rôle (différent selon le type d'appareil)
#ifdef CONFIG_DEVICE_TYPE_END_DEVICE
        // Pour les end devices: log toutes les 50 itérations
        static uint32_t log_counter = 0;
        if ((log_counter++ % 50) == 0) {
            ESP_LOGI(TAG, "Device role: %d (0=disabled, 1=detached, 2=child, 3=router, 4=leader)", role);
        }
#else
        // Pour les autres appareils: log une fois devenu leader
        static bool role_printed = false;
        if (!role_printed && role == OT_DEVICE_ROLE_LEADER) {
            ESP_LOGI(TAG, "Device role: %d (leader)", role);
            role_printed = true;
        }
#endif

        // Contrôle LED selon le rôle réseau
        if (role == OT_DEVICE_ROLE_LEADER || role == OT_DEVICE_ROLE_ROUTER) {
            // Leader/Router: clignotement vert rapide
            led_strip_set_pixel(led_strip, 0, 0, 50, 0);  // Vert
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(100));
            led_strip_clear(led_strip);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else if (role == OT_DEVICE_ROLE_CHILD) {
            // Child: couleur selon la commande UDP reçue
            if (sCurrentLedColor == 0x47) {
            //    led_strip_set_pixel(led_strip, 0, 0, 50, 0);  // Vert pour commande 0x47
                for (int i = 1; i < 10; i++) {
            led_strip_set_pixel(led_strip, i, 50, 30, 0);
        }
            } else {
                led_strip_set_pixel(led_strip, 0, 0, 0, 0);  // Noir pour commande 0x42 (défaut)
            }
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(200));
            led_strip_clear(led_strip);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            // État détaché/désactivé: clignotement rouge lent
            led_strip_set_pixel(led_strip, 0, 50, 0, 0);  // Rouge
            for (int i = 1; i < 4; i++) {
            led_strip_set_pixel(led_strip, i, 0, 0, 0);
        }
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(500));
            led_strip_clear(led_strip);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(500));
            
        }
    }
}

/**
 * @brief Tâche de lecture UART pour débogage et contrôle
 *
 * Cette tâche FreeRTOS lit en continu les données reçues sur l'interface UART
 * et les traite pour le débogage et le contrôle des broches GPIO. Elle permet
 * de recevoir des commandes via le port série et de les retransmettre.
 *
 * La tâche:
 * - Alloue un buffer pour les données UART
 * - Lit les données avec un timeout de 2 secondes
 * - Affiche les données reçues en hexadécimal
 * - Traite les données via check_uart_and_control_pin()
 * - Retransmet les données sur UART (echo)
 *
 * @param pvParameters Paramètres de la tâche (non utilisés)
 */
static void uart_read_task(void *pvParameters)
{
    otInstance *instance = (otInstance *)pvParameters;

    // Allocation du buffer UART
    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate UART buffer");
        vTaskDelete(NULL);
        return;
    }

    // Boucle de lecture UART
    while (1) {
        // Lecture des données UART avec timeout
        int len = uart_read_bytes(UART_NUM, data, UART_BUF_SIZE, pdMS_TO_TICKS(2000));
        if (len > 0) {
            ESP_LOGI(TAG, "UART received %d bytes:", len);
            ESP_LOG_BUFFER_HEX(TAG, data, len);

            // Traitement des données reçues
            check_uart_and_control_pin(data, len);
                        // 🔥 ENVOI UDP DIRECT
            esp_openthread_lock_acquire(portMAX_DELAY);
            bool ok = send_to_child_locked(instance, data, len);
            esp_openthread_lock_release();

            if (ok) {
                ESP_LOGI(TAG, "UDP sent from UART (%d bytes)", len);
            } else {
                ESP_LOGW(TAG, "UDP send failed");
            }
            // Echo des données sur UART
            uart_write_bytes(UART_NUM, (const char *)data, len);
        } else {
            ESP_LOGI(TAG, "UART: Waiting for data on GPIO%d...", UART_RX_PIN);
        }
    }
}



/**
 * @brief Tâche d'exemple d'envoi périodique de données aux enfants
 *
 * Cette tâche démontre l'envoi automatique de commandes de couleur aux appareils
 * enfants toutes les SEND_PERIOD_MS millisecondes. Elle alterne entre l'envoi
 * de la commande bleue (0x42) et verte (0x47).
 *
 * La tâche:
 * - Attend 5 secondes au démarrage pour laisser le réseau s'initialiser
 * - Envoie alternativement les commandes de couleur bleu/vert
 * - Utilise le verrou OpenThread pour la sécurité des threads
 * - Log les succès/échecs d'envoi
 *
 * @param pvParameters Instance OpenThread passée en paramètre
 */
/*static void send_data_example_task(void *pvParameters)
{
    otInstance *instance = (otInstance *)pvParameters;
    bool blue_color = true;

    // Attendre que le réseau Thread soit prêt
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Boucle d'envoi périodique
    while (1) {
        // Préparer la commande de couleur (alternance bleu/vert)
        uint8_t color_command = blue_color ? 0x42 : 0x47;

        // Envoi avec verrouillage thread-safe
        esp_openthread_lock_acquire(portMAX_DELAY);
        bool ok = send_to_child_locked(instance, &color_command, 1);
        esp_openthread_lock_release();

        if (ok) {
            ESP_LOGI(TAG, "Sent color command: %c (%s)",
                     color_command, blue_color ? "BLUE" : "GREEN");
            blue_color = !blue_color;  // Alterner la couleur pour le prochain envoi
        } else {
            ESP_LOGW(TAG, "Send skipped or failed");
        }

        // Attendre avant le prochain envoi
        vTaskDelay(pdMS_TO_TICKS(SEND_PERIOD_MS));
    }
}*/

/**
 * @brief Configure l'UART et les GPIO pour le débogage et le contrôle
 *
 * Cette fonction initialise l'interface UART pour la communication série
 * et configure une broche GPIO en sortie pour le contrôle matériel.
 * L'UART est configuré à 115200 bauds avec 8 bits de données, pas de parité.
 *
 * Configuration UART:
 * - Baud rate: 115200
 * - Data bits: 8
 * - Parity: None
 * - Stop bits: 1
 * - Flow control: None
 *
 * Configuration GPIO:
 * - Broche CONTROL_PIN en mode sortie
 */
static void configure_uart_and_gpio(void)
{
    // Configuration UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Installation et configuration du driver UART
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, -1, -1));

    // Configuration de la broche GPIO de contrôle
    gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << CONTROL_PIN_1) |
                            (1ULL << CONTROL_PIN_2) |
                            (1ULL << CONTROL_PIN_3),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

/**
 * @brief Remplit le dataset opérationnel OpenThread avec les paramètres réseau
 *
 * Cette fonction configure un dataset OpenThread complet avec tous les paramètres
 * nécessaires pour former ou rejoindre un réseau Thread. Le dataset inclut:
 * - Nom du réseau: "OpenThread"
 * - PAN ID: 0x676b
 * - Canal: 15
 * - Clé réseau: séquence prédéfinie
 * - Extended PAN ID: séquence prédéfinie
 *
 * @param dataset Pointeur vers la structure otOperationalDataset à remplir
 */
static void fill_dataset(otOperationalDataset *dataset)
{
    memset(dataset, 0, sizeof(*dataset));

    // Timestamp actif
    dataset->mActiveTimestamp.mSeconds = 1;
    dataset->mComponents.mIsActiveTimestampPresent = true;

    // Nom du réseau
    strcpy((char *)dataset->mNetworkName.m8, "OpenThread");
    dataset->mComponents.mIsNetworkNamePresent = true;

    // PAN ID
    dataset->mPanId = 0x676b;
    dataset->mComponents.mIsPanIdPresent = true;

    // Canal de communication
    dataset->mChannel = 15;
    dataset->mComponents.mIsChannelPresent = true;

    // Clé réseau (16 octets)
    const uint8_t networkKey[16] = {
        0x00, 0x11, 0x22, 0x33,
        0x44, 0x55, 0x66, 0x77,
        0x88, 0x99, 0xaa, 0xbb,
        0xcc, 0xdd, 0xee, 0xff
    };
    memcpy(dataset->mNetworkKey.m8, networkKey, sizeof(networkKey));
    dataset->mComponents.mIsNetworkKeyPresent = true;

    // Extended PAN ID (8 octets)
    const uint8_t extPanId[8] = {
        0x11, 0x11, 0x11, 0x11,
        0x11, 0x11, 0x11, 0x11
    };
    memcpy(dataset->mExtendedPanId.m8, extPanId, sizeof(extPanId));
    dataset->mComponents.mIsExtendedPanIdPresent = true;
}

/**
 * @brief Fonction principale de l'application ESP32
 *
 * Cette fonction initialise tous les composants nécessaires au fonctionnement
 * de l'application Thread avec contrôle LED et communication UDP.
 *
 * Séquence d'initialisation:
 * 1. Initialisation du système (NVS, event loop, netif, VFS)
 * 2. Configuration OpenThread selon le type d'appareil
 * 3. Configuration UART et GPIO
 * 4. Création des tâches FreeRTOS
 *
 * L'application supporte deux modes de fonctionnement:
 * - End Device (enfant): reçoit des commandes UDP et contrôle la LED
 * - Leader/Router (parent): envoie des commandes UDP aux enfants
 *
 * @note Cette fonction ne retourne jamais (boucle infinie dans les tâches)
 */
void app_main(void)
{
    // Configuration VFS pour les descripteurs de fichiers d'événements
    esp_vfs_eventfd_config_t eventfd_config = {
        .max_fds = 3,
    };

    // Initialisation des composants système de base
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));

    // Initialisation de l'interface CLI OpenThread si activée
#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_init();
#endif

    // Configuration OpenThread
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

    // Démarrage d'OpenThread
    ESP_ERROR_CHECK(esp_openthread_start(&config));
    otInstance *instance = esp_openthread_get_instance();

    // Configuration spécifique selon le type d'appareil
#ifdef CONFIG_DEVICE_TYPE_END_DEVICE
    // Configuration pour un appareil enfant (End Device)
    esp_openthread_lock_acquire(portMAX_DELAY);

    otOperationalDataset dataset;
    fill_dataset(&dataset);

    otError error = otDatasetSetActive(instance, &dataset);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to set active dataset: %d", error);
    }

    // Configuration du mode de liaison pour un enfant
    otLinkModeConfig mode = {
        .mRxOnWhenIdle = true,    // Récepteur toujours actif
        .mDeviceType = false,     // Pas un routeur
        .mNetworkData = false     // Ne gère pas les données réseau
    };
    otThreadSetLinkMode(instance, mode);
    otThreadSetChildTimeout(instance, CHILD_TIMEOUT_S);

    // Activation des protocoles réseau
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

    // Initialisation du socket de réception UDP
    init_receive_socket_locked(instance);
    esp_openthread_lock_release();

    // Création de la tâche de contrôle LED
    xTaskCreate(led_blink_task, "led_blink", 4096, NULL, 5, NULL);
   
#else
    // Configuration pour un appareil parent (Leader/Router)
    esp_openthread_lock_acquire(portMAX_DELAY);

    otOperationalDataset dataset;
    fill_dataset(&dataset);

    otError error = otDatasetSetActive(instance, &dataset);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to set active dataset: %d", error);
    }

    // Activation des protocoles réseau
    error = otIp6SetEnabled(instance, true);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to enable IP6: %d", error);
    }

    error = otThreadSetEnabled(instance, true);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to enable thread: %d", error);
    }

    // Initialisation du socket d'envoi UDP
    init_udp_socket_locked(instance);
    esp_openthread_lock_release();

    // Attendre un peu pour la stabilité
    vTaskDelay(pdMS_TO_TICKS(500));

    // Tenter de devenir leader du réseau
    esp_openthread_lock_acquire(portMAX_DELAY);
    error = otThreadBecomeLeader(instance);
    if (error != OT_ERROR_NONE) {
        ESP_LOGW(TAG, "Failed to become leader explicitly: %d", error);
    }
    esp_openthread_lock_release();

    // Configuration UART et GPIO pour le débogage
    configure_uart_and_gpio();

    // Création des tâches de contrôle LED, lecture UART et envoi périodique
   
    xTaskCreate(uart_read_task, "uart_read", 4096, instance, 5, NULL);
 //   xTaskCreate(send_data_example_task, "send_example", 4096, instance, 4, NULL);
    xTaskCreate(led_blink_task, "led_blink", 4096, NULL, 5, NULL);

#endif

    // Initialisation des commandes CLI personnalisées si activées
#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
    esp_cli_custom_command_init();
#endif


}