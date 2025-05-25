#ifndef RC76XX_MQTT_H
#define RC76XX_MQTT_H

#include "FreeRTOS.h"
#include <stdint.h>
#include <stdbool.h>

#define RC76XX_DEBUG_LEVEL RC76XX_DEBUG_LEVEL_VERBOSE

//===========================================================================
//  Debug configuration
//===========================================================================
// Debug levels
#define RC76XX_DEBUG_LEVEL_NONE    0
#define RC76XX_DEBUG_LEVEL_ERROR   1
#define RC76XX_DEBUG_LEVEL_WARNING 2
#define RC76XX_DEBUG_LEVEL_INFO    3
#define RC76XX_DEBUG_LEVEL_VERBOSE 4

// Set the current debug level (can be overridden in build system)
#ifndef RC76XX_DEBUG_LEVEL
#define RC76XX_DEBUG_LEVEL RC76XX_DEBUG_LEVEL_NONE
#endif

// Debug macros for different levels
#if RC76XX_DEBUG_LEVEL >= RC76XX_DEBUG_LEVEL_ERROR
#define RC76XX_ERROR(fmt, ...) printf("[%lu] ERROR: " fmt, HAL_GetTick(), ##__VA_ARGS__)
#else
#define RC76XX_ERROR(fmt, ...) do {} while(0)
#endif

#if RC76XX_DEBUG_LEVEL >= RC76XX_DEBUG_LEVEL_WARNING
#define RC76XX_WARNING(fmt, ...) printf("[%lu] WARNING: " fmt, HAL_GetTick(), ##__VA_ARGS__)
#else
#define RC76XX_WARNING(fmt, ...) do {} while(0)
#endif

#if RC76XX_DEBUG_LEVEL >= RC76XX_DEBUG_LEVEL_INFO
#define RC76XX_INFO(fmt, ...) printf("[%lu] INFO: " fmt, HAL_GetTick(), ##__VA_ARGS__)
#else
#define RC76XX_INFO(fmt, ...) do {} while(0)
#endif

#if RC76XX_DEBUG_LEVEL >= RC76XX_DEBUG_LEVEL_VERBOSE
#define RC76XX_VERBOSE(fmt, ...) printf("[%lu] VERBOSE: " fmt, HAL_GetTick(), ##__VA_ARGS__)
#else
#define RC76XX_VERBOSE(fmt, ...) do {} while(0)
#endif

// Always print critical messages regardless of debug level
#define RC76XX_CRITICAL(fmt, ...) printf("[%lu] CRITICAL: " fmt, HAL_GetTick(), ##__VA_ARGS__)

#define RC76XX_IMEI_LEN 16
#define RC76XX_MODEL_LEN 32
#define RC76XX_IP_LEN 16
#define RC76XX_CLIENTID_LEN 32
#define RC76XX_TOPIC_LEN 64
#define RC76XX_PAYLOAD_LEN 128
#define RC76XX_RX_PAYLOAD_LEN 1024

/**
 * @brief Enumeration of possible result codes for the RC76xx driver
 *
 * Defines the various status and error conditions that can be returned
 * by RC76xx driver functions to indicate the outcome of operations.
 */
typedef enum
{
    RC76XX_OK = 0,
    RC76XX_ERR_AT,
    RC76XX_ERR_PARSE,
    RC76XX_ERR_STATE,
    RC76XX_ERR_INTERNAL,
    RC76XX_ERR_BUFFER_OVERFLOW,
    RC76XX_ERR_INVALID_PARAM,  // New: for invalid parameters
    RC76XX_ERR_CME,            // New: for CME errors
    RC76XX_ERR_CMS             // New: for CMS errors
} RC76XX_Result_t;

/**
 * @brief Enumeration of driver states for the RC76xx MQTT driver
 *
 * Defines the various operational states that the RC76xx modem can be in
 * during MQTT communication and network interactions.
 * 
 * RC76XX MQTT State Machine
 * =========================
 *
 * State Transition Diagram:
 *
 * ┌─────────────────┐     RC76XX_Reset()     ┌─────────────────┐
 * │                 │◄────────────────────────┤                 │
 * │ RC76XX_STATE_   │                         │ Any State       │
 * │ RESET           │                         │                 │
 * │                 │                         └─────────────────┘
 * └────────┬────────┘
 *          │ RC76XX_Initialize()
 *          ▼
 * ┌─────────────────┐
 * │ RC76XX_STATE_   │
 * │ INITIALIZED     │
 * │                 │
 * └────────┬────────┘
 *          │ RC76XX_NetworkAttach()
 *          ▼
 * ┌─────────────────┐
 * │ RC76XX_STATE_   │
 * │ NETWORK_READY   │
 * │                 │
 * └────────┬────────┘
 *          │ RC76XX_ConfigMQTT()
 *          ▼
 * ┌─────────────────┐
 * │ RC76XX_STATE_   │
 * │ MQTT_CONFIGURED │
 * │                 │
 * └────────┬────────┘
 *          │ RC76XX_ConnectMQTT()
 *          ▼
 * ┌─────────────────┐
 * │ RC76XX_STATE_   │
 * │ MQTT_CONNECTING │
 * │                 │
 * └────────┬────────┘
 *          │ URC: +KMQTT_IND: 0,1
 *          ▼
 * ┌─────────────────┐
 * │ RC76XX_STATE_   │◄───────┐
 * │ MQTT_CONNECTED  │        │
 * │                 │        │ RC76XX_Subscribe()
 * └────────┬────────┘        │ RC76XX_Publish()
 *          │                 │
 *          │ RC76XX_Disconnect()
 *          ▼
 * ┌─────────────────┐
 * │ RC76XX_STATE_   │
 * │ RESET           │
 * │                 │
 * └─────────────────┘
 *
 * Error Transitions:
 * - Any AT command failure → RC76XX_STATE_ERROR
 * - URC: +KMQTT_IND: 0,0 or +KMQTT_IND: 0,5 → RC76XX_STATE_ERROR
 * - Reset URC → RC76XX_STATE_RESET
 *
 * Driver states */
typedef enum
{
    RC76XX_STATE_RESET = 0,
    RC76XX_STATE_INITIALIZED,
    RC76XX_STATE_NETWORK_READY,
    RC76XX_STATE_MQTT_CONFIGURED,
    RC76XX_STATE_MQTT_CONNECTING,
    RC76XX_STATE_MQTT_CONNECTED,
    RC76XX_STATE_ERROR
} RC76XX_State_t;

/**
 * @brief Opaque driver handle for the RC76xx MQTT driver
 *
 * Represents an internal structure containing the state and configuration
 * for an RC76xx modem instance. This handle is used to track and manage
 * the modem's connection, network, and MQTT-related parameters.
 */
/** Opaque driver handle */
typedef struct
{
    RC76XX_State_t state;
    char imei[RC76XX_IMEI_LEN];
    char model[RC76XX_MODEL_LEN];
    char ip[RC76XX_IP_LEN];
    int cnx_id; // ID returned by +KCNXCFG
    int mqtt_cfg_id; // ID returned by +KMQTTCFG
    char client_id[RC76XX_CLIENTID_LEN];
} RC76XX_Handle_t;

/**
 * @brief  Reset the modem
 *
 * @param h driver handle 
 * @return RC76XX_Result_t RC76XX_OK or error
 */
RC76XX_Result_t RC76XX_Reset(RC76XX_Handle_t *h);


/**
 * @brief  Initialize the modem, get IMEI, disable echo.
 * @param  h  driver handle (uninitialized)
 * @return RC76XX_OK or error
 */
RC76XX_Result_t RC76XX_Initialize(RC76XX_Handle_t *h);

/**
 * @brief  Attach to network, configure PDP context, get IP.
 * @param  h
 * @return RC76XX_OK or error
 */
RC76XX_Result_t RC76XX_NetworkAttach(RC76XX_Handle_t *h);

/**
 * @brief  Configure MQTT parameters (server, port, clientID, authentication, SSL).
 * @param  h  driver handle
 * @param  host   MQTT broker DNS or IP
 * @param  port   MQTT broker port number
 * @param  clientID  Unique client identifier for the MQTT connection
 * @param  topic  Optional topic for subscription
 * @param  username  Optional username for MQTT authentication
 * @param  password  Optional password for MQTT authentication
 * @param  useTLS  true to enable SSL/TLS encryption
 * @param  subscribe  true to subscribe to the specified topic
 * @param  keepAliveInterval  MQTT keep-alive interval in seconds
 * @return RC76XX_OK or error
 */
RC76XX_Result_t RC76XX_ConfigMQTT(RC76XX_Handle_t *h,
                                  const char *host,
                                  uint16_t port,
                                  const char *clientID,
                                  const char *topic,
                                  const char *username,
                                  const char *password,
                                  bool useTLS,
                                  bool subscribe,
                                  uint16_t keepAliveInterval);

/**
 * @brief  Connect (open) the MQTT session.
 * @param  h
 * @return RC76XX_OK or error
 */
RC76XX_Result_t RC76XX_ConnectMQTT(RC76XX_Handle_t *h);

/**
 * @brief  Publish a message to a specified MQTT topic.
 * @param  h  driver handle
 * @param  topic  MQTT topic to publish the message to
 * @param  payload  Message payload to be published
 * @return RC76XX_OK or error
 */
RC76XX_Result_t RC76XX_Publish(RC76XX_Handle_t *h,
                               const char *topic,
                               const char *payload);

/**
 * @brief  Subscribe to a topic.
 * @param  h  driver handle
 * @param  topic  MQTT topic to subscribe to
 * @return RC76XX_OK or error
 */
RC76XX_Result_t RC76XX_Subscribe(RC76XX_Handle_t *h,
                                 const char *topic);

/**
 * @brief  Disconnect MQTT and tear down.
 * @param  h  driver handle
 * @return RC76XX_OK or error
 */
RC76XX_Result_t RC76XX_Disconnect(RC76XX_Handle_t *h);

/**
 * @brief  Register URC (Unsolicited Result Code) handlers for MQTT communication.
 * @param  h  driver handle
 * @return RC76XX_OK or error
 */
RC76XX_Result_t RC76XX_RegisterURCHandlers(RC76XX_Handle_t *h);

// URC handlers
/**
 * @brief  Handle the CREG (Circuit/Network Registration) URC event.
 * @param  args  Arguments associated with the CREG URC
 */
void rc76xx_creg_handler(const char *args);
/**
 * @brief  Handle the CGREG (Circuit/GPRS Network Registration) URC event.
 * @param  args  Arguments associated with the CGREG URC
 */
void rc76xx_cgreg_handler(const char *args);

/**
 * @brief  Handle the MQTT data received URC event.
 * @param  args  Arguments associated with the MQTT data URC
 */
void rc76xx_mqtt_data_handler(const char *args);

/**
 * @brief  Handle the MQTT indication URC event.
 * @param  args  Arguments associated with the MQTT indication URC
 */
void rc76xx_mqtt_ind_handler(const char *args);

/**
 * @brief  Handle the reset URC event.
 * @param  args  Arguments associated with the reset URC
 */
void rc76xx_reset_handler(const char *args);

/**
 * @brief  Handle the OK response URC event.
 * @param  args  Arguments associated with the OK URC
 */
void rc76xx_ok_handler(const char *args);

/**
 * @brief  Handle the error URC event.
 * @param  args  Arguments associated with the error URC
 */
void rc76xx_error_handler(const char *args);

#endif // RC76XX_MQTT_H
