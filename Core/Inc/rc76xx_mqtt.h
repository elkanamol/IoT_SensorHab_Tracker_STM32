#ifndef RC76XX_MQTT_H
#define RC76XX_MQTT_H

#include "FreeRTOS.h"
#include <stdint.h>
#include <stdbool.h>

#define RC76XX_IMEI_LEN 16
#define RC76XX_MODEL_LEN 32
#define RC76XX_IP_LEN 16
#define RC76XX_CLIENTID_LEN 32
#define RC76XX_TOPIC_LEN 64
#define RC76XX_PAYLOAD_LEN 128

/** High-level result codes for the RC76xx driver */
typedef enum
{
    RC76XX_OK = 0,
    RC76XX_ERR_TIMEOUT,
    RC76XX_ERR_AT,
    RC76XX_ERR_STATE,
    RC76XX_ERR_PARSE,
    RC76XX_ERR_INTERNAL
} RC76XX_Result_t;

/** Driver states */
typedef enum
{
    RC76XX_STATE_RESET = 0,
    RC76XX_STATE_INITIALIZED,
    RC76XX_STATE_NETWORK_READY,
    RC76XX_STATE_MQTT_CONFIGURED,
    RC76XX_STATE_MQTT_CONNECTED,
    RC76XX_STATE_ERROR
} RC76XX_State_t;

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
 * @brief  Configure MQTT parameters (server, port, clientID, SSL).
 * @param  h
 * @param  host   MQTT broker DNS or IP
 * @param  port
 * @param  clientID
 * @param  useTLS  true to enable SSL/TLS
 * @return RC76XX_OK or error
 */
RC76XX_Result_t RC76XX_ConfigMQTT(RC76XX_Handle_t *h,
                                  const char *host,
                                  uint16_t port,
                                  const char *clientID,
                                  const char *username,
                                  const char *password,
                                  bool useTLS);

/**
 * @brief  Connect (open) the MQTT session.
 * @param  h
 * @return RC76XX_OK or error
 */
RC76XX_Result_t RC76XX_ConnectMQTT(RC76XX_Handle_t *h);

/**
 * @brief  Publish a message.
 * @param  h
 * @param  topic
 * @param  payload
 * @return RC76XX_OK or error
 */
RC76XX_Result_t RC76XX_Publish(RC76XX_Handle_t *h,
                               const char *topic,
                               const char *payload);

/**
 * @brief  Subscribe to a topic.
 * @param  h
 * @param  topic
 * @return RC76XX_OK or error
 */
RC76XX_Result_t RC76XX_Subscribe(RC76XX_Handle_t *h,
                                 const char *topic);

/**
 * @brief  Disconnect MQTT and tear down.
 * @param  h
 * @return RC76XX_OK or error
 */
RC76XX_Result_t RC76XX_Disconnect(RC76XX_Handle_t *h);

#endif // RC76XX_MQTT_H
