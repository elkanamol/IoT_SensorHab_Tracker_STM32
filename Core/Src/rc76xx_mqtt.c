
#include "rc76xx_mqtt.h"
#include "uat_freertos.h"
#include "uat_parser.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

//===========================================================================
//  Timeouts in RTOS ticks
//===========================================================================
#define CMD_TIMEOUT pdMS_TO_TICKS(5000)
#define URC_TIMEOUT pdMS_TO_TICKS(15000)

static RC76XX_Handle_t *g_mqtt_handle = NULL;


RC76XX_Result_t RC76XX_Reset(RC76XX_Handle_t *h)
{
    if (!h)
    {
        return RC76XX_ERR_INTERNAL;
    }
    char resp[64];
    // 1) Reset
    if (uAT_SendReceive("AT+CFUN=1,1", "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }

    h->state = RC76XX_STATE_RESET;
    return RC76XX_OK;
}


RC76XX_Result_t RC76XX_Initialize(RC76XX_Handle_t *h)
{
    if (!h || h->state != RC76XX_STATE_RESET)
    {
        return RC76XX_ERR_STATE;
    }

    char resp[256];

    // 1) Basic AT check
    if (uAT_SendReceive("AT", "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }

    // 2) Get IMEI
    if (uAT_SendReceive("ATI", "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }
    // parse full line into h->imei
    if (!uAT_ParseString(resp, "IMEI: ", h->imei, sizeof(h->imei)))
    {
        return RC76XX_ERR_PARSE;
    }

    // parse model from resp to h->model
    if (!uAT_ParseString(resp, "Model: ", h->model, sizeof(h->model)))
    {
        return RC76XX_ERR_PARSE;
    }

    // 3) Disable echo
    if (uAT_SendReceive("ATE0", "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }

    // State transition: RESET → INITIALIZED
    h->state = RC76XX_STATE_INITIALIZED;
    return RC76XX_OK;
}


RC76XX_Result_t RC76XX_NetworkAttach(RC76XX_Handle_t *h)
{
    if (!h || h->state != RC76XX_STATE_INITIALIZED)
    {
        return RC76XX_ERR_STATE;
    }

    char resp[64], param[64];

    if (uAT_SendReceive("AT+CREG?", "OK", resp, sizeof(resp), URC_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }
    if (!uAT_ParseString(resp, "+CREG: ", param, sizeof(param)))
    {
        return RC76XX_ERR_PARSE;
    }
    if ((strstr(param, "0,1")) || (strstr(param, "0,5")))
    {
        h->state = RC76XX_STATE_ERROR;
        return RC76XX_ERR_STATE;
    }

    // 1) Attach to network
    if (uAT_SendReceive("AT+CGATT=1", "OK", resp, sizeof(resp), URC_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }

    // 2) Define PDP context (CID=1, IPv4)
    if (uAT_SendReceive("AT+CGDCONT=1,\"IP\",\"uinternet\"",
                        "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }

    // // 3) Activate the context
    // if (uAT_SendReceive("AT+CGACT=1,1", "OK", resp, sizeof(resp), URC_TIMEOUT) != UAT_OK)
    // {
    //     return RC76XX_ERR_AT;
    // }

    // 4) Query IP
    if (uAT_SendReceive("AT+CGPADDR=1", "+CGPADDR:", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }
    if (!uAT_ParseString(resp, "+CGPADDR: 1,\"", h->ip, sizeof(h->ip)))
    {
        return RC76XX_ERR_PARSE;
    }

    h->state = RC76XX_STATE_NETWORK_READY;
    return RC76XX_OK;
}


RC76XX_Result_t RC76XX_ConfigMQTT(RC76XX_Handle_t *h,
                                  const char *host,
                                  uint16_t port,
                                  const char *clientID,
                                  const char *topic,
                                  const char *username,
                                  const char *password,
                                  bool useTLS,
                                  bool subscribe,
                                  uint16_t keepAliveInterval)
{
    if (!h || h->state != RC76XX_STATE_NETWORK_READY)
    {
        return RC76XX_ERR_STATE;
    }

    char cmd[256], resp[64];
    int written = 0;
    
    // Format the command string
    if (subscribe)
    {
        written = snprintf(cmd, sizeof(cmd),
                 "AT+KMQTTCFG=%d,%s,%u,4,\"%s\",%u,1,0,\"%s\",\"field1=0\",1,0,\"%s\",\"%s\",,\"\"",
                 useTLS ? 1 : 0,
                 host,
                 port,
                 clientID, keepAliveInterval, topic, username, password);
    }
    else
    {
        written = snprintf(cmd, sizeof(cmd),
                 "AT+KMQTTCFG=%d,%s,%u,4,\"%s\",%u,1,0,,,0,0,\"%s\",\"%s\",,\"\"",
                 useTLS ? 1 : 0,
                 host,
                 port,
                 clientID, keepAliveInterval, username, password);
    }
    
    // Check if the buffer was large enough
    if (written < 0 || written >= sizeof(cmd))
    {
        return RC76XX_ERR_BUFFER_OVERFLOW;
    }
    
    if (uAT_SendReceive(cmd, "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }
    if (uAT_SendReceive("AT+KMQTTCFG?", "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }
    if (!uAT_ParseInt(resp, "+KMQTTCFG:", ',', &h->mqtt_cfg_id))
    {
        return RC76XX_ERR_PARSE;
    }

    h->state = RC76XX_STATE_MQTT_CONFIGURED;
    return RC76XX_OK;
}

RC76XX_Result_t RC76XX_ConnectMQTT(RC76XX_Handle_t *h)
{
    if (!h || h->state != RC76XX_STATE_MQTT_CONFIGURED)
    {
        return RC76XX_ERR_STATE;
    }

    char cmd[64], resp[64];

    // AT+KMQTTCNX=<session_id>
    snprintf(cmd, sizeof(cmd), "AT+KMQTTCNX=%d", h->mqtt_cfg_id);

    // 1) wait for +KMQTTCNX: <session_id>
    if (uAT_SendReceive(cmd, "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }
    // note, this handle in URC rgistration.
    // if (uAT_SendReceive("", "+KMQTT_IND: 0,6", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    // {
    //     return RC76XX_ERR_AT;
    // }

    h->state = RC76XX_STATE_MQTT_CONNECTING;
    return RC76XX_OK;
}


RC76XX_Result_t RC76XX_Publish(RC76XX_Handle_t *h,
                               const char *topic,
                               const char *payload)
{
    if (!h || h->state != RC76XX_STATE_MQTT_CONNECTED)
    {
        return RC76XX_ERR_STATE;
    }

    char cmd[RC76XX_TOPIC_LEN + RC76XX_PAYLOAD_LEN + 64];
    char resp[64];

    // AT+KMQTTPUB=<session_id>,<topic>,<qos>,<retained>,"<payload>"
    snprintf(cmd, sizeof(cmd),
             "AT+KMQTTPUB=%d,\"%s\",0,0,\"%s\"",
             h->mqtt_cfg_id,
             topic,
             payload);

    // 1) wait for +KMQTTPUB: <session_id>
    if (uAT_SendReceive(cmd, "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }

    return RC76XX_OK;
}


RC76XX_Result_t RC76XX_Subscribe(RC76XX_Handle_t *h,
                                 const char *topic)
{
    if (!h || h->state != RC76XX_STATE_MQTT_CONNECTED)
    {
        return RC76XX_ERR_STATE;
    }

    char cmd[RC76XX_TOPIC_LEN + 32];
    char resp[64];

    // AT+KMQTTSUB=<session_id>,"<topic>",<qos>
    snprintf(cmd, sizeof(cmd),
             "AT+KMQTTSUB=%d,\"%s\",0",
             h->mqtt_cfg_id,
             topic);

    // 1) wait for +KMQTTSUB: <session_id>
    if (uAT_SendReceive(cmd, "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }


    return RC76XX_OK;
}


RC76XX_Result_t RC76XX_Disconnect(RC76XX_Handle_t *h)
{
    if (!h)
    {
        return RC76XX_ERR_INTERNAL;
    }

    char cmd[32], resp[64];

    // AT+KMQTTCLOSE=<session_id>
    snprintf(cmd, sizeof(cmd), "AT+KMQTTCLOSE=%d", h->mqtt_cfg_id);

    // 1) wait for +KMQTTCLOSE: <session_id>
    if (uAT_SendReceive(cmd, "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }

    h->state = RC76XX_STATE_RESET;
    return RC76XX_OK;
}

// URC handler registration function
RC76XX_Result_t RC76XX_RegisterURCHandlers(RC76XX_Handle_t *h)
{
    if (!h)
    {
        return RC76XX_ERR_INTERNAL;
    }

    // Store the handle in the global variable for use by handlers
    g_mqtt_handle = h;

    // Register all URC handlers
    uAT_Result_t result;

    result = uAT_RegisterURC("+CREG", rc76xx_creg_handler);
    if (result != UAT_OK)
    {
        printf("Failed to register +CREG handler: %d\n", result);
        return RC76XX_ERR_AT;
    }

    result = uAT_RegisterURC("+CGREG", rc76xx_cgreg_handler);
    if (result != UAT_OK)
    {
        printf("Failed to register +CGREG handler: %d\n", result);
        return RC76XX_ERR_AT;
    }

    result = uAT_RegisterURC("+KMQTT_DATA:", rc76xx_mqtt_data_handler);
    if (result != UAT_OK)
    {
        printf("Failed to register +KMQTT_DATA handler: %d\n", result);
        return RC76XX_ERR_AT;
    }

    result = uAT_RegisterURC("+KMQTT_IND: 0,", rc76xx_mqtt_ind_handler);
    if (result != UAT_OK)
    {
        printf("Failed to register +KMQTT_IND handler: %d\n", result);
        return RC76XX_ERR_AT;
    }

    result = uAT_RegisterURC("+QCEUICCSUPPORT:0", rc76xx_reset_handler);
    if (result != UAT_OK)
    {
        printf("Failed to register reset handler: %d\n", result);
        return RC76XX_ERR_AT;
    }

    // this can do an errors when sending sendReceive commands. be curefull!!
    // result = uAT_RegisterCommand("OK", rc76xx_ok_handler);
    // if (result != UAT_OK)
    // {
    //     printf("Failed to register OK handler: %d\n", result);
    //     return RC76XX_ERR_AT;
    // }

    result = uAT_RegisterCommand("ERROR", rc76xx_error_handler);
    if (result != UAT_OK)
    {
        printf("Failed to register ERROR handler: %d\n", result);
        return RC76XX_ERR_AT;
    }

    return RC76XX_OK;
}

/* creg_handler: called whenever a line beginning with "+CREG" is received */
void rc76xx_creg_handler(const char *args)
{
    // args might be e.g. ": 1" or ": 0,1"
    printf("[%lu] >>> Network registration URC: %s", HAL_GetTick(), args);
    if (strcmp(args, ": 1,1") == 0)
    {
        printf("[%lu] >>> Network EPS registration successful", HAL_GetTick());
    }
}

/* cgreg_handler: called whenever a line beginning with "+CGREG" is received */
void rc76xx_cgreg_handler(const char *args)
{
    // args might be e.g. ": 1" or ": 0,1"
    printf("[%lu] >>> Network registration URC: %s", HAL_GetTick(), args);
    if (strcmp(args, ": 1") == 0)
    {
        printf("[%lu] >>> Network GPRS registration successful!!", HAL_GetTick());
    }
}

/* mqtt_data_handler: called whenever a line beginning with "+MQTT_DATA" is received */
void rc76xx_mqtt_data_handler(const char *args)
{
    printf("[%lu] >>> MQTT data URC: %s", HAL_GetTick(), args);
    // TODO: Extract the data, "home/LWTMessage" => topic, "Test 40447" => payload
    // char topic[100] = {""};
    // char payload[1024] = {""};
    // sscanf(args, "0,%[^,],%[^,]", topic, payload);
    // printf("[%lu] >>> MQTT data topic: %s, payload: %s", HAL_GetTick(), topic, payload);
}

/* mqtt_ind_handler: called whenever a line beginning with "+MQTT_IND" is received */
void rc76xx_mqtt_ind_handler(const char *args)
{
    if (!g_mqtt_handle)
    {
        printf("[%lu] >>> MQTT indication URC but no handle available: %s", HAL_GetTick(), args);
        return;
    }

    printf("[%lu] >>> MQTT indication URC: %s", HAL_GetTick(), args);
    switch (atol(args))
    {
    case 0:
        printf("[%lu] >>> MQTT connection aborted\n", HAL_GetTick());
        g_mqtt_handle->state = RC76XX_STATE_ERROR;
        break;
    case 1:
        printf("[%lu] >>> MQTT connection successful\n", HAL_GetTick());
        g_mqtt_handle->state = RC76XX_STATE_MQTT_CONNECTED;
        break;
    case 2:
        printf("[%lu] >>> MQTT subscribed to a topic successful\n", HAL_GetTick());
        break;
    case 3:
        printf("[%lu] >>> MQTT unsubscribed to a topic successful\n", HAL_GetTick());
        break;
    case 4:
        printf("[%lu] >>> MQTT message published successful\n", HAL_GetTick());
        break;
    case 5:
        printf("[%lu] >>> MQTT generic error\n", HAL_GetTick());
        g_mqtt_handle->state = RC76XX_STATE_ERROR;
        break;
    case 6:
        printf("[%lu] >>> MQTT socket open successful\n", HAL_GetTick());
        g_mqtt_handle->state = RC76XX_STATE_MQTT_CONNECTING;
        break;
    default:
        printf("[%lu] >>> MQTT unknown indication\n", HAL_GetTick());
        break;
    }
}

// Reset handler to handle the reset URC from the module
void rc76xx_reset_handler(const char *args)
{
    if (!g_mqtt_handle)
    {
        printf("[%lu] >>> RC76xx modem has reset but no handle available!", HAL_GetTick());
        return;
    }

    g_mqtt_handle->state = RC76XX_STATE_RESET;
    printf("[%lu] >>> RC76xx modem has reset!!, state transition to RC76XX_STATE_RESET:", HAL_GetTick());
}

// This will be called whenever a line "OK\r\n" arrives.
void rc76xx_ok_handler(const char *args)
{
    // args points just past the "OK" in your buffer.
    printf("[%lu] >>> Got OK response%s", HAL_GetTick(), args);
}

// This will be called whenever a line "ERROR\r\n" arrives.
void rc76xx_error_handler(const char *args)
{
    printf("[%lu] >>> Got ERROR response%s", HAL_GetTick(), args);
}