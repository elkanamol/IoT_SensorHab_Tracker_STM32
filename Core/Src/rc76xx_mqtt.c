#include "rc76xx_mqtt.h"
#include "uat_freertos.h"
#include "uat_parser.h"
#include <string.h>
#include <stdio.h>

//===========================================================================
//  Timeouts in RTOS ticks
//===========================================================================
#define CMD_TIMEOUT pdMS_TO_TICKS(5000)
#define URC_TIMEOUT pdMS_TO_TICKS(15000)

//===========================================================================
// Reset the modem
//===========================================================================
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

//===========================================================================
//  Initialize: AT → get IMEI → echo off
//===========================================================================
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

    h->state = RC76XX_STATE_INITIALIZED;
    return RC76XX_OK;
}

//===========================================================================
//  Network attach & get IP
//===========================================================================
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

//===========================================================================
//  Configure MQTT parameters
//===========================================================================
RC76XX_Result_t RC76XX_ConfigMQTT(RC76XX_Handle_t *h,
                                  const char *host,
                                  uint16_t port,
                                  const char *clientID,
                                  const char *topic,
                                  const char *username,
                                  const char *password,
                                  bool useTLS,
                                  uint16_t keepAliveInterval)
{
    if (!h || h->state != RC76XX_STATE_NETWORK_READY)
    {
        return RC76XX_ERR_STATE;
    }

    char cmd[256], resp[64];

    snprintf(cmd, sizeof(cmd),
             "AT+KMQTTCFG=%d,%s,%u,4,\"%s\",%u,1,0,\"%s\",\"field1=0\",1,0,\"%s\",\"%s\",,\"\"",
             useTLS ? 1 : 0,
             host,
             port,
             clientID, keepAliveInterval, topic, username, password);

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

//===========================================================================
//  Connect MQTT
//===========================================================================
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
    if (uAT_SendReceive("", "+KMQTT_IND: 0,6", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }

    h->state = RC76XX_STATE_MQTT_CONNECTED;
    return RC76XX_OK;
}

//===========================================================================
//  Publish
//===========================================================================
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

//===========================================================================
//  Subscribe
//===========================================================================
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

//===========================================================================
//  Disconnect
//===========================================================================
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
    uAT_SendReceive(cmd, "+KMQTTCLOSE:", resp, sizeof(resp), CMD_TIMEOUT);
    // 2) wait for OK
    uAT_SendReceive("", "OK", resp, sizeof(resp), CMD_TIMEOUT);

    h->state = RC76XX_STATE_RESET;
    return RC76XX_OK;
}

