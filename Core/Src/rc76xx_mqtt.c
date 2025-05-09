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
//  Initialize: AT → get IMEI → echo off
//===========================================================================
RC76XX_Result_t RC76XX_Initialize(RC76XX_Handle_t *h)
{
    if (!h || h->state != RC76XX_STATE_RESET)
    {
        return RC76XX_ERR_STATE;
    }

    char resp[64];

    // 1) Basic AT check
    if (uAT_SendReceive("AT", "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }

    // 2) Get IMEI
    if (uAT_SendReceive("AT+CGSN", "", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }
    // Parse the full line into h->imei
    if (!uAT_ParseString(resp, "", h->imei, sizeof(h->imei)))
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

    char resp[64];

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

    // 3) Activate the context
    if (uAT_SendReceive("AT+CGACT=1,1", "OK", resp, sizeof(resp), URC_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }

    // 4) Query IP
    if (uAT_SendReceive("AT+CGPADDR=1", "+CGPADDR:", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }
    // Use parser helper to extract the IP after "+CGPADDR:"
    if (!uAT_ParseIPAddress(resp, "+CGPADDR:", h->ip, sizeof(h->ip)))
    {
        return RC76XX_ERR_PARSE;
    }
    printf("IP: %s\n", h->ip);

    // 2) Define PDP context (CID=1, IPv4)
    if (uAT_SendReceive("AT+CNXCFG=1,\"GPRS\",\"uinternet\"",
                        "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }
    // Use parser helper to extract the IP after "+CGPADDR:"
    if (!uAT_ParseIPAddress(resp, "+CNXCFG:", h->cnx_id, sizeof(h->ip)))
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
                                  bool useTLS)
{
    if (!h || h->state != RC76XX_STATE_NETWORK_READY)
    {
        return RC76XX_ERR_STATE;
    }

    char cmd[128], resp[64];

    // AT+KMQTTCFG=1,0,iot.eclipse.org,1883,4,"BX3101",30,1,1,"home/LWTMessage "," BX3101offline ",0,1,username,password
        // Build configuration command
        snprintf(cmd, sizeof(cmd),
                 "AT+KMQTTCFG=\"%s:%u\",\"%s\",60,1,%d",
                 host, port, clientID, useTLS ? 1 : 0);

    // 1) Send config, wait for +KMQTTCFG: <id>
    if (uAT_SendReceive(cmd, "+KMQTTCFG:", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }
    // Parse the integer ID after the prefix
    if (!uAT_ParseInt(resp, "+KMQTTCFG:", ',', &h->mqtt_cfg_id))
    {
        return RC76XX_ERR_PARSE;
    }

    // 2) Wait for final OK
    if (uAT_SendReceive("", "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
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
    snprintf(cmd, sizeof(cmd), "AT+KMQTTCNX=%d", h->mqtt_cfg_id);

    if (uAT_SendReceive(cmd, "OK", resp, sizeof(resp), URC_TIMEOUT) != UAT_OK)
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

    char cmd[RC76XX_TOPIC_LEN + RC76XX_PAYLOAD_LEN + 32];
    char resp[64];

    snprintf(cmd, sizeof(cmd),
             "AT+KMQTTPUB=%d,\"%s\",\"%s\",0,0",
             h->mqtt_cfg_id, topic, payload);

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

    char cmd[RC76XX_TOPIC_LEN + 32], resp[64];
    snprintf(cmd, sizeof(cmd),
             "AT+KMQTTSUB=%d,\"%s\",0",
             h->mqtt_cfg_id, topic);

    // Expect +KMQTTSUB: <id>,<topic>
    if (uAT_SendReceive(cmd, "+KMQTTSUB:", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }

    // You could parse the topic back if needed:
    // uAT_ParseQuotedString(resp, ",", parsedTopic, ...);

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
    snprintf(cmd, sizeof(cmd), "AT+KMQTTCLOSE=%d", h->mqtt_cfg_id);
    uAT_SendReceive(cmd, "OK", resp, sizeof(resp), CMD_TIMEOUT);

    h->state = RC76XX_STATE_RESET;
    return RC76XX_OK;
}
