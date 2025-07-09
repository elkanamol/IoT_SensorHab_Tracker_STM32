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

/**
 * @brief Maps parser error codes to RC76XX error codes
 * 
 * @param parseResult The parser result code
 * @return RC76XX_Result_t The corresponding RC76XX error code
 */
static RC76XX_Result_t MapParserError(uAT_ParseResult_t parseResult)
{
    switch (parseResult)
    {
        case UAT_PARSE_OK:
            return RC76XX_OK;
        case UAT_PARSE_BUFFER_TOO_SMALL:
            return RC76XX_ERR_BUFFER_OVERFLOW;
        case UAT_PARSE_NULL_ARG:
        case UAT_PARSE_INVALID_VALUE:
            return RC76XX_ERR_INVALID_PARAM;
        case UAT_PARSE_PREFIX_NOT_FOUND:
        case UAT_PARSE_INVALID_FORMAT:
        default:
            return RC76XX_ERR_PARSE;
    }
}

/**
 * @brief Check for CME/CMS errors in response
 * 
 * @param resp Response string to check
 * @return RC76XX_Result_t RC76XX_OK if no error, or appropriate error code
 */
static RC76XX_Result_t CheckModemErrors(const char *resp)
{
    int errorCode;
    
    if (uAT_IsCMEError(resp, &errorCode))
    {
        RC76XX_ERROR("Modem reported CME ERROR: %u\n", errorCode);
        return RC76XX_ERR_CME;
    }
    
    if (uAT_IsCMSError(resp, &errorCode))
    {
        RC76XX_ERROR("Modem reported CMS ERROR: %u\n", errorCode);
        return RC76XX_ERR_CMS;
    }
    
    return RC76XX_OK;
}

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
        RC76XX_ERROR("Invalid state for initialization: %u\n", h ? h->state : -1);
        return RC76XX_ERR_STATE;
    }

    char resp[256];
    uAT_ParseResult_t parseResult;

    RC76XX_VERBOSE("Sending basic AT check\n");
    // 1) Basic AT check
    if (uAT_SendReceive("AT", "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        RC76XX_Result_t result = CheckModemErrors(resp);
        if (result != RC76XX_OK)
        {
            return result;
        }
        RC76XX_ERROR("AT command failed\n");
        return RC76XX_ERR_AT;
    }

    RC76XX_VERBOSE("Getting device information\n");
    // 2) Get IMEI
    if (uAT_SendReceive("ATI", "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        RC76XX_Result_t result = CheckModemErrors(resp);
        if (result != RC76XX_OK)
        {
            return result;
        }
        RC76XX_ERROR("ATI command failed\n");
        return RC76XX_ERR_AT;
    }
    
    // Parse IMEI with enhanced error handling
    parseResult = uAT_ParseString(resp, "IMEI: ", h->imei, sizeof(h->imei));
    if (parseResult != UAT_PARSE_OK)
    {
        RC76XX_ERROR("Failed to parse IMEI: %d\n", parseResult);
        return MapParserError(parseResult);
    }
    RC76XX_INFO("Device IMEI: %s\n", h->imei);

    // Parse model with enhanced error handling
    parseResult = uAT_ParseString(resp, "Model: ", h->model, sizeof(h->model));
    if (parseResult != UAT_PARSE_OK)
    {
        RC76XX_ERROR("Failed to parse model: %d\n", parseResult);
        return MapParserError(parseResult);
    }
    RC76XX_INFO("Device model: %s\n", h->model);

    RC76XX_VERBOSE("Disabling command echo\n");
    // 3) Disable echo
    if (uAT_SendReceive("ATE0", "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        RC76XX_Result_t result = CheckModemErrors(resp);
        if (result != RC76XX_OK)
        {
            return result;
        }
        RC76XX_ERROR("ATE0 command failed\n");
        return RC76XX_ERR_AT;
    }

    // State transition: RESET → INITIALIZED
    h->state = RC76XX_STATE_INITIALIZED;
    RC76XX_INFO("Modem initialized successfully\n");
    return RC76XX_OK;
}


RC76XX_Result_t RC76XX_NetworkAttach(RC76XX_Handle_t *h)
{
    if (!h || h->state != RC76XX_STATE_INITIALIZED)
    {
        RC76XX_ERROR("Modem not initialized\n");
        return RC76XX_ERR_STATE;
    }

    char resp[64], param[64];
    uAT_ParseResult_t parseResult;

    if (uAT_SendReceive("AT+CREG?", "OK", resp, sizeof(resp), URC_TIMEOUT) != UAT_OK)
    {
        RC76XX_Result_t result = CheckModemErrors(resp);
        if (result != RC76XX_OK)
        {
            RC76XX_ERROR("Failed to get CREG status: %d\n", result);
            return result;
        }
        return RC76XX_ERR_AT;
    }
    
    parseResult = uAT_ParseString(resp, "+CREG: ", param, sizeof(param));
    if (parseResult != UAT_PARSE_OK)
    {
        RC76XX_ERROR("Failed to parse CREG status: %d\n", parseResult);
        return MapParserError(parseResult);
    }
    
    if ((strstr(param, "0,1")) || (strstr(param, "0,5")))
    {
        h->state = RC76XX_STATE_ERROR;
        return RC76XX_ERR_STATE;
    }

    // 1) Attach to network
    if (uAT_SendReceive("AT+CGATT=1", "OK", resp, sizeof(resp), URC_TIMEOUT) != UAT_OK)
    {
        RC76XX_Result_t result = CheckModemErrors(resp);
        if (result != RC76XX_OK)
        {
            return result;
        }
        RC76XX_ERROR("AT+CGATT=1 command failed\n");
        return RC76XX_ERR_AT;
    }

    // 2) Define PDP context (CID=1, IPv4)
    if (uAT_SendReceive("AT+CGDCONT=1,\"IP\",\"uinternet\"",
                        "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        RC76XX_Result_t result = CheckModemErrors(resp);
        if (result != RC76XX_OK)
        {
            return result;
        }
        RC76XX_ERROR("AT+CGDCONT=1,\"IP\",\"uinternet\" command failed\n");
        return result;
    }

    // // 3) Activate the context
    // if (uAT_SendReceive("AT+CGACT=1,1", "OK", resp, sizeof(resp), URC_TIMEOUT) != UAT_OK)
    // {
    //     return RC76XX_ERR_AT;
    // }

    // 4) Query IP
    if (uAT_SendReceive("AT+CGPADDR=1", "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        RC76XX_Result_t result = CheckModemErrors(resp);
        if (result != RC76XX_OK)
        {
            RC76XX_ERROR("AT+CGPADDR=1 command failed: %d\n", result);
            return result;
        }
        return RC76XX_ERR_AT;
    }
    
    // Looking at the actual response, we need to use a different parsing approach
    // The response format is: "+CGPADDR: 1,"100.96.109.51""
    
    // First, check if the response contains the expected prefix
    if (!strstr(resp, "+CGPADDR: 1,"))
    {
        RC76XX_ERROR("IP address response format unexpected\n");
        return RC76XX_ERR_PARSE;
    }
    
    // Use the string parsing function instead of IP address parsing
    // This is more flexible with the actual response format
    parseResult = uAT_ParseQuotedString(resp, "+CGPADDR: 1,", h->ip, sizeof(h->ip));
    if (parseResult != UAT_PARSE_OK)
    {
        RC76XX_ERROR("Failed to parse IP address: %d\n", parseResult);
        return MapParserError(parseResult);
    }
    
    // Validate that what we parsed looks like an IP address
    if (!h->ip[0] || !strchr(h->ip, '.'))
    {
        RC76XX_ERROR("Parsed string doesn't appear to be an IP address: %s\n", h->ip);
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
    
    // Validate input parameters
    if (!host || !clientID || !username || !password)
    {
        return RC76XX_ERR_INVALID_PARAM;
    }
    
    if (subscribe && !topic)
    {
        return RC76XX_ERR_INVALID_PARAM;
    }

    char cmd[256], resp[64];
    memset(cmd, 0, sizeof(cmd));
    memset(resp, 0, sizeof(resp));
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
        RC76XX_Result_t result = CheckModemErrors(resp);
        if (result != RC76XX_OK)
        {
            return result;
        }
        return RC76XX_ERR_AT;
    }
    if (UAT_PARSE_OK != uAT_ParseInt(resp, "+KMQTTCFG: ", ',', &h->mqtt_cfg_id))
    {
        return RC76XX_ERR_PARSE;
    }
    if (uAT_SendReceive("AT+KMQTTCFG?", "OK", resp, sizeof(resp), CMD_TIMEOUT) != UAT_OK)
    {
        return RC76XX_ERR_AT;
    }
    RC76XX_INFO("MQTT configured with session ID: %d\n", h->mqtt_cfg_id);

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
        RC76XX_ERROR("Failed to register +CREG handler: %d\n", result);
        return RC76XX_ERR_AT;
    }

    result = uAT_RegisterURC("+CGREG", rc76xx_cgreg_handler);
    if (result != UAT_OK)
    {
        RC76XX_ERROR("Failed to register +CGREG handler: %d\n", result);
        return RC76XX_ERR_AT;
    }

    result = uAT_RegisterURC("+KMQTT_DATA:", rc76xx_mqtt_data_handler);
    if (result != UAT_OK)
    {
        RC76XX_ERROR("Failed to register +KMQTT_DATA handler: %d\n", result);
        return RC76XX_ERR_AT;
    }

    result = uAT_RegisterURC("+KMQTT_IND: 0,", rc76xx_mqtt_ind_handler);
    if (result != UAT_OK)
    {
        RC76XX_ERROR("Failed to register +KMQTT_IND handler: %d\n", result);
        return RC76XX_ERR_AT;
    }

    result = uAT_RegisterURC("+QCEUICCSUPPORT:0", rc76xx_reset_handler);
    if (result != UAT_OK)
    {
        RC76XX_ERROR("Failed to register reset handler: %d\n", result);
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
        RC76XX_ERROR("Failed to register ERROR handler: %d\n", result);
        return RC76XX_ERR_AT;
    }

    return RC76XX_OK;
}

/* creg_handler: called whenever a line beginning with "+CREG" is received */
void rc76xx_creg_handler(const char *args)
{
    // args might be e.g. ": 1" or ": 0,1"
    RC76XX_INFO("[%u] >>> Network registration URC: %s", HAL_GetTick(), args);
    if (strcmp(args, ": 1,1") == 0)
    {
        RC76XX_INFO("[%u] >>> Network EPS registration successful", HAL_GetTick());
    }
}

/* cgreg_handler: called whenever a line beginning with "+CGREG" is received */
void rc76xx_cgreg_handler(const char *args)
{
    // args might be e.g. ": 1" or ": 0,1"
    RC76XX_INFO("[%u] >>> Network registration URC: %s", HAL_GetTick(), args);
    if (strcmp(args, ": 1") == 0)
    {
        RC76XX_INFO("[%u] >>> Network GPRS registration successful!!", HAL_GetTick());
    }
}

/* mqtt_data_handler: called whenever a line beginning with "+KMQTT_DATA:" is received */
void rc76xx_mqtt_data_handler(const char *args)
{
    RC76XX_INFO("[%u] >>> MQTT data URC: %s", HAL_GetTick(), args);
    
    // // Extract topic and payload from the data
    // char topic[RC76XX_TOPIC_LEN] = {0};
    // char payload[RC76XX_PAYLOAD_LEN] = {0};
    
    // // Format is: "<socket_ID>,\"<topic_name>\",\"<payload>\""
    // // First, skip the socket ID
    // const char *topicStart = strchr(args, ',');
    // if (!topicStart) {
    //     printf("[%u] >>> Error: Invalid MQTT data format (no comma found)\n", HAL_GetTick());
    //     return;
    // }
    
    // topicStart++; // Skip the comma
    
    // // Extract the topic (quoted string)
    // if (*topicStart != '\"') {
    //     printf("[%u] >>> Error: Invalid MQTT data format (topic not quoted)\n", HAL_GetTick());
    //     return;
    // }
    
    // topicStart++; // Skip opening quote
    // const char *topicEnd = strchr(topicStart, '\"');
    // if (!topicEnd) {
    //     printf("[%u] >>> Error: Invalid MQTT data format (topic not properly quoted)\n", HAL_GetTick());
    //     return;
    // }
    
    // size_t topicLen = topicEnd - topicStart;
    // if (topicLen >= RC76XX_TOPIC_LEN) {
    //     printf("[%u] >>> Error: Topic too long (%zu bytes)\n", HAL_GetTick(), topicLen);
    //     topicLen = RC76XX_TOPIC_LEN - 1; // Truncate
    // }
    
    // memcpy(topic, topicStart, topicLen);
    // topic[topicLen] = '\0';
    
    // // Now look for the payload
    // const char *payloadStart = topicEnd + 1; // Move past the closing quote
    
    // // Find the next comma
    // payloadStart = strchr(payloadStart, ',');
    // if (!payloadStart) {
    //     printf("[%u] >>> Error: Invalid MQTT data format (no payload delimiter)\n", HAL_GetTick());
    //     return;
    // }
    
    // payloadStart++; // Skip the comma
    
    // // Check if payload is quoted
    // if (*payloadStart != '\"') {
    //     printf("[%u] >>> Error: Invalid MQTT data format (payload not quoted)\n", HAL_GetTick());
    //     return;
    // }
    
    // payloadStart++; // Skip opening quote
    
    // // For the payload, we need to handle nested quotes in JSON
    // // We'll look for the last quote in the string
    // // We'll look for the last quote in the string
    // const char *payloadEnd = NULL;
    // const char *lastQuote = payloadStart;
    
    // // Find the last quote in the string
    // while ((lastQuote = strchr(lastQuote + 1, '\"')) != NULL) {
    //     payloadEnd = lastQuote;
    // }
    
    // if (!payloadEnd) {
    //     printf("[%u] >>> Error: Invalid MQTT data format (payload not properly quoted)\n", HAL_GetTick());
    //     return;
    // }
    
    // size_t payloadLen = payloadEnd - payloadStart;
    // if (payloadLen >= RC76XX_RX_PAYLOAD_LEN)
    // {
    //     printf("[%u] >>> Error: Payload too long (%zu bytes)\n", HAL_GetTick(), payloadLen);
    //     payloadLen = RC76XX_RX_PAYLOAD_LEN - 1; // Truncate
    // }

    // memcpy(payload, payloadStart, payloadLen);
    // payload[payloadLen] = '\0';
    
    // // Print the extracted data
    // printf("[%u] >>> MQTT data:\n", HAL_GetTick());
    // printf("[%u] >>> Topic: %s\n", HAL_GetTick(), topic);
    // printf("[%u] >>> Payload: %s\n", HAL_GetTick(), payload);
    
    // // If the payload is JSON, you might want to parse it further
    // // For example, if it contains a field1 vaue as shown in your example
    // if (strstr(payload, "field1")) {
    //     // You could extract specific fields here if needed
    //     printf("[%u] >>> JSON payload detected\n", HAL_GetTick());
    // }
}

/* mqtt_ind_handler: called whenever a line beginning with "+MQTT_IND" is received */
void rc76xx_mqtt_ind_handler(const char *args)
{
    if (!g_mqtt_handle)
    {
        RC76XX_ERROR("MQTT indication URC but no handle available: %s\n", args);
        return;
    }

    RC76XX_VERBOSE("MQTT indication URC: %s\n", args);
    
    int indicator = atoi(args);
    switch (indicator)
    {
    case 0:
        RC76XX_WARNING("MQTT connection aborted\n");
        g_mqtt_handle->state = RC76XX_STATE_ERROR;
        break;
    case 1:
        RC76XX_INFO("MQTT connection successful\n");
        g_mqtt_handle->state = RC76XX_STATE_MQTT_CONNECTED;
        break;
    case 2:
        RC76XX_INFO("MQTT subscribed to a topic successful\n");
        break;
    case 3:
        RC76XX_INFO("MQTT unsubscribed to a topic successful\n");
        break;
    case 4:
        RC76XX_INFO("MQTT message published successful\n");
        break;
    case 5:
        RC76XX_WARNING("MQTT generic error\n");
        g_mqtt_handle->state = RC76XX_STATE_ERROR;
        break;
    case 6:
        RC76XX_INFO("MQTT socket open successful\n");
        g_mqtt_handle->state = RC76XX_STATE_MQTT_CONNECTING;
        break;
    default:
        RC76XX_WARNING("MQTT unknown indication: %d\n", indicator);
        break;
    }
}

// Reset handler to handle the reset URC from the module
void rc76xx_reset_handler(const char *args)
{
    (void)args;
    if (!g_mqtt_handle)
    {
        RC76XX_ERROR("[%u] >>> RC76xx modem has reset but no handle available!\r\n", HAL_GetTick());
        return;
    }

    g_mqtt_handle->state = RC76XX_STATE_RESET;
    RC76XX_INFO("[%u] >>> RC76xx modem has reset!!, state transition to RC76XX_STATE_RESET:\r\n", HAL_GetTick());
}

// This will be called whenever a line "OK\r\n" arrives.
void rc76xx_ok_handler(const char *args)
{
    // args points just past the "OK" in your buffer.
    RC76XX_INFO("[%u] >>> Got OK response%s", HAL_GetTick(), args);
}

// This will be called whenever a line "ERROR\r\n" arrives.
void rc76xx_error_handler(const char *args)
{
    RC76XX_ERROR("[%u] >>> Got ERROR response%s", HAL_GetTick(), args);
}
