#include "rc76xx_mqtt.h"
#include "uat_parser.h"
#include "uat_freertos.h"
#include "string.h"
#include "uat_parser.h"
#include "usart.h"
#include "stdio.h"

RC76xx_Handle_t mqtt_handle;
RC76xx_Result_t RC76xx_Initialize(RC76xx_Handle_t *handle)
{
    // check if the handle is NULL
    // if it is, return RC76xx_ERR_INVALID_ARG

    // if it is not, check if the state machine is initialized
    // check if the device is ready to receive AT commands wait for OK.
    // if not, change the state to RC76xx_STATE_ERROR, return RC76xx_ERR_MODEM

    // if it is, change the state to RC76xx_STATE_INITIALIZED
    // get IMEI number AT+CGSN, wait OK
    // set the state machine to RC76xx_STATE_INITIALIZED
    // if it is, return RC76xx_OK

    if (handle == NULL)
        return RC76xx_ERR_INVALID_ARG;
    // Initialize state machine
    handle->stateMachine.currentState = RC76xx_STATE_UNINITIALIZED;
    handle->stateMachine.errorRetryCount = 0;

    printf("[RC76xx_Initialize] starting RC76 init");

    // Send basic AT command to check if modem is responsive
    char response[512];
    if (uAT_SendReceive("AT", "OK", response, sizeof(response), pdMS_TO_TICKS(1000)) != UAT_OK)
        return RC76xx_ERR_MODEM;
    printf("[RC76xx_Initialize] response: %s", response);

    // Get modem information
    if (uAT_SendReceive("ATI", "OK", response, sizeof(response), pdMS_TO_TICKS(1000)) != UAT_OK)
        return RC76xx_ERR_MODEM;
    printf("[RC76xx_Initialize] response: %s", response);

    // Register URC handlers
    RC76xx_RegisterURCHandlers(handle);
    printf("[RC76xx_Initialize] registered URC handlers");

    // Update state
    handle->stateMachine.currentState = RC76xx_STATE_INITIALIZED;

    return RC76xx_OK;
}
RC76xx_Result_t RC76xx_Check_device_ready(RC76xx_Handle_t *handle)
{
    // check if the device is initialized
    // if not, change the state to RC76xx_STATE_UNINITIALIZED, return RC76xx_ERR_MODEM

    // check SIM card with AT+CPIN?, wait for OK.
    // check theresponse for "+CPIN: READY"
    // if the SIM card is not ready, change the state to RC76xx_STATE_ERROR

    // if the SIM card is ready, change the state to RC76xx_STATE_READY
    // get the SIM IMSI number AT+CIMI, wait OK

    // if the device is ready, change the state to RC76xx_STATE_READY
    printf("[RC76xx_Check_device_ready] starting RC76xx_Check_device_ready");
    return RC76xx_OK;
}

RC76xx_Result_t RC76xx_Deinitialize(RC76xx_Handle_t *handle)
{
    // check if the handle is NULL
    // if it is, return RC76xx_ERR_INVALID_ARG
    
    // issue disconnection function RC76xx_Disconnect, if it is successful, change the state to RC76xx_STATE_UNINITIALIZED
    // if it is not successful, change the state to RC76xx_STATE_ERROR
    printf("[RC76xx_Deinitialize] starting RC76xx_Deinitialize");
    return RC76xx_OK;
}

RC76xx_Result_t RC76xx_SetNetworkConfig(RC76xx_Handle_t *handle, RC76xx_Network_Config_t *networkConfig)
{
    // check if the state machine is RC76xx_STATE_READY or RC76xx_STATE_INITIALIZED
    // if not, change the state to RC76xx_STATE_ERROR, return RC76xx_ERR_INVALID_STATE

    // check if the network configuration is valid
    // if not, change the state to RC76xx_STATE_ERROR, return RC76xx_ERR_INVALID_ARG

    // set the network configuration
    // example: AT+CGDCONT=1,"IP","internet"
    // wait for OK

    // set the AT+KCNXCFG=1,"GPRS","internet"
    // wait for OK
    
    // if the network configuration is set successfully, change the state to RC76xx_STATE_NETWORK_CONFIGURED
    // if it is not successful, change the state to RC76xx_STATE_ERROR
    printf("[RC76xx_SetNetworkConfig] starting RC76xx_SetNetworkConfig");
    return RC76xx_OK;
}
RC76xx_Result_t RC76xx_SetMQTTConfig(RC76xx_Handle_t *handle, RC76xx_MQTT_Config_t *mqttConfig)
{
    // sprintf the AT+KMQTTCFG command with the provided configuration parameters
    // example: AT+KMQTTCFG=1,1,"iot.us-west-2.amazonaws.com",8883,4,"RC7620",,,,,,,,,,0,"x-amzn-mqtt-ca"
    // send the command and wait for OK
    // check the response for +KMQTTCFG: 1

    // set up the SSL/TLS if required for the MQTT connection by using AT+KSSLCRYPTO=0,8,3,25392,12,4,3,0
    // wait for OK

    // update state machine to reflect the MQTT configuration RC76xx_STATE_MQTT_CONFIGURED

    if (!handle || !mqttConfig)
        return RC76xx_ERR_INVALID_ARG;

    // Store configuration
    memcpy(&handle->mqttConfig, mqttConfig, sizeof(RC76xx_MQTT_Config_t));
    printf("[RC76xx_SetMQTTConfig] starting RC76xx_SetMQTTConfig");

    // Format KMQTTCFG command
    char cmd[256] = "AT+KMQTTCFG=1,1,\"a298afy0zf1mqz-ats.iot.us-west-2.amazonaws.com\",8883,4,\"RC7620\",,,,,,,,,,0,\"x-amzn-mqtt-ca\"";
    // snprintf(cmd, sizeof(cmd), "AT+KMQTTCFG=%d,%d,\"%s\",%s,%d,\"%s\",\"%s\",\"%s\",%d,%d,%d,%d,%d,%d,%d,%d,\"%s\"",
    //          mqttConfig->sessionId,
    //          mqttConfig->useTls ? 1 : 0,
    //          mqttConfig->server,
    //          mqttConfig->port,
    //          mqttConfig->mqttVersion,
    //          mqttConfig->client_id,
    //          mqttConfig->username ? mqttConfig->username : "",
    //          mqttConfig->password ? mqttConfig->password : "",
    //          mqttConfig->keepalive,
    //          mqttConfig->cleanSession ? 1 : 0,
    //          mqttConfig->willFlag ? 1 : 0,
    //          mqttConfig->willRetained ? 1 : 0,
    //          mqttConfig->willQos,
    //          mqttConfig->willTopic ? mqttConfig->willTopic : "",
    //          mqttConfig->willMessage ? mqttConfig->willMessage : "",
    //          mqttConfig->securityMethod,
    //          mqttConfig->alpnList ? mqttConfig->alpnList : "");

    // Send command and check response
    char response[128];
    if (uAT_SendReceive(cmd, "+KMQTTCFG:", response, sizeof(response), pdMS_TO_TICKS(5000)) != UAT_OK)
        return RC76xx_ERR_COMMUNICATION;

    // Parse session ID from response
    uint8_t sessionId;
    if (!RC76xx_ParseSessionId(response, &sessionId))
        return RC76xx_ERR_COMMUNICATION;

    // Store session ID
    handle->mqttConfig.sessionId = sessionId;

    // Configure SSL/TLS if needed
    if (mqttConfig->useTls)
    {
        // Hardcoded SSL/TLS configuration as mentioned
        if (uAT_SendReceive("AT+KSSLCRYPTO=0,8,3,25392,12,4,3,0", "OK", response, sizeof(response), pdMS_TO_TICKS(1000)) != UAT_OK)
            return RC76xx_ERR_COMMUNICATION;
    }

    // Update state
    handle->stateMachine.currentState = RC76xx_STATE_MQTT_CONFIGURED;
    printf("[RC76xx_SetMQTTConfig] MQTT configuration successful\n");
    return RC76xx_OK;
}
RC76xx_Result_t RC76xx_Connect(RC76xx_Handle_t *handle)
{
    // check network status. CREG/CEGREG and COPS
    // if not connected, change state machine to RC76xx_STATE_NETWORK_CONNECTING and return

    // Connects to MQTT broker using AT+KMQTTCNX=<CID> command, wait to OK and check the result for CID +KMQTTCNX: 1
    // wait and handle the URC +KMQTT_IND  


    // if successful, update state machine to RC76xx_STATE_MQTT_CONNECTING
    // if failed, update state machine to RC76xx_STATE_ERROR

    if (!handle)
        return RC76xx_ERR_INVALID_ARG;

    printf("[RC76xx_Connect] starting RC76xx_Connect");

    // Check if already connected
    if (handle->stateMachine.currentState == RC76xx_STATE_MQTT_CONNECTED)
        return RC76xx_OK;

    // Check network status first
    char response[128];
    if (uAT_SendReceive("AT+CREG?", "OK", response, sizeof(response), pdMS_TO_TICKS(1000)) != UAT_OK)
        return RC76xx_ERR_COMMUNICATION;

    int regStatus;
    if (!RC76xx_ParseRegistrationStatus(response, &regStatus))
        return RC76xx_ERR_COMMUNICATION;

    // If not registered (status != 1 and status != 5), return error
    if (regStatus != 1 && regStatus != 5)
        return RC76xx_ERR_NOT_CONNECTED;

    // Format connect command
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+KMQTTCNX=%d", handle->mqttConfig.sessionId);

    // Send connect command
    if (uAT_SendReceive(cmd, "+KMQTTCNX:", response, sizeof(response), pdMS_TO_TICKS(10000)) != UAT_OK)
        return RC76xx_ERR_COMMUNICATION;

    // Update state - actual connection confirmation will come through URC
    handle->stateMachine.currentState = RC76xx_STATE_MQTT_CONNECTING;
    printf("[RC76xx_Connect] MQTT connecting\n");
    return RC76xx_OK;
}
RC76xx_Result_t RC76xx_Disconnect(RC76xx_Handle_t *handle)
{
    // check if already disconnected and return RC76xx_ERR_INVALID_STATE if so
    // Closes MQTT connection using AT+KMQTTCLOSE command, wait to OK

    // update state machine to RC76xx_STATE_MQTT_DISCONNECTED
    printf("[RC76xx_Disconnect] starting RC76xx_Disconnect");
    return RC76xx_OK;
}
RC76xx_Result_t RC76xx_Subscribe(RC76xx_Handle_t *handle, const char *topic, uint8_t qos)
{
    // check if already subscribed and return RC76xx_ERR_INVALID_STATE if so
    // Subscribes to MQTT topic using Sends AT+KMQTTSUB=<session_id>,<topic_name>,<qos> (Numeric parameter that is given by +KMQTTCFG.)
    // command with the topic name command wait OK.
    // check response for +KMQTTSUB: <session id>

    // update state machine to RC76xx_STATE_MQTT_SUBSCRIBED
    printf("[RC76xx_Subscribe] starting RC76xx_Subscribe");
    return RC76xx_OK;
}
RC76xx_Result_t RC76xx_Unsubscribe(RC76xx_Handle_t *handle, const char *topic)
{
    // check if already unsubscribed and return RC76xx_ERR_INVALID_STATE if so
    // Unsubscribes from MQTT topic using AT+KMQTTUNSUB=<CID>,topic command with the topic name command, wait OK.
    // check response for +KMQTTUNSUB: 1
    // register the URC +KMQTT_IND if it is not already registered and wait for +KMQTT_IND: 1,3

    printf("[RC76xx_Unsubscribe] starting RC76xx_Unsubscribe");
    return RC76xx_OK;
}
RC76xx_Result_t RC76xx_Publish(RC76xx_Handle_t *handle, const char *topic, const char *message, uint8_t qos)
{
    // check if the existing state is RC76xx_STATE_MQTT_CONNECTED
    // if not, return RC76xx_ERR_INVALID_STATE

    // Publish the message to the MQTT topic using AT+KMQTTPUB=<CID>,<topic_name>,<qos>,<retained>,<message> command
    //  e.g. "AT+KMQTTPUB=1,"\test_RC7620"\,1,1,"\Hello World! - PUB 0001 - QOS 1"\", wait OK.

    // check response for +KMQTTPUB: 1
    // update state machine to RC76xx_STATE_MQTT_PUBLISHED

    if (!handle || !topic || !message)
        return RC76xx_ERR_INVALID_ARG;

    // Check if connected
    if (handle->stateMachine.currentState != RC76xx_STATE_MQTT_CONNECTED &&
        handle->stateMachine.currentState != RC76xx_STATE_MQTT_SUBSCRIBED)
        return RC76xx_ERR_INVALID_STATE;
    printf("[RC76xx_Publish] starting RC76xx_Publish");
    // Format publish command
    char cmd[512];
    snprintf(cmd, sizeof(cmd), "AT+KMQTTPUB=%d,\"%s\",%d,%d,\"%s\"",
             handle->mqttConfig.sessionId,
             topic,
             qos,
             1, // retained flag set to 1
             message);

    // Send publish command
    char response[128];
    if (uAT_SendReceive(cmd, "+KMQTTPUB:", response, sizeof(response), pdMS_TO_TICKS(5000)) != UAT_OK)
        return RC76xx_ERR_COMMUNICATION;

    // Temporarily set state to publishing
    RC76xx_State_t previousState = handle->stateMachine.currentState;
    handle->stateMachine.currentState = RC76xx_STATE_MQTT_PUBLISHING;

    // Note: The actual publish confirmation will come through URC
    // We'll return to the previous state when that happens
    handle->stateMachine.previousState = previousState;
    printf("[RC76xx_Publish] MQTT publishing\n");
    return RC76xx_OK;
}
RC76xx_Result_t RC76xx_GetState(RC76xx_Handle_t *handle, RC76xx_State_t *state)
{
    // check if the state is valid
    // if not, return RC76xx_ERR_INVALID_ARG

    // get the current state
    // if the state is valid, return RC76xx_OK
    // if the state is not valid, return RC76xx_ERR_INVALID_STATE

    printf("[RC76xx_GetState] starting RC76xx_GetState");
    return RC76xx_OK;
}

// State machine function
RC76xx_Result_t RC76xx_SetState(RC76xx_Handle_t *handle, RC76xx_State_t newState, uint32_t timeout)
{
    // check if the state is valid
    // if not, return RC76xx_ERR_INVALID_ARG
    // check if the state
    // if it is, return RC76xx_OK

    printf("[RC76xx_SetState] starting RC76xx_SetState");
    return RC76xx_OK;
}

/**
 * @brief Handler for MQTT indication URCs (+KMQTT_IND)
 *
 * This function is registered with uAT_RegisterURC() to handle
 * +KMQTT_IND unsolicited responses from the modem.
 *
 * @param args URC arguments from UAT parser
 */
static void RC76xx_MQTT_IND_Handler(const char *args)
{
    printf("MQTT_IND: %s\n", args);
}

/**
 * @brief Handler for MQTT data URCs (+KMQTT_DATA)
 *
 * This function is registered with uAT_RegisterURC() to handle
 * +KMQTT_DATA unsolicited responses containing published messages.
 *
 * @param args URC arguments from UAT parser
 */
static void RC76xx_MQTT_DATA_Handler(const char *args)
{
    printf("MQTT_DATA: %s\n", args);
}

/**
 * @brief Handler for network connection URCs (+KCNX_IND)
 *
 * This function is registered with uAT_RegisterURC() to handle
 * +KCNX_IND unsolicited responses for network status changes.
 *
 * @param args URC arguments from UAT parser
 */
static void RC76xx_KCNX_IND_Handler(const char *args)
{
    // Extract handle from global context or passed via uAT_RegisterURC
    RC76xx_Handle_t *handle = &mqtt_handle; // Example - adjust as needed

    // Parse the URC arguments
    int sessionId, status;
    if (sscanf(args, "%d,%d", &sessionId, &status) != 2)
        return;

    // Process based on status code
    switch (status)
    {
    case RC76xx_MQTT_STATUS_CONNECTED: // 1
        handle->stateMachine.currentState = RC76xx_STATE_MQTT_CONNECTED;
        RC76xx_NotifyEvent(handle, RC76xx_EVENT_MQTT_CONNECTED, NULL);
        break;

    case RC76xx_MQTT_STATUS_SUBSCRIBED: // 2
        handle->stateMachine.currentState = RC76xx_STATE_MQTT_SUBSCRIBED;
        break;

    case RC76xx_MQTT_STATUS_PUBLISHED: // 4
        // If we were in publishing state, return to previous state
        if (handle->stateMachine.currentState == RC76xx_STATE_MQTT_PUBLISHING)
            handle->stateMachine.currentState = handle->stateMachine.previousState;

        RC76xx_NotifyEvent(handle, RC76xx_EVENT_MQTT_PUBLISHED, NULL);
        break;

    case RC76xx_MQTT_STATUS_CONNECTION_ABORTED: // 0
        handle->stateMachine.currentState = RC76xx_STATE_MQTT_CONFIGURED;
        RC76xx_NotifyEvent(handle, RC76xx_EVENT_MQTT_DISCONNECTED, NULL);
        break;

        // Handle other status codes...
    }
}

/**
 * @brief Registers all required URC handlers with the UAT API
 *
 * @param handle Pointer to RC76xx handle
 * @return RC76xx_Result_t Result code
 */
static RC76xx_Result_t RC76xx_RegisterURCHandlers(RC76xx_Handle_t *handle)
{
    printf("[RC76xx_RegisterURCHandlers] starting RC76xx_RegisterURCHandlers");
    // Register URC handlers with UAT API
    if (uAT_RegisterURC("+KMQTT_IND:", RC76xx_MQTT_IND_Handler) != UAT_OK)
        return RC76xx_ERR_RESOURCE;

    if (uAT_RegisterURC("+KMQTT_DATA:", RC76xx_MQTT_DATA_Handler) != UAT_OK)
        return RC76xx_ERR_RESOURCE;

    if (uAT_RegisterURC("+KCNX_IND:", RC76xx_KCNX_IND_Handler) != UAT_OK)
        return RC76xx_ERR_RESOURCE;

    return RC76xx_OK;
}

/**
 * @brief Notifies application of events through callback
 *
 * @param handle Pointer to RC76xx handle
 * @param eventType Type of event
 * @param data Event-specific data
 * @return RC76xx_Result_t Result code
 */
static RC76xx_Result_t RC76xx_NotifyEvent(RC76xx_Handle_t *handle, RC76xx_EventType_t eventType, void *data)
{
    printf("[RC76xx_NotifyEvent] starting RC76xx_NotifyEvent");
    return RC76xx_OK;
}
/**
 * @brief Parses MQTT session ID from KMQTTCFG response
 *
 * Uses uAT_ParseInt() to extract session ID from +KMQTTCFG response
 *
 * @param response Response string from AT command
 * @param sessionId Pointer to store session ID
 * @return bool True if successful, false otherwise
 */
static bool RC76xx_ParseSessionId(const char *response, uint8_t *sessionId)
{
    printf("[RC76xx_ParseSessionId] starting RC76xx_ParseSessionId");
    return false;
}

/**
 * @brief Parses network registration status
 *
 * Uses uAT_ParseInt() to extract registration status from +CREG/+CEREG response
 *
 * @param response Response string from AT command
 * @param status Pointer to store registration status
 * @return bool True if successful, false otherwise
 */
static bool RC76xx_ParseRegistrationStatus(const char *response, int *status)
{
    printf("[RC76xx_ParseRegistrationStatus] starting RC76xx_ParseRegistrationStatus");
    return uAT_ParseInt(response, "+GREG:", ",", status);
}

/**
 * @brief Parses IP address from CGPADDR response
 *
 * Uses uAT_ParseIPAddress() to extract IP address from +CGPADDR response
 *
 * @param response Response string from AT command
 * @param ipAddress Buffer to store IP address
 * @param bufferSize Size of buffer
 * @return bool True if successful, false otherwise
 */
static bool RC76xx_ParseIPAddress(const char *response, char *ipAddress, size_t bufferSize)
{
    printf("[RC76xx_ParseIPAddress] starting RC76xx_ParseIPAddress");
    return false;
}
