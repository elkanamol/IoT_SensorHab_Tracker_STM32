#include "rc76xx_mqtt.h"
#include "uat_freertos.h"
#include "string.h"
#include "uat_parser.h"
#include "usart.h"



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
    
}

RC76xx_Result_t RC76xx_Deinitialize(RC76xx_Handle_t *handle)
{
    // check if the handle is NULL
    // if it is, return RC76xx_ERR_INVALID_ARG
    
    // issue disconnection function RC76xx_Disconnect, if it is successful, change the state to RC76xx_STATE_UNINITIALIZED
    // if it is not successful, change the state to RC76xx_STATE_ERROR
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
}
RC76xx_Result_t RC76xx_Connect(RC76xx_Handle_t *handle)
{
    // check network status. CREG/CEGREG and COPS
    // if not connected, change state machine to RC76xx_STATE_NETWORK_CONNECTING and return

    // Connects to MQTT broker using AT+KMQTTCNX=<CID> command, wait to OK and check the result for CID +KMQTTCNX: 1
    // wait and handle the URC +KMQTT_IND  


    // if successful, update state machine to RC76xx_STATE_MQTT_CONNECTING
    // if failed, update state machine to RC76xx_STATE_ERROR
}
RC76xx_Result_t RC76xx_Disconnect(RC76xx_Handle_t *handle)
{
    // check if already disconnected and return RC76xx_ERR_INVALID_STATE if so
    // Closes MQTT connection using AT+KMQTTCLOSE command, wait to OK

    // update state machine to RC76xx_STATE_MQTT_DISCONNECTED
}
RC76xx_Result_t RC76xx_Subscribe(RC76xx_Handle_t *handle, const char *topic, uint8_t qos)
{
    // check if already subscribed and return RC76xx_ERR_INVALID_STATE if so
    // Subscribes to MQTT topic using Sends AT+KMQTTSUB=<session_id>,<topic_name>,<qos> (Numeric parameter that is given by +KMQTTCFG.)
    // command with the topic name command wait OK.
    // check response for +KMQTTSUB: <session id>

    // update state machine to RC76xx_STATE_MQTT_SUBSCRIBED
}
RC76xx_Result_t RC76xx_Unsubscribe(RC76xx_Handle_t *handle, const char *topic)
{
    // check if already unsubscribed and return RC76xx_ERR_INVALID_STATE if so
    // Unsubscribes from MQTT topic using AT+KMQTTUNSUB=<CID>,topic command with the topic name command, wait OK.
    // check response for +KMQTTUNSUB: 1
    // register the URC +KMQTT_IND if it is not already registered and wait for +KMQTT_IND: 1,3
}
RC76xx_Result_t RC76xx_Publish(RC76xx_Handle_t *handle, const char *topic, const char *message, uint8_t qos)
{
    // check if the existing state is RC76xx_STATE_MQTT_CONNECTED
    // if not, return RC76xx_ERR_INVALID_STATE

    // Publish the message to the MQTT topic using AT+KMQTTPUB=<CID>,<topic_name>,<qos>,<retained>,<message> command
    //  e.g. "AT+KMQTTPUB=1,"\test_RC7620"\,1,1,"\Hello World! - PUB 0001 - QOS 1"\", wait OK.

    // check response for +KMQTTPUB: 1
    // update state machine to RC76xx_STATE_MQTT_PUBLISHED
}
RC76xx_Result_t RC76xx_GetState(RC76xx_Handle_t *handle, RC76xx_State_t *state)
{
    // check if the state is valid
    // if not, return RC76xx_ERR_INVALID_ARG

    // get the current state
    // if the state is valid, return RC76xx_OK
    // if the state is not valid, return RC76xx_ERR_INVALID_STATE
}

// State machine function
RC76xx_Result_t RC76xx_SetState(RC76xx_Handle_t *handle, RC76xx_State_t newState, uint32_t timeout)
{
    // check if the state is valid
    // if not, return RC76xx_ERR_INVALID_ARG
    // check if the state
    // if it is, return RC76xx_OK
}

/**
 * @brief Handler for MQTT indication URCs (+KMQTT_IND)
 *
 * This function is registered with uAT_RegisterURC() to handle
 * +KMQTT_IND unsolicited responses from the modem.
 *
 * @param args URC arguments from UAT parser
 */
static void RC76xx_MQTT_IND_Handler(const char *args);

/**
 * @brief Handler for MQTT data URCs (+KMQTT_DATA)
 *
 * This function is registered with uAT_RegisterURC() to handle
 * +KMQTT_DATA unsolicited responses containing published messages.
 *
 * @param args URC arguments from UAT parser
 */
static void RC76xx_MQTT_DATA_Handler(const char *args);

/**
 * @brief Handler for network connection URCs (+KCNX_IND)
 *
 * This function is registered with uAT_RegisterURC() to handle
 * +KCNX_IND unsolicited responses for network status changes.
 *
 * @param args URC arguments from UAT parser
 */
static void RC76xx_KCNX_IND_Handler(const char *args);

/**
 * @brief Registers all required URC handlers with the UAT API
 *
 * @param handle Pointer to RC76xx handle
 * @return RC76xx_Result_t Result code
 */
static RC76xx_Result_t RC76xx_RegisterURCHandlers(RC76xx_Handle_t *handle);

/**
 * @brief Notifies application of events through callback
 *
 * @param handle Pointer to RC76xx handle
 * @param eventType Type of event
 * @param data Event-specific data
 * @return RC76xx_Result_t Result code
 */
static RC76xx_Result_t RC76xx_NotifyEvent(RC76xx_Handle_t *handle,
                                          RC76xx_EventType_t eventType,
                                          void *data);
/**
 * @brief Parses MQTT session ID from KMQTTCFG response
 *
 * Uses uAT_ParseInt() to extract session ID from +KMQTTCFG response
 *
 * @param response Response string from AT command
 * @param sessionId Pointer to store session ID
 * @return bool True if successful, false otherwise
 */
static bool RC76xx_ParseSessionId(const char *response, uint8_t *sessionId);

/**
 * @brief Parses network registration status
 *
 * Uses uAT_ParseInt() to extract registration status from +CREG/+CEREG response
 *
 * @param response Response string from AT command
 * @param status Pointer to store registration status
 * @return bool True if successful, false otherwise
 */
static bool RC76xx_ParseRegistrationStatus(const char *response, int *status);

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
static bool RC76xx_ParseIPAddress(const char *response, char *ipAddress, size_t bufferSize);
