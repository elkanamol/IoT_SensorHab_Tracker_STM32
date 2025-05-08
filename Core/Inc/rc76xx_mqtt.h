#ifndef __RC76XX_MQTT_H__
#define __RC76XX_MQTT_H__

#include "uat_freertos.h"
#include "stdint.h"
#include "stddef.h"
#include "stdbool.h"


// Return codes
typedef enum
{
    RC76xx_OK = 0,     
    RC76xx_ERR_INVALID_ARG,
    RC76xx_ERR_NOT_INITIALIZED,
    RC76xx_ERR_ALREADY_INITIALIZED,
    RC76xx_ERR_TIMEOUT,
    RC76xx_ERR_COMMUNICATION,
    RC76xx_ERR_BUSY,
    RC76xx_ERR_NOT_CONNECTED,
    RC76xx_ERR_RESOURCE,
    RC76xx_ERR_INVALID_STATE,
    RC76xx_ERR_MODEM
} RC76xx_Result_t;

// State machine states
typedef enum
{
    RC76xx_STATE_UNINITIALIZED = 0,  // Driver not initialized 
    RC76xx_STATE_INITIALIZED,        // Driver initialized but not connected
    RC76xx_STATE_MODEM_READY,        // Modem responded to basic AT commands
    RC76xx_STATE_NETWORK_CONFIGURED, // Setting up network parameters
    RC76xx_STATE_NETWORK_CONNECTING, // Attempting to connect to cellular network
    RC76xx_STATE_NETWORK_CONNECTED,  // Connected to cellular network
    RC76xx_STATE_MQTT_CONFIGURED,    // Configuring MQTT client
    RC76xx_STATE_MQTT_CONNECTING,    // Connecting to MQTT broker, waiting for URC +KMQTT_IND: 1,1
    RC76xx_STATE_MQTT_CONNECTED,     // Connected to MQTT broker
    RC76xx_STATE_MQTT_SUBSCRIBING,   // Subscribing to MQTT topics
    RC76xx_STATE_MQTT_SUBSCRIBED,    // Subscribed to MQTT topics
    RC76xx_STATE_MQTT_PUBLISHING,    // Publishing a message
    RC76xx_STATE_MQTT_DISCONNECTING, // Disconnecting from MQTT broker
    RC76xx_STATE_ERROR               // Error state with recovery mechanism
} RC76xx_State_t;

// Event types
typedef enum
{
    RC76xx_EVENT_STATE_CHANGED = 0,
    RC76xx_EVENT_NETWORK_REGISTERED,
    RC76xx_EVENT_NETWORK_LOST,
    RC76xx_EVENT_MQTT_CONNECTED,
    RC76xx_EVENT_MQTT_DISCONNECTED,
    RC76xx_EVENT_MQTT_MESSAGE_RECEIVED,
    RC76xx_EVENT_MQTT_PUBLISHED,
    RC76xx_EVENT_ERROR
} RC76xx_EventType_t;

// MQTT status codes from +KMQTT_IND
typedef enum {
    RC76xx_MQTT_STATUS_CONNECTION_ABORTED = 0,
    RC76xx_MQTT_STATUS_CONNECTED = 1,
    RC76xx_MQTT_STATUS_SUBSCRIBED = 2,
    RC76xx_MQTT_STATUS_UNSUBSCRIBED = 3,
    RC76xx_MQTT_STATUS_PUBLISHED = 4,
    RC76xx_MQTT_STATUS_GENERIC_ERROR = 5
} RC76xx_MQTT_Status_t;

// Event structure
typedef struct {
    RC76xx_EventType_t type;
    union {
        struct {
            RC76xx_State_t oldState;
            RC76xx_State_t newState;
        } stateChange;
        
        struct {
            char topic[64];
            char message[256];
            uint8_t qos;
        } mqttMessage;
        
        struct {
            RC76xx_Result_t code;
            char message[64];
        } error;
    } data;
} RC76xx_Event_t;

// Event callback type
typedef void (*RC76xx_EventCallback_t)(const RC76xx_Event_t *event, void *userData);

// State machine context - essential for operation
typedef struct {
    RC76xx_State_t currentState;
    RC76xx_State_t previousState;
    uint32_t stateEntryTime;
    uint32_t stateTimeout;
    bool errorRecoveryMode;
    uint8_t errorRetryCount;
    RC76xx_Result_t lastError;
} RC76xx_StateMachine_t;

// Network configuration - minimal required fields
typedef struct {
    char *apn;               // Access Point Name
    char *username;          // APN username (can be NULL)
    char *password;          // APN password (can be NULL)
    char *ip_address;        // IP address (for status)
    char *port;              // Port (for status)
    uint8_t pdpContextId;    // PDP context ID (1-16)
    char *ipType;            // "IP" or "IPV6" or "IPV4V6"
    
} RC76xx_Network_Config_t;

// SSL/TLS configuration
typedef struct {
    uint8_t securityMethod;  // 0=No security, 1=TLS
    uint8_t cipherIndex;     // Cipher suite profile index
    uint8_t authMode;        // Authentication mode
    uint16_t negotiationBitmask; // TLS version negotiation bitmask
    uint8_t encryptAlgoBitmask;  // Encryption algorithm bitmask
    uint8_t certValidLevel;      // Certificate validation level
    uint8_t caCertIndex;         // CA certificate index
    uint8_t clientCertIndex;     // Client certificate index
    char *alpnList;              // ALPN protocol list
} RC76xx_SSL_Config_t;

// MQTT configuration - minimal required fields
typedef struct {
    char *server;            // MQTT broker server
    char *port;              // MQTT broker port
    char *client_id;         // MQTT client ID
    char *username;          // MQTT username
    char *password;          // MQTT password
    char *topic;             // Default topic
    char *message;           // Default message
    uint16_t qos;            // Quality of Service
    uint16_t keepalive;      // Keepalive interval in seconds
    uint16_t timeout;        // Connection timeout
    uint16_t retry_count;    // Number of retries
    uint16_t retry_delay;    // Delay between retries
    
    uint8_t sessionId;       // MQTT session ID (assigned by driver)
    uint8_t mqttVersion;     // 3=MQTT 3.1, 4=MQTT 3.1.1
    bool cleanSession;       // Clean session flag
    bool useTls;             // Whether to use TLS
    
    // Will message configuration
    bool willFlag;           // Enable Last Will and Testament
    char *willTopic;         // Will topic
    char *willMessage;       // Will message
    bool willRetained;       // Will retained flag
    uint8_t willQos;         // Will QoS
    
    // SSL/TLS configuration
    uint8_t securityMethod;  // 0=No security, 1=TLS
    uint8_t cipherIndex;     // Cipher suite profile index
    char *alpnList;          // ALPN protocol list
    
} RC76xx_MQTT_Config_t;

// Main driver handle
typedef struct {
    RC76xx_StateMachine_t stateMachine;
    RC76xx_MQTT_Config_t mqttConfig;
    RC76xx_Network_Config_t networkConfig;
    
    // Event callback
    void (*eventCallback)(RC76xx_EventType_t event, void *data, void *userData);
    void *userData;
    
    // Internal state
    bool networkRegistered;
    int8_t signalQuality;
    char lastErrorMessage[64];
    
    // UART handle
    void *uartHandle;
} RC76xx_Handle_t;


// Core API functions - essential functions only
RC76xx_Result_t RC76xx_Check_device_ready(RC76xx_Handle_t *handle);
RC76xx_Result_t RC76xx_Initialize(RC76xx_Handle_t *handle);
RC76xx_Result_t RC76xx_Deinitialize(RC76xx_Handle_t *handle);
RC76xx_Result_t RC76xx_SetNetworkConfig(RC76xx_Handle_t *handle, RC76xx_Network_Config_t *networkConfig);
RC76xx_Result_t RC76xx_SetMQTTConfig(RC76xx_Handle_t *handle, RC76xx_MQTT_Config_t *mqttConfig);
RC76xx_Result_t RC76xx_Connect(RC76xx_Handle_t *handle);
RC76xx_Result_t RC76xx_Disconnect(RC76xx_Handle_t *handle);
RC76xx_Result_t RC76xx_Subscribe(RC76xx_Handle_t *handle, const char *topic, uint8_t qos);
RC76xx_Result_t RC76xx_Unsubscribe(RC76xx_Handle_t *handle, const char *topic);
RC76xx_Result_t RC76xx_Publish(RC76xx_Handle_t *handle, const char *topic, const char *message, uint8_t qos);
RC76xx_Result_t RC76xx_GetState(RC76xx_Handle_t *handle, RC76xx_State_t *state);

// State machine function
RC76xx_Result_t RC76xx_SetState(RC76xx_Handle_t *handle, RC76xx_State_t newState, uint32_t timeout);

/* These functions should be internal (static) in the implementation file, not in the header */
// void RC76xx_ExecuteStateEntryActions(RC76xx_Handle_t *handle, RC76xx_State_t state);
// void RC76xx_StateMonitorTask(void *pvParameters);
// void RC76xx_HandleStateTimeout(RC76xx_Handle_t *handle, RC76xx_State_t state);
// void RC76xx_ExecuteStateActions(RC76xx_Handle_t *handle, RC76xx_State_t state);
// void RC76xx_SetError(RC76xx_Handle_t *handle, RC76xx_Result_t errorCode, const char *errorMessage);

#endif // __RC76XX_MQTT_H__
