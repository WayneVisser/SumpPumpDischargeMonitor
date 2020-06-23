// ***********************************************************************
// put stuff in config_t to be managed by the WiFiManager

// ***********************************************************************
#define MQTT_SERVER_LEN       40
#define MQTT_PORT_LEN         6
#define MQTT_USERNAME_LEN     32
#define MQTT_PASSWORD_LEN     32

// ***********************************************************************
typedef struct {
  char server[MQTT_SERVER_LEN];
  char port[MQTT_PORT_LEN];
  char username[MQTT_USERNAME_LEN];
  char password[MQTT_PASSWORD_LEN];
} cfg_data_t;


// ***********************************************************************
extern  cfg_data_t     cfg_data;

// ***********************************************************************
extern  bool          configLoad();
extern  void          configSave();



