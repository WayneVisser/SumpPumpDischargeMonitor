/* ***********************************************************************
 *
 * Sump Pump Discharge Monitor
 *
 * May 2019
 * 
 * Version History
 * 
 * 1.0  2019-05-01    Initial version for ESP-02.  Didn't work - not able
 *                    to reliably use GPIO as input on ESP-02.
 * 1.1  2019-11-15    Switch to WeMos D1 Mini Pro.
 * 1.2  2020-06-21    Use WifiManager library to manage WiFi and MQTT 
 *                    parameters
 * 1.3  2020-06-22    Use NTP for clock sync
 * 1.4  2020-06-22    Use SPIFFS for config
 * 1.5  2020-06-22    Fix start time publish to MQTT broker
 * 1.6  2020-06-22    remove start time as a separate MQTT publish
 * 1.61 2020-06-23    fix startTime initialization
 * 
 * Hardware required: ESP8266 with Arduino platform.  1.5" flow rate sensor
 *
 * Flow rate sensor connected to ESP-8266 GPIO input.  Flow rate sensor
 * specs:
 *
 *    Sensor: YF-DN40
 *    Freq. = 0.45 * Q where Q=L/min
 *    27.0 pulses/litre
 *    Flow Range = 5-150L/min
 * 
 *    Product item no. :YF-DN40
 *    Material : Plastic
 *    Specification : 
 *      Function : Sensor, flow rate
 *      Thread size :  BSP 1 1/2"
 *      Size:L 92x OD46  X1.5"
 *      Color : Black  color 
 *      Flow rate : 5~150L/min
 *      Flow Pulse:  F(Hz)=(0.45xQ) +/-3%    Q=L/min
 *      Max. Working Current : 15mA (DC5 V)
 *      Min. Working Voltage: DC 4.5V
 *      Working Voltage:DC 5V~18V
 *      Load Capacity: =10 mA (DC 5 V)
 *      Operating  Working  Temperature : 
 *      -25 Degree centigrade to +80 Degree centigrade
 *      Liquid Temperature: 85
 *      Accuracy : 5% 
 *
 *
 *  License
 *    The MIT License (MIT)
 *
 *    Copyright (c) 2017 Wayne Visser
 *
 *    Permission is hereby granted, free of charge, to any person obtaining a
 *    copy of this software and associated documentation files (the
 *    "Software"), to deal in the Software without restriction, including
 *    without limitation the rights to use, copy, modify, merge, publish,
 *    distribute, sublicense, and/or sell copies of the Software, and to
 *    permit persons to whom the Software is furnished to do so, subject to
 *    the following conditions:
 *
 *    The above copyright notice and this permission notice shall be included
 *    in all copies or substantial portions of the Software.
 *
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 *    OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 *    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 *    CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 *    TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 *    SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * **********************************************************************/

#include <Arduino.h>
#include <Time.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>    
#include <PubSubClient.h>
#include <TimeAlarms.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include "debug.h"
#include "config.h"
#include "version.h"

// *********************************************************************
// INCREASE mqtt PACKET SIZE (Default is 128 bytes
#undef  MQTT_MAX_PACKET_SIZE        // un-define max packet size
#define MQTT_MAX_PACKET_SIZE 256    // fix for MQTT client dropping messages over 128B


// ***********************************************************************
#define VERSION               1.61
#define VERSION_HEADER        "\n\nSump Pump Discharge Monitor - V" \
                                + String(VERSION, 2) \
                                + "\n\n"
  
#define PIN_FLOW              D4
#define PIN_POWER_INPUT       A0

// internal AP info
#define AP_SSID               "ESP_AP"
#define AP_PASSWORD           "sooperSecr3t"

// NTP Server
#define NTP_SERVER            "0.ca.pool.ntp.org"

// internal webserver port
#define WEBSERVER_PORT        8080

// mqtt topics
#define MQTT_TOPIC_STATUS     "sump/status"
//#define mqtt_TOPIC_START      "sump/start_time"
//#define mqtt_TOPIC_POWER      "sump/power"

// secs - how often to sync the RTC
#define RTC_UPDATE_INTERVAL   (12 * SECS_PER_HOUR)

// sample rates for external sensors.  Although not ideal, a number of
// processes depend on the sensor sample rate being 1 minute.  Be careful
// if this is changed.
#define FLOW_SAMPLE_RATE      60       // seconds

// ***********************************************************************

volatile  unsigned long       iFlowPulse;
//volatile  unsigned long       iLastFlowPulse;
volatile  unsigned long       dtLastPulse;

WiFiManagerParameter          custom_mqtt_server("server", "mqtt server", cfg_data.server, MQTT_SERVER_LEN);
WiFiManagerParameter          custom_mqtt_port("port", "mqtt port", cfg_data.port, MQTT_PORT_LEN);
WiFiManagerParameter          custom_mqtt_username("username", "mqtt username", cfg_data.username, MQTT_USERNAME_LEN);
WiFiManagerParameter          custom_mqtt_password("password", "mqtt password", cfg_data.password, MQTT_PASSWORD_LEN);

WiFiClient                    espClient;

PubSubClient                  mqttClient(espClient);

WiFiUDP                       ntpUDP;
NTPClient                     timeClient(ntpUDP, NTP_SERVER);

// real time clock
time_t                        lastClockSyncTime = 0;      // time when clock last sync'd
bool                          acqSync;

// trigger alarms
bool                          trigSample;

// when monitor was started
time_t                        startTime = 0;

// alarm ID's
AlarmID_t                     flowAlarmID;

ESP8266WebServer              webServer(WEBSERVER_PORT);

// *********************************************************************
// sync RTC from a NTP Server
//
static time_t syncClockFromNTP() {
  time_t  t = 0;

  if ( timeClient.update() ) {
    setSyncInterval(RTC_UPDATE_INTERVAL);
    lastClockSyncTime = now();
    acqSync = false;
    Sprintln("RTC acquired");    
    t = timeClient.getEpochTime();
    // record time of first sync.
    if ( startTime == 0 ) 
      startTime = t;
    Sprintln(t);
  } else {
    // try again sooner
    setSyncInterval(60);    
  }
  return t;
}

// *********************************************************************
// sync clock with a known source.
//
static time_t syncClock() {
  // return syncClockFromRTC();
  // return syncClockFromWeb();
   return syncClockFromNTP();
}

// *********************************************************************
// makeTimeString
//
#ifdef DEBUG
static String makeTimeString(time_t t) {
  String s;
  TimeElements tm;

  breakTime(t, tm);
  s = String(tm.Hour) + ":";
  if (tm.Minute < 10) s = s + "0";
  s = s + String(tm.Minute) + ":";
  if (tm.Second < 10) s = s + "0";
  s = s + String(tm.Second);
  s = s + " -- "
    + String(tm.Day) + "/"
    + String(tm.Month) + "/"
    + String(tm.Year + 1970);

  return s;
}
#endif

// *********************************************************************
void handleRoot(){
  String sText;

  sText = getVersionInfo(VERSION_HEADER);
  sText += "\n";
  webServer.send(200, "text/plain", sText);
}

// *********************************************************************
void handleResetWiFi(){
  webServer.send(200, "text/plain","Reset and reboot in 5 secs...");
  delay(5000);
  /*for (int i=0; i<50; ++i ) {
    webServer.handleClient();
    delay(100);
  }
  */
  WiFiManager wifiManager;
  wifiManager.resetSettings();

  ESP.reset();
}

// ***********************************************************************
void ICACHE_RAM_ATTR isrFlow()
{
  unsigned long dtNow;

  dtNow = millis();
  if ( (dtNow - dtLastPulse) > 1 ) {
    dtLastPulse = dtNow;
    ++iFlowPulse;
  }
}

// ***********************************************************************
static void mqttConnect()
{
  while ( !mqttClient.connected() ) 
  {
    Sprintln(" Connecting to mqtt broker...");
    if ( mqttClient.connect("ESP8266Client", cfg_data.username, cfg_data.password) )
      Sprintln("  Connected!");
    else {
      Sprint("  failed with state ");
      Sprintln(mqttClient.state());
      delay(2000);
    }
  }
}

// *********************************************************************
// tickFlowSample
//
static void tickFlowSample() {
  trigSample = true;
}

// ***********************************************************************
// doSample
//
// sample sensors (flow and power) and send to MQTT broker.
//
static void doSample()
{
  String sCount;
  String sFlowRate;
  String sVolume;
  float fLitrePerMin;
  float fVolume;
  float fVin;
  unsigned long iCurFlowPulse;
  StaticJsonDocument<512> doc;
  char buf [512];

  // time to sample sensors?
  if ( trigSample ) {
    trigSample = false;
    Sprintln("\nSampling flow sensor...");

    // capture amount of flow since last sample.
    noInterrupts();
    iCurFlowPulse = iFlowPulse;
    // reset pulse counter
    iFlowPulse = 0;
    interrupts();

    fLitrePerMin = iCurFlowPulse / 27.0 * 60;

//    fVolume = iFlowPulse / 27.0;
    fVolume = iCurFlowPulse / 27.0;

    sCount = String(iCurFlowPulse);
    sFlowRate = String(fLitrePerMin);
    sVolume = String(fVolume);

    Sprintln("\nSampling power level...");
    // sample power status
    // 10-bit A/D. input voltage is monitored through a resistor divider.  520/100
    // therefore at 5v Vin, max input voltage on A0 is 0.96V.  That translates to an 
    // A/D value of about 984.
    fVin = map(analogRead(PIN_POWER_INPUT), 0, 984, 0, 5000) / 1000.0;

    if ( !mqttClient.connected() )
      mqttConnect();

    if ( mqttClient.connected() ) {
      // push sensor values
      doc.clear();
      doc["time"] = now();
      doc["count"] = iCurFlowPulse;
      doc["flow"] = fLitrePerMin;
      doc["volume"] = fVolume;
      doc["power"] = fVin;
      doc["start"] = startTime;
      size_t n = serializeJson(doc, buf, 511);

      Sprintln(" Publishing data to MQTT Broker");
      Sprintln(" Payload: " + String(buf));
      mqttClient.publish(MQTT_TOPIC_STATUS, buf, n);
    }
  }  
}

// ***********************************************************************
void doAcqSync()
{
  if ( timeStatus() == timeSet ) {
    if ( !acqSync ) {
      if ( second(now()) == 0 ) {
        // (re-)align alarms with RTC.
        Alarm.free(flowAlarmID);
        Sprintln("Creating flowAlarm");
        flowAlarmID = Alarm.timerRepeat(FLOW_SAMPLE_RATE, tickFlowSample);

        acqSync = true;
        Sprintln("Acquisition sync'd at: " + String(makeTimeString(now())));        
      }
    }
  }
}

// ***********************************************************************
void saveConfigCallback() {
  Sprintln("Saving MQTT parameters...");
  strcpy(cfg_data.server, custom_mqtt_server.getValue());
  strcpy(cfg_data.port, custom_mqtt_port.getValue());
  strcpy(cfg_data.username, custom_mqtt_username.getValue());
  strcpy(cfg_data.password, custom_mqtt_password.getValue());

  configSave();
}

// ***********************************************************************
void setup() {
  String sInfo;

  Serial.begin(115200);
  delay(1000);

  Sprint(getVersionInfo(VERSION_HEADER));

  WiFiManager wifiManager;
#ifndef DEBUG
  wifiManager.setDebugOutput(false);
#endif
  // if we cannot load MQTT stuff, start all over.
  if (!configLoad() )
    wifiManager.resetSettings();
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_username);
  wifiManager.addParameter(&custom_mqtt_password);
  wifiManager.setTimeout(600);

  // block until a connection is made or parameters entered, or else reboot.
  if ( !wifiManager.autoConnect(AP_SSID, AP_PASSWORD)) {
    Sprintln("Not able to make WiFi connection...Rebooting.");
    delay(3000);
    ESP.reset();
  }
  
  // Print the IP address
  Sprintln("");
  Sprint("WiFi connected with IP: ");
  Sprintln(WiFi.localIP());
  
  // NTP Client
  timeClient.begin();

  setSyncProvider( syncClock );
  // we can't do anything without time, so set the sync interval quickly at first
  // in case there are problems and back off later, once the initial sync is made.
  setSyncInterval( 60 );

  // load configuration
  configLoad();
  Sprint("MQTT Broker: ");
  Sprintln(cfg_data.server);
  Sprint("MQTT Port: ");
  Sprintln(cfg_data.port);

  // establish MQTT client.
  mqttClient.setServer(cfg_data.server, atoi(cfg_data.port));

  // internal webserver
  webServer.on("/", handleRoot); 
  webServer.on("/resetWiFi", handleResetWiFi); 
  webServer.begin(WEBSERVER_PORT);

  // prepare ISR from flow meter
  pinMode(PIN_FLOW, INPUT);
  attachInterrupt (digitalPinToInterrupt(PIN_FLOW), isrFlow, RISING);
}

// ***********************************************************************
void loop()
{
  doAcqSync();
  doSample();
  mqttClient.loop();
  webServer.handleClient();
  Alarm.delay(0);
}

