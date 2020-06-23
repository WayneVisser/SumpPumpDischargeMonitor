#include <Arduino.h>

#include "version.h"

// ***********************************************************************
String  getVersionInfo(String sHeader) {
  static String sStatus;

  sStatus = sHeader
    + "ESP8266 Info\n"
    + "  CPU Speed: "
    + String(ESP.getCpuFreqMHz())
    + " MHz\n"
    + "  Core Version: "
    + ESP.getCoreVersion()
    + "\n"
    + "  SDK Version: "
    + String(ESP.getSdkVersion())
    + "\n"
    + "  ESP8266 Chip ID: "
    + "0x" + String(ESP.getChipId(), HEX)
    + "\n"
    + "  Flash Chip ID: "
    + "0x" + String(ESP.getFlashChipId(), HEX)
    + "\n"
    + "  Flash Size: "
    + String(ESP.getFlashChipSize() / 1048576.0, 1) + " MBytes"
    + "\n"
    + "  Flash Speed: "
    + String(ESP.getFlashChipSpeed() / 1000000.0, 2) + " MHz"
    + "\n"
    + "  Sketch Size: "
    + String(ESP.getSketchSize()) + " bytes"
    + "\n"
    + "  Sketch MD5: "
    + String(ESP.getSketchMD5())
    + "\n\n";

  return (sStatus);
}
