#include <FS.h>
#include "debug.h"
#include "config.h"

// ***********************************************************************
#define CFG_FILE        "/config.dat"

// ***********************************************************************
bool            bSaveConfig = false;
cfg_data_t      cfg_data;

// ***********************************************************************
bool configLoad() {
  bool bStatus = false;

  if ( SPIFFS.begin() ) {
    if (SPIFFS.exists(CFG_FILE)) {
      //file exists, reading and loading
      Sprintln("Config READ");
      Sprintln("  opening file: " + String(CFG_FILE));
      File configFile = SPIFFS.open(CFG_FILE, "r");
      if (configFile) {
        Sprintln("  reading file");
        if ( configFile.size() == sizeof(cfg_data_t)) {
          Sprintln("  file size = OK");
          configFile.readBytes((char *)&cfg_data, sizeof(cfg_data_t));
          bStatus= true;
        } else {
          Sprintln("  file size error");
        }
        configFile.close();
      }
    }
  }
  return (bStatus);
}

// ***********************************************************************
void  configSave() {
  // open config file for writing.  make it if it does not exist
  Sprintln("Config WRITE");
  File configFile = SPIFFS.open(CFG_FILE, "w");
  if (configFile) {
    Sprintln("  opened config file for writing: " + String(CFG_FILE));
    configFile.write((char *)&cfg_data, sizeof(cfg_data_t));
    configFile.close();
  }
}
