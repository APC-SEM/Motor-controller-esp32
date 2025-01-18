#include <Preferences.h>
Preferences preferences;

void read_memory() {
  // retrieve saved data
  preferences.begin("ThrottleData", false);
  max_throttle = preferences.getUInt("max_throttle", 0);
  min_throttle = preferences.getUInt("min_throttle", 0);
  preferences.end();
}

void store_memory(){
  // store data
  preferences.begin("ThrottleData", false);
  preferences.putUInt("max_throttle", max_throttle-50); // offset by 50, so it is easier to reach max throttle
  preferences.putUInt("min_throttle", min_throttle);
  preferences.end();
}
