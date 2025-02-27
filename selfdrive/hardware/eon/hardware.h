#pragma once

#include <cstdlib>
#include <fstream>
#include <cstdio>

#include <gui/ISurfaceComposer.h>
#include <gui/SurfaceComposerClient.h>
#include <hardware/hwcomposer_defs.h>

#include "selfdrive/common/util.h"
#include "selfdrive/hardware/base.h"

class HardwareEon : public HardwareNone {
public:
  static constexpr float MAX_VOLUME = 0.85;
  static constexpr float MIN_VOLUME = 0.45;

  static bool EON() { return true; }
  static std::string get_os_version() {
    return "NEOS " + util::read_file("/VERSION");
  };

  static void reboot() { std::system("reboot"); };
  static void poweroff() { std::system("LD_LIBRARY_PATH= svc power shutdown"); };
  static void set_brightness(int percent) {
    std::ofstream brightness_control("/sys/class/leds/lcd-backlight/brightness");
    if (brightness_control.is_open()) {
      if (percent > 50 ) {
        percent *= 0.75;
      }
      brightness_control << (int)(percent * (255/100.)) << "\n";
      brightness_control.close();
    }
  };
  static void set_display_power(bool on) {
    auto dtoken = android::SurfaceComposerClient::getBuiltInDisplay(android::ISurfaceComposer::eDisplayIdMain);
    android::SurfaceComposerClient::setDisplayPowerMode(dtoken, on ? HWC_POWER_MODE_NORMAL : HWC_POWER_MODE_OFF);
  };

  static bool get_ssh_enabled() {
    return std::system("getprop persist.neos.ssh | grep -qF '1'") == 0;
  };
  static void set_ssh_enabled(bool enabled) {
    std::string cmd = util::string_format("setprop persist.neos.ssh %d", enabled ? 1 : 0);
    std::system(cmd.c_str());
  };

  // android only
  inline static bool launched_activity = false;
  static void check_activity() {
    int ret = std::system("dumpsys SurfaceFlinger --list | grep -Fq 'com.android.settings'");
    launched_activity = ret == 0;
  }

  static void close_activities() {
    if(launched_activity) {
      std::system("pm disable com.android.settings && pm enable com.android.settings");
    }
  }

  static void launch_activity(std::string activity, std::string opts = "") {
    if (!launched_activity) {
      std::string cmd = "am start -n " + activity + " " + opts +
                        " --ez extra_prefs_show_button_bar true \
                         --es extra_prefs_set_next_text ''";
      std::system(cmd.c_str());
    }
    launched_activity = true;
  }
  static void launch_wifi() {
    launch_activity("com.android.settings/.wifi.WifiPickerActivity", "-a android.net.wifi.PICK_WIFI_NETWORK");
  }
  static void launch_tethering() {
    launch_activity("com.android.settings/.TetherSettings");
  }

  // added jc01rho
  static void touch_prebuilt() {
    std::ofstream output("/data/openpilot/prebuilt"); //touch prebuilt
  }
  static void rm_prebuilt() {
    std::remove("/data/openpilot/prebuilt"); //rm prebuilt
  }
  static void git_clean_reset() {
    std::system("/system/bin/su -c LD_LIBRARY_PATH=/data/phonelibs:/data/data/com.termux/files/usr/lib data/data/com.termux/files/usr/bin/git -C /data/openpilot reset --hard");
    std::system("/system/bin/su -c LD_LIBRARY_PATH=/data/phonelibs:/data/data/com.termux/files/usr/lib data/data/com.termux/files/usr/bin/git -C /data/openpilot clean -xfd");
  }
  static void clean_build_cache() {
    std::system("/system/bin/su -c rm -rf /data/build_cache");
  }
  static void update_reboot() {

    rm_prebuilt();
    reboot();
  }
  static void clean_build_reboot() {
    git_clean_reset();
    clean_build_cache();
    update_reboot();

  }
};
