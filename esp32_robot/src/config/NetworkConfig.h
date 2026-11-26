#ifndef NETWORK_CONFIG_H
#define NETWORK_CONFIG_H

// Nạp thông tin mạng / OTA thực tế từ Secrets.h (file cục bộ, bị loại trừ bởi .gitignore).
// Sao chép Secrets.h.example -> Secrets.h và điền thông tin thực tế trước khi nạp firmware.
#if defined(__has_include)
#if __has_include("Secrets.h")
#include "Secrets.h"
#endif
#endif

// ======================= WIFI & OTA (Placeholder mặc định khi chưa có Secrets.h) =======================
#ifndef WIFI_SSID
#define WIFI_SSID     "CHANGE_ME_WIFI_SSID"
#endif

#ifndef WIFI_PASS
#define WIFI_PASS     "CHANGE_ME_WIFI_PASS"
#endif

#ifndef AP_SSID
#define AP_SSID       "RobotLauNha"
#endif

#ifndef AP_PASS
#define AP_PASS       "CHANGE_ME_AP_PASS"
#endif

#ifndef MDNS_HOSTNAME
#define MDNS_HOSTNAME "robot"
#endif

#ifndef OTA_HOSTNAME
#define OTA_HOSTNAME  "robot"
#endif

#ifndef OTA_PASSWORD
#define OTA_PASSWORD  "CHANGE_ME_OTA_PASS"
#endif

#endif // NETWORK_CONFIG_H
