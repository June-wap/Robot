#ifndef NETWORK_CONFIG_H
#define NETWORK_CONFIG_H

// Tùy chọn nạp thông tin mạng nội bộ từ Secrets.h (bị bỏ qua bởi .gitignore)
// để có thể ghi đè credentials cục bộ mà không sửa mã nguồn chia sẻ.
#if defined(__has_include)
#if __has_include("Secrets.h")
#include "Secrets.h"
#endif
#endif

// ======================= WIFI & OTA =======================
#ifndef WIFI_SSID
#define WIFI_SSID  "Ngoc Tram_2G"
#endif

#ifndef WIFI_PASS
#define WIFI_PASS  "trungkien"
#endif

#ifndef AP_SSID
#define AP_SSID    "RobotLauNha"
#endif

#ifndef AP_PASS
#define AP_PASS    "88888888"
#endif

#ifndef MDNS_HOSTNAME
#define MDNS_HOSTNAME "robot"
#endif

#ifndef OTA_HOSTNAME
#define OTA_HOSTNAME  "robot"
#endif

#ifndef OTA_PASSWORD
#define OTA_PASSWORD  "robot1234"
#endif

#endif // NETWORK_CONFIG_H
