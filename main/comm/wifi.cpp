// Wi-Fi 与 UDP 实现
//
// 移植说明（ESP-IDF）：
//   * Arduino WiFi 类 -> esp_wifi + esp_netif + esp_event
//   * WiFi.setSleep(false)      -> esp_wifi_set_ps(WIFI_PS_NONE)
//   * WiFi.softAPgetStationNum()-> WIFI_EVENT_AP_STACONNECTED/DISCONNECTED 计数
//   * WiFiUDP                   -> 非阻塞 BSD socket
//   * 凭据沿用原 NVS 键名 WIFI_AP_SSID / WIFI_AP_PASS / WIFI_STA_SSID / WIFI_STA_PASS

#include "comm/wifi.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"

#include "app/cli.h"
#include "app/globals.h"
#include "app/parameters.h"
#include "hal/board.h"

static const char* TAG = "wifi";

// WiFi 模式常量（与原 wifi.ino 保持一致：0=关闭, 1=AP, 2=STA）
// 配置变量 wifiMode / udpLocalPort / udpRemotePort 定义于 app/comm_config.cpp

static constexpr int W_DISABLED = 0;
static constexpr int W_AP = 1;
static constexpr int W_STA = 2;

static esp_netif_t* s_netif = nullptr;
static int s_udpSock = -1;
static struct sockaddr_in s_remoteAddr = {};
static volatile int s_staCount = 0;
static volatile bool s_staConnected = false;

bool wifiIsApMode() {
	return wifiMode == W_AP;
}

static void udpOpen() {
	if (s_udpSock >= 0) return;

	s_udpSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (s_udpSock < 0) {
		ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
		return;
	}

	struct sockaddr_in local = {};
	local.sin_family = AF_INET;
	local.sin_port = htons((uint16_t)udpLocalPort);
	local.sin_addr.s_addr = htonl(INADDR_ANY);
	if (bind(s_udpSock, (struct sockaddr*)&local, sizeof(local)) < 0) {
		ESP_LOGE(TAG, "bind(%d) failed: errno=%d", udpLocalPort, errno);
		close(s_udpSock);
		s_udpSock = -1;
		return;
	}

	// 必须非阻塞：否则飞控主循环会被 recvfrom 卡住
	int flags = fcntl(s_udpSock, F_GETFL, 0);
	fcntl(s_udpSock, F_SETFL, flags | O_NONBLOCK);

	// 默认远端：广播地址（与原 udpRemoteIP 初值 255.255.255.255 一致）
	s_remoteAddr.sin_family = AF_INET;
	s_remoteAddr.sin_port = htons((uint16_t)udpRemotePort);
	s_remoteAddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);

	int broadcast = 1;
	setsockopt(s_udpSock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

	ESP_LOGI(TAG, "UDP listening on port %d", udpLocalPort);
}

static void wifiEventHandler(void* arg, esp_event_base_t base, int32_t id, void* data) {
	(void)arg;

	if (base == WIFI_EVENT) {
		switch (id) {
			case WIFI_EVENT_AP_STACONNECTED: {
				int n = s_staCount + 1;
				s_staCount = n;
				ESP_LOGI(TAG, "station joined, count=%d", n);
				break;
			}
			case WIFI_EVENT_AP_STADISCONNECTED: {
				int n = (s_staCount > 0) ? (s_staCount - 1) : 0;
				s_staCount = n;
				ESP_LOGI(TAG, "station left, count=%d", n);
				break;
			}
			case WIFI_EVENT_STA_START:
				esp_wifi_connect();
				break;
			case WIFI_EVENT_STA_CONNECTED:
				s_staConnected = true;
				break;
			case WIFI_EVENT_STA_DISCONNECTED:
				s_staConnected = false;
				ESP_LOGW(TAG, "STA disconnected, retrying");
				esp_wifi_connect();
				break;
			default:
				break;
		}
	} else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t* event = (ip_event_got_ip_t*)data;
		ESP_LOGI(TAG, "got IP: " IPSTR, IP2STR(&event->ip_info.ip));
	}
}

void setupWiFi() {
	print("Setup Wi-Fi\n");

	if (wifiMode == W_DISABLED) {
		print("WiFi 已禁用 (WIFI_MODE=0)\n");
		return;
	}

	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	wifi_init_config_t initCfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&initCfg));

	ESP_ERROR_CHECK(esp_event_handler_instance_register(
		WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandler, nullptr, nullptr));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(
		IP_EVENT, IP_EVENT_STA_GOT_IP, &wifiEventHandler, nullptr, nullptr));

	wifi_config_t cfg = {};

	if (wifiMode == W_AP) {
		s_netif = esp_netif_create_default_wifi_ap();

		const char* ssid = storage.getString("WIFI_AP_SSID", "阿瓦达");
		const char* pass = storage.getString("WIFI_AP_PASS", "12345678");
		strlcpy((char*)cfg.ap.ssid, ssid, sizeof(cfg.ap.ssid));
		cfg.ap.ssid_len = strlen(ssid);
		strlcpy((char*)cfg.ap.password, pass, sizeof(cfg.ap.password));
		cfg.ap.channel = 1;
		cfg.ap.max_connection = 4;
		cfg.ap.authmode = (strlen(pass) >= 8) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
		ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &cfg));
		ESP_LOGI(TAG, "AP mode: SSID='%s'", ssid);
	} else {
		s_netif = esp_netif_create_default_wifi_sta();

		const char* ssid = storage.getString("WIFI_STA_SSID", "");
		const char* pass = storage.getString("WIFI_STA_PASS", "");
		strlcpy((char*)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid));
		strlcpy((char*)cfg.sta.password, pass, sizeof(cfg.sta.password));
		cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;

		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
		ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
		ESP_LOGI(TAG, "STA mode: SSID='%s'", ssid);
	}

	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // 关闭省电（等价 WiFi.setSleep(false)）
	ESP_ERROR_CHECK(esp_wifi_start());

	udpOpen();
}

void sendWiFi(const uint8_t* buf, int len) {
	if (s_udpSock < 0 || buf == nullptr || len <= 0) return;
	// 与原实现一致：无任何客户端连接时不下发
	if (s_staCount == 0 && !s_staConnected) return;

	sendto(s_udpSock, buf, (size_t)len, 0,
	       (struct sockaddr*)&s_remoteAddr, sizeof(s_remoteAddr));
}

int receiveWiFi(uint8_t* buf, int len) {
	if (s_udpSock < 0 || buf == nullptr || len <= 0) return 0;

	struct sockaddr_in from = {};
	socklen_t fromLen = sizeof(from);
	int n = (int)recvfrom(s_udpSock, buf, (size_t)len, 0,
	                      (struct sockaddr*)&from, &fromLen);
	if (n <= 0) return 0;

	// 记住对端地址，后续遥测发回该地址（等价原 udp.remoteIP() 逻辑）
	if (from.sin_addr.s_addr != 0) {
		s_remoteAddr = from;
		s_remoteAddr.sin_port = htons((uint16_t)udpRemotePort);
	}
	return n;
}

void printWiFiInfo() {
	print("Mode: %s\n", wifiMode == W_AP ? "Access Point (AP)" : "Client (STA)");

	uint8_t mac[6] = {};
	char ipStr[16] = "0.0.0.0";

	if (wifiMode == W_AP) {
		esp_wifi_get_mac(WIFI_IF_AP, mac);
		esp_netif_ip_info_t ip = {};
		if (s_netif != nullptr && esp_netif_get_ip_info(s_netif, &ip) == ESP_OK) {
			snprintf(ipStr, sizeof(ipStr), IPSTR, IP2STR(&ip.ip));
		}
		print("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
		      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		print("SSID: %s\n", storage.getString("WIFI_AP_SSID", "Drone_WiFi"));
		print("Password: ***\n");
		print("Clients: %d\n", s_staCount);
		print("IP: %s\n", ipStr);
	} else {
		esp_wifi_get_mac(WIFI_IF_STA, mac);
		esp_netif_ip_info_t ip = {};
		if (s_netif != nullptr && esp_netif_get_ip_info(s_netif, &ip) == ESP_OK) {
			snprintf(ipStr, sizeof(ipStr), IPSTR, IP2STR(&ip.ip));
		}
		print("Connected: %d\n", s_staConnected ? 1 : 0);
		print("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
		      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		print("SSID: %s\n", storage.getString("WIFI_STA_SSID", ""));
		print("IP: %s\n", ipStr);
	}
	print("Local UDP port: %d\n", udpLocalPort);
	print("Remote UDP port: %d\n", udpRemotePort);
	print("MAVLink connected: %d\n", mavlinkConnected ? 1 : 0);
}

void configWiFi(bool ap, const char* ssid, const char* password) {
	if (ssid == nullptr) ssid = "";
	if (password == nullptr) password = "";

	if (ap) {
		storage.putString("WIFI_AP_SSID", ssid);
		storage.putString("WIFI_AP_PASS", password);
	} else {
		storage.putString("WIFI_STA_SSID", ssid);
		storage.putString("WIFI_STA_PASS", password);
	}
	print("✓ 重启后生效 Reboot to apply new settings\n");
}
