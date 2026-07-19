/*
 * micro-ROS Wi-Fi TCP transport for ESP32 / ESP32-S3.
 * Pairs with Orin: micro_ros_agent tcp4 --port <MICROROS_AGENT_PORT>
 *
 * TCP_NODELAY is enabled so 20 ms IMU samples are not delayed by Nagle.
 */
#if defined(ESP32)

#include "wifi_tcp_transport.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <micro_ros_arduino.h>

extern "C" {

static WiFiClient tcp_client;

static bool tcp_ensure_connected(struct micro_ros_agent_locator *locator) {
  if (tcp_client.connected()) {
    return true;
  }
  tcp_client.stop();
  /* Keepalive-ish: short connect timeout via WiFiClient default */
  if (!tcp_client.connect(locator->address,
                          static_cast<uint16_t>(locator->port))) {
    return false;
  }
  tcp_client.setNoDelay(true);
  return true;
}

bool arduino_wifi_tcp_transport_open(struct uxrCustomTransport *transport) {
  struct micro_ros_agent_locator *locator =
      (struct micro_ros_agent_locator *)transport->args;
  return tcp_ensure_connected(locator);
}

bool arduino_wifi_tcp_transport_close(struct uxrCustomTransport *transport) {
  (void)transport;
  tcp_client.stop();
  return true;
}

size_t arduino_wifi_tcp_transport_write(struct uxrCustomTransport *transport,
                                        const uint8_t *buf, size_t len,
                                        uint8_t *errcode) {
  (void)errcode;
  struct micro_ros_agent_locator *locator =
      (struct micro_ros_agent_locator *)transport->args;
  if (!tcp_ensure_connected(locator)) {
    return 0;
  }
  const size_t sent = tcp_client.write(buf, len);
  tcp_client.flush();
  return sent;
}

size_t arduino_wifi_tcp_transport_read(struct uxrCustomTransport *transport,
                                       uint8_t *buf, size_t len, int timeout,
                                       uint8_t *errcode) {
  (void)errcode;
  (void)transport;

  const uint32_t start = millis();
  while ((int32_t)(millis() - start) < timeout) {
    if (!tcp_client.connected()) {
      return 0;
    }
    if (tcp_client.available() > 0) {
      break;
    }
    delay(1);
  }
  if (tcp_client.available() <= 0) {
    return 0;
  }
  const int n = tcp_client.read(buf, len);
  return (n < 0) ? 0 : (size_t)n;
}

} /* extern "C" */

#endif /* ESP32 */
