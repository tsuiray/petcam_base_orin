#pragma once

#if defined(ESP32)

#include <uxr/client/transport.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool arduino_wifi_tcp_transport_open(struct uxrCustomTransport *transport);
bool arduino_wifi_tcp_transport_close(struct uxrCustomTransport *transport);
size_t arduino_wifi_tcp_transport_write(struct uxrCustomTransport *transport,
                                        const uint8_t *buf, size_t len,
                                        uint8_t *errcode);
size_t arduino_wifi_tcp_transport_read(struct uxrCustomTransport *transport,
                                       uint8_t *buf, size_t len, int timeout,
                                       uint8_t *errcode);

#ifdef __cplusplus
}
#endif

#endif /* ESP32 */
