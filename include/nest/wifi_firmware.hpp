#ifndef NEST_WIFI_FIRMWARE_H
#define NEST_WIFI_FIRMWARE_H

#include <libopencm3/stm32/iwdg.h>

extern "C" {
#include <driver/include/m2m_wifi.h>
#include <spi_flash/include/spi_flash.h>
}
#include <nest/uart.hpp>

#define CMD_READ_FLASH 0x01
#define CMD_WRITE_FLASH 0x02
#define CMD_ERASE_FLASH 0x03
#define CMD_MAX_PAYLOAD_SIZE 0x50
#define CMD_HELLO 0x99

typedef struct __attribute__((__packed__)) {
  uint8_t command;
  uint32_t address;
  uint32_t arg1;
  uint16_t payloadLength;

  // payloadLenght bytes of data follows...
} UartPacket;

void wifi_firmware_update();
void wifi_firmware_setup();
void wifi_firmware_receive_packet(UartPacket *packet, uint8_t *dest_payload);
void wifi_firmware_loop();
bool isBigEndian();
uint32_t fromNetwork32(uint32_t from);
uint16_t fromNetwork16(uint16_t from);
uint32_t toNetwork32(uint32_t to);
uint16_t toNetwork16(uint16_t to);

#endif // NEST_WIFI_FIRMWARE_H
