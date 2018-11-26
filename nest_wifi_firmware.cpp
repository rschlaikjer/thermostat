#include "nest_wifi_firmware.h"

static const int MAX_PAYLOAD_SIZE = 1024;

// Allocated statically so the compiler can tell us
// about the amount of used RAM
static UartPacket pkt;
static uint8_t payload[MAX_PAYLOAD_SIZE];

void wifi_firmware_update() {
    printf("Entering firmware update mode\n");
    wifi_firmware_setup();
    while (true) {
        wifi_firmware_loop();
        iwdg_reset();
    }
}


void wifi_firmware_setup() {
    nm_bsp_init();
    if (m2m_wifi_download_mode() != M2M_SUCCESS) {
        printf("Failed to put the WiFi module in download mode\n");
        while (true);
    }
}

void wifi_firmware_receive_packet(UartPacket *packet, uint8_t *dest_payload) {
    // Read command
    uint8_t *p = reinterpret_cast<uint8_t *>(packet);
    uint16_t l = sizeof(UartPacket);
    while (l > 0) {
        int c = uart_getc();
        if (c == -1)
            continue;
        *p++ = c;
        l--;
    }

    // Convert parameters from network byte order to cpu byte order
    packet->address = fromNetwork32(packet->address);
    packet->arg1 = fromNetwork32(packet->arg1);
    packet->payloadLength = fromNetwork16(packet->payloadLength);

    // Read payload
    l = packet->payloadLength;
    while (l > 0) {
        int c = uart_getc();
        if (c == -1)
            continue;
        *dest_payload++ = c;
        l--;
    }
}

void wifi_firmware_loop() {

    wifi_firmware_receive_packet(&pkt, payload);

    if (pkt.command == CMD_HELLO) {
        if (pkt.address == 0x11223344 && pkt.arg1 == 0x55667788)
            printf("v10000");
    }

    if (pkt.command == CMD_MAX_PAYLOAD_SIZE) {
        uint16_t res = toNetwork16(MAX_PAYLOAD_SIZE);
        uart_write(reinterpret_cast<uint8_t *>(&res), sizeof(res));
    }

    if (pkt.command == CMD_READ_FLASH) {
        uint32_t address = pkt.address;
        uint32_t len = pkt.arg1;
        if (spi_flash_read(payload, address, len) != M2M_SUCCESS) {
            printf("ER");
        } else {
            uart_write(payload, len);
            printf("OK");
        }
    }

    if (pkt.command == CMD_WRITE_FLASH) {
        uint32_t address = pkt.address;
        uint32_t len = pkt.payloadLength;
        if (spi_flash_write(payload, address, len) != M2M_SUCCESS) {
            printf("ER");
        } else {
            printf("OK");
        }
    }

    if (pkt.command == CMD_ERASE_FLASH) {
        uint32_t address = pkt.address;
        uint32_t len = pkt.arg1;
        if (spi_flash_erase(address, len) != M2M_SUCCESS) {
            printf("ER");
        } else {
            printf("OK");
        }
    }
}

bool isBigEndian() {
  uint32_t test = 0x11223344;
  uint8_t *pTest = reinterpret_cast<uint8_t *>(&test);
  return pTest[0] == 0x11;
}

uint32_t fromNetwork32(uint32_t from) {
  static const bool be = isBigEndian();
  if (be) {
    return from;
  } else {
    uint8_t *pFrom = reinterpret_cast<uint8_t *>(&from);
    uint32_t to;
    to = pFrom[0]; to <<= 8;
    to |= pFrom[1]; to <<= 8;
    to |= pFrom[2]; to <<= 8;
    to |= pFrom[3];
    return to;
  }
}

uint16_t fromNetwork16(uint16_t from) {
  static bool be = isBigEndian();
  if (be) {
    return from;
  } else {
    uint8_t *pFrom = reinterpret_cast<uint8_t *>(&from);
    uint16_t to;
    to = pFrom[0]; to <<= 8;
    to |= pFrom[1];
    return to;
  }
}

uint32_t toNetwork32(uint32_t to) {
  return fromNetwork32(to);
}

uint16_t toNetwork16(uint16_t to) {
  return fromNetwork16(to);
}
