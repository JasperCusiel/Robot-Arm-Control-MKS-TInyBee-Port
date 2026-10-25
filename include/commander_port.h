#pragma once
/*
 * Which serial port carries the PAROL Commander protocol.
 *
 * COMMANDER_ON_UART2 = 0 : USB (UART0, the CH340) - plug the TinyBee into the PC.
 * COMMANDER_ON_UART2 = 1 : UART2 on the EXP2 header - wire it to a Raspberry Pi.
 *
 * The default UART2 pins GPIO16/17 are NOT usable on the TinyBee: they are on
 * EXP1, level-shifted to 5 V and output-only (and GPIO16/17 are J2 DIR/STEP here).
 * EXP2 pins are direct 3.3 V ESP32 GPIOs, so they are used instead.
 */
#define COMMANDER_ON_UART2 1

constexpr int COMMANDER_UART2_RX_PIN = 14;  // EXP2 pin 3 <- Pi TXD (GPIO14, header pin 8)
constexpr int COMMANDER_UART2_TX_PIN = 19;  // EXP2 pin 1 -> Pi RXD (GPIO15, header pin 10)
