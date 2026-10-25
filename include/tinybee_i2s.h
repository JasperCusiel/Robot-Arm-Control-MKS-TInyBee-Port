#pragma once

#include <Arduino.h>
#include <stdint.h>

using I2SSampleCallback = uint32_t (*)();

// Drives the TinyBee's cascaded output shift registers at 250 ksamples/s.
// The callback returns one complete 32-bit output word for each sample.
bool tinybeeI2SBegin(I2SSampleCallback callback);

