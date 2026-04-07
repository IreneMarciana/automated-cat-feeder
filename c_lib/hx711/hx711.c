#include "hx711.h"
#include <unistd.h>
#include <string.h>
#include <stdio.h>

// =========================
// MEMORY BARRIER (CRITICAL)
// =========================
#define MEM_BARRIER() __sync_synchronize()

int setupGPIO(HX711 *hx)
{
  setup_io();

  INP_GPIO(hx->data_pin);
  INP_GPIO(hx->clock_pin);
  OUT_GPIO(hx->clock_pin);

  // Ensure clock starts LOW
  setPinState(hx->clock_pin, 0);
  usleep(10);

  return 0;
}

void cleanGPIO(HX711 *hx)
{
  GPIO_PULL = 0;
  GPIO_PULLCLK0 = 1 << hx->data_pin;
  GPIO_PULL = 0;
  GPIO_PULLCLK0 = 0;
  restore_io();
}

// =========================
// PRIORITY
// =========================
int setPriority(int priority)
{
  struct sched_param sched;
  memset(&sched, 0, sizeof(sched));

  sched.sched_priority = priority;
  if (sched_setscheduler(0, SCHED_FIFO, &sched))
  {
    return 1;
  }
  return 0;
}

// =========================
// RESET HX711
// =========================
void reset(HX711 *hx)
{
  setPinState(hx->clock_pin, 1);
  usleep(100);
  setPinState(hx->clock_pin, 0);
  usleep(100);
}

// =========================
// GAIN SETTING
// =========================
void setGain(HX711 *hx)
{
  int pulses = 1;

  if (hx->wanted_channel == 'B')
    pulses = 2;
  else if (hx->gain_channel_A == 64)
    pulses = 3;

  for (int i = 0; i < pulses; i++)
  {
    setPinState(hx->clock_pin, 1);
    usleep(1);
    setPinState(hx->clock_pin, 0);
    usleep(1);
  }
}

// =========================
// RAW DATA READ (FIXED)
// =========================
int getRawData(HX711 *hx)
{
  unsigned int bits = 0;

  // Wait until DATA goes LOW (ready)
  while (getPinState(hx->data_pin))
  {
    usleep(100);
  }

  // Read 24 bits
  for (int i = 0; i < 24; i++)
  {
    setPinState(hx->clock_pin, 1);
    usleep(1);

    bits = (bits << 1) | (getPinState(hx->data_pin) ? 1 : 0);

    setPinState(hx->clock_pin, 0);
    usleep(1);
  }

  // Set gain for next read
  setGain(hx);

  // Sign extend 24-bit value
  if (bits & 0x800000)
  {
    bits |= ~0xFFFFFF;
  }

  return (int)bits;
}

// =========================
// GPIO READ (FIXED)
// =========================
bool getPinState(unsigned pin_number)
{
  if (pin_number > 31)
  {
    printf("getPinState - wrong pin number: %d\n", pin_number);
    return false;
  }

  MEM_BARRIER();

  int value = (*(gpio + 13) & (1 << pin_number));

  MEM_BARRIER();

  return value != 0;
}

// =========================
// GPIO WRITE (FIXED)
// =========================
int setPinState(unsigned pin_number, bool state)
{
  if (pin_number > 31)
  {
    printf("setPinState - wrong pin number: %d\n", pin_number);
    return 1;
  }

  MEM_BARRIER();

  if (state)
    *(gpio + 7) = (1 << pin_number);
  else
    *(gpio + 10) = (1 << pin_number);

  MEM_BARRIER();

  return 0;
}

// =========================
// INIT
// =========================
int initHX711(HX711 *hx, unsigned char clock_pin, unsigned char data_pin)
{
  if (clock_pin > 31 || data_pin > 31)
  {
    printf("Invalid GPIO pins\n");
    return 1;
  }

  hx->clock_pin = clock_pin;
  hx->data_pin = data_pin;
  hx->gain_channel_A = 128;
  hx->current_channel = 'A';
  hx->wanted_channel = 'A';

  hx->offset_A_128 = 0;
  hx->offset_A_64 = 0;
  hx->offset_B = 0;

  hx->scale_ratio_A_128 = 0.0;
  hx->scale_ratio_A_64 = 0.0;
  hx->scale_ratio_B = 0.0;

  hx->filterPtr = NULL;

  return 0;
}

// =========================
// ZERO SCALE
// =========================
int zeroScale(HX711 *hx)
{
  double result = getRawDataMean(hx, 50);

  if (hx->current_channel == 'A' && hx->gain_channel_A == 128)
  {
    hx->offset_A_128 = result;
    return 0;
  }

  return 1;
}

// =========================
// MEAN RAW DATA
// =========================
int getRawDataMean(HX711 *hx, int samples)
{
  long sum = 0;

  for (int i = 0; i < samples; i++)
  {
    int val = getRawData(hx);
    sum += val;
    usleep(5000);
  }

  return (int)(sum / samples);
}

// =========================
// DATA (OFFSET APPLIED)
// =========================
int getDataMean(HX711 *hx, int samples)
{
  int result = getRawDataMean(hx, samples);
  return result - hx->offset_A_128;
}

// =========================
// WEIGHT
// =========================
int getWeightMean(HX711 *hx, int samples)
{
  int result = getRawDataMean(hx, samples);

  if (hx->scale_ratio_A_128 == 0)
    return 0;

  return (int)((result - hx->offset_A_128) / hx->scale_ratio_A_128);
}