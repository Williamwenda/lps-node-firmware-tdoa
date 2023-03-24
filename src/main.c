/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stm32f0xx_hal.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "spi.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "led.h"
#include "button.h"

#include "cfg.h"
#include "eeprom.h"

#include "usb_device.h"
#include "usbcomm.h"

#include "lps25h.h"
#include "test_support.h"
#include "production_test.h"

#include "uwb.h"


#define POWER_LEVELS 10

const uint8_t *uid = (uint8_t*)MCU_ID_ADDRESS;

static void restConfig();
static void changeAddress(uint8_t addr);
static void handleSerialInput(char ch);
static void handleRangeRequest(char* ch);
static void handleButton(void);
static void changeMode(unsigned int newMode);
static void changeRadioMode(unsigned int newMode);
static void changePower(uint8_t power);
static void printModeList();
static void printRadioModeList();
static void printMode();
static void printRadioMode();
static void printPowerHelp();
static void help();
static void bootload(void);

typedef enum {mainMenu, modeMenu, idMenu, radioMenu, powerMenu} Menu_t;
typedef struct {
  bool configChanged;
  Menu_t currentMenu;
  unsigned int tempId;
} MenuState;

static void main_task(void *pvParameters) {
  int i;
  char ch;
  char data_buf[3];
  bool selftestPasses = true;

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();

  // Light up all LEDs to test
  ledOn(ledRanging);
  ledOn(ledSync);
  ledOn(ledMode);
  buttonInit(buttonIdle);

  // printf("\r\n\r\n====================\r\n");

  // printf("SYSTEM\t: CPU-ID: ");
  // for (i=0; i<12; i++) {
  //   printf("%02x", uid[i]);
  // }
  // printf("\r\n");

  // Initializing pressure sensor (if present ...)
  // lps25hInit(&hi2c1);
  // testSupportPrintStart("Initializing pressure sensor");
  // if (lps25hTestConnection()) {
  //   printf("[OK]\r\n");
  //   lps25hSetEnabled(true);
  // } else {
  //   printf("[FAIL] (%u)\r\n", (unsigned int)hi2c1.ErrorCode);
  //   selftestPasses = false;
  // }

  // testSupportPrintStart("Pressure sensor self-test");
  // testSupportReport(&selftestPasses, lps25hSelfTest());

  // Initializing i2c eeprom
  eepromInit(&hi2c1);
  testSupportPrintStart("EEPROM self-test");
  testSupportReport(&selftestPasses, eepromTest());


  cfgInit();

  // Initialising radio
  testSupportPrintStart("Initialize UWB ");
  uwbInit();
  if (uwbTest()) {
    printf("[OK]\r\n");
  } else {
    printf("[ERROR]: %s\r\n", uwbStrError());
    selftestPasses = false;
  }

  if (!selftestPasses) {
    printf("TEST\t: One or more self-tests failed, blocking startup!\r\n");
    usbcommSetSystemStarted(true);
  }

  // Printing UWB configuration
  // struct uwbConfig_s * uwbConfig = uwbGetConfig();
  // printf("CONFIG\t: Address is 0x%X\r\n", uwbConfig->address[0]);
  // printf("CONFIG\t: Mode is %s\r\n", uwbAlgorithmName(uwbConfig->mode));
  // printf("CONFIG\t: Tag mode anchor list (%i): ", uwbConfig->anchorListSize);
  // for (i = 0; i < uwbConfig->anchorListSize; i++) {
  //   printf("0x%02X ", uwbConfig->anchors[i]);
  // }
  // printf("\r\n");
  // printf("CONFIG\t: Anchor position enabled: %s\r\n",
  //        uwbConfig->positionEnabled?"true":"false");
  // if (uwbConfig->positionEnabled) {
  //   printf("CONFIG\t: Anchor position: %f %f %f\r\n", uwbConfig->position[0],
  //                                                     uwbConfig->position[1],
  //                                                     uwbConfig->position[2]);
  // }
  // printf("CONFIG\t: SmartPower enabled: %s\r\n", uwbConfig->smartPower?"True":"False");
  // printf("CONFIG\t: Force TX power: %s\r\n", uwbConfig->forceTxPower?"True":"False");
  // if(uwbConfig->forceTxPower) {
  //   printf("CONFIG\t: TX power setting: %08X\r\n", (unsigned int)uwbConfig->txPower);
  // }
  // printf("CONFIG\t: Bitrate: %s\r\n", uwbConfig->lowBitrate?"low":"normal");
  // printf("CONFIG\t: Preamble: %s\r\n", uwbConfig->longPreamble?"long":"normal");

  HAL_Delay(500);

  ledOff(ledRanging);
  ledOff(ledSync);
  ledOff(ledMode);

  // printf("SYSTEM\t: Node started ...\r\n");
  // printf("SYSTEM\t: Press 'h' for help.\r\n");

  usbcommSetSystemStarted(true);

  // Starts UWB protocol
  uwbStart();

  // Main loop ...
  while(1) {
    // usbcommPrintWelcomeMessage();

    ledTick();
    // handleButton();
    // // Measure pressure
    // if (uwbConfig.mode != modeSniffer) {
    //   if(lps25hGetData(&pressure, &temperature, &asl)) {
    //     pressure_ok = true;
    //   } else {
    //     printf("Fail reading pressure\r\n");
    //     printf("pressure not ok\r\n");
    //   }
    // }

    // Accepts serial commands
#ifdef USE_FTDI_UART
    if (HAL_UART_Receive(&huart1, (uint8_t*)&ch, 1, 0) == HAL_OK) {
#else
    if(usbcommRead(&data_buf, 3)) {
#endif
      // handleSerialInput(ch);
      handleRangeRequest(data_buf);
    }
  }
}

/* Function required to use "printf" to print on serial console */
int _write (int fd, const void *buf, size_t count)
{
  // stdout
  if (fd == 1) {
    #ifdef USE_FTDI_UART
      HAL_UART_Transmit(&huart1, (uint8_t *)buf, count, HAL_MAX_DELAY);
    #else
      usbcommWrite(buf, count);
    #endif
  }

  // stderr
  if (fd == 2) {
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, count, HAL_MAX_DELAY);
  }

  return count;
}

static void handleRangeRequest(char* data_buf)
{
  // req_anchor_id;
  if(data_buf[0] == 0x3c &&
     data_buf[1] == 0x3c)
    {
      reqRange(data_buf[2]);
    }

}

static void handleButton(void) {
  ButtonEvent be = buttonGetState();

  if (be == buttonShortPress) {
    ledBlink(ledRanging, true);
    // TODO: Implement and remove ledblink
  }  else if (be == buttonLongPress) {
    ledBlink(ledSync, true);
    // TODO: Implement and remove ledblink
  }

  buttonProcess();
}

static void restConfig() {
  printf("Resetting EEPROM configuration...");
  if (cfgReset()) {
    printf("OK\r\n");
  } else {
    printf("ERROR\r\n");
  }
}

static void changeAddress(uint8_t addr) {
  printf("Updating address to 0x%02X\r\n", addr);
  cfgWriteU8(cfgAddress, addr);
  if (cfgReadU8(cfgAddress, &addr)) {
    printf("Device address: 0x%X\r\n", addr);
  } else {
    printf("Device address: Not found!\r\n");
  }
}

static void changeMode(unsigned int newMode) {
    printf("Previous device mode: ");
    printMode();

    cfgWriteU8(cfgMode, newMode);

    printf("New device mode: ");
    printMode();
}

static void changePower(uint8_t power) { //expects [0, POWER_LEVELS-1] interval
  // 000 11111 = 0x1F = 33.5dB = max power
  // 110 00000 = 0xC0 = 0dB = min power
  // first 3 bits can have 7 values 000 - 110  (111 means OFF)

  // convert interval to [1, POWER_LEVELS]
  if(power > POWER_LEVELS-1) {
    power = POWER_LEVELS;
  } else {
    power++;
  }

  float desired_db = (power * 1. / POWER_LEVELS )*33.5;

  //split desired power between amplifier and mixer
  uint8_t db_amp = (uint8_t) roundf( (desired_db * 18/33.5) / 3 ) * 3;  //rounded to 3dB steps (supported by amplifier)
  float db_mix = roundf( (desired_db - db_amp) / 0.5 ) * 0.5; //rounded to 0.5dB steps (supported by mixer)

  // 7=111 minus (normalized db_amp values 1-6), all shifted to first 3 bits:
  uint8_t amp_3bits = 0xE0 & (7-( db_amp / 3 + 1 ))<<5;  // 0xE0 bitmask 11100000
  // mixer is 5 bits:
  uint8_t mix_5bits = 0x1F & ( (uint8_t) (db_mix / 0.5) ); // 0x1F bitmask 00011111

  uint8_t power_bits = amp_3bits | mix_5bits;

  //copy power byte to all 4 bytes of txPower
  uint32_t txPower = power_bits | (power_bits<<8) | (power_bits<<16) | (power_bits<<24);
  printf("Setting txpower to: 0x%lX = %.1fdB\r\n", txPower, db_amp+db_mix);
  cfgWriteU32(cfgTxPower, txPower);
}

static void printModeList()
{
  unsigned int count = uwbAlgorithmCount();
  int current_mode = -1;
  uint8_t mode;

  if (cfgReadU8(cfgMode, &mode)) {
    current_mode = mode;
  }

  printf("-------------------\r\n");
  printf("Available UWB modes:\r\n");
  for (int i=0; i<count; i++) {
    printf(" %d - %s%s\r\n", i, uwbAlgorithmName(i),
                             (i == current_mode)?" (Current mode)":"");
  }
}


static StaticTask_t xMainTask;
static StackType_t ucMainStack[configMINIMAL_STACK_SIZE];

int main() {
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();

  // Configure the system clock
  SystemClock_Config();

  // Setup main task
  xTaskCreateStatic( main_task, "main", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, ucMainStack, &xMainTask );

  // Start the FreeRTOS scheduler
  vTaskStartScheduler();

  // Should never reach there
  while(1);

  return 0;
}

// Enter bootloader from software: Taken from micropython machine_bootloader function
static void bootload(void) {
    printf("Entering DFU Mode\r\n");
    HAL_Delay(500);

    HAL_RCC_DeInit();
    HAL_DeInit();

    __HAL_REMAPMEMORY_SYSTEMFLASH();

    // arm-none-eabi-gcc 4.9.0 does not correctly inline this
    //     //     // MSP function, so we write it out explicitly here.
    //__set_MSP(*((uint32_t*) 0x00000000));
    __ASM volatile ("movs r3, #0\nldr r3, [r3, #0]\nMSR msp, r3\n" : : : "r3", "sp");

    ((void (*)(void)) *((uint32_t*) 0x00000004))();

    while (1);
}

// Freertos required callbacks
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  static StaticTask_t xIdleTaskTCB;
  static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  static StaticTask_t xTimerTaskTCB;
  static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void vAssertCalled( unsigned long ulLine, const char * const pcFileName )
{
  printf("Assert failed at %s:%lu", pcFileName, ulLine);
  while(1);
}
