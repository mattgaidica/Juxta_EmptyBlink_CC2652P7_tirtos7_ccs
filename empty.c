/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty.c ========
 */

/* For usleep() */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <MC3635.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/Watchdog.h>

/* Driver configuration */
#include "ti_drivers_config.h"

// see https://github.com/mcubemems/mCube_mc36xx_arduino_driver/blob/master/MC36XX.cpp
#define SPI_MSG_LENGTH 2
#define READ_CMD 0x80
#define ADDR_BIT 0x40
unsigned char masterRxBuffer[SPI_MSG_LENGTH];
unsigned char masterTxBuffer[SPI_MSG_LENGTH];
SPI_Handle spiHandle;

void blinkLoop(uint8_t runOnce)
{
    while (1)
    {
        GPIO_write(LED_1, 1);
        usleep(50000);
        GPIO_write(LED_1, 0);
        usleep(50000);
        if (runOnce == 1)
        {
            break;
        }
    }
}

void sensorSniff(void)
{
    MC3635_stop(spiHandle);
    MC3635_SetSniffThreshold(spiHandle, MC36XX_AXIS_X, 2);
    MC3635_SetSniffThreshold(spiHandle, MC36XX_AXIS_Y, 2);
    MC3635_SetSniffThreshold(spiHandle, MC36XX_AXIS_Z, 2);
    MC3635_SetSniffDetectCount(spiHandle, MC36XX_AXIS_X, 3);
    MC3635_SetSniffDetectCount(spiHandle, MC36XX_AXIS_Y, 3);
    MC3635_SetSniffDetectCount(spiHandle, MC36XX_AXIS_Z, 3);
    MC3635_SetSniffAndOrN(spiHandle, MC36XX_ANDORN_OR);
    MC3635_SetSniffDeltaMode(spiHandle, MC36XX_DELTA_MODE_C2P);
    MC3635_SetINTCtrl(spiHandle, 0, 0, 0, 0, 1); //Enable wake-up INT
    MC3635_sniff(spiHandle);
}

void sniffInterrupt(uint_least8_t index)
{
    MC36XX_interrupt_event_t evt_mc36xx = {0};
    if (spiHandle == NULL)
    {
        return;
    }
    GPIO_write(LED_1, 1);
    MC3635_INTHandler(spiHandle, &evt_mc36xx);
    sensorSniff();
    GPIO_write(LED_1, 0);
}

/*
 *  ======== mainThread ========
 */
void* mainThread(void *arg0)
{
    MC36XX_interrupt_event_t evt_mc36xx = {0};
    blinkLoop(1);

    GPIO_init();
    SPI_init();

    GPIO_write(LED_0, 0);
    GPIO_write(LED_1, 0);

    spiHandle = MC3635_init(CONFIG_SPI);
    if (spiHandle == NULL)
    {
        blinkLoop(0);
    }
    MC3635_start(spiHandle);

    // scratch example
//    MC3635_writeReg(spiHandle, MC36XX_REG_SCRATCH, 0xA2);
//    uint8_t value = MC3635_readReg(spiHandle, MC36XX_REG_SCRATCH);

    sensorSniff();
    GPIO_enableInt(AXY_INT);

    while (1)
    {
//        GPIO_write(LED_1, 0);
//        uint8_t intVal = GPIO_read(AXY_INT);
//        if (intVal == 0) {
//            GPIO_write(LED_1, 1);
////            MC36XX_acc_t rawAccel = MC3635_readRawAccel(spiHandle);
//            MC3635_INTHandler(spiHandle, &evt_mc36xx);
//            sensorSniff();
//            sleep(1);
//        }
        usleep(10);
    }

    SPI_close(spiHandle);

    while (1)
    {
        GPIO_write(LED_0, 1);
        usleep(25000);
        GPIO_write(LED_0, 0);
        sleep(1);
    }
}
