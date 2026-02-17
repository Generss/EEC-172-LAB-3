//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - SPI Demo
// Application Overview - The demo application focuses on showing the required 
//                        initialization sequence to enable the CC3200 SPI 
//                        module in full duplex 4-wire master and slave mode(s).
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup SPI_Demo
//! @{
//
//*****************************************************************************

// Standard includes
#include <string.h>
#include <stdio.h>
#include <stdint.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"
#include "oled_test.h"
#include "Adafruit_SSD1351.h"
#include "gpio.h"
#include "hw_nvic.h"
#include "hw_apps_rcm.h"
#include "systick.h"
#include "typing.h"


// Common interface includes
#include "uart_if.h"
#include "pin_mux_config.h"

#include "systick_utils.h"


#define APPLICATION_VERSION     "1.4.0"


#define SPI_IF_BIT_RATE  100000

#define SENDER 0


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

int messageIndex = 0;
volatile char cBuff[625];
volatile int rx_write = 0;
volatile int rx_read  = 0;
volatile int charIndex = 0;
volatile uint8_t newCharSet = 0;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif



typedef struct {
    uint16_t code;
    const char *name;
} IRKeyEntry;

static const IRKeyEntry ir_key_table[] = {
    { 255,   "KEY_0" },
    { 32895, "KEY_1" },
    { 16575, "KEY_2" },
    { 49215, "KEY_3" },
    { 8415,  "KEY_4" },
    { 41055, "KEY_5" },
    { 24735, "KEY_6" },
    { 57375, "KEY_7" },
    { 4335,  "KEY_8" },
    { 36975, "KEY_9" },
    { 765,   "KEY_LAST" },
    { 0xE817, "KEY_ENTER"}

};


extern void (* const g_pfnVectors[])(void);

volatile unsigned long int current_char_timeout = 0;




volatile uint8_t newValueSet = 0;
volatile int currentCode = 0x00;
volatile int lastCode = 0x00;

volatile unsigned int bufferIntro = 0x00;
volatile unsigned int bufferData = 0x00;
volatile uint8_t edgesDetected = 0;
volatile int typing = 0;


typedef struct PinSetting {
    unsigned long port;
    unsigned int pin;
} PinSetting;
static const PinSetting infared = { .port = GPIOA0_BASE, .pin = 0x1};
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


const IRKeyEntry* ir_lookup(uint16_t code) {
    int i;
    for (i = 0; i < sizeof(ir_key_table)/sizeof(ir_key_table[0]); i++) {
        if (ir_key_table[i].code == code) {
            return &ir_key_table[i];
        }
    }
    return NULL;
}


static void BoardInit(void) {

    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);

    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);


    PRCMCC3200MCUInit();


    /* In case of TI-RTOS vector table is initialize by OS itself */
    #ifndef USE_TIRTOS
      //
      // Set vector table base
      //
    #if defined(ccs)
        MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
    #endif
    #if defined(ewarm)
        MAP_IntVTableBaseSet((unsigned long)&__vector_table);
    #endif
    #endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}




static void GPIOInfaredHandler(void) { //Pin 55 infared handler
    unsigned long ulStatus;

    //printf("Made it to handler");
    ulStatus = MAP_GPIOIntStatus(infared.port, true);
    MAP_GPIOIntClear(infared.port, ulStatus);

    // read the countdown register and compute elapsed cycles
    uint64_t delta = SYSTICK_RELOAD_VAL - SysTickValueGet();
    // convert elapsed cycles to microseconds
    uint64_t delta_us = TICKS_TO_US(delta);

    //The systick wrapped around itself, need to restart
    if (systick_cnt > 0) {
        edgesDetected = 0;
        bufferData = 0;
        SysTickReset();
        return;
    }

    //for the start bit
    if (delta_us > 12000 && delta_us < 16000) {
        edgesDetected = 0;
        bufferData = 0;
        SysTickReset();
        return;
    }

    //to deal with any other large gap, if there is one it's not our remote
    if (delta_us > 4000) {
        edgesDetected = 0;
        bufferData = 0;
        SysTickReset();
        return;
    }

    uint8_t changeBit;
    if (delta_us > 900 && delta_us < 1500) {
        changeBit = 0;
    }
    else if (delta_us > 1800 && delta_us < 2800) {
        changeBit = 1;
    }
    else {
        // leader / gap / noise
        SysTickReset();
        return;
    }

    if(edgesDetected < 32) {
        bufferData = bufferData << 1;
        bufferData += changeBit;
        edgesDetected++;
    }


    SysTickReset();
    if(edgesDetected >= 32) {

        uint16_t intro = bufferData >> 16;

        if (intro == 0x02FD) {
            lastCode = currentCode;
            currentCode = bufferData & 0xFFFF;;
            newValueSet = 1;
        }
        edgesDetected = 0;
        bufferIntro = 0x00;
        bufferData = 0x00;

    }


}

typedef enum{
    STATE_START,
    STATE_START_MESSAGE,
    STATE_INVALID_DELETE,
    STATE_LOOP_CHAR,
    STATE_NEW_CHAR,
    STATE_DELETE,
    STATE_DELETE_REPEAT
} typeState;

typeState transition(typeState current){
    switch(current){
    case STATE_START:
        return STATE_LOOP_CHAR;
    case STATE_START_MESSAGE:
        if(currentCode == KEY_LAST){
            return STATE_INVALID_DELETE;
        }
        return STATE_LOOP_CHAR;
    case STATE_INVALID_DELETE:
        return STATE_START_MESSAGE;
    case STATE_LOOP_CHAR:
        if(currentCode == KEY_LAST){
            return STATE_DELETE;
        }
        if((lastCode != currentCode) || current_char_timeout > 20){
            return STATE_NEW_CHAR;
        }
        return STATE_LOOP_CHAR;
    case STATE_NEW_CHAR:
        if(currentCode == KEY_LAST){
            return STATE_DELETE;
        }
        if(currentCode == lastCode && current_char_timeout <= 20){
            return STATE_LOOP_CHAR;
        }
        return STATE_NEW_CHAR;
    case STATE_DELETE_REPEAT:
        if(currentCode == KEY_LAST){
            return STATE_DELETE_REPEAT;
        }
        return STATE_LOOP_CHAR;
    case STATE_DELETE:
        if(currentCode == KEY_LAST){
            return STATE_DELETE_REPEAT;
        }
        return STATE_LOOP_CHAR;
    }
    return STATE_START;
    // need an error state
}

void keyMappingTest(void) {
    fillScreen(BLACK);
    setTextSize(1);
    setTextColor(WHITE, BLACK);
    setCursor(0,0);
    int x = 0;
    int y = 0;
    char curr =' ';

    typeState currentState = STATE_START;

    while(1) {
        Report("checking if ValueSet ");
        while(!newValueSet) {}
        const IRKeyEntry *key = ir_lookup(currentCode);
        Report("Pressed (0x%X)\r\n", currentCode);
        if(!key){
            return;
        }

        currentState = transition(currentState);
        if(key->code == KEY_ENTER){
            while(x!=114){
                x+=6;
                ++messageIndex;
                cBuff[messageIndex] = ' ';
            }
            newValueSet = 0;
            return;
        }

        switch(currentState){
        case STATE_START_MESSAGE:
            drawChar(x,y,curr,WHITE,BLACK,1);
            cBuff[messageIndex] = curr;
            ++messageIndex;
            break;
        case STATE_LOOP_CHAR:
            typing = 1;
            current_char_timeout = 0;
            curr = setChar(key->code);
            drawChar(x,y,curr,WHITE,BLACK,1);
            cBuff[messageIndex] = curr;
            break;
        case STATE_DELETE:
            typing = 0;
            current_char_timeout = 0;
            drawChar(x,y,' ',WHITE,BLACK,1);
            --messageIndex;
            break;
        case STATE_NEW_CHAR:
            typing = 1;
            current_char_timeout = 0;
            if(x < 120){
                x+=6;
            }else{
                x=0;
                y+=8;
            }
            nextChar();
            curr = setChar(key->code);
            drawChar(x,y,curr,WHITE,BLACK,1);
            ++messageIndex;
            cBuff[messageIndex] = curr;
            break;
        case STATE_DELETE_REPEAT:
            typing = 0;
            current_char_timeout = 0;
            if(x!=0){
                x-=6;
                drawChar(x,y,' ',WHITE,BLACK,1);
            }else{
                x=120;
                y-=8;
                drawChar(x,y,' ',WHITE,BLACK,1);
            }
            --messageIndex;
            break;
        case STATE_INVALID_DELETE:
            typing = 0;
            current_char_timeout = 0;
            break;
        }
        newValueSet = 0;

    }
}
void UARTISR(void) {
    unsigned long st = MAP_UARTIntStatus(UARTA1_BASE, true);
    MAP_UARTIntClear(UARTA1_BASE, st);

    while (MAP_UARTCharsAvail(UARTA1_BASE)) {
        int ch = MAP_UARTCharGetNonBlocking(UARTA1_BASE);
        if (ch == -1) break;

        if (rx_write < (int)sizeof(cBuff)) {
            cBuff[rx_write++] = (char)ch;
            newCharSet = 1;
        }
    }
}

void InitUartCommunication(void) {
    MAP_UARTConfigSetExpClk(UARTA1_BASE,
        MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
        UART_BAUD_RATE,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    MAP_UARTFIFOEnable(UARTA1_BASE);

    // REGISTER ISR FIRST
    MAP_UARTIntRegister(UARTA1_BASE, UARTISR);

    // Clear any pending UART interrupts
    unsigned long st = MAP_UARTIntStatus(UARTA1_BASE, true);
    MAP_UARTIntClear(UARTA1_BASE, st);

    // Enable UART RX + RX timeout interrupts
    MAP_UARTIntEnable(UARTA1_BASE, UART_INT_RX | UART_INT_RT);


    MAP_IntEnable(INT_UARTA1);
}


void main()
{
    unsigned long ulStatus;

    BoardInit();

    PinMuxConfig();

    MAP_SPIReset(GSPI_BASE);
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,
                     SPI_MODE_MASTER,
                     SPI_SUB_MODE_0,
                     (SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVELOW |
                     SPI_WL_8));
    MAP_SPIFIFOEnable(GSPI_BASE, SPI_TX_FIFO);
    MAP_SPIEnable(GSPI_BASE);


    SysTickInit();


    if(SENDER == 1) {
        MAP_GPIOIntRegister(infared.port, GPIOInfaredHandler);

        MAP_GPIOIntTypeSet(infared.port, infared.pin, GPIO_FALLING_EDGE);

        ulStatus = MAP_GPIOIntStatus(infared.port, false); // clear interrupts for GPIO0
        MAP_GPIOIntClear(infared.port, ulStatus);

        MAP_GPIOIntEnable(infared.port, infared.pin);
    }
    InitUartCommunication();


    InitTerm();
    ClearTerm();

    Message("\n\n\n\r");
    Message("\t\t   ********************************************\n\r");
    Message("\t\t        CC3200 Lab 3  \n\r");
    Message("\t\t   ********************************************\n\r");
    Message("\n\n\n\r");

    Adafruit_Init();



    if(SENDER == 0) {
        fillScreen(BLACK);
        setTextSize(1);
        setTextColor(WHITE, BLACK);
        setCursor(0,0);

        int x = 0;
        int y = 0;
        while (1) {

            while (!newCharSet) {}

            while (rx_read < rx_write) {
                char curr = cBuff[rx_read++];
                x += 6;
                drawChar(x, y, curr, WHITE, BLACK, 1);
                if (x >= 120) { x = 0; y += 7; }
            }

            MAP_IntMasterDisable();
            if (rx_read >= rx_write) {
                newCharSet = 0;
             }
            MAP_IntMasterEnable();
        }
     }
     else {
         while(1){
             keyMappingTest();
             int index = 0;

             while(index <= messageIndex) {
                 UARTCharPut(UARTA1_BASE,cBuff[index]);
                 index++;
             }
             messageIndex = 0;
         }
     }
}
