/****************************************************************************
** Copyright (c) 2022, Carsten Schmidt. All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions
** are met:
**
** 1. Redistributions of source code must retain the above copyright
**    notice, this list of conditions and the following disclaimer.
**
** 2. Redistributions in binary form must reproduce the above copyright
**    notice, this list of conditions and the following disclaimer in the
**    documentation and/or other materials provided with the distribution.
**
** 3. Neither the name of the copyright holder nor the names of its
**    contributors may be used to endorse or promote products derived from
**    this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************/

#include "IfxAsclin_Asc.h"
#include "IfxCpu_Irq.h"

#include "I_ser.h"

////// Macros ////////////////////////////////////////////////////////////////

#define SER_ISRPRIO_RX    4
#define SER_ISRPRIO_TX    8
#define SER_ISRPRIO_ERR  12

#define SER_PIN_RX  IfxAsclin0_RXA_P14_1_IN
#define SER_PIN_TX  IfxAsclin0_TX_P14_0_OUT

#define SER_RX_BUFFER_SIZE  64
#define SER_TX_BUFFER_SIZE  64

////// Global ////////////////////////////////////////////////////////////////

static IfxAsclin_Asc g_ascHandle;

static uint8 g_uartTxBuffer[SER_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
static uint8 g_uartRxBuffer[SER_RX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];

////// ISR ///////////////////////////////////////////////////////////////////

IFX_INTERRUPT(asc0RxISR, 0, SER_ISRPRIO_RX)
{
    IfxAsclin_Asc_isrReceive(&g_ascHandle);
}

IFX_INTERRUPT(asc0TxISR, 0, SER_ISRPRIO_TX)
{
    IfxAsclin_Asc_isrTransmit(&g_ascHandle);
}

IFX_INTERRUPT(asc0ErrISR, 0, SER_ISRPRIO_ERR)
{
    IfxAsclin_Asc_isrError(&g_ascHandle);
}

////// Public ////////////////////////////////////////////////////////////////

void I_ser_init(uint32_t baud)
{
    IfxAsclin_Asc_Config ascConf;

    /* Set default configurations */
    IfxAsclin_Asc_initModuleConfig(&ascConf, &MODULE_ASCLIN0);

    /* Set the desired baud rate */
    ascConf.baudrate.baudrate     = (float32)baud;
    ascConf.baudrate.oversampling = IfxAsclin_OversamplingFactor_16;

    /* Configure the sampling mode */
    ascConf.bitTiming.medianFilter        = IfxAsclin_SamplesPerBit_three;
    ascConf.bitTiming.samplePointPosition = IfxAsclin_SamplePointPosition_8;

    /* ISR priorities and interrupt target */
    ascConf.interrupt.txPriority    = SER_ISRPRIO_TX;
    ascConf.interrupt.rxPriority    = SER_ISRPRIO_RX;
    ascConf.interrupt.erPriority    = SER_ISRPRIO_ERR;
    ascConf.interrupt.typeOfService = IfxCpu_Irq_getTos(IfxCpu_getCoreIndex());

    /* Pin configuration */
    const IfxAsclin_Asc_Pins pins = {
            .cts        = NULL_PTR,
            .ctsMode    = IfxPort_InputMode_pullUp,
            .rx         = &SER_PIN_RX,
            .rxMode     = IfxPort_InputMode_pullUp,
            .rts        = NULL_PTR,
            .rtsMode    = IfxPort_OutputMode_pushPull,
            .tx         = &SER_PIN_TX,
            .txMode     = IfxPort_OutputMode_pushPull,
            .pinDriver  = IfxPort_PadDriver_cmosAutomotiveSpeed1
    };
    ascConf.pins = &pins;

    /* FIFO buffers configuration */
    ascConf.txBuffer     = &g_uartTxBuffer[0];
    ascConf.txBufferSize = SER_TX_BUFFER_SIZE;
    ascConf.rxBuffer     = &g_uartRxBuffer[0];
    ascConf.rxBufferSize = SER_RX_BUFFER_SIZE;

    /* Init ASCLIN module */
    IfxAsclin_Asc_initModule(&g_ascHandle, &ascConf);
}

void I_ser_putc(const char c)
{
    IfxAsclin_Asc_blockingWrite(&g_ascHandle, (uint8)c);
}

void I_ser_puts(const char *s)
{
    Ifx_SizeT len = 0;

    if( s != NULL ) {
        const char *p = s;
        while( *p++ != '\0' ) {
            len++;
        }
    }

    if( len < 1 ) {
        return;
    }

    IfxAsclin_Asc_write(&g_ascHandle, s, &len, TIME_INFINITE);
}

size_t I_ser_available(void)
{
    return (size_t)IfxAsclin_Asc_getReadCount(&g_ascHandle);
}

uint8_t I_ser_getc(void)
{
    return (uint8_t)IfxAsclin_Asc_blockingRead(&g_ascHandle);
}
