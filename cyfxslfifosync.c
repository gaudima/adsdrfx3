/*
 ## Cypress USB 3.0 Platform source file (cyfxbulksrcsink.c)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2011,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
 */

/* This file illustrates the bulk source sink application example using the DMA MANUAL_IN
 * and DMA MANUAL_OUT mode */

/*
   This example illustrates USB endpoint data source and data sink mechanism. The example
   comprises of vendor class USB enumeration descriptors with 2 bulk endpoints. A bulk OUT
   endpoint acts as the producer of data and acts as the sink to the host. A bulk IN endpoint
   acts as the consumer of data and acts as the source to the host.

   The data source and sink is achieved with the help of a DMA MANUAL IN channel and a DMA
   MANUAL OUT channel. A DMA MANUAL IN channel is created between the producer USB bulk
   endpoint and the CPU. A DMA MANUAL OUT channel is created between the CPU and the consumer
   USB bulk endpoint. Data is received in the IN channel DMA buffer from the host through the
   producer endpoint. CPU is signalled of the data reception using DMA callbacks. The CPU
   discards this buffer. This leads to the sink mechanism. A constant patern data is loaded
   onto the OUT Channel DMA buffer whenever the buffer is available. CPU issues commit of
   the DMA data transfer to the consumer endpoint which then gets transferred to the host.
   This leads to a constant source mechanism.

   The DMA buffer size is defined based on the USB speed. 64 for full speed, 512 for high speed
   and 1024 for super speed. CY_FX_BULKSRCSINK_DMA_BUF_COUNT in the header file defines the
   number of DMA buffers.

   For performance optimizations refer the readme.txt
 */

#include <cyu3gpio.h>
#include <spi_regs.h>
#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyfxslfifosync.h"
#include "cyu3usb.h"
#include "cyu3uart.h"
#include "cyu3gpif.h"
#include "cyu3pib.h"
#include "cyu3spi.h"
#include "pib_regs.h"
#include "cyfxspi_bb.h"
//#include "cyfxgpif2configNew.h"
#include "debug.h"
//#include "cyfxgpif2configOE2.h"
#include "cyfxgpif2configOE.h"
#include "spi_patch.h"


uint8_t glEp0Buffer[32];
uint8_t glEp0MultipleDataBuffer[33]; // 33rd byte is error code, just for debugging.
uint16_t glRecvdLen;
CyU3PThread bulkSrcSinkAppThread;     /* Application thread structure */
CyU3PDmaChannel glChHandleBulkSink;      /* DMA MANUAL_IN channel handle.          */
CyU3PDmaMultiChannel glChHandleBulkSrc;       /* DMA MANUAL_OUT channel handle.         */
CyBool_t dmaActive = CyTrue;

CyBool_t glIsApplnActive = CyFalse;      /* Whether the source sink application is active or not. */
CyBool_t glStartAd9269Gpif = CyFalse;
uint32_t glDMARxCount = 0;               /* Counter to track the number of buffers received. */
uint32_t glDMATxCount = 0;               /* Counter to track the number of buffers transmitted. */

/* Application Error Handler */
void CyFxAppErrorHandler(
        CyU3PReturnStatus_t apiRetStatus
) {
    /* Application failed with the error code apiRetStatus */

    CyU3PDebugPrintL(1, "ERROR HANDLER CALLED status: %d", apiRetStatus);
    /* Add custom debug or recovery actions here */

    /* Loop Indefinitely */
    for (;;) {
        /* Thread sleep : 100 ms */
        CyU3PDebugPrintL(1, "ERROR HANDLER CALLED status: %d", apiRetStatus);
        CyU3PThreadSleep(100);
    }
}

/* This function initializes the debug module. The debug prints
 * are routed to the UART and can be seen using a UART console
 * running at 115200 baud rate. */
void CyFxBulkSrcSinkApplnDebugInit(void) {
    CyU3PUartConfig_t uartConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Initialize the UART for printing debug messages */
    apiRetStatus = CyU3PUartInit();
    CyU3PDebugPreamble(CyFalse);
    //apiRetStatus = CyU3PUartInit();
    if (apiRetStatus != CY_U3P_SUCCESS) {
        /* Error handling */
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Set UART configuration */
    CyU3PMemSet((uint8_t *) &uartConfig, 0, sizeof(uartConfig));
    uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
    uartConfig.stopBit = CY_U3P_UART_ONE_STOP_BIT;
    uartConfig.parity = CY_U3P_UART_NO_PARITY;
    uartConfig.txEnable = CyTrue;
    uartConfig.rxEnable = CyFalse;
    uartConfig.flowCtrl = CyFalse;
    uartConfig.isDma = CyTrue;

    //apiRetStatus = CyU3PUartSetConfig (&uartConfig, NULL);
    apiRetStatus = CyU3PUartSetConfig(&uartConfig, NULL);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Set the UART transfer to a really large value. */
    //apiRetStatus = CyU3PUartTxSetBlockXfer (0xFFFFFFFF);
    apiRetStatus = CyU3PUartTxSetBlockXfer(0xFFFFFFFF);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Initialize the debug module. */
    apiRetStatus = CyU3PDebugInit(CY_U3P_LPP_SOCKET_UART_CONS, 8);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyFxAppErrorHandler(apiRetStatus);
    }
	CyU3PDebugPrintL (4, "\n\n\n====================\n");
}

CyU3PDmaChannel glChHandleUtoCPU;   /* DMA Channel handle for U2CPU transfer. */
CyU3PDmaChannelConfig_t dmaCfg1;

void CyFxBulkSrcSinkApplnDmaCB(CyU3PDmaMultiChannel *handle, CyU3PDmaCbType_t type, CyU3PDmaCBInput_t *input) {
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PDmaBuffer_t buf_p;
    CyU3PDebugPrintL(4, "DMA CB\n");
//    if(type == CY_U3P_DMA_CB_PROD_EVENT) {
//        for(int index = 0; index < 2; index++)
//        {
//            status = CyU3PDmaMultiChannelGetBuffer(handle, &buf_p, CYU3P_NO_WAIT);
//            if (status != CY_U3P_SUCCESS)
//            {
//                /* This can be a valid case. Just break. */
//                break;
//            }
//
//            /* This is a produce event notification to the CPU. This notification is
//             * received upon reception of every buffer. The buffer will not be sent
//             * out unless it is explicitly committed. The call shall fail if there
//             * is a bus reset / usb disconnect or if there is any application error. */
////            if(dmaActive) {
//                status = CyU3PDmaMultiChannelCommitBuffer(handle, buf_p.count, 0);
////            } else {
////                status = CyU3PDmaMultiChannelDiscardBuffer(handle);
////            }
//            if (status != CY_U3P_SUCCESS)
//            {
////                CyU3PDebugPrintL(4, "CyU3PDmaChannelCommitBuffer failed, Error code = %d\n", status);
//            }
//        }
//    }
}

/* This function starts the application. This is called
 * when a SET_CONF event is received from the USB host. The endpoints
 * are configured and the DMA pipe is setup in this function. */
void CyFxBulkSrcSinkApplnStart(void) {
    uint16_t size = 0;
    CyU3PEpConfig_t epCfg;

    CyU3PDmaMultiChannelConfig_t dmaCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();

    /* First identify the usb speed. Once that is identified,
     * create a DMA channel and start the transfer on this. */

    /* Based on the Bus Speed configure the endpoint packet size */
    switch (usbSpeed) {
        case CY_U3P_FULL_SPEED:
            size = 64;
            break;

        case CY_U3P_HIGH_SPEED:
            size = 512;
            break;

        case CY_U3P_SUPER_SPEED:
            size = 1024;
            break;

        default:
            CyU3PDebugPrintL(4, "Error! Invalid USB speed.\n");
            CyFxAppErrorHandler(CY_U3P_ERROR_FAILURE);
            break;
    }
    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_INTR;
    epCfg.burstLen = 1;
    epCfg.streams = 0;
    epCfg.streams = 0;
    epCfg.pcktSize = size;

    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_DEBUG, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    CyU3PUsbFlushEp(CY_FX_EP_DEBUG);

//    apiRetStatus = CyU3PDebugInit (CY_FX_EP_DEBUG_SOCKET, 8);
//    if (apiRetStatus != CY_U3P_SUCCESS)
//    {
//        CyFxAppErrorHandler (apiRetStatus);
//    }

    CyU3PMemSet((uint8_t *) &epCfg, 0, sizeof(epCfg));
    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_BULK;
    epCfg.burstLen = (usbSpeed == CY_U3P_SUPER_SPEED) ?
                     (CY_FX_EP_BURST_LENGTH) : 1;
    epCfg.streams = 0;
    epCfg.pcktSize = size;
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrintL(4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    CyU3PMemSet((uint8_t *) &dmaCfg, 0, sizeof(dmaCfg));

    dmaCfg.size = (usbSpeed == CY_U3P_SUPER_SPEED) ?
                  (size * CY_FX_EP_BURST_LENGTH) : (size);
    dmaCfg.count = CY_FX_BULKSRCSINK_DMA_BUF_COUNT;
    dmaCfg.validSckCount = 2;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    dmaCfg.notification = 0;
    dmaCfg.cb = NULL;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;
    dmaCfg.prodSckId[0] = CY_U3P_PIB_SOCKET_0;
    dmaCfg.prodSckId[1] = CY_U3P_PIB_SOCKET_1;
    dmaCfg.consSckId[0] = CY_FX_EP_CONSUMER_SOCKET;
    apiRetStatus = CyU3PDmaMultiChannelCreate(&glChHandleBulkSrc, CY_U3P_DMA_TYPE_AUTO_MANY_TO_ONE, &dmaCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrintL(4, "CyU3PDmaChannelCreate failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    apiRetStatus = CyU3PDmaMultiChannelSetXfer(&glChHandleBulkSrc, CY_FX_BULKSRCSINK_DMA_TX_SIZE, 0);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrintL(4, "CyU3PDmaChannelSetXfer failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);

    glIsApplnActive = CyTrue;
}

/* This function stops the application. This shall be called whenever a RESET
 * or DISCONNECT event is received from the USB host. The endpoints are
 * disabled and the DMA pipe is destroyed by this function. */
void
CyFxBulkSrcSinkApplnStop(
        void) {
    CyU3PEpConfig_t epCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Update the flag so that the application thread is notified of this. */
    glIsApplnActive = CyFalse;

    /* Disable endpoints. */
    CyU3PMemSet((uint8_t *) &epCfg, 0, sizeof(epCfg));
    epCfg.enable = CyFalse;

#if 0
    /* Destroy the channels */
    CyU3PDmaChannelDestroy (&glChHandleBulkSink);
    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
    /* Producer endpoint configuration. */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_PRODUCER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrintL (4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

#else
    CyU3PDmaMultiChannelDestroy(&glChHandleBulkSrc);
    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
    /* Consumer endpoint configuration. */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrintL(4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }
#endif

    CyU3PDebugDeInit ();

    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_DEBUG);

    /* Disable endpoints. */
    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyFalse;

    /* Consumer endpoint configuration. */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_DEBUG, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
}

/* Callback to handle the USB setup requests. */
CyBool_t CyFxBulkSrcSinkApplnUSBSetupCB(uint32_t setupdat0, uint32_t setupdat1) {
    uint8_t bRequest, bReqType;
    uint8_t bType, bTarget;
    uint16_t wValue;
    uint16_t wLength;
    uint16_t wIndex;

    /* Decode the fields from the setup request. */
    bReqType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
    bType = (bReqType & CY_U3P_USB_TYPE_MASK);
    bTarget = (bReqType & CY_U3P_USB_TARGET_MASK);
    bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
    wValue = ((setupdat0 & CY_U3P_USB_VALUE_MASK) >> CY_U3P_USB_VALUE_POS);
    wLength = ((setupdat1 & CY_U3P_USB_LENGTH_MASK) >> CY_U3P_USB_LENGTH_POS);
    wIndex = ((setupdat1 & CY_U3P_USB_INDEX_MASK) >> CY_U3P_USB_INDEX_POS);

//    glChHandleBulkSrcSusp = CyFalse;
//    int timeout = 1000;
//    CyBool_t retVal = CyFalse;
//
//    CyU3PDmaMultiChannelSetSuspend(&glChHandleBulkSrc, CY_U3P_DMA_SCK_SUSP_NONE, CY_U3P_DMA_SCK_SUSP_CUR_BUF);
//
//    while (!glChHandleBulkSrcSusp/* && timeout >= 0*/) {
//        CyU3PThreadSleep(1);
//        timeout--;
//    }
//
//    if (timeout < 0) {
//        return CyFalse;
//    }

//    dmaActive = CyFalse;
    CyU3PDebugPrintL(4, "Request\n");
    if (bRequest == 0x05) {

        CyU3PDebugPrintL(4, "\n\n\r Vendor Req = 0x%x Received", bRequest);
        CyU3PDebugPrintL(4, "\n\n\r wValue = 0x%d, wLength = 0x%x, wIndex = 0x%x", wValue, wLength, wIndex);
        if (wValue) /* Read */
        {
            //SpiReadWrite(wIndex, wValue, glEp0Buffer, wLength);
            /*
                    tmpbuf = glEp0Buffer;
                    CyU3PDebugPrintL (4, "\n\n\r glRecvdLen = 0x%x status = 0x%x\n\n\r", glRecvdLen, apiRetStatus);
                    for(wLength=0; wLength<glRecvdLen; wLength++ )
                    {
                        CyU3PDebugPrintL (4, "\t\t  0x%x ", *(tmpbuf));
                        tmpbuf++;
                    }
             */
            //CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
        } else {

            /*apiRetStatus = */
            //CyU3PUsbGetEP0Data (32, glEp0Buffer, &glRecvdLen);
            //SpiReadWrite(wIndex, wValue, glEp0Buffer, wLength);


        }

        return CyTrue;
    } else
        //if (bType == CY_U3P_USB_VENDOR_RQT)
    {
        if (bRequest == 0xB3) {
            //Reset function
            //CyU3PDeviceReset(CyFalse);
            //if ((bReqType & 0x80) == 0)
            CyU3PDebugPrintL(4, "Reset request 1\n");
            {
                CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
                if (glEp0Buffer[2] != 0xFF) {
                    //SPI send
                    CyU3PSpiSetSsnLine(CyFalse);
                    CyU3PSpiTransmitWords(glEp0Buffer, 2);
                    CyU3PSpiSetSsnLine(CyTrue);
                } else {
                    CyU3PDebugPrintL(4, "Reset request\n");
                    CyU3PDeviceReset(CyFalse);
                }
                return CyTrue;
            }
        } else if (bRequest == 0xB5) {
            //Old recieve function
            //CyU3PDeviceReset(CyFalse);
            //if ((bReqType & 0x80) == 0)
            {
                //CyU3PUsbGetEP0Data (wLength, glEp0Buffer, NULL);
                //SPI send
                glEp0Buffer[2] = wValue;
                glEp0Buffer[1] = (wIndex & 0xff);
                glEp0Buffer[0] = (wIndex >> 8);

                //CyU3PSpiSetSsnLine (CyFalse);
                CyU3PGpioSetValue(ADSPI_EN, CyFalse);
                CyU3PSpiTransmitReceiveWords(glEp0Buffer, 3);
                //CyU3PSpiTransmitWords(glEp0Buffer, 3);
                //CyU3PSpiSetSsnLine (CyTrue);
                CyU3PGpioSetValue(ADSPI_EN, CyTrue);

                CyU3PUsbSendEP0Data(wLength, glEp0Buffer);

                return CyTrue;
            }
        } else if (bRequest == 0xB6) {
            //Old transmit function
            //CyU3PDeviceReset(CyFalse);
            //if ((bReqType & 0x80) == 0)
            {
                CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
                //SPI send

                CyU3PGpioSetValue(ADSPI_EN, CyFalse);
                //CyU3PSpiSetSsnLine (CyFalse);
                //CyU3PSpiTransmitReceiveWords(glEp0Buffer, 3);
                CyU3PSpiTransmitWords(glEp0Buffer, 3);
                //CyU3PSpiSetSsnLine (CyTrue);
                CyU3PGpioSetValue(ADSPI_EN, CyTrue);

                //CyU3PUsbSendEP0Data (wLength, glEp0Buffer);

                return CyTrue;
            }
        } else if (bRequest == 0xBA) {
            //Start device
            CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
            CyU3PGpifControlSWInput(CyTrue);
            return CyTrue;
        } else if (bRequest == 0xBB) {
            //Stop device
            CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
            CyU3PGpifControlSWInput(CyFalse);
            return CyTrue;
        } else if (bRequest == 0xC0) {
            //Transmit and receive data;
            if (wLength > 32) {
                return CyFalse;
            }
            CyU3PUsbGetEP0Data(wLength, glEp0MultipleDataBuffer, NULL);
            CyU3PGpioSetValue(ADSPI_EN, CyFalse);
            glEp0MultipleDataBuffer[32] = CyU3PSpiTransmitReceiveWords(glEp0MultipleDataBuffer, wValue);
            CyU3PGpioSetValue(ADSPI_EN, CyTrue);
            return CyTrue;
        } else if (bRequest == 0xC1) {
            //Send received data;
            if (wLength > 33) {
                return CyFalse;
            }
            CyU3PUsbSendEP0Data(wLength, glEp0MultipleDataBuffer);
            return CyTrue;
        } else if (bRequest == 0xC2) {
            //blink led
            CyU3PGpifControlSWInput(wValue);
            CyU3PUsbSendEP0Data(1, glEp0MultipleDataBuffer);
            return CyTrue;
        } else if (bRequest == 0xC3) {
            //Do nothing
            CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
            return CyTrue;
        }
    }

//    CyU3PDmaMultiChannelSetSuspend(&glChHandleBulkSrc, CY_U3P_DMA_SCK_SUSP_NONE, CY_U3P_DMA_SCK_SUSP_NONE);
//    CyFx3BusyWait(100);
//    CyU3PDmaMultiChannelResume(&glChHandleBulkSrc, CyTrue, CyTrue);
//    dmaActive = CyTrue;

    return CyFalse;
}

/* This is a callback function to handle gpif events */
void CyFxBulkSrcSinkApplnGPIFEventCB(CyU3PGpifEventType event, uint8_t currentState) {
    CyU3PDebugPrintL(4, "\n\r !!!!GPIF INTERRUPT\n");

    switch (event) {
        case CYU3P_GPIF_EVT_SM_INTERRUPT:
            CyU3PDebugPrintL(4, "\n\r GPIF overflow INT received\n");
            break;

        default:
            break;
    }
}

/* This is the callback function to handle the USB events. */
void CyFxBulkSrcSinkApplnUSBEventCB(CyU3PUsbEventType_t evtype, uint16_t evdata) {
    CyU3PDebugPrintL(4, "\n\r USB EVENT\n");
    switch (evtype) {
        case CY_U3P_USB_EVENT_SETCONF:
            /* If the application is already active
             * stop it before re-enabling. */
            if (glIsApplnActive) {
                CyFxBulkSrcSinkApplnStop();
            }
            /* Start the source sink function. */
            CyFxBulkSrcSinkApplnStart();
            break;

        case CY_U3P_USB_EVENT_RESET:
        case CY_U3P_USB_EVENT_DISCONNECT:
            /* Stop the source sink function. */
            if (glIsApplnActive) {
                CyFxBulkSrcSinkApplnStop();
            }
            break;
        case CY_U3P_USB_EVENT_EP0_STAT_CPLT:
            break;
        default:
            break;
    }
}

void CyFxStartAd9269Gpif(void) {
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    /* Start the state machine. */
    apiRetStatus = CyU3PGpifSMStart(RESET, ALPHA_RESET);
    //apiRetStatus = CyU3PGpifSMStart (START, ALPHA_START);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrintL(4, "CyU3PGpifSMStart failed, Error Code = %d\n", apiRetStatus);

    }

    CyU3PDebugPrintL(4, "CyU3PGpifSMStart Done = %d\n", apiRetStatus);
}


/* This function initializes the USB Module, sets the enumeration descriptors.
 * This function does not start the bulk streaming and this is done only when
 * SET_CONF event is received. */
void CyFxBulkSrcSinkApplnInit(void) {
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    /**************************************GPIF****************************************************/
    CyU3PPibClock_t pibClock;

    /* Initialize the p-port block. */
    pibClock.clkDiv = 2;
    pibClock.clkSrc = CY_U3P_SYS_CLK;
    pibClock.isHalfDiv = CyFalse;
    /* Disable DLL for sync GPIF */
    pibClock.isDllEnable = CyFalse;
    apiRetStatus = CyU3PPibInit(CyTrue, &pibClock);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrintL(4, "P-port Initialization failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Load the GPIF configuration for Slave FIFO sync mode. */
    apiRetStatus = CyU3PGpifLoad(&CyFxGpifConfig);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrintL(4, "CyU3PGpifLoad failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }


    CyU3PGpifRegisterCallback(CyFxBulkSrcSinkApplnGPIFEventCB);

    /**********************************************************************************************/
    /* Start the USB functionality. */
    apiRetStatus = CyU3PUsbStart();
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrintL(4, "CyU3PUsbStart failed to Start, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* The fast enumeration is the easiest way to setup a USB connection,
     * where all enumeration phase is handled by the library. Only the
     * class / vendor requests need to be handled by the application. */
    CyU3PUsbRegisterSetupCallback(CyFxBulkSrcSinkApplnUSBSetupCB, CyTrue);

    /* Setup the callback to handle the USB events. */
    CyU3PUsbRegisterEventCallback(CyFxBulkSrcSinkApplnUSBEventCB);

    /* Set the USB Enumeration descriptors */

    /* Super speed device descriptor. */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, NULL, (uint8_t *) CyFxUSB30DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrintL(4, "USB set device descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* High speed device descriptor. */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, NULL, (uint8_t *) CyFxUSB20DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrintL(4, "USB set device descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* BOS descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, NULL, (uint8_t *) CyFxUSBBOSDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrintL(4, "USB set configuration descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Device qualifier descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, NULL, (uint8_t *) CyFxUSBDeviceQualDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrintL(4, "USB set device qualifier descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Super speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, NULL, (uint8_t *) CyFxUSBSSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrintL(4, "USB set configuration descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* High speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, NULL, (uint8_t *) CyFxUSBHSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrintL(4, "USB Set Other Speed Descriptor failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Full speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, NULL, (uint8_t *) CyFxUSBFSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrintL(4, "USB Set Configuration Descriptor failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 0 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *) CyFxUSBStringLangIDDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrintL(4, "USB set string descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 1 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *) CyFxUSBManufactureDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrintL(4, "USB set string descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 2 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *) CyFxUSBProductDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrintL(4, "USB set string descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Connect the USB Pins with super speed operation enabled. */
    apiRetStatus = CyU3PConnectState(CyTrue, CyTrue);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrintL(4, "USB Connect failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }


}


/* Entry function for the BulkSrcSinkAppThread. */
void BulkSrcSinkAppThread_Entry(uint32_t input) {
    /* Initialize the debug module */
    CyFxBulkSrcSinkApplnDebugInit();

    /* Initialize GPIO module. */
    CyFxGpioInit();
    /* Initialize the application */
    CyFxBulkSrcSinkApplnInit();

    //CyFxConfigureAd9269(3);

    CyFxStartAd9269Gpif();
    uint8_t  logPriority;
    uint16_t logId;
    uint32_t logParam;
    logPriority = 1;
    logId       = 0x1111;
    logParam    = 0x12345678;
    CyU3PDebugPrintL(6, "\n\rSTART DBM");
    for (;;) {
//        if (glIsApplnActive)
//        {
//            CyU3PDebugPrintL(2, "USB Debug logger: time from start in ticks: %d\n", CyU3PGetTime());
//        }
        uint8_t curState_p;

        CyU3PGpifGetSMState(&curState_p);

        CyU3PDebugPrintL(4, "%d| Current status = %d\n", CyU3PGetTime(), curState_p);
        CyU3PThreadSleep(300);


        /*if (glIsApplnActive)
        {

            if(!glStartAd9269Gpif)
            {
                glStartAd9269Gpif = CyTrue;
                CyFxStartAd9269Gpif();
            }*/


        /* Print the number of buffers received / transmitted so far from the USB host. */
        //	CyU3PDebugPrintL (4, "\n\rData tracker: buffers received: %d, buffers sent: %d\n", glDMARxCount, glDMATxCount);
        //}
    }
}

/* Application define function which creates the threads. */
void CyFxApplicationDefine(void) {
    void *ptr = NULL;
    uint32_t retThrdCreate = CY_U3P_SUCCESS;

    /* Allocate the memory for the threads */
    ptr = CyU3PMemAlloc(CY_FX_BULKSRCSINK_THREAD_STACK);

    /* Create the thread for the application */
    retThrdCreate = CyU3PThreadCreate (&bulkSrcSinkAppThread,      /* App thread structure */
                                       "21:Bulk_src_sink",                      /* Thread ID and thread name */
                                       BulkSrcSinkAppThread_Entry,              /* App thread entry function */
                                       0,                                       /* No input parameter to thread */
                                       ptr,                                     /* Pointer to the allocated thread stack */
                                       CY_FX_BULKSRCSINK_THREAD_STACK,          /* App thread stack size */
                                       CY_FX_BULKSRCSINK_THREAD_PRIORITY,       /* App thread priority */
                                       CY_FX_BULKSRCSINK_THREAD_PRIORITY,       /* App thread priority */
                                       CYU3P_NO_TIME_SLICE,                     /* No time slice for the application thread */
                                       CYU3P_AUTO_START                         /* Start the thread immediately */
    );

    /* Check the return code */
    if (retThrdCreate != 0) {
        /* Thread Creation failed with the error code retThrdCreate */

        /* Add custom recovery or debug actions here */

        /* Application cannot continue */
        /* Loop indefinitely */
        while (1);
    }
}

/*
 * Main function
 */
int main(void) {
    CyU3PIoMatrixConfig_t io_cfg;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Initialize the device */
    status = CyU3PDeviceInit(NULL);
    if (status != CY_U3P_SUCCESS) {
        goto handle_fatal_error;
    }

    /* Initialize the caches. Enable instruction cache and keep data cache disabled.
     * The data cache is useful only when there is a large amount of CPU based memory
     * accesses. When used in simple cases, it can decrease performance due to large
     * number of cache flushes and cleans and also it adds to the complexity of the
     * code. */
    status = CyU3PDeviceCacheControl(CyTrue, CyFalse, CyFalse);
    if (status != CY_U3P_SUCCESS) {
        goto handle_fatal_error;
    }
#if 0
    /* Configure the IO matrix for the device. On the FX3 DVK board, the COM port
     * is connected to the IO(53:56). This means that either DQ32 mode should be
     * selected or lppMode should be set to UART_ONLY. Here we are choosing
     * UART_ONLY configuration. */
    io_cfg.isDQ32Bit = CyTrue;
    io_cfg.useUart   = CyTrue;
    io_cfg.useI2C    = CyFalse;
    io_cfg.useI2S    = CyFalse;
    io_cfg.useSpi    = CyFalse;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
    /* No GPIOs are enabled. */
    io_cfg.gpioSimpleEn[0]  = 0;
    io_cfg.gpioSimpleEn[1]  = 0;
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    status = CyU3PDeviceConfigureIOMatrix (&io_cfg);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }
#endif
#if 1
    /* Configure the IO matrix for the device. On the FX3 DVK board, the COM port
     * is connected to the IO(53:56). This means that either DQ32 mode should be
     * selected or lppMode should be set to UART_ONLY. Here we are choosing
     * UART_ONLY configuration. */
    io_cfg.isDQ32Bit = CyFalse;//CyTrue;
    io_cfg.useUart = CyTrue;
    io_cfg.useI2C = CyFalse;
    io_cfg.useI2S = CyFalse;
    io_cfg.useSpi = CyTrue;//CyFalse;
    io_cfg.lppMode = CY_U3P_IO_MATRIX_LPP_DEFAULT;
    /* No GPIOs are enabled. */
    io_cfg.gpioSimpleEn[0] = 0;
    io_cfg.gpioSimpleEn[1] = 0x00002000;
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    status = CyU3PDeviceConfigureIOMatrix(&io_cfg);
    if (status != CY_U3P_SUCCESS) {
        goto handle_fatal_error;
    }

    {
        CyU3PSpiConfig_t spiConfig;
        CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
        apiRetStatus = CyU3PSpiInit();
        if (apiRetStatus != CY_U3P_SUCCESS) {
            CyU3PDebugPrintL(4, "SPI init failed, Error Code = %d\n", apiRetStatus);
        }
        /* Start the SPI master block. Run the SPI clock at 25MHz
         * and configure the word length to 8 bits. Also configure
         * the slave select using FW. */
        CyU3PMemSet((uint8_t *) &spiConfig, 0, sizeof(spiConfig));
        spiConfig.isLsbFirst = CyFalse;
        spiConfig.cpol = CyTrue;
        spiConfig.ssnPol = CyFalse;
        spiConfig.cpha = CyFalse;//CyTrue;
        spiConfig.leadTime = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
        spiConfig.lagTime = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
        spiConfig.ssnCtrl = CY_U3P_SPI_SSN_CTRL_HW_EACH_WORD;
        //spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_FW;
        spiConfig.clock = 10000000;
        spiConfig.wordLen = 8;

        apiRetStatus = CyU3PSpiSetConfig(&spiConfig, NULL);
        if (apiRetStatus != CY_U3P_SUCCESS) {
            CyU3PDebugPrintL(4, "SPI config failed, Error Code = %d\n", apiRetStatus);
        }

    }
#endif

    /* This is a non returnable call for initializing the RTOS kernel */
    CyU3PKernelEntry();

    /* Dummy return to make the compiler happy */
    return 0;

    handle_fatal_error:

    CyU3PDebugPrintL(1, "FATAL ERROR\n");
    /* Cannot recover from this error. */
    while (1);
}

/* [ ] */

