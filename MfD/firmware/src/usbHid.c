/* ************************************************************************** */
/** Descriptive File Name

  @Company
 Rose GmbH

  @File Name
 usbHid.c

  @Summary
 USB implementation.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
#include "usbHid.h"
#include "spi.h"
#include "adc.h"

#include <stdbool.h>
#include "system_definitions.h"
#include "system_config.h"
#include "driver/usb/drv_usb.h"
#include "usb/usb_device.h"

// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
/* Receive Data Buffer */
uint8_t USB_MAKE_BUFFER_DMA_READY receiveDataBuffer[USB_STACK_SIZE] ;
/* Transmit data buffer for sending values to PC */
uint8_t USB_MAKE_BUFFER_DMA_READY transmitVal2PCbuffer[USB_STACK_SIZE];

/* intern variables */
USB_DATA usbData;

static void processUSB_auto(uint8_t typeID);
static void processUSB_manu(void);
static void receiveAndclearUsbRx ( void );
static void sendAndclearUsbTx ( void );
static void sendAndclearUsbTxRx ( void );

// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
USB_DEVICE_HID_EVENT_RESPONSE APP_USBDeviceHIDEventHandler
(
    USB_DEVICE_HID_INDEX iHID,
    USB_DEVICE_HID_EVENT event,
    void * eventData,
    uintptr_t userData
)
{
    USB_DEVICE_HID_EVENT_DATA_REPORT_SENT * reportSent;
    USB_DEVICE_HID_EVENT_DATA_REPORT_RECEIVED * reportReceived;

    /* Check type of event */
    switch (event)
    {
        case USB_DEVICE_HID_EVENT_REPORT_SENT:

            /* The eventData parameter will be USB_DEVICE_HID_EVENT_REPORT_SENT
             * pointer type containing details about the report that was
             * sent. */
            reportSent = (USB_DEVICE_HID_EVENT_DATA_REPORT_SENT *) eventData;
            if(reportSent->handle == usbData.txTransferHandle )
            {
                // Transfer progressed.
                usbData.hidDataTransmitted = true;
            }
            
            break;

        case USB_DEVICE_HID_EVENT_REPORT_RECEIVED:

            /* The eventData parameter will be USB_DEVICE_HID_EVENT_REPORT_RECEIVED
             * pointer type containing details about the report that was
             * received. */

            reportReceived = (USB_DEVICE_HID_EVENT_DATA_REPORT_RECEIVED *) eventData;
            if(reportReceived->handle == usbData.rxTransferHandle )
            {
                // Transfer progressed.
                usbData.hidDataReceived = true;
            }
          
            break;

        case USB_DEVICE_HID_EVENT_SET_IDLE:

            /* For now we just accept this request as is. We acknowledge
             * this request using the USB_DEVICE_HID_ControlStatus()
             * function with a USB_DEVICE_CONTROL_STATUS_OK flag */

            USB_DEVICE_ControlStatus(usbData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);

            /* Save Idle rate recieved from Host */
            usbData.idleRate = ((USB_DEVICE_HID_EVENT_DATA_SET_IDLE*)eventData)->duration;
            break;

        case USB_DEVICE_HID_EVENT_GET_IDLE:

            /* Host is requesting for Idle rate. Now send the Idle rate */
            USB_DEVICE_ControlSend(usbData.usbDevHandle, & (usbData.idleRate),1);

            /* On successfully reciveing Idle rate, the Host would acknowledge back with a
               Zero Length packet. The HID function drvier returns an event
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the application upon
               receiving this Zero Length packet from Host.
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates this control transfer
               event is complete */

            break;
        default:
            // Nothing to do.
            break;
    }
    return USB_DEVICE_HID_EVENT_RESPONSE_NONE;
}

void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context)
{
    switch(event)
    {
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:
        {
            /* Host has de configured the device or a bus reset has happened.
             * Device layer is going to de-initialize all function drivers.
             * Hence close handles to all function drivers (Only if they are
             * opened previously. */
            usbData.deviceConfigured = false;
            
            OnBoardLED_2Off();
            
            usbData.state = USB_STATE_WAIT_FOR_CONFIGURATION;
            break;
        }
        case USB_DEVICE_EVENT_CONFIGURED:
        {
            /* Set the flag indicating device is configured. */
            usbData.deviceConfigured = true;

            /* Save the other details for later use. */
            usbData.configurationValue = ((USB_DEVICE_EVENT_DATA_CONFIGURED*)eventData)->configurationValue;

            /* Register application HID event handler */
            USB_DEVICE_HID_EventHandlerSet(USB_DEVICE_HID_INDEX_0, APP_USBDeviceHIDEventHandler, (uintptr_t)&usbData);

            OnBoardLED_2On();
            
            break;
        }
        case USB_DEVICE_EVENT_SUSPENDED:
        {
            /* Switch on green and orange, switch off red */
//            OnBoardLED_1Off();
//            OnBoardLED_2On();
            break;
        }
        case USB_DEVICE_EVENT_POWER_DETECTED:
        {
            /* VBUS was detected. We can attach the device */

            USB_DEVICE_Attach (usbData.usbDevHandle);
            break;
        }
        case USB_DEVICE_EVENT_POWER_REMOVED:
        {
            /* VBUS is not available */
            USB_DEVICE_Detach(usbData.usbDevHandle);
            
            OnBoardLED_2Off();
            
            break;
        }
        /* These events are not used in this demo */
        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            printf("USB_DEVICE_EVENT_RESUMED;USB_DEVICE_EVENT_ERROR;default;\n");
            break;
    }
}

// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************

static void processUSB_manu(void)
{    
    /* --- HEADER --- */
    /* Protocol ID */
    usbData.val2PC->protIDbyte = HID_PROT_VAL2PC;
    /* Number of Sensor */
    usbData.val2PC->automaticMode = receiveDataBuffer[0];
    usbData.autoMode = receiveDataBuffer[0];
    /* Control letter */
    usbData.val2PC->typeIDbyte = receiveDataBuffer[1];
    
    /* Fill individual part of the frame */
    switch(receiveDataBuffer[1])
    {
        /* Air Temperature Sensor*/
        case HID_PROT_AS:
        {
            /* --- DATA ---- */
            getAdcAirTemperatureSensorValue(ADC_AS1_idx);
            usbData.val2PC->adcValue1 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage1 = adcData.d_AS_ioVoltage;
            usbData.val2PC->calcedValue1 = adcData.d_temperature;
            printf("AS1 temp: %u | ", adcData.d_temperature);

            getAdcAirTemperatureSensorValue(ADC_AS2_idx);
            usbData.val2PC->adcValue2 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage2 = adcData.d_AS_ioVoltage;
            usbData.val2PC->calcedValue2 = adcData.d_temperature;
            printf("AS2 temp: %u | ", adcData.d_temperature);

            getAdcAirTemperatureSensorValue(ADC_AS3_idx);
            usbData.val2PC->adcValue3 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage3 = adcData.d_AS_ioVoltage;
            usbData.val2PC->calcedValue3 = adcData.d_temperature;
            printf("AS3 temp: %u | ", adcData.d_temperature);

            /* Send USB frame */
            sendAndclearUsbTx();

            break;
        }

        /* Gyro Sensor*/
        case HID_PROT_GY:
        {
            /* Read out the actual sensor value*/
            getAdcAccelerometerValues();

            /* --- DATA ---- */
            usbData.val2PC->adcValue1 = adcData.u16_gyro_x;
            usbData.val2PC->adcValue2 = adcData.u16_gyro_y;
            usbData.val2PC->adcValue3 = adcData.u16_gyro_z;

            /* Send USB frame */
            sendAndclearUsbTx();
            break;
        }

        /* Manifold Pressure Sensor*/
        case HID_PROT_MPS:
        {
            /* --- DATA ---- */
            getAdcManifoldPressureSensorValue(ADC_MPS_idx);
            usbData.val2PC->adcValue1 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage1 = adcData.d_ioVoltage;
            usbData.val2PC->calcedValue1 = adcData.d_bar;
            usbData.val2PC->calcedValue2 = adcData.d_psi; 
            printf("MPS bar: %u | ", adcData.d_bar);
            printf("MPS psi: %u | ", adcData.d_psi);

            /* Send USB frame */
            sendAndclearUsbTx();
            break;
        }

        /* OnBoard Temperature Sensor */
        case HID_PROT_OBT:
        {
            /* --- DATA ---- */
            usbData.val2PC->calcedValue1 = spiData.pre_position;
            usbData.val2PC->calcedValue2 = spiData.post_position;
            printf("OBT: %u,%2u | ", spiData.pre_position, spiData.post_position);

            /* Whole send procedure */
            sendAndclearUsbTx();
            break;
        }

        /* Pressure Sensor*/
        case HID_PROT_PS:
        {
            /* --- DATA ---- */
            getAdcPressureSensorValue(ADC_PS1_idx);
            usbData.val2PC->adcValue1 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage1 = adcData.d_ioVoltage;
            usbData.val2PC->calcedValue1 = adcData.d_bar;
            usbData.val2PC->calcedValue2 = adcData.d_psi;
            printf("PS1 bar: %u | ", adcData.d_bar);
            printf("PS1 psi: %u | ", adcData.d_psi);

            getAdcPressureSensorValue(ADC_PS2_idx);
            usbData.val2PC->adcValue3 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage3 = adcData.d_ioVoltage;
            usbData.val2PC->calcedValue3 = adcData.d_bar;
            usbData.val2PC->calcedValue4 = adcData.d_psi;
            printf("PS2 bar: %u | ", adcData.d_bar);
            printf("PS2 psi: %u | ", adcData.d_psi);

            /* Send USB frame */
            sendAndclearUsbTx();
            break;
        }

        /* Relais Switch*/
        case HID_PROT_RL:
        {
             /* --- DATA ---- */
             /* Check which Relais wants to switch */
             if ( usbData.val2PC->calcedValue1 == APP_REL1_idx)
             {
//                 appData.b_relais1_state = BSP_nRelais_En(BSP_Relais_nGate1, (bool)receiveDataBuffer[2]);
                 usbData.val2PC->calcedValue1 = appData.b_relais1_state;
             }
             else if ( usbData.val2PC->calcedValue2 == APP_REL2_idx )
             {
//                 appData.b_relais2_state = BSP_nRelais_En(BSP_Relais_nGate2, (bool)receiveDataBuffer[2]);
                 usbData.val2PC->calcedValue2 = appData.b_relais2_state;
             }
             else if ( usbData.val2PC->calcedValue3 == APP_REL3_idx )
             {
//                 appData.b_relais2_state = BSP_nRelais_En(BSP_Relais_nGate3, (bool)receiveDataBuffer[2]);
                 /* --- DATA --- */
                 usbData.val2PC->calcedValue3 = appData.b_relais2_state;
             }

             /* Send USB frame */
             sendAndclearUsbTx();

            break;
        }

        /* Liquid Temperature Sensor */
        case HID_PROT_TS:
        {            
            /* --- DATA ---- */
            getAdcTemperatureSensorValue(ADC_TS1_idx);
            usbData.val2PC->adcValue1 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage1 = adcData.d_ioVoltage;
            usbData.val2PC->calcedValue1 = adcData.d_temperature;
            printf("TS1 temp: %.1f | ", adcData.d_temperature);

            getAdcTemperatureSensorValue(ADC_TS2_idx);
            usbData.val2PC->adcValue2 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage2 = adcData.d_ioVoltage;
            usbData.val2PC->calcedValue2 = adcData.d_temperature;
            printf("TS2 temp: %.1f | ", adcData.d_temperature);

            getAdcTemperatureSensorValue(ADC_TS3_idx);
            usbData.val2PC->adcValue3 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage3 = adcData.d_ioVoltage;
            usbData.val2PC->calcedValue3 = adcData.d_temperature;
            printf("TS3 temp: %.1f | ", adcData.d_temperature);

            getAdcTemperatureSensorValue(ADC_TS4_idx);
            usbData.val2PC->adcValue4 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage4 = adcData.d_ioVoltage;
            usbData.val2PC->calcedValue4 = adcData.d_temperature;
            printf("TS4 temp: %.1f | ", adcData.d_temperature);

            /* Send USB frame */
            sendAndclearUsbTx();

            break;
        }
        case HID_PROT_V:
        {
            /* --- DATA ---- */
            usbData.val2PC->calcedValue1 = getAdcVin();
            printf("Vin: %.1f | ", usbData.val2PC->calcedValue1);
            usbData.val2PC->calcedValue2 = getAdc5V0();
            printf("5V0: %.1f | ", usbData.val2PC->calcedValue2);
            usbData.val2PC->calcedValue3 = getAdc3V3();
            printf("3V3: %.1f | ", usbData.val2PC->calcedValue3);

            /* Send USB frame */
            sendAndclearUsbTx();
            break;
        }
    }
}

static void processUSB_auto(uint8_t typeID)
{    
    /* --- HEADER --- */
    /* Protocol ID */
    usbData.val2PC->protIDbyte = HID_PROT_VAL2PC;
    /* Number of Sensor */
    usbData.val2PC->automaticMode = usbData.autoMode;
    /* Control letter */
    usbData.val2PC->typeIDbyte = typeID;
    
    /* Fill individual part of the frame */
    switch(typeID)
    {
        /* Air Temperature Sensor*/
        case HID_PROT_AS:
        {
            /* --- DATA ---- */
            getAdcAirTemperatureSensorValue(ADC_AS1_idx);
            usbData.val2PC->adcValue1 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage1 = adcData.d_AS_ioVoltage;
            usbData.val2PC->calcedValue1 = adcData.d_temperature;
            printf("AS1 temp: %u | ", adcData.d_temperature);

            getAdcAirTemperatureSensorValue(ADC_AS2_idx);
            usbData.val2PC->adcValue2 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage2 = adcData.d_AS_ioVoltage;
            usbData.val2PC->calcedValue2 = adcData.d_temperature;
            printf("AS2 temp: %u | ", adcData.d_temperature);

            getAdcAirTemperatureSensorValue(ADC_AS3_idx);
            usbData.val2PC->adcValue3 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage3 = adcData.d_AS_ioVoltage;
            usbData.val2PC->calcedValue3 = adcData.d_temperature;
            printf("AS3 temp: %u | ", adcData.d_temperature);

            /* Send USB frame */
            sendAndclearUsbTx();

            break;
        }

        /* Gyro Sensor*/
        case HID_PROT_GY:
        {
            /* Read out the actual sensor value*/
            getAdcAccelerometerValues();

            /* --- DATA ---- */
            usbData.val2PC->adcValue1 = adcData.u16_gyro_x;
            usbData.val2PC->adcValue2 = adcData.u16_gyro_y;
            usbData.val2PC->adcValue3 = adcData.u16_gyro_z;

            /* Send USB frame */
            sendAndclearUsbTx();
            break;
        }

        /* Manifold Pressure Sensor*/
        case HID_PROT_MPS:
        {
            /* --- DATA ---- */
            getAdcManifoldPressureSensorValue(ADC_MPS_idx);
            usbData.val2PC->adcValue1 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage1 = adcData.d_ioVoltage;
            usbData.val2PC->calcedValue1 = adcData.d_bar;
            usbData.val2PC->calcedValue2 = adcData.d_psi; 
            printf("MPS bar: %u | ", adcData.d_bar);
            printf("MPS psi: %u | ", adcData.d_psi);

            /* Send USB frame */
            sendAndclearUsbTx();
            break;
        }

        /* OnBoard Temperature Sensor */
        case HID_PROT_OBT:
        {
            /* --- DATA ---- */
            usbData.val2PC->calcedValue1 = spiData.pre_position;
            usbData.val2PC->calcedValue2 = spiData.post_position;
            printf("OBT: %u,%2u | ", spiData.pre_position, spiData.post_position);

            /* Whole send procedure */
            sendAndclearUsbTx();
            break;
        }

        /* Pressure Sensor*/
        case HID_PROT_PS:
        {
            /* --- DATA ---- */
            getAdcPressureSensorValue(ADC_PS1_idx);
            usbData.val2PC->adcValue1 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage1 = adcData.d_ioVoltage;
            usbData.val2PC->calcedValue1 = adcData.d_bar;
            usbData.val2PC->calcedValue2 = adcData.d_psi;
            printf("PS1 bar: %u | ", adcData.d_bar);
            printf("PS1 psi: %u | ", adcData.d_psi);

            getAdcPressureSensorValue(ADC_PS2_idx);
            usbData.val2PC->adcValue3 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage3 = adcData.d_ioVoltage;
            usbData.val2PC->calcedValue3 = adcData.d_bar;
            usbData.val2PC->calcedValue4 = adcData.d_psi;
            printf("PS2 bar: %u | ", adcData.d_bar);
            printf("PS2 psi: %u | ", adcData.d_psi);

            /* Send USB frame */
            sendAndclearUsbTx();
            break;
        }

        /* Liquid Temperature Sensor */
        case HID_PROT_TS:
        {            
            /* --- DATA ---- */
            getAdcTemperatureSensorValue(ADC_TS1_idx);
            usbData.val2PC->adcValue1 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage1 = adcData.d_ioVoltage;
            usbData.val2PC->calcedValue1 = adcData.d_temperature;
            printf("TS1 temp: %.1f | ", adcData.d_temperature);

            getAdcTemperatureSensorValue(ADC_TS2_idx);
            usbData.val2PC->adcValue2 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage2 = adcData.d_ioVoltage;
            usbData.val2PC->calcedValue2 = adcData.d_temperature;
            printf("TS2 temp: %.1f | ", adcData.d_temperature);

            getAdcTemperatureSensorValue(ADC_TS3_idx);
            usbData.val2PC->adcValue3 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage3 = adcData.d_ioVoltage;
            usbData.val2PC->calcedValue3 = adcData.d_temperature;
            printf("TS3 temp: %.1f | ", adcData.d_temperature);

            getAdcTemperatureSensorValue(ADC_TS4_idx);
            usbData.val2PC->adcValue4 = adcData.u16_adcValue;
            usbData.val2PC->ioVoltage4 = adcData.d_ioVoltage;
            usbData.val2PC->calcedValue4 = adcData.d_temperature;
            printf("TS4 temp: %.1f | ", adcData.d_temperature);

            /* Send USB frame */
            sendAndclearUsbTx();

            break;
        }
        case HID_PROT_V:
        {
            /* --- DATA ---- */
            usbData.val2PC->calcedValue1 = getAdcVin();
            printf("Vin: %.1f | ", usbData.val2PC->calcedValue1);
            usbData.val2PC->calcedValue2 = getAdc5V0();
            printf("5V0: %.1f | ", usbData.val2PC->calcedValue2);
            usbData.val2PC->calcedValue3 = getAdc3V3();
            printf("3V3: %.1f | ", usbData.val2PC->calcedValue3);

            /* Send USB frame */
            sendAndclearUsbTx();
            break;
        }
    }
}

/*------------------------------------------------------------------------------ 
 * Function: sendAndclearUsbRx()
 * receive data from USB
 * 
 * read data from USB and write to buffer. Get ready for next transaction
 -----------------------------------------------------------------------------*/
static void receiveAndclearUsbRx ( void )
{
    usbData.hidDataReceived = false;

    /* Place a new read request. */
    USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
            &usbData.rxTransferHandle, usbData.receiveDataBuffer, USB_STACK_SIZE );
}
/*------------------------------------------------------------------------------ 
 * Function: sendAndclearUsbTx()
 * send USB transceiver
 * 
 * send struct via usb and make it ready for next transaction
 -----------------------------------------------------------------------------*/
static void sendAndclearUsbTx ( void )
{
    /* Prepare the USB module to send the data packet to the host */
    USB_DEVICE_HID_ReportSend (USB_DEVICE_HID_INDEX_0,
            &usbData.txTransferHandle, transmitVal2PCbuffer, USB_STACK_SIZE );
}
/*------------------------------------------------------------------------------ 
 * Function: sendAndclearUsbTxRx()
 * send and receive from and to USB
 * 
 * send struct via usb and make it ready for next transaction
 * read data from USB and write to buffer. Get ready for next transaction
 -----------------------------------------------------------------------------*/
static void sendAndclearUsbTxRx ( void )
{
    /* Prepare the USB module to send the data packet to the host */
    sendAndclearUsbTx();
    /* Place a new read request. */
    receiveAndclearUsbRx();
}

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */
void initUSB ( void )
{
    /* Place the App state machine in its initial state. */
    usbData.state = USB_STATE_INIT;
    
    usbData.usbDevHandle = USB_DEVICE_HANDLE_INVALID;
    usbData.deviceConfigured = false;
    usbData.txTransferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;
    usbData.rxTransferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;
    usbData.hidDataReceived = false;
    usbData.hidDataTransmitted = true;
    usbData.receiveDataBuffer = &receiveDataBuffer[0];

    memset(transmitVal2PCbuffer, 0, USB_STACK_SIZE);
    usbData.val2PC = (UI_USB_DATA*)transmitVal2PCbuffer;

    usbData.typeID = 2;
    
    printf("initUSB() done!\n");
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */
void taskUSB (void )
{
    /* Check if device is configured.  See if it is configured with correct
     * configuration value  */
    switch(usbData.state)
    {
        case USB_STATE_INIT:
        {
            /* Open the device layer */
            usbData.usbDevHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );
            
            /* Check if usb is ready for work */
            if(usbData.usbDevHandle != USB_DEVICE_HANDLE_INVALID)
            {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(usbData.usbDevHandle, APP_USBDeviceEventHandler, 0);
                printf("USB init succeeded!\n");
                usbData.state = USB_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                printf("USB init fail!");
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;
        }
        case USB_STATE_WAIT_FOR_CONFIGURATION:
        {
            if(usbData.deviceConfigured == true)
            {
                /* Device is ready to run the main task */
                usbData.hidDataReceived = false;
                usbData.hidDataTransmitted = true;
                usbData.state = USB_STATE_MAIN_TASK;
                
                /* Place a new read request. */
                USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                        &usbData.rxTransferHandle, &receiveDataBuffer[0], 64);
            }
            break;
        }
        case USB_STATE_MAIN_TASK:
        {
            if(!usbData.deviceConfigured)
            {
                /* Device is not configured */
                usbData.state = USB_STATE_WAIT_FOR_CONFIGURATION;
            }
            else if( usbData.hidDataReceived )
            {
                processUSB_manu();
                return;
            }
            
            /* if automatic send mode is active, count through the different Sensors */
            if(usbData.autoMode == 1)
            {
                processUSB_auto(usbData.typeID);
                
                if (usbData.typeID < 10)
                {
                    /* increase counter for next sensor value frame */
                    usbData.typeID++;
                }
                else
                {
                    /* reset value to 2 */
                    usbData.typeID = 2;
                }
            }
            break;
        }    
        case USB_STATE_ERROR:
        {
            printf("USB state Error | ");
            break;
        }
        default:
        {
            printf("USB default | ");
            break;
        }
    }
}
/* *****************************************************************************
 End of File
 */