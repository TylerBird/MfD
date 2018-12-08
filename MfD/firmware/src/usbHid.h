/* ************************************************************************** */
/** Descriptive File Name

  @Company
 Rose GmbH

  @File Name
    usbHid.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef USB_H    /* Guard against multiple inclusion */
#define USB_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "usb/usb_chapter_9.h"
#include "usb/usb_device.h"

// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
#define USB_STACK_SIZE      64
#define USB_MAKE_BUFFER_DMA_READY   __attribute__ ((coherent, aligned(16)))

// *****************************************************************************
// Definition of available States in taskUSB
typedef enum
{
    /* Application is initializing */
    USB_STATE_INIT,
    /* Application is waiting for configuration */
    USB_STATE_WAIT_FOR_CONFIGURATION,
    /* Application is running the main tasks */
    USB_STATE_MAIN_TASK,
    /* Application is in an error state */
    USB_STATE_ERROR
            
} USB_STATES;

// Definition of available States in automatic mode
typedef enum
{
    /* Application is running the main tasks */
    USB_AUTO_STATE_MAIN_TASK,
    USB_A_STATE_AS,
    USB_A_STATE_GY,
    USB_A_STATE_MPS,
    USB_A_STATE_OTS,
    USB_A_STATE_PS,
    USB_A_STATE_TS, 
    /* Application is in an error state */
    USB_AUTO_STATE_ERROR
            
} USB_AUTO_STATES;

// Definition of HID Protocol 
typedef enum
{
    HID_PROT_INVALID = 0,   // Application's state machine's initial state
    HID_PROT_VAL2PC,        // 1 - Protocol ID
    HID_PROT_AS,            // 2 - Air Temperature Sensor
    HID_PROT_GY,            // 3 - Accelerator values
    HID_PROT_MPS,           // 4 - Manifold Pressure Sensor
    HID_PROT_PS,            // 5 - Pressure Sensor
    HID_PROT_TS,            // 6 - Liquid Temperature Sensor
    HID_PROT_V,             // 7 - Voltage Values
    HID_PROT_OBT,           // 8 - OnBoard Temperature Sensor
    HID_PROT_KTYPE,         // 9 - K-Type Element
    HID_PROT_RL,            //10 - Relais Control
    HID_PROT_ERROR
} HID_PROTOCOL;

// Definition of USB Frame Structure
typedef struct
{
    uint8_t protIDbyte;     // 1 Defines the acutal Protocol version
    uint8_t automaticMode;  // 2 0-manuell(request, response), 1-automatic (send out frames)
    uint8_t typeIDbyte;     // 3 Descripes which sensor the PC wants to read
    
    uint16_t adcValue1;     // 4
    double ioVoltage1;      // 6
    double calcedValue1;    // 10
    
    uint16_t adcValue2;     // 14
    double ioVoltage2;      // 16
    double calcedValue2;    // 20
    
    uint16_t adcValue3;     // 24
    double ioVoltage3;      // 26
    double calcedValue3;    // 30
    
    uint16_t adcValue4;     // 34
    double ioVoltage4;      // 36
    double calcedValue4;    // 40
    
} UI_USB_DATA;

// Structure of USB 
typedef struct
{
    /* The application's current state */
    USB_STATES state;
    UI_USB_DATA* val2PC;
    USB_AUTO_STATES state_A;
    uint8_t typeID;
    uint8_t autoMode;
    
      /* Device layer handle returned by device layer open function */
    USB_DEVICE_HANDLE  usbDevHandle;

    /* Recieve data buffer */
    uint8_t * receiveDataBuffer;

    /* Transmit data buffer */
    uint8_t * transmitDataBuffer;

    /* Device configured */
    bool deviceConfigured;

    /* Send report transfer handle*/
    USB_DEVICE_HID_TRANSFER_HANDLE txTransferHandle;

    /* Receive report transfer handle */
    USB_DEVICE_HID_TRANSFER_HANDLE rxTransferHandle;

    /* Configuration value selected by the host*/
    uint8_t configurationValue;

    /* HID data received flag*/
    bool hidDataReceived;

    /* HID data transmitted flag */
    bool hidDataTransmitted;

    /* USB HID current Idle */
    uint8_t idleRate;

} USB_DATA;

// make stuff availabe in other components
extern USB_DATA usbData;
extern uint8_t USB_MAKE_BUFFER_DMA_READY receiveDataBuffer[USB_STACK_SIZE] ;
extern uint8_t USB_MAKE_BUFFER_DMA_READY transmitDataBuffer[USB_STACK_SIZE];
extern uint8_t USB_MAKE_BUFFER_DMA_READY transmitVal2PCbuffer[USB_STACK_SIZE];

// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// init and task USB function prototypes
void initUSB ( void );
void taskUSB ( void );

/* Send things to short the code */
void sendUiUSB ( void );

//void USB_AutoSend_Task ( void );

// this has to be add to the code
extern const USB_DEVICE_FUNCTION_REGISTRATION_TABLE funcRegistrationTable[1];
extern const USB_DEVICE_MASTER_DESCRIPTOR usbMasterDescriptor;

#endif /* _USB_H */

/* *****************************************************************************
 End of File
 */
