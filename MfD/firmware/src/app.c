/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Rosemann
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
#include "app.h"

#include "adc.h"
//#include "can.h"
//#include "ic.h"
#include "spi.h"
#include "usbHID.h"

// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
APP_DATA appData;

// counter variables
// TODO: check if specific int 32- or 16- or 8-bit are better
int i;
int j;


// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */
void initApp ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
    // Programming done LED
    OnBoardLED_2On();
    
//    // Turn off Relais
//    appData.b_relais1_state = Relais_nGate1Off();
//    appData.b_relais2_state = Relais_nGate2Off();
//    appData.b_relais3_state = Relais_nGate3Off();
    
    // start up paramters for counter variables
    i = 0;
    j = 0;
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */
void taskApp ( void )
{
    /* Check the application's current state. */
    switch ( appData.state )
    {
        // Application's initial state.
        case APP_STATE_INIT:
        {
			printf("%c%s%c%s", 27, "[2J", 27,"[H"); //clearscreen!!!
            printf("######################################\n");
            printf("####                              ####\n");
            printf("####          M  F  D  yo         ####\n");
            printf("####                              ####\n");
            printf("######################################\r\n");
            printf("Version \t%d.%d\n", (uint8_t)VERSION, ((uint16_t)(VERSION * 10)) % 10);
            printf("Build date \t%s - %s\n", __DATE__, __TIME__);
            printf("APP: Initialization started:\n");
			
            /* Init all components */
            initADC();
//            IC_Init();
//            CAN_Init();
            initSpiTemperatureSensor();
            initSpiKtype();
            initUSB();
            
            /* swap state */
            appData.state = APP_STATE_SERVICE_TASKS;
            break;
        }

        // Application's service task.
        case APP_STATE_SERVICE_TASKS:
        {
            /* Play USB Task */
            taskUSB();
//            printf("USB task finished |");
            
            // Play ADC Task
            //TODO: Initiate a Timer for triggering the read out cycle
            if (i > 500000)
            {
                /* add tasks */
                taskADC();
//                CAN_Task();
                taskSpiKtype();
                taskSpiTemperatureSensor();
//                IC_Task();
                                
                /* Control the print out on uart */
                if (j !=0 )
                {
                    printf("\n%i: ", j);
                }
                j++;
                
                // LED toggle showing system is alive
                OnBoardLED_2Toggle();
                
                /* Reset counter variable */
                i=0;
            }
            i++;
            
            break;
        }
        
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            printf("Something went wrong!!\n");
            break;
        }
    }
}

/*******************************************************************************
 End of File
 */
