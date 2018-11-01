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
//#include "spi.h"
//#include "usb.h"
//#include "bsp_config.h"

// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
APP_DATA appData;

// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */
void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

int i = 0;
int j = 0;
/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */
void APP_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
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
            printf("APP: initialization started | ");
			
            /* Programming done LED */
            OnBoardLED_2On();
            
            /* Init all components */
            ADC_Init();
            //IC_Init();
//            CAN_Init();
//            SPI_TempSen_Init();
//            SPI_KType_Init();
//            USB_Initialize();
            
            /* Set or Reset Relais */
//            appData.b_relais1_state = BSP_nRelais_En(BSP_Relais_nGate1, false);
//            appData.b_relais2_state = BSP_nRelais_En(BSP_Relais_nGate2, false);
//            appData.b_relais3_state = BSP_nRelais_En(BSP_Relais_nGate3, false);
            
            /* swap state */
            appData.state = APP_STATE_SERVICE_TASKS;
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            /* Play USB Task */
//            USB_Task();
            //printf("USB task finished |");
            
            /* Play ADC Task */
            if (i > 500000)
            {
                /* add tasks */
                ADC_Task();
                //CAN_Task();
//                SPI_KType_Task();
//                SPI_TempSen_Task();
                //IC_Task();
                                
                /* Control the print out on uart */
                if (j !=0 )
                {
                    printf("\n%i: ", j);
                }
                j++;
                
                /* Toggle LED for check program is running */
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
            break;
        }
    }
}

/*******************************************************************************
 End of File
 */
