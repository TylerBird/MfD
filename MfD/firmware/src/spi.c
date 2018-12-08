/* ************************************************************************** */
/** Descriptive File Name

  @Company
 Rosemann GmbH

  @File Name
 spi.c

  @Summary
 SPI implementation.

  @Description
 In this file the SPI components are implemented.
 There are two SPI used: 1 for read the operating temperature
                         2 for read out the K-Type temperature sensor
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include "spi.h"
#include "app.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */
SPI_DATA spiData;

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */
void initSpiTemperatureSensor( void )
{
    /* Open the SPI Driver */
    spiData.drvSPIHandle = DRV_SPI_Open(DRV_SPI_INDEX_0,DRV_IO_INTENT_READ );
    if(spiData.drvSPIHandle != DRV_HANDLE_INVALID)
    {
        printf("initSpiTemperatureSensor() done!\n");
    }
    
    /* swap state */
    spiData.ots_state = SPI_STATE_OTS_GET_DATA;
}

void initSpiKtype( void )
{
    /* Open the SPI Driver */
    spiData.drvSPIHandle_KT = DRV_SPI_Open(DRV_SPI_INDEX_1,DRV_IO_INTENT_READ );
    if(spiData.drvSPIHandle_KT != DRV_HANDLE_INVALID)
    {
        printf("initSpiKtype() done!\n");
    }
//    spiData.drvSPIHandle = DRV_SPI_Open(DRV_SPI_INDEX_1,DRV_IO_INTENT_READ );
//    if(spiData.drvSPIHandle != DRV_HANDLE_INVALID)
//    {
//        printf("SPI_KType_Init finished |\n");
//    }
    
    /* swap state */
    spiData.kt_state = SPI_STATE_KT_GET_DATA;
}

/* This function reads the temperature out of the temperature IC via SPI
 * and waits until the conversation is finished.
 * TODO: add an errorCounter for more than 10 read trys to avoid stuck in
 * while in failure case
 * The Return value is a 8-bit pointer.
 * in byte[0] is the pre-postion of the temperature converted to decimal
 * in byte[1] is the post-position converted in to decimal
 */
void taskSpiTemperatureSensor(void)
{
    uint16_t temperature;
    uint8_t  sign = 0;
//    uint8_t  pre_position;
//    uint8_t  post_position;
    
    switch ( spiData.ots_state )
    {
        case SPI_STATE_OTS_GET_DATA:
        {
            //printf("reached TS_GET_DATA | ");
            
            /* Enable SPI Chip Select for TemperatureSensor */
            SPI_OnTempSen_nCS_SELECT();

            /* Start Read out of Temperature Value */
            SPI_Read(&spiData.drvSPIRXbuffer[0], 2);
                
            int i;
            for(i=0; i<50; i++){;}
            
            /* swap state */
            spiData.ots_state = SPI_STATE_OTS_BUSY_CHECK;
            break;
        }
        case SPI_STATE_OTS_BUSY_CHECK:
        {
            //printf("reached TS_GET_DATA | ");
            
            if(SPI_Check_Transfer_Status(spiData.drvSPIRDBUFHandle))
            {
                /* Disable SPI Chip Select for TemperatureSensor*/
                SPI_OnTempSen_nCS_DESELECT();
                
                //printf("reached SPI_Check_Transfer_Status | ");
                
                /* save read buffer in variable for further calculations */
                temperature = (spiData.drvSPIRXbuffer[0] << 8) | spiData.drvSPIRXbuffer[1];
                /* check the most significant bit for minus or plus degreess */
                sign = (temperature >> 15);
                /* cut of the two lowest bits these are crap */
                temperature = (temperature >> 2);

                /* pre-decimal position */
                spiData.pre_position = 0x00FF & (temperature >> 5);
                /* post-decimal position */
                spiData.post_position = (0x001F & temperature);
                spiData.post_position = (spiData.post_position * 3125 / 1000);

                spiData.data[0] = spiData.pre_position;
                spiData.data[1] = spiData.post_position;

                printf("OnboardTemp: %u,%u | ", spiData.data[0], spiData.data[1]);

                /* Reset state and done variable */
                spiData.ots_state = SPI_STATE_OTS_GET_DATA;
                //return &spiData.data[0];
            }
            break;
        }
        
        case SPI_STATE_OTS_ERROR:
        {
            printf("OnTS state Error | ");
            break;
        }
        default:
        {
            printf("OnTS default | ");
            break;
        }
    }
}

void taskSpiKtype(void)
{
    uint32_t temperature;
    uint32_t thermoTemperature;
    uint32_t refTemperature;
    uint8_t  sign = 0;
    uint32_t pre_position;
    uint32_t post_position;
    uint32_t pre_position_ref;
    uint32_t post_position_ref;
    
    switch ( spiData.kt_state )
    {
        case SPI_STATE_KT_GET_DATA:
        {
            /* Enable SPI Chip Select for TemperatureSensor */
            SPI_KType_nCS_SELECT();

            /* Start Read out of Temperature Value */
            SPI_KT_Read(&spiData.drvSPIRXbuffer_KT[10], 4);
            
            int i;
            for(i=0; i<50; i++){;}
            
            /* swap state */
            spiData.kt_state = SPI_STATE_KT_BUSY_CHECK;
            break;
        }

        case SPI_STATE_KT_BUSY_CHECK:
        {
            if(SPI_Check_Transfer_Status(spiData.drvSPIRDBUFHandle_KT))
//            if(SPI_Check_Transfer_Status(spiData.drvSPIRDBUFHandle))
            {
                /* Disable SPI Chip Select for TemperatureSensor*/
                SPI_KType_nCS_DESELECT();

                /* save read buffer in variable for further calculations */
                temperature = ((spiData.drvSPIRXbuffer_KT[10] << 24) | 
                               (spiData.drvSPIRXbuffer_KT[11] << 16) |
                               (spiData.drvSPIRXbuffer_KT[12] <<  8) |
                               (spiData.drvSPIRXbuffer_KT[13] <<  0));
//                temperature = ((spiData.drvSPIRXbuffer[10] << 24) | 
//                               (spiData.drvSPIRXbuffer[11] << 16) |
//                               (spiData.drvSPIRXbuffer[12] <<  8) |
//                               (spiData.drvSPIRXbuffer[13] <<  0));

//                printf("%u:%u:%u:%u | ",
//                        spiData.drvSPIRXbuffer_KT[10],
//                        spiData.drvSPIRXbuffer_KT[11],
//                        spiData.drvSPIRXbuffer_KT[12],
//                        spiData.drvSPIRXbuffer_KT[13]);
//                printf("%u:%u:%u:%u | ",
//                        spiData.drvSPIRXbuffer[10],
//                        spiData.drvSPIRXbuffer[11],
//                        spiData.drvSPIRXbuffer[12],
//                        spiData.drvSPIRXbuffer[13]);
                
                /* check the most significant bit for minus or plus degrees */
                sign = 0x1 & (temperature >> 30);
                
//                printf("sign:%u", sign);
                
                /* save just the needed bytes without sign */
                thermoTemperature   = 0x1FFF & (temperature >> 18);
                refTemperature      = 0x07FF & (temperature >> 4);
                
//                printf("TT:%u RT:%u", thermoTemperature, refTemperature);
                
                
                /* calculate: THERMO-TEMPERATURE 
                 * negate all bits if it is an negative temperature
                 * for easier calucations */
                if (sign == 0)
                {
                    /* post-decimal position */
                    post_position = (0x0003 & thermoTemperature);
                    post_position = (post_position * 2500 / 100); // 2-digits after the comma
                    /* pre-decimal position */
                    pre_position = 0x07FF & (thermoTemperature >> 2);
                }
                else if (sign == 1)
                {
                    /* post-decimal position */
                    post_position = (0x0003 & thermoTemperature);
                    post_position = 100 - (post_position * 2500 / 100); // 2-digits after the comma

                    /* check if there is a post-comma position*/
                    if (post_position > 0)
                    {
                        pre_position = 0x07FF & (thermoTemperature >> 2);
                        pre_position = 2048 - pre_position - 1;
                    }
                    else
                    {
                        pre_position = 0x07FF & (thermoTemperature >> 2);
                        pre_position = 2048 - pre_position;
                    }
                }

                /* calculate: REFERENCE-TEMPERATURE 
                 * negate all bits if it is an negative temperature
                 * for easier calucations */
                if (sign == 0)
                {
                    /* post-decimal position */
                    post_position_ref = (0x000F & refTemperature);
                    post_position_ref = (post_position_ref * 6250 / 10); //4-digits after the comma
                    /* pre-decimal position */
                    pre_position_ref = 0x007F & (refTemperature >> 4);
                }
                else if (sign == 1)
                {
                    /* post-decimal position */
                    post_position_ref = (0x000F & refTemperature);
                    post_position_ref = 10000 - (post_position_ref * 6250 / 10); //4-digits after the comma

                    /* check if there is a post-comma position*/
                    if (post_position_ref > 0)
                    {
                        pre_position_ref = (0x007F & (refTemperature >> 4));
                        pre_position_ref = 128 - pre_position_ref - 1;
                    }
                    else
                    {
                        pre_position_ref = (0x007F & (refTemperature >> 4));
                        pre_position_ref = 128 - pre_position_ref;
                    }
                }

                /* save thermoTemperature in return value*/
                spiData.data_KT[0] = 0x00ff & (pre_position >> 8);
                spiData.data_KT[1] = 0x00ff & pre_position;
                spiData.data_KT[2] = 0x00ff & post_position;

                spiData.data_KT[3] = 0x00ff & pre_position_ref;
                spiData.data_KT[4] = 0x00ff & post_position_ref;
                
//                spiData.data[10] = 0x00ff & (pre_position >> 8);
//                spiData.data[11] = 0x00ff & pre_position;
//                spiData.data[12] = 0x00ff & post_position;
//
//                spiData.data[13] = 0x00ff & pre_position_ref;
//                spiData.data[14] = 0x00ff & post_position_ref;
                
                printf("KT: %u%u,%u ref: %u,%u | ", spiData.data_KT[0], spiData.data_KT[1],
                        spiData.data_KT[2],spiData.data_KT[3],spiData.data_KT[4]);
//                printf("KT: %u%u,%u ref: %u,%u | ", spiData.data[10], spiData.data[11],
//                        spiData.data[12],spiData.data[13],spiData.data[14]);
                
                /* Reset state and done variable */
                spiData.kt_state = SPI_STATE_KT_GET_DATA;
            }
            break;
        }
        case SPI_STATE_KT_ERROR:
        {
            printf("KT state Error | ");
            break;
        }
        default:
        {
            printf("KT default | ");
            break;
        }
    }
}


/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */
void SPI_Read(SPI_DATA_TYPE *rxbuffer, uint32_t num_of_bytes)
{
    /* Add the buffer pointer to read the data from EEPROM */
    spiData.drvSPIRDBUFHandle = DRV_SPI_BufferAddRead(spiData.drvSPIHandle,
            rxbuffer, num_of_bytes, 0, 0);
}

void SPI_KT_Read(SPI_DATA_TYPE *rxbuffer, uint32_t num_of_bytes)
{
    /* Add the buffer pointer to read the data from EEPROM */
    spiData.drvSPIRDBUFHandle_KT = DRV_SPI_BufferAddRead(spiData.drvSPIHandle_KT,
            rxbuffer, num_of_bytes, 0, 0);
}

uint8_t SPI_Check_Transfer_Status(DRV_SPI_BUFFER_HANDLE drvBufferHandle)
{
    if(DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus (drvBufferHandle))
    {
        return true;
    }
    else
    {
        return false;
    }
}


/* *****************************************************************************
 End of File
 */
