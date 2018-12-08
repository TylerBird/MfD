/* ************************************************************************** */
/** Descriptive File Name

  @Company
 Rosemann GmbH

  @File Name
 spi.h

  @Summary
 SPI header.

  @Description
 In this file the SPI components are implemented.
 There are two SPI used: 1 for read the operating temperature
                         2 for read out the K-Type temperature sensor
 */
/* ************************************************************************** */

#ifndef SPI_H    /* Guard against multiple inclusion */
#define SPI_H

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Include Files                                                     */
/* ************************************************************************** */
/* ************************************************************************** */
#include <app.h>

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Defines                                                           */
/* ************************************************************************** */
/* ************************************************************************** */
#define SPI_OnTempSen_nCS_SELECT()      SPI1_nCS_TempSOff(); //BSP_SPI_nCS_TempS_En(true)
#define SPI_OnTempSen_nCS_DESELECT()    SPI1_nCS_TempSOn(); //BSP_SPI_nCS_TempS_En(false)
#define SPI_KType_nCS_SELECT()          SPI2_nCS_KTypeOff(); //BSP_SPI_nCS_KType_En(true)
#define SPI_KType_nCS_DESELECT()        SPI2_nCS_KTypeOn(); //BSP_SPI_nCS_KType_En(false)
#define MAX_NUM_OF_BYTES                100

typedef unsigned char SPI_DATA_TYPE;

// *****************************************************************************
/* SPI states

  Summary:
    SPI states enumeration

  Description:
    This enumeration defines the valid SPI states.  These states
    determine the behavior of the SPI at various times.
*/
typedef enum
{
	/* SPI's state machine's initial state. */
	SPI_STATE_OTS_GET_DATA=0,
    SPI_STATE_OTS_BUSY_CHECK,
    SPI_STATE_OTS_ERROR,
} SPI_OTS_STATES;

typedef enum
{
	/* SPI's state machine's initial state. */
	SPI_STATE_KT_GET_DATA=0,
    SPI_STATE_KT_BUSY_CHECK,
    SPI_STATE_KT_ERROR,
} SPI_KT_STATES;

// *****************************************************************************
/* SPI Data

  Summary:
    Holds SPI data

  Description:
    This structure holds the SPI's data.

  Remarks:
    SPI strings and buffers are be defined outside this structure.
 */
typedef struct
{
    /* The ADC's current state */
    SPI_OTS_STATES ots_state;
    SPI_KT_STATES  kt_state;

    /* SPI Driver Handle  */
    DRV_HANDLE              drvSPIHandle;
    DRV_HANDLE              drvSPIHandle_KT;
    /* Read buffer handle */
    DRV_SPI_BUFFER_HANDLE   drvSPIRDBUFHandle;
    DRV_SPI_BUFFER_HANDLE   drvSPIRDBUFHandle_KT;
    /* SPI Driver RX buffer  */
    SPI_DATA_TYPE           drvSPIRXbuffer[MAX_NUM_OF_BYTES];
    SPI_DATA_TYPE           drvSPIRXbuffer_KT[MAX_NUM_OF_BYTES];
   
    unsigned char data[MAX_NUM_OF_BYTES];
    unsigned char data_KT[MAX_NUM_OF_BYTES];
    
    uint8_t pre_position;
    uint8_t post_position;
    
} SPI_DATA;

extern SPI_DATA spiData;
/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Function prototypes                                               */
/* ************************************************************************** */
/* ************************************************************************** */
void initSpiTemperatureSensor(void);
void taskSpiTemperatureSensor(void);

void initSpiKtype(void);
void taskSpiKtype(void);

void SPI_Read(SPI_DATA_TYPE *rxbuffer, uint32_t num_of_bytes);
void SPI_KT_Read(SPI_DATA_TYPE *rxbuffer, uint32_t num_of_bytes);
uint8_t SPI_Check_Transfer_Status(DRV_SPI_BUFFER_HANDLE drvBufferHandle);

#endif /* SPI_H */

/* *****************************************************************************
 End of File
 */
