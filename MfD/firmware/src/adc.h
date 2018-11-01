/* ************************************************************************** */
/** Descriptive File Name

  @Company
 Rosemann GmbH

  @File Name
 adc.h

  @Summary
 ADC header.

  @Description
 There are 15 ADC Channel with the following configuration.
 * (Channel Instance 01) - ADCHS_AN0:  ADC_TempSen4
 * (Channel Instance 02) - ADCHS_AN1:  ADC_TempSen3
 * (Channel Instance 03) - ADCHS_AN2:  ADC_TempSen2
 * (Channel Instance 04) - ADCHS_AN3:  ADC_TempSen1
 * (Channel Instance 05) - ADC Channel 7
 * (Instance 00) - ADCHS_AN11: ADC_Gyro_X
 * (Instance 01) - ADCHS_AN12: ADC_Gyro_Y
 * (Instance 02) - ADCHS_AN13: ADC_Gyro_Z
 * (Instance 03) - ADCHS_AN15: ADC_AirSen1
 * (Instance 04) - ADCHS_AN16: ADC_AirSen2
 * (Instance 05) - ADCHS_AN17: ADC_AirSen3
 * (Instance 06) - ADCHS_AN30: ADC_PressSen2
 * (Instance 07) - ADCHS_AN31: ADC_PressSen1
 * (Instance 08) - ADCHS_AN32: ADC_3V3
 * (Instance 09) - ADCHS_AN33: ADC_5V
 * (Instance 10) - ADCHS_AN34: ADC_Vin
 * (Instance 11) - ADCHS_AN23: ADC_ManiPressSen1
 */
/* ************************************************************************** */

#ifndef ADC_H    /* Guard against multiple inclusion */
#define ADC_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* ************************************************************************** */
/* Section: Defines                                                           */
/* ************************************************************************** */
#define ADC_TS1_idx     3
#define ADC_TS2_idx     2
#define ADC_TS3_idx     1
#define ADC_TS4_idx     0
 
#define ADC_AS1_idx     0
#define ADC_AS2_idx     1
#define ADC_AS3_idx     2

#define ADC_PS1_idx     1
#define ADC_PS2_idx     0
 
#define ADC_MPS_idx     0

// *****************************************************************************
/* ADC states

  Summary:
    ADC states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/
typedef enum
{
	/* Application's state machine's initial state. */
	ADC_STATE_INIT=0,
	ADC_STATE_SERVICE_TASKS,
    ADC_STATE_ERROR,
} ADC_STATES;

// *****************************************************************************
/* ADC Data

  Summary:
    Holds ADC data

  Description:
    This structure holds the ADC's data.

  Remarks:
    ADC strings and buffers are be defined outside this structure.
 */
typedef struct
{
    /* The ADC's current state */
    ADC_STATES state;

    /* general variables */
    uint16_t u16_adcValue;
    uint16_t u16_gyro_x;
    uint16_t u16_gyro_y;
    uint16_t u16_gyro_z;
    double d_ioVoltage;
    double d_voltage;
    double d_res2;
    double d_temperature;
    double d_bar;
    double d_psi;
    
    /* Air Sensor*/
    uint16_t u16_AS_adcValue;
    double d_AS_ioVoltage;
    double d_AS_res2;
    
} ADC_DATA_1;

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Function prototypes                                                           */
/* ************************************************************************** */
/* ************************************************************************** */
void ADC_Init(void);
void ADC_Task(void);

/* Temperature Sensor - Oil, Water, etc. */
double ADC_Read_TS_Temperature(uint8_t);
double ADC_Read_TS_Resistor(uint8_t);

/* Accelormeter */
void ADC_Read_Gyro(void);

/* Air Temperature */
double ADC_Read_AS_Temperature(uint8_t);
double ADC_Read_AS_Resistor(uint8_t);

/* Pressure Sensor */
double ADC_Read_PS_Press(uint8_t);
double ADC_Read_PS_Voltage(uint8_t);

/* Manifold pressure Sensor */
double ADC_Read_MPS_Press(uint8_t);
double ADC_Read_MPS_Voltage(uint8_t);


double ADC_Read_3V3(void);
double ADC_Read_5V0(void);
double ADC_Read_Vin(void);
#endif /* ADC_H */

/* *****************************************************************************
 End of File
 */
