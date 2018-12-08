/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Rosemann

  @File Name
    adc.c

  @Summary
    ADC implementation.

  @Description
 There are 15 ADC Channel with the following configuration.
 * (Channel Instance 01) - ADCHS_AN0:  ADC_TempSen4
 * (Channel Instance 02) - ADCHS_AN1:  ADC_TempSen3
 * (Channel Instance 03) - ADCHS_AN2:  ADC_TempSen2
 * (Channel Instance 04) - ADCHS_AN3:  ADC_TempSen1
 * (Channel Instance 05) - ADC Channel 7
// * (Instance 00) - ADCHS_AN11: ADC_Gyro_X
 * (Instance 00) - ADCHS_AN19: ADC_Gyro_X
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

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include "adc.h"
#include "app.h"
#include "driver/adc/drv_adc_static.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data or Defines                              */
/* ************************************************************************** */
/* ************************************************************************** */
#define VCC_REF_3V3     3.3
#define ADC_MAX_VALUE   4096

/* These are the values with max range of the supply lines*/
#define VCC_3V3_MAX     3.65
#define VCC_5V0_MAX     7.3
#define VCC_12V_MAX     16.3

#define VCC_PS_MAX      5.34
#define VCC_MPS_MAX     3.3

ADC_DATA adcData;

/* ************************************************************************** */
// Section: Globle Functions                                                  */
/* ************************************************************************** */
void initADC()
{
    /* Enable ADC */
    DRV_ADC0_Open();
    DRV_ADC1_Open();
    DRV_ADC2_Open();
    DRV_ADC3_Open();
    DRV_ADC4_Open();
    
    /* swap state */
    adcData.state = ADC_STATE_INIT;
    
    printf("initADC() done!\n");
}

void taskADC()
{   
    switch ( adcData.state)
    {
        case ADC_STATE_INIT:
        {
            /* swap state */
            adcData.state = ADC_STATE_SERVICE_TASKS;
            break;
        }
            
        case ADC_STATE_SERVICE_TASKS:
        {
            /* Read Voltage */
            getAdcVin();
            getAdc5V0();
            getAdc3V3();
            
            /* Read Temperature Sensors */
            getAdcTemperatureSensorValue(ADC_TS1_idx);
            getAdcTemperatureSensorValue(ADC_TS2_idx);
            getAdcTemperatureSensorValue(ADC_TS3_idx);
            getAdcTemperatureSensorValue(ADC_TS4_idx);
            
            /* Read Gyro */
            getAdcAccelerometerValues();
            
            /* Read Air Temperature Sensors */
            getAdcAirTemperatureSensorValue(ADC_AS1_idx);
            getAdcAirTemperatureSensorValue(ADC_AS2_idx);
            getAdcAirTemperatureSensorValue(ADC_AS3_idx);
            
            /* Read Pressure Sensors */
            getAdcPressureSensorValue(ADC_PS1_idx);
            getAdcPressureSensorValue(ADC_PS2_idx);
            
            break;
        }
        case ADC_STATE_ERROR:
        {
            printf("ADC state Error | ");
            break;
        }
        default:
        {
            printf("ADC state default | ");
            break;
        }
    }
}

/* ************************************************************************** */
// Section: Functions                                                         */
/* ************************************************************************** */
double getAdcTemperatureSensorValue(uint8_t bufindex)
{
    static double res1 = 100;   //100ohm
    
    /* Start ADC */
    DRV_ADC_Start();
    
    /* There are 4 adc channels for temperature Sensor 1 to 4  */
    if (bufindex >= 0 && bufindex < 4)
    {
        /* check if samples are availabe */
        while(!DRV_ADC_SamplesAvailable(ADCHS_AN0 + bufindex)){;}
        /* read availabe sample from ADC */
        adcData.u16_adcValue = DRV_ADC_SamplesRead(ADCHS_AN0 + bufindex);
        
        /* calc the actual IO voltage */
        adcData.d_ioVoltage = VCC_REF_3V3 / ADC_MAX_VALUE * adcData.u16_adcValue;
        /* calc the resistance R2 out of it */
        adcData.d_res2 = res1 * ((VCC_REF_3V3/(VCC_REF_3V3-adcData.d_ioVoltage))-1);
        
        /* Calc temperature values */
        if      (adcData.d_res2 > 27 && adcData.d_res2 < 68)
        {
            adcData.d_temperature = -(0.83 * (adcData.d_res2)) + 176.67;
        }
        else if (adcData.d_res2 >= 68 && adcData.d_res2 < 120)
        {
            adcData.d_temperature = -(0.39 * (adcData.d_res2)) + 146.15;
        }
        else if (adcData.d_res2 >= 120 && adcData.d_res2 < 224)
        {
            adcData.d_temperature = -(0.19 * (adcData.d_res2)) + 123.08;
        }
        else if (adcData.d_res2 >= 224 && adcData.d_res2 < 438)
        {
            adcData.d_temperature = -(0.09 * (adcData.d_res2)) + 100.93;
        }
        else if (adcData.d_res2 >= 438 && adcData.d_res2 < 950) // eig 925 - 25 for savety
        {
            adcData.d_temperature = -(0.04 * (adcData.d_res2)) + 77.99;
        }
        else
        {
            adcData.d_temperature = 0;
        }
        
        if (adcData.d_temperature != 0)
        {
            printf("TS%u: %.2f C | ", 4-bufindex, adcData.d_temperature);
        }
    }
    else
    {
        printf(" -- FAILURE: -- ADC channel not available! \n");
    }
    
    return adcData.d_temperature;
}

double getAdcTemperatureSensorResistorValue(uint8_t bufindex)
{
    static double res1 = 100;   //100ohm
    
    /* Start ADC */
    DRV_ADC_Start();
    
    /* There are 4 adc channels for temperature Sensor 1 to 4  */
    if (bufindex >= 0 && bufindex < 4)
    {
        /* check if samples are availabe */
        while(!DRV_ADC_SamplesAvailable(ADCHS_AN0 + bufindex)){;}
        /* read availabe sample from ADC */
        adcData.u16_adcValue = DRV_ADC_SamplesRead(ADCHS_AN0 + bufindex);
        
        /* calc the actual IO voltage */
        adcData.d_ioVoltage = VCC_REF_3V3 / ADC_MAX_VALUE * adcData.u16_adcValue;
        /* calc the resistance R2 out of it */
        adcData.d_res2 = res1 * ((VCC_REF_3V3/(VCC_REF_3V3-adcData.d_ioVoltage))-1);
        
        /* print out value for Debug */
        printf("TS%u: %.2fOhm | ", 4-bufindex, adcData.d_res2);
    }
    else
    {
        printf(" -- FAILURE: -- ADC channel not available! \n");
    }
    
    /* Return resistor value*/
    return adcData.d_res2;
}


void getAdcAccelerometerValues(void)
{    
    /* Start ADC */
    DRV_ADC_Start();
    
    /* check if samples are availabe */
    while(!DRV_ADC_SamplesAvailable(ADCHS_AN12)){;}
    /* read availabe sample from ADC */
    adcData.u16_gyro_x = DRV_ADC_SamplesRead(ADCHS_AN19);
    adcData.u16_gyro_y = DRV_ADC_SamplesRead(ADCHS_AN12);
    adcData.u16_gyro_z = DRV_ADC_SamplesRead(ADCHS_AN13);
    
    printf("X(%u) | Y(%u) | Z(%u) | ", adcData.u16_gyro_x, adcData.u16_gyro_y, adcData.u16_gyro_z);
    //printf(" -- FAILURE: -- ADC channel not available! \n");
}


double getAdcAirTemperatureSensorValue(uint8_t bufindex)
{
    static double res1 = 1000;  //1kohm
     
    /* Start ADC */
    DRV_ADC_Start();
    
    /* There are 3 adc channels for temperature Sensor
     * 0 to 2  */
    if (bufindex >= 0 && bufindex < 3)
    {
        /* check if samples are availabe */
        while(!DRV_ADC_SamplesAvailable(ADCHS_AN15 + bufindex)){;}
        /* read availabe sample from ADC */
        adcData.u16_adcValue = DRV_ADC_SamplesRead(ADCHS_AN15 + bufindex);
        
        /* calc the actual IO voltage */
        adcData.d_ioVoltage = VCC_REF_3V3 / ADC_MAX_VALUE * adcData.u16_adcValue;
        /* calc the resistance R2 out of it */
        adcData.d_res2 = res1 * ((VCC_REF_3V3/(VCC_REF_3V3-adcData.d_ioVoltage))-1);
         
        /* Try to calc temperature value */
        if      (adcData.d_res2 > 300 && adcData.d_res2 < 599)
        {
            adcData.d_temperature = -(0.11 * (adcData.d_res2)) + 183.5;
        }
        else if (adcData.d_res2 >= 599 && adcData.d_res2 < 1268)
        {
            adcData.d_temperature = -(0.045 * (adcData.d_res2)) + 146.86;
        }
        else if (adcData.d_res2 >= 1268 && adcData.d_res2 < 3020)
        {
            adcData.d_temperature = -(0.017 * (adcData.d_res2)) + 111.71;
        }
        else if (adcData.d_res2 >= 3020 && adcData.d_res2 < 5830)
        {
            adcData.d_temperature = -(0.007 * (adcData.d_res2)) + 81.5;
        }
        else if (adcData.d_res2 >= 5830 && adcData.d_res2 < 62350) // eig 62300 - 50 für savety
        {
            adcData.d_temperature = -(0.001 * (adcData.d_res2)) + 45.99;
        }
        else
        {
            adcData.d_temperature = 0;
        }
        
        //printf("AirSen%i: %f ohm, %f degree\n", bufindex+1, adcData.d_res2, temp_val);
        if (adcData.d_temperature != 0)
        {
            printf("AS%u: %.2f C(%.2f) | ", bufindex+1, adcData.d_temperature, adcData.d_res2);
        }
    }
    else
    {
        printf(" -- FAILURE: -- ADC channel not available! \n");
    }
    
    /* Return value */
    return adcData.d_temperature;
}

double getAdcAirTemperatureSensorResistorValue(uint8_t bufindex)
{
    static double res1 = 1000;  //1kohm
     
    /* Start ADC */
    DRV_ADC_Start();
    
    /* There are 3 adc channels for temperature Sensor
     * 0 to 2  */
    if (bufindex >= 0 && bufindex < 3)
    {
        /* check if samples are availabe */
        while(!DRV_ADC_SamplesAvailable(ADCHS_AN15 + bufindex)){;}
        /* read availabe sample from ADC */
        adcData.u16_adcValue = DRV_ADC_SamplesRead(ADCHS_AN15 + bufindex);
        
        /* calc the actual IO voltage */
        adcData.d_ioVoltage = VCC_REF_3V3 / ADC_MAX_VALUE * adcData.u16_adcValue;
        /* calc the resistance R2 out of it */
        adcData.d_res2 = res1 * ((VCC_REF_3V3/(VCC_REF_3V3-adcData.d_ioVoltage))-1);
        
        printf("AS%u: %.2fOhm | ", bufindex+1, adcData.d_res2);
    }
    else
    {
        printf(" -- FAILURE: -- ADC channel not available! \n");
    }
    
    /* Return value */
    return adcData.d_res2;
}

double getAdcPressureSensorValue(uint8_t bufindex)
{
    /* Start ADC */
    DRV_ADC_Start();
    
    /* There are 2 adc channels for temperature Sensor
     * 0 to 1  */
    if (bufindex >= 0 && bufindex < 2)
    {
        /* check if samples are availabe */
        while(!DRV_ADC_SamplesAvailable(ADCHS_AN30 + bufindex)){;}
        /* read availabe sample from ADC */
        adcData.u16_adcValue = DRV_ADC_SamplesRead(ADCHS_AN30 + bufindex);
        
        /* calc the actual IO voltage */
        adcData.d_ioVoltage = VCC_REF_3V3 / ADC_MAX_VALUE * adcData.u16_adcValue;
        
        /* Calc real 5V0 voltage */
        adcData.d_voltage = VCC_PS_MAX / VCC_REF_3V3 * adcData.d_ioVoltage;
        
        /* Try to calc temperature value */
        adcData.d_bar = 1.9981 * adcData.d_voltage;
        adcData.d_psi = adcData.d_bar / 0.0689;
        
        if(adcData.d_bar < 1)
        {
            adcData.d_bar = 0;
            adcData.d_psi = 0;
        }
        
        //printf("PresSen%i: %.2f ioVoltage, %.2f pressure(%u)\n", 2-bufindex, adcData.d_ioVoltage, adcData.d_bar, adcData.u16_adcValue);
        if ( adcData.d_psi != 0 )
        {
            printf("PS%u: %.0fPSI | ", 2-bufindex, adcData.d_psi);
        }
    }
    else
    {
        printf(" -- FAILURE: -- ADC channel not available! \n");
    }
    
    return adcData.d_psi;   
}

double getAdcPressureSensorVoltageValue(uint8_t bufindex)
{
    /* Start ADC */
    DRV_ADC_Start();
    
    /* There are 2 adc channels for temperature Sensor
     * 0 to 1  */
    if (bufindex >= 0 && bufindex < 2)
    {
        /* check if samples are availabe */
        while(!DRV_ADC_SamplesAvailable(ADCHS_AN30 + bufindex)){;}
        /* read availabe sample from ADC */
        adcData.u16_adcValue = DRV_ADC_SamplesRead(ADCHS_AN30 + bufindex);
        
        /* calc the actual IO voltage */
        adcData.d_ioVoltage = VCC_REF_3V3 / ADC_MAX_VALUE * adcData.u16_adcValue;
        
        /* Calc real 5V0 voltage */
        adcData.d_voltage = VCC_PS_MAX / VCC_REF_3V3 * adcData.d_ioVoltage;
        
        printf("PS%u: %.0fVoltage | ", 2-bufindex, adcData.d_voltage);
    }
    else
    {
        printf(" -- FAILURE: -- ADC channel not available! \n");
    }
    
    return adcData.d_voltage;   
}


double getAdcManifoldPressureSensorValue(uint8_t bufindex)
{
    /* Start ADC */
    DRV_ADC_Start();
    
    /* There are 2 adc channels for temperature Sensor
     * 0 to 1  */
    if (bufindex >= 0 && bufindex < 1)
    {
        /* check if samples are availabe */
        while(!DRV_ADC_SamplesAvailable(ADCHS_AN23 + bufindex)){;}
        /* read availabe sample from ADC */
        adcData.u16_adcValue = DRV_ADC_SamplesRead(ADCHS_AN23 + bufindex);
        
        /* calc the actual IO voltage */
        adcData.d_ioVoltage = VCC_REF_3V3 / ADC_MAX_VALUE * adcData.u16_adcValue;
        
        /* Calc real 3V3 voltage */
        adcData.d_voltage = VCC_MPS_MAX / VCC_REF_3V3 * adcData.d_ioVoltage;
        
        /* Try to calc temperature value */
        adcData.d_bar = 1.9981 * adcData.d_voltage;
        adcData.d_psi = adcData.d_bar / 0.0689;
        
        if(adcData.d_bar < 1)
        {
            adcData.d_bar = 0;
            adcData.d_psi = 0;
        }
        
        //printf("PresSen%i: %.2f ioVoltage, %.2f pressure(%u)\n", 2-bufindex, adcData.d_ioVoltage, adcData.d_bar, adcData.u16_adcValue);
        if ( adcData.d_bar != 0 )
        {
            printf("PS%u: %.1fbar | ", 2-bufindex, adcData.d_bar);
        }
    }
    else
    {
        printf(" -- FAILURE: -- ADC channel not available! \n");
    }
    
    return adcData.d_bar;
}

double getAdcManifoldPressureSensorVoltageValue(uint8_t bufindex)
{
    /* Start ADC */
    DRV_ADC_Start();
    
    /* There are 2 adc channels for temperature Sensor
     * 0 to 1  */
    if (bufindex >= 0 && bufindex < 1)
    {
        /* check if samples are availabe */
        while(!DRV_ADC_SamplesAvailable(ADCHS_AN23 + bufindex)){;}
        /* read availabe sample from ADC */
        adcData.u16_adcValue = DRV_ADC_SamplesRead(ADCHS_AN23 + bufindex);
        
        /* calc the actual IO voltage */
        adcData.d_ioVoltage = VCC_REF_3V3 / ADC_MAX_VALUE * adcData.u16_adcValue;
    }
    else
    {
        printf(" -- FAILURE: -- ADC channel not available! \n");
    }
}


double getAdc3V3(void)
{
    /* Start ADC */
    DRV_ADC_Start();
    
    /* check if samples are availabe */
    while(!DRV_ADC_SamplesAvailable(ADCHS_AN32)){;}
    /* read availabe sample from ADC */
    adcData.u16_adcValue = DRV_ADC_SamplesRead(ADCHS_AN32);
    
    /* calc the actual IO voltage */
    adcData.d_ioVoltage = VCC_REF_3V3 / ADC_MAX_VALUE * adcData.u16_adcValue;
    
    /* Calc real 5V0 voltage */
    adcData.d_voltage = VCC_3V3_MAX / VCC_REF_3V3 * adcData.d_ioVoltage;
    
    printf("3V3: %.1f | ", adcData.d_voltage);
    
    return adcData.d_voltage;
}


double getAdc5V0(void)
{
    /* Start ADC */
    DRV_ADC_Start();
    
    /* check if samples are availabe */
    while(!DRV_ADC_SamplesAvailable(ADCHS_AN33)){;}
    /* read availabe sample from ADC */
    adcData.u16_adcValue = DRV_ADC_SamplesRead(ADCHS_AN33);
    
    /* calc the actual IO voltage */
    adcData.d_ioVoltage = VCC_REF_3V3 / ADC_MAX_VALUE * adcData.u16_adcValue;
    
    /* Calc real 5V0 voltage */
    adcData.d_voltage = VCC_5V0_MAX / VCC_REF_3V3 * adcData.d_ioVoltage;
    
    //printf("5V: %f ioVoltage, %f voltage\n", adcData.d_ioVoltage, voltage);
//    printf("5V0: %.1f | ", adcData.d_voltage);
    
    return adcData.d_voltage;
}


double getAdcVin(void)
{
    /* Start ADC */
    DRV_ADC_Start();
    
    /* check if samples are availabe */
    while(!DRV_ADC_SamplesAvailable(ADCHS_AN34)){;}
    /* read availabe sample from ADC */
    adcData.u16_adcValue = DRV_ADC_SamplesRead(ADCHS_AN34);
    
    /* calc the actual IO voltage */
    adcData.d_ioVoltage = VCC_REF_3V3 / ADC_MAX_VALUE * adcData.u16_adcValue;
    
    /* Calc real Vin voltage */
    adcData.d_voltage = VCC_12V_MAX / VCC_REF_3V3 * adcData.d_ioVoltage;
    
    /* print out value */
//    printf("Vin: %.1f | ", adcData.d_voltage);
    //printf("\n----Vin: %.2f ioV, %.2f V (%u)\n", adcData.d_ioVoltage, voltage, adcData.u16_adcValue);
    
    return adcData.d_voltage;
}

/* *****************************************************************************
 End of File
 */
