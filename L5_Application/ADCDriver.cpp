#include <stdint.h>

#include "ADCDriver.hpp"
#include "LPC17xx.h"

const float ADCDriver::V_REF = 3.3;

ADCDriver::ADCDriver()
{

}

void ADCDriver::adcInitBurstMode()
{
//    /* Power on ADC */
//    LPC_SC->PCONP |= (1 << 12);
//    LPC_SC->PCLKSEL0 |= (3 << 24);
//    LPC_ADC->ADCR &= ~(0xFF << 8);
//    /* Enable ADC */
//    LPC_ADC->ADCR |= (1 << 21);
//    /* Start burst mode */
//    LPC_ADC->ADCR &= ~(7 << 24);
//    LPC_ADC->ADCR |= (1 << 16);
    LPC_SC->PCONP |= (1 << 12);
    LPC_SC->PCLKSEL0 |= (3 << 24);
    LPC_PINCON->PINSEL1 &= ~(3 << 18);
    LPC_PINCON->PINSEL1 |= (1 << 18);
    LPC_ADC->ADCR |= (1 << 21);
    LPC_ADC->ADCR |= (2 << 8);
    LPC_ADC->ADINTEN &= ~(1 << 8);
    LPC_ADC->ADCR |= (1 << 16);

}

void ADCDriver::adcSelectPin(ADC_PIN adc_pin_arg)
{
    switch(adc_pin_arg)
    {
        case ADC_PIN_0_25:
            /* AD0.2 */
            LPC_ADC->ADCR |= (1 << 2);
            LPC_PINCON->PINSEL1 &= ~(3 << 18);
            LPC_PINCON->PINSEL1 |= (1 << 18);
            break;

        case ADC_PIN_0_26:
            /* AD0.3 */
            LPC_ADC->ADCR |= (1 << 3);
            LPC_PINCON->PINSEL1 &= ~(3 << 20);
            LPC_PINCON->PINSEL1 |= (1 << 20);
            break;

        case ADC_PIN_1_30:
            /* AD0.4 */
            LPC_ADC->ADCR |= (1 << 4);
            LPC_PINCON->PINSEL3 |= (3 << 28);
            break;

        case ADC_PIN_1_31:
            /* AD0.5 */
            LPC_ADC->ADCR |= (1 << 5);
            LPC_PINCON->PINSEL3 |= (3 << 30);
            break;

        default:
            break;
    }
}

float ADCDriver::readADCVoltageByChannel(uint8_t adc_channel_arg)
{
    LPC_ADC->ADCR &= ~( (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) );         //clear all pre
    LPC_ADC->ADCR |= (1 << adc_channel_arg);
    uint16_t result = (LPC_ADC->ADGDR >> 4) & (0xFFF);

    return result;
}
