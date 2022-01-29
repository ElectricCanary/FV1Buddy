/*
 * FV1Buddy.c
 *
 * Created: 23/01/2022 00:51:16
 * Author : Antoine Ricoux for Electric Canary
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 20000000UL
#include <util/delay.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <avr/eeprom.h>

#define CLK_PIN 6
#define PWM_PIN 3
//#define TAP_PIN 6
#define TAP_PIN 0
#define LED_PIN 7
#define POT_PIN 1
#define DIV_PIN 2
#define DEBOUNCE_TIME 900
#define TAP_EEPROM 1
#define TEMPO_EEPROM 2

volatile uint16_t pot;
volatile uint8_t divsw;
volatile uint16_t pwm = 500;
volatile uint16_t ms;
volatile uint16_t ledms;

void IO_Init(void) 
{
    //PWM & LED as outputs
    PORTA.DIRSET |= (1 << PWM_PIN) | (1 << LED_PIN) | (1<<CLK_PIN);
    //pull up for div switch & tap button
    PORTA_PIN0CTRL |= PORT_PULLUPEN_bm;
    //PORTA_PIN6CTRL |= PORT_PULLUPEN_bm;
    PORTA_PIN2CTRL |= PORT_PULLUPEN_bm;
}

void ADC_Config(void) 
{
    //freerunning mode, enable ADC
    ADC0.CTRLA |= ADC_FREERUN_bm | ADC_ENABLE_bm;
    //reference to VCC with the right sampling capacitor 
    ADC0.CTRLC |= ADC_SAMPCAP_bm | ADC_REFSEL0_bm;
    //Start with ADC channel 1
    ADC0.MUXPOS = POT_PIN;
    //enable result ready interrupt
    ADC0.INTCTRL |= ADC_RESRDY_bm;
    //Start conversion
    ADC0.COMMAND |= ADC_STCONV_bm;
}

ISR(ADC0_RESRDY_vect) 
{
    switch (ADC0.MUXPOS) 
    {
        case POT_PIN:
            pot = ADC0.RES;
            ADC0.MUXPOS = DIV_PIN;
            break;

        case DIV_PIN:
            divsw = ADC0.RESH;
            ADC0.MUXPOS = POT_PIN;
            break;

        default:
            ADC0.MUXPOS = POT_PIN;
            break;
    }
}

void TCA_Config(void) 
{
    //Enable PWM output 0 (PA3), single slope PWM
    TCA0.SINGLE.CTRLB |= TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP0EN_bm;
    //Set period to 1000 (20kHz for 20MHz clock)
    TCA0.SINGLE.PER = 999;
    TCA0.SINGLE.CMP0 = 500;
    //Enable overflow interrupt
    TCA0.SINGLE.INTCTRL |= TCA_SINGLE_OVF_bm;
    //Enable timer
    TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm;
}

ISR(TCA0_OVF_vect) 
{
    TCA0.SINGLE.CMP0BUF = pwm;
    static uint8_t count;
    count++;
    if (count >= 20) 
    {
        count = 0;
        ms++;
        ledms++;
    }

    //Clear interrupt flag
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}

void TCB_Config(void) 
{
    //8bit PWM Mode, enable output pin
    TCB0.CTRLB = TCB_CNTMODE_PWM8_gc | TCB_CCMPEN_bm;
    //Set period at 208 (10MHz/208 = 48 077Hz)
    TCB0.CCMPL = 208;
    //Set duty cycle at 50%
    TCB0.CCMPH = 104;
    //Enable timer and dividing clock by 2 (10MHz)
    TCB0.CTRLA = TCB_CLKSEL0_bm | TCB_ENABLE_bm;
}

uint8_t debounce(void) {
    if (!(PORTA.IN & (1 << TAP_PIN))) 
    {
        _delay_us(DEBOUNCE_TIME);
        
        if (!(PORTA.IN & (1 << TAP_PIN))) 
        {
            return (1);
        } 
        
        else 
        {
            return (0);
        }
    }
    
    else 
    {
        return (0);
    }
}

int main(void) 
{
    //Unlocking protected registers and setting main clock to 20MHz
    CPU_CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = 0;
    CLKCTRL.MCLKLOCK |= CLKCTRL_LOCKEN_bm;

    uint8_t tap = eeprom_read_byte((uint8_t*) TAP_EEPROM); //1 for tap tempo control, 0 for pot control
    uint8_t laststate = 0; //laststate of tap button
    uint8_t nbtap = 0; //number of taps
    uint8_t tapping = 0; //1 if user is currently tapping
    uint16_t mstempo = eeprom_read_word((uint16_t*) TEMPO_EEPROM); //Current tempo in ms
    uint16_t divtempo; //Current tempo in ms multiplied by divmult
    float divmult = 1; //Multiplier for tempo division
    uint16_t delaymax = 1000; //Maximum delay in ms allowed
    uint16_t previousdiv = 300;
    uint16_t previouspot;
    uint8_t timepresetactive = 0;

    //enable interrupts
    sei();
    IO_Init();
    TCA_Config();
    TCB_Config();
    ADC_Config();

    while (1) 
    {
        //TIME POT

        if ((tap == 1 && abs(previouspot - pot) >= 15) || (timepresetactive == 1 && tap == 0 && abs(previouspot - pot) >= 15) || (timepresetactive == 0 && tap == 0 && abs(previouspot - pot) >= 1))//if pot move of more than 5%, changing to pot control
        {
            mstempo = pot / 1000; //mstempo used to stock directly digi pot wiper position (for presets)
            pwm = divtempo - 1;

            previouspot = pot;
            timepresetactive = 0;
            tap = 0;
            TCA0.SINGLE.CNT = 0;
            eeprom_update_byte((uint8_t*) TAP_EEPROM, 0);
            eeprom_update_word((uint16_t*) TEMPO_EEPROM, mstempo);
        }

        //TEMPO DIV

        if (abs(previousdiv - divsw) > 50) //if first time or if div toggle changed position : update div
        {
            if (debounce() == 0) //if tap button not pressed while changing 3 first div
            {
                if (divsw <= 50) 
                {
                    divmult = 1;
                } //fourth

                if (divsw > 50 && divsw < 230) 
                {
                    divmult = 0.75;
                } //dotted eighth

                if (divsw >= 230) 
                {
                    divmult = 0.5;
                } //eighth
            }

            if (debounce() == 1) //if tap button pressed while changing 3 last div
            {
                if (divsw <= 50) 
                {
                    divmult = 0.333333;
                } //triplet

                if (divsw > 50 && divsw < 230) 
                {
                    divmult = 0.25;
                } //sixteenth

                if (divsw >= 230) 
                {
                    divmult = 0.1666666;
                } //sextuplet

                nbtap = 0; //don't count press as tap
                tapping = 0;
                ms = 0;
                laststate = 1;
            }

            if (tap == 1) //if in tap control, update digital pot value
            {
                divtempo = round(mstempo * divmult);
                if (divtempo > delaymax) 
                {
                    divtempo = delaymax;
                    mstempo = delaymax / divmult;
                }
                pwm = divtempo;
            }

            ledms = 0;
            previousdiv = divsw; //reseting previousdiv to detect next move
        }

        //TAP TEMPO

        if (debounce() && laststate == 0 && nbtap == 0) //first tap
        {
            TCA0.SINGLE.CNT = 0; //starts counting
            ms = 0;
            nbtap++;
            laststate = 1;
            tapping = 1;
        }

        if (nbtap > 1 && ms > (3 * mstempo) && !debounce() && laststate == 0) //if too long between taps  : resets
        {
            ms = 0;
            nbtap = 0;
            tapping = 0;
        }

        if (nbtap == 1 && ms > ((delaymax / divmult) + 800) && !debounce() && laststate == 0) //if tapped only once, reset once the max tempo for the div + 800ms passed
        {
            ms = 0;
            nbtap = 0;
            tapping = 0;
            eeprom_update_word((uint16_t*) TEMPO_EEPROM, mstempo);
            eeprom_update_byte((uint8_t*) TAP_EEPROM, 1);
        }

        if (!debounce() && laststate == 1) //release tap button
        {
            laststate = 0;
            PORTA.OUTCLR = (1 << LED_PIN);
        }

        if (debounce() && laststate == 0 && nbtap != 0) //not first tap
        {
            if (TCA0.SINGLE.CNT >= 500) 
            {
                ms++;
            } //round up value if timer counter more than 500µs

            if (nbtap == 1) //if second tap, tempo = time elapsed between the 2 button press
            {
                divtempo = ms * divmult;
                mstempo = ms;
            }
            else //if not second tap, average every tap
            {
                divtempo = round((divtempo + (ms * divmult)) / 2);
                mstempo = round((mstempo + ms) / 2);
            }
            if (divtempo > delaymax) 
            {
                divtempo = delaymax;
                mstempo = delaymax / divmult;
            }
            pwm = divtempo - 1;

            nbtap++; //updating number of tap and last state of tap button
            laststate = 1;
            TCA0.SINGLE.CNT = 0; //reseting counter and ms
            ms = 0;
            ledms = 0;
            tap = 1; //now in tap control mode
            PORTA.OUTSET = (1 << LED_PIN);
        }

        //---------LED CONTROL
        if (tapping == 1 && nbtap == 1 && laststate == 1)//keep the light off when long button press
        {
            PORTA.OUTCLR = (1 << LED_PIN);
        }

        if (tap == 1 && tapping == 0 && !debounce()) 
        {
            if (ledms >= divtempo - 4) //turns LED on every downbeat
            {
                ledms = 0;
                PORTA.OUTSET = (1 << LED_PIN);
            }

            if (ledms >= 8) //turns LED off 4ms after downbeat
            {
                PORTA.OUTCLR = (1 << LED_PIN);
            }
        }
    }
}
