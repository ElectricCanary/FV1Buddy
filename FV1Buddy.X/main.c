/* 
 * FV1 Buddy
 * January 2022
 * by Antoine Ricoux for Electric Canary
 * 
 *  https://electric-canary.com/FV1Buddy
 *  support@electric-canary.com
 * 
 * This code is an ATtiny402 Tap Tempo & Clock for the Spin Semiconductors FV1 DSP
 * It can be calibrated for 700, 800, 900 & 1000ms delays.
 * 
 * Pinout:
 * 1: VDD
 * 2: 48kHz Clock Output
 * 3: LED+ Output
 * 4: Time Potentiometer Analog Input
 * 5: Tempo Division Switch (On-Off-On) Analog Input
 * 6: Momentary Tap Tempo Button Input
 * 7: PWM Output
 * 8: GND
 * 
 * This code is shared shared under a BY-NC-SA Creative Commons License
 * Go here for complete license : https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 20000000UL
#include <util/delay.h>
#include <avr/eeprom.h>
#include <stdlib.h>

#define CLK_PIN 6
#define PWM_PIN 3
#define TAP_PIN 0
#define LED_PIN 7
#define POT_PIN 1
#define DIV_PIN 2
#define DEBOUNCE_TIME 900
#define EEPROM_TAP 0
#define EEPROM_TEMPO 1
#define EEPROM_DELAYMAX 3

volatile uint16_t pot;
volatile uint16_t divsw;
volatile uint16_t pwm = 500;
volatile uint16_t ms;
volatile uint16_t ledms;

//This set of fuses will deactivate the reset, it will be impossible to reprogram the chip without high voltage programming
FUSES = {
	.WDTCFG = 0x00, // WDTCFG {PERIOD=OFF, WINDOW=OFF}
	.BODCFG = 0x00, // BODCFG {SLEEP=DIS, ACTIVE=DIS, SAMPFREQ=1KHz, LVL=BODLEVEL0}
	.OSCCFG = 0x02, // OSCCFG {FREQSEL=20MHZ, OSCLOCK=CLEAR}
	.TCD0CFG = 0x00, // TCD0CFG {CMPA=CLEAR, CMPB=CLEAR, CMPC=CLEAR, CMPD=CLEAR, CMPAEN=CLEAR, CMPBEN=CLEAR, CMPCEN=CLEAR, CMPDEN=CLEAR}
	.SYSCFG0 = 0xF2, // SYSCFG0 {EESAVE=CLEAR, RSTPINCFG=GPIO, CRCSRC=NOCRC}
	.SYSCFG1 = 0x07, // SYSCFG1 {SUT=64MS}
	.APPEND = 0x00, // APPEND {APPEND=User range:  0x0 - 0xFF}
	.BOOTEND = 0x00, // BOOTEND {BOOTEND=User range:  0x0 - 0xFF}
};

void IO_Init(void)
{
  //PWM & LED as outputs
  PORTA.DIRSET = (1<<PWM_PIN) | (1<<LED_PIN) | (1<<CLK_PIN);
  //pull up for div switch & tap button
  PORTA_PIN0CTRL |= PORT_PULLUPEN_bm;
  PORTA_PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
  PORTA_PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
}

void ADC_Config(void)
{
  //reference to VCC with the right sampling capacitor 
  ADC0.CTRLC = ADC_SAMPCAP_bm | ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV32_gc;
  //ADC0.CTRLB = ADC_SAMPNUM_ACC32_gc;
  //Start with ADC channel 1
  ADC0.MUXPOS = ADC_MUXPOS_AIN2_gc;
  //enable result ready interrupt
  ADC0.INTCTRL = ADC_RESRDY_bm;
  //10bits freerun & enable
  ADC0.CTRLA |= ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc | ADC_FREERUN_bm;
  //Start conversion
  ADC0.COMMAND = ADC_STCONV_bm;
}

ISR(ADC0_RESRDY_vect)
{
  switch(ADC0.MUXPOS)
  {
    case ADC_MUXPOS_AIN1_gc:
        divsw = ADC0.RES;
        ADC0.MUXPOS = ADC_MUXPOS_AIN2_gc;
    break;
    
    case ADC_MUXPOS_AIN2_gc:
        pot = ADC0.RES;
        ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc;
    break;
    
    default:
        ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc;
    break;
  }
    ADC0.INTFLAGS = ADC_RESRDY_bm;
}

//ms timer for tap tempo
void TCA_Config(void)
{
  //Enable interrupts
  sei();
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

//32kHz Timer for FV-1
void TCB_Config(void)
{
    //8bit PWM Mode, enable output pin
    TCB0.CTRLB = TCB_CNTMODE_PWM8_gc | TCB_CCMPEN_bm;
    //Set period at 208 (5MHz/153 = 32 680Hz)
    TCB0.CCMPL = 153;
    //Set duty cycle at 50%
    TCB0.CCMPH = 127;
    //Enable timer and dividing clock by 2 (10MHz)
    TCB0.CTRLA = TCB_CLKSEL_1_bm | TCB_ENABLE_bm;
}

uint8_t debounce(void)
{
    if (!(PORTA.IN & (1<<TAP_PIN)))
    {
        _delay_us(DEBOUNCE_TIME);
        if (!(PORTA.IN & (1<<TAP_PIN)))
        {
            return(1);
        }
        else
        {
            return(0);
        }
    }
    
    else
    {
        return(0);
    }
}

void blink(void)
{
    PORTA.OUTTGL = (1<<LED_PIN);
    _delay_ms(150);
    PORTA.OUTTGL = (1<<LED_PIN);
    _delay_ms(150);
}

uint16_t calib(void)
{
    uint16_t count = 0;
    
    if (debounce())
    {
        while(debounce())
        {
            _delay_us(1000 - DEBOUNCE_TIME);
            count++;
            if (count >= 5000)
            {
                PORTA.OUTCLR = (1<<LED_PIN);
                blink();
                blink();
                for (uint16_t c = 0; c < 4000; c++ )
                {
                    _delay_us(1000 - DEBOUNCE_TIME);

                    if (!debounce())
                    {
                        blink();
                        blink();
                        return(((pot>>8)+7)*100);
                    }
                }
            }
        }
    }
    
    else
    {
        if(eeprom_read_word((uint16_t*)EEPROM_DELAYMAX) > 1000)
        {
            eeprom_update_word((uint16_t*)EEPROM_DELAYMAX,1000);
            return(1000);
        }
        else
        {
            return(eeprom_read_word((uint16_t*)EEPROM_DELAYMAX));
        }
    }
}

int main(void)
{
  //Unlocking protected registers and setting main clock to 20MHz
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0;
  CLKCTRL.MCLKLOCK |= CLKCTRL_LOCKEN_bm;
  
  uint8_t tap = eeprom_read_byte((uint8_t*)EEPROM_TAP); //1 for tap tempo control, 0 for pot control
  uint8_t laststate = 0; //laststate of tap button
  uint8_t nbtap = 0; //number of taps
  uint8_t tapping = 0; //1 if user is currently tapping
  uint16_t mstempo = eeprom_read_word((uint16_t*)EEPROM_TEMPO); //Current tempo in ms
  uint16_t divtempo = 500; //Current tempo in ms multiplied by divmult
  float divmult = 1; //Multiplier for tempo division
  uint16_t previousdiv = 60000;
  uint16_t previouspot;
  uint8_t updown;
  
  //enable interrupts
  sei();
  IO_Init();
  TCA_Config();
  TCB_Config();
  ADC_Config();
  uint16_t delaymax = calib(); //Maximum delay in ms allowed
  
  if (tap == 1)
  {
      previouspot = pot;
  }
  else
  {
      previouspot = 60000;
  }
  
  _delay_ms(500);
  
    while (1)
    {
        //TEMPO DIV
        if (abs(previousdiv - divsw) > 300)		//if first time or if div toggle changed position : update div
		{
			if (debounce() == 0)	//if tap button not pressed while changing 3 first div
			{
				if (divsw <= 100){divmult = 1;}	//fourth
			
				if (divsw > 101 && divsw < 900){divmult = 0.75;}	//dotted eighth
				
				if (divsw >= 900){divmult = 0.5;}	//eighth
			}
			
			if (debounce() == 1)	//if tap button pressed while changing 3 last div
			{
				if (divsw <= 100){divmult = 0.333333;}	//triplet
				
				if (divsw > 101 && divsw < 900){divmult = 0.25;}	//sixteenth
				
				if (divsw >= 900){divmult = 0.1666666;}	//sextuplet
				
				nbtap = 0;			//don't count press as tap
				tapping = 0;
				ms = 0;
				laststate = 1;
			}
			
			if (tap == 1)	//if in tap control, update pwm value
			{
				divtempo = round(mstempo * divmult);
				pwm = divtempo;
			}
			
			ledms = 0;
			previousdiv = divsw;	//reseting previousdiv to detect next move
		}
        
        //TIME POT
        //if pot move of more than 7%, changing to pot control
        if ((tap == 1 && tapping == 0 && abs(previouspot-pot) >= 72) || (tap == 0 && tapping == 0 && abs(previouspot-pot) >= 1))
		{
			pwm = pot * (0x3FF/delaymax);
			previouspot = pot;
			tap = 0;
            eeprom_update_byte((uint8_t*)EEPROM_TAP,0);
		}
        
        //TAP TEMPO
        
        if (debounce() && laststate==0 && nbtap==0) //first tap
		{
			TCA0.SINGLE.CNT = 0;			//starts counting
			ms=0;
			nbtap++;
			laststate = 1;
			tapping = 1;
		}
        
		if (nbtap > 1 && !debounce() && laststate==0) //if too long between taps  : resets
		{
            if (ms > (1.5*mstempo))
            {
                eeprom_update_byte((uint8_t*)EEPROM_TAP,1);
                eeprom_update_word((uint16_t*)EEPROM_TEMPO,mstempo);
            }
            
            if (ms > (3*mstempo))
            {
                ms = 0;
                nbtap = 0;
                tapping = 0;
            }
		}
		
		if (nbtap == 1 && ms > ((delaymax/divmult) + 800) && !debounce() && laststate==0) //if tapped only once, reset once the max tempo for the div + 800ms passed
		{
			ms = 0;
			nbtap = 0;
			tapping = 0;
		}
		
		if (!debounce() && laststate == 1)	//release tap button
		{
			laststate = 0;
			PORTA.OUTCLR = (1<<LED_PIN);
            previouspot = pot;
            previousdiv = divsw;
		}
		
		if (debounce() && laststate==0 && nbtap!=0 && ms >= 100) //not first tap
		{
			if (TCA0.SINGLE.CNT >= 500){ms++;}	//round up value if timer counter more than 500µs
			
			if (nbtap == 1)		//if second tap, tempo = time elapsed between the 2 button press
			{
				divtempo = ms * divmult;
				mstempo = ms;
			}
			
			else		//if not second tap, average every tap
			{
				divtempo = (divtempo + (ms*divmult)) / 2;
				mstempo = (mstempo + ms)/2;
			}
            
            pwm = divtempo;
            
			nbtap++;                //updating number of tap and last state of tap button
			laststate = 1;
			TCA0.SINGLE.CNT = 0;    //reseting counter and ms
			ms = 0;
			ledms = 0;
			tap = 1;                //now in tap control mode
			PORTA.OUTSET = (1<<LED_PIN);
		}
        
        // RAMP
        if (debounce() && ms >= 2000 && laststate==1)	//if button pressed more than 2s
		{
            while (debounce())
            {
                if(updown == 0)
                {
                    divtempo += 1;
                }
                
                else
                {
                    divtempo -= 1;
                }
                
                pwm = divtempo;
                
                if (divtempo == 0 || divtempo == delaymax) 
                {
                    updown ^= 1;
                    PORTA.OUTTGL = (1<<LED_PIN);
                }
                
                for (uint16_t i = 0; i < pot; i++)
                {
                    _delay_us(1);
                }
            }
            if (tap == 1)
            {
                divtempo = mstempo * divmult;
                pwm = divtempo;
            }
			else
            {
                pwm = pot * (0x3FF/delaymax);
            }
            updown = 0;
            previouspot = pot;
        }
        
        //---------LED CONTROL
        if (tap != 1 && tapping == 0)
        {
            PORTA.OUTSET = (1<<LED_PIN);
        }
        
        if (tapping == 1 && nbtap == 1 && laststate == 1)//keep the light off when long button press
        {
            PORTA.OUTCLR = (1<<LED_PIN);
        }	
        
		if (tap == 1 && tapping == 0 && !debounce())
		{	
			if (ledms >= mstempo - 4)	//turns LED on every downbeat
			{
				ledms = 0;
				PORTA.OUTSET = (1<<LED_PIN);
			}
		
			if (ledms >= 8)			//turns LED off 4ms after downbeat
			{
				PORTA.OUTCLR = (1<<LED_PIN);
			}
		}
    }
}