//TASK 2
//*************************************************************************
// Name of Group Members: Colin Gregg, Owen Funk, Eli Juvan
// Lab this program is associated with: ECE 257 Senior Design
// Hardware Inputs used: PF0 (SW1), PF4 (SW2)
// Hardware Outputs used: PF1 (Red LED), PF3 (Green LED)
// Additional files needed: xxxxxx
// Date of last modification: 11/11/25
//*************************************************************************

//include statements to provide additional resource and definitions to be used
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/interrupt.h"    // Interrupt driver library


 //global variable declaration
volatile int count = 1;

//prototype functions
void setup_PWM(unsigned long ulPeriod);
void update_frequency(int count);
void portF_input_setup(int input_pins);
void frequency_control(void);



int main(void) {

    unsigned long ulPeriod;
    int freq;

    //set system clock to 80 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    //configure PWM Clock scaled down
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    //enable peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    // Get PWM for designated Frequency
    freq = 25000;
    ulPeriod = ((80000000 / 64) / freq) - 1;

    // Initialize PWM
    setup_PWM(ulPeriod);

    //configure PF0 and PF4 as inputs
    portF_input_setup(GPIO_PIN_0 | GPIO_PIN_4);

    //configure interrupts
    GPIOIntRegister(GPIO_PORTF_BASE, frequency_control);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_4);
    IntMasterEnable();

    while (1) {} //keep program running return 0;
}



//function to configure PWM settings
void setup_PWM(unsigned long ulPeriod) {

    //enable peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1)) {}

    //configure PF1 and PF3 as PWM outputs
    GPIOPinConfigure(GPIO_PF1_M1PWM5);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);

    //configure PWM Generator
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, ulPeriod); //set PWM_signal period
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 62); // set PWM_disable period

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, ulPeriod/2);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 31);

    //enable PWM generator
    PWMGenEnable(PWM1_BASE, PWM_GEN_2); //enable PWM output
    PWMGenEnable(PWM1_BASE, PWM_GEN_3); //enable PWM output
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);
}


// Function to update PWM frequency for PF1 (M1PWM5) only
void update_frequency(int count) {
    unsigned long ulPeriod;

    switch(count) {
    case 1:
        ulPeriod = ((80000000 / 64) / 25000) - 1;
        PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, ulPeriod);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, ulPeriod/2);
        break;
    case 2:
        ulPeriod = ((80000000 / 64) / 30000) - 1;
        PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, ulPeriod);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, ulPeriod/2);
        break;
    case 3:
        ulPeriod = ((80000000 / 64) / 35000) - 1;
        PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, ulPeriod);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, ulPeriod/2);
        break;
    case 4:
        ulPeriod = ((80000000 / 64) / 40000) - 1;
        PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, ulPeriod);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, ulPeriod/2);
        break;
    }
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, ulPeriod); //set PWM_signal period
}

// Interrupt handler to adjust frequency
void frequency_control(void) {
    IntMasterDisable(); //disable global interrupts
    uint32_t status = GPIOIntStatus(GPIO_PORTF_BASE, true);

    if (status & GPIO_INT_PIN_4) {
        //if PF4 (SW1) pressed
        if (count == 1){
            // do nothing, don't wrap around and can't decrement count
        }
        else
        {
            count -= 1; // decrement to lower state
        }
    }

    if (status & GPIO_INT_PIN_0) {
        //if PF0 (SW2) pressed
        if (count == 4) {
            // do nothing, don't wrap and can't increment count
        }
        else {
            count += 1; // increment to higher state
        }
    }

    update_frequency(count); //update PWM with new frequency selector
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_4); //clear interrupt flag
    IntMasterEnable(); //re-enable global interrupts
}

//configure PF0 and PF4 as inputs
void portF_input_setup(int input_pins) {
    SYSCTL_RCGCGPIO_R |= 0x20; //enable clock for port F
    GPIO_PORTF_LOCK_R = 0x4C4F434B; //unlock port F
    GPIO_PORTF_CR_R |= input_pins; //enable commit register for selected pins
    GPIO_PORTF_DIR_R &= ~(input_pins); //set as input
    GPIO_PORTF_DEN_R |= input_pins; //enable digital function
    GPIO_PORTF_PUR_R |= input_pins; //enable pull-up resistors
}
