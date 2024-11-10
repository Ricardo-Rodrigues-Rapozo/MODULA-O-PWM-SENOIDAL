#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "inc/hw_uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/fpu.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "math.h"


uint32_t ui32ADC0Value[1];
#define VECTOR_SIZE 1000
uint16_t dataVector[VECTOR_SIZE];
uint32_t ui32SysClkFreq;
uint32_t FS = 40000;
int idx=0;
uint32_t ui32PWMClockRate;
uint32_t freq_port = 2000;
uint32_t duty_cycle = 10;

float amplitude = 50.0; // Amplitude máxima para 50% do duty cycle (ajustável)
float offset = 50.0;    // Offset para centralizar entre 0% a 100%
float frequency = 1000;  // Frequência da senoide (ajustável)
float time = 0.0;
float time_step = 1.0f / 40000.0f; // Passo de tempo (ajustável)


void InitPWM(void)
{

    //
//    // hABILITA O MODULO PWM
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    //
    //hABILITA O GPIO
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // CONFIGURA OS PINOS
    //
    MAP_GPIOPinConfigure(GPIO_PF2_M0PWM2);
    MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // configurando o clk do pwm
    //
    MAP_PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_8);
    //
    // Use a local variable to store the PWM clock rate which will be
    // 120 MHz / 8 = 15 MHz. This variable will be used to set the
    // PWM generator period.
    //
    ui32PWMClockRate = ui32SysClkFreq / 8;

    //
    // Configure PWM2 to count up/down without synchronization.
    //
    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
                        PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    //
    // Set the PWM period to 250Hz.  To calculate the appropriate parameter
    // use the following equation: N = (1 / f) * PWMClk.  Where N is the
    // function parameter, f is the desired frequency, and PWMClk is the
    // PWM clock frequency based on the system clock.
    //
    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, (ui32PWMClockRate / freq_port));

    //
    // Set PWM2 to a duty cycle of 25%.  You set the duty cycle as a function
    // of the period.  Since the period was set above, you can use the
    // PWMGenPeriodGet() function.  For this example the PWM will be high for
    // 25% of the time or (PWM Period / 4).
    //Configurar frequência do
    //gerador e razão cíclica inicial
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                         MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1) / 4);

    //
    // Enable PWM Out Bit 2 (PF2) output signal.
    //
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);

    //
    // Enable the PWM generator block.
    //
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_1);

}


void ConfigureADC(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32SysClkFreq/FS - 1);

    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);

    TimerEnable(TIMER0_BASE, TIMER_A);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0);

    ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);

    ADCSequenceEnable(ADC0_BASE, 1);

    ADCIntClear(ADC0_BASE, 1);


}

int main(void)
{
    // Configuração do clock do sistema

    ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
        SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    // Inicializa o PWM e ADC
    InitPWM();
    ConfigureADC();

    while(1)
    {
        if (ADCIntStatus(ADC0_BASE, 1, false))
                      {
                          //Limpa a flag de interrupção do ADC e obtém os dados do sequenciador
                          ADCIntClear(ADC0_BASE, 1);
                          ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
                          dataVector[idx] =  ui32ADC0Value[0];  // Mantém apenas os 16 bits menos significativos
                          //MAP_PWMPulseWidthSet(PWM0_BASE,PWM_OUT2 , duty_cycle);
                          idx = (idx+1)%VECTOR_SIZE;
                          // Calcular o valor da senoide para ajustar o ciclo de trabalho
                          float duty_cycle = offset + amplitude * sin(2 * 3.14159 * frequency * time);

                          // Converter o valor do ciclo de trabalho para a largura de pulso do PWM
                          uint32_t pulse_width = (uint32_t)((duty_cycle / 100.0) * PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1));
                          PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pulse_width);
                          // Incrementa o tempo
                          time += time_step;
                      }
    }
}
