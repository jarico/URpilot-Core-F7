/***************************************************************************************
**  reset.c - Fichero de las funciones de reset del microcontrolador
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 03/12/2020
**  Fecha de modificacion: 03/12/2020
**
**  El proyecto URpilot NO es libre. No se puede distribuir y/o modificar este fichero
**  bajo ningun concepto.
**
**  En caso de modificacion y/o solicitud de informacion pongase en contacto con
**  el grupo de investigacion ICON a traves de: www.unirioja.es/urpilot
**
**
**  Control de versiones del fichero
**
**  v1.0  Ramon Rico. Se ha liberado la primera version estable
**
****************************************************************************************/

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include "tiempo.h"
#include "atomico.h"
#include "nvic.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static uint32_t usTicks = 0;                       // Ciclos por microsegundo
volatile uint32_t tiempoSysTick = 0;               // Variable que se incrementa cada ms en la interrupcion del systick timer
volatile uint32_t ciclosSysTick = 0;               // Valor del systick en la interrupcion
volatile int32_t sysTickPendiente = 0;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void iniciarContadorCiclos(void)
**  Descripcion:    Inicia la variable que dicta los ciclos por microsegundo. Variable
**                  utilizada en las funciones de tiempo
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarContadorCiclos(void)
{
    usTicks = HAL_RCC_GetSysClockFreq() / 1000000;
}


/***************************************************************************************
**  Nombre:         uint32_t microsISR(void)
**  Descripcion:    Retorna los microsegundos cuando se esta ejecutando una interrupcion
**                  (maximo 70 minutos)
**  Parametros:     Ninguno
**  Retorno:        Microsegundos transcurridos
****************************************************************************************/
uint32_t microsISR(void)
{
    register uint32_t ms, pendiente, cntCiclos;

    BLOQUE_ATOMICO(NVIC_PRIO_MAX) {
        cntCiclos = SysTick->VAL;

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
            // Hay pendiente una interrupcion del SysTick
            sysTickPendiente = 1;

            // Lee el valor del SysTick para asegurar que leemos el valor despues del rollover.
            cntCiclos = SysTick->VAL;
        }
        ms = tiempoSysTick;
        pendiente = sysTickPendiente;
    }

    return ((ms + pendiente) * 1000) + (usTicks * 1000 - cntCiclos) / usTicks;
}


/***************************************************************************************
**  Nombre:         uint32_t micros(void)
**  Descripcion:    Retorna los microsegundos (maximo 70 minutos)
**  Parametros:     Ninguno
**  Retorno:        Microsegundos transcurridos
****************************************************************************************/
uint32_t micros(void)
{
    register uint32_t ms, cntCiclos;

    // Llama a microsISR() si estamos dentro de una interrupcion
    if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) || (__get_BASEPRI()))
        return microsISR();

    do {
        ms = tiempoSysTick;
        cntCiclos = SysTick->VAL;
    } while (ms != tiempoSysTick || cntCiclos > ciclosSysTick);

    return (ms * 1000) + (usTicks * 1000 - cntCiclos) / usTicks;
}


/***************************************************************************************
**  Nombre:         uint32_t millis(void)
**  Descripcion:    Retorna los milisegundos  (maximo 49 dias)
**  Parametros:     Ninguno
**  Retorno:        Milisegundos transcurridos
****************************************************************************************/
uint32_t millis(void)
{
    return tiempoSysTick;
}


/***************************************************************************************
**  Nombre:         void delayMicroseconds(uint32_t us)
**  Descripcion:    Retardo en microsegundos
**  Parametros:     Microsegundos que se quieren retardar
**  Retorno:        Ninguno
****************************************************************************************/
void delayMicroseconds(uint32_t us)
{
    uint32_t usInicio = micros();
    while ((micros() - usInicio) < us);
}


/***************************************************************************************
**  Nombre:         void delay(uint32_t ms)
**  Descripcion:    Retardo en milisegundos
**  Parametros:     Milisegundos que se quiere retardar
**  Retorno:        Ninguno
****************************************************************************************/
void delay(uint32_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}


/***************************************************************************************
**  Nombre:         void HAL_Delay(uint32_t Delay)
**  Descripcion:    Retardo en milisegundos para las funciones HAL
**  Parametros:     Milisegundos que se quiere retardar
**  Retorno:        Ninguno
****************************************************************************************/
void HAL_Delay(uint32_t Delay)
{
    delay(Delay);
}
