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
#include "reset.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
uint32_t valorRCCcsr;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void resetSistema(void)
**  Descripcion:    Resetea la CPU
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void resetSistema(void)
{
    __disable_irq();
    NVIC_SystemReset();
}


/***************************************************************************************
**  Nombre:         void resetSistemaAlBootloader(void)
**  Descripcion:    Resetea para poner el sistema en modo bootloader
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void resetSistemaAlBootloader(void)
{
    // Flag remanente para decirle a la CPU que se ponga en modo bootloader
    (*(__IO uint32_t *) (BKPSRAM_BASE + 4)) = 0xDEADBEEF;
    __disable_irq();
    NVIC_SystemReset();
}


/***************************************************************************************
**  Nombre:         void chequearSolicitudBootLoader(void)
**  Descripcion:    Comprueba mediante el flag remanente de la funcion "resetSistemaAlBootloader"
**                  si hay solicitud de bootloader
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void chequearSolicitudBootLoader(void)
{
    uint32_t dato;

    __PWR_CLK_ENABLE();
    __BKPSRAM_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    dato = (*(__IO uint32_t *) (BKPSRAM_BASE + 4)) ;
    if (dato == 0xDEADBEEF) {
        (*(__IO uint32_t *) (BKPSRAM_BASE + 4)) =  0xCAFEFEED; // Resetea el valor para que no entre siempre que encendamos
        // La zona Backup SRAM es write-back por defecto, asegurarse de que el valor se escribe correctamente
        // Otra solucion seria hacer la zona write-through en las opciones de la Memory Protection Unit
        SCB_CleanDCache_by_Addr((uint32_t *) (BKPSRAM_BASE + 4), sizeof(uint32_t));

        void (*SaltoSisMemBoot)(void);
        __SYSCFG_CLK_ENABLE();
        SYSCFG->MEMRMP |= SYSCFG_MEM_BOOT_ADD0 ;
        uint32_t p =  (*((uint32_t *) 0x1ff00000));
        __set_MSP(p);                                                    // Pone el puntero del stack al valor por defecto
        SaltoSisMemBoot = (void (*)(void)) (*((uint32_t *) 0x1ff00004)); // Pone el PC en la posicion del System Memory reset vector (+4)
        SaltoSisMemBoot();
        while (1);
    }
}


/***************************************************************************************
**  Nombre:         bool rccCSR(void)
**  Descripcion:    Obtiene el valor de RCC->CSR para comprobar si es reset por software
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void rccCSR(void)
{
	valorRCCcsr = RCC->CSR;
}


/***************************************************************************************
**  Nombre:         bool resetSoftware(void)
**  Descripcion:    Comprueba si ha habido un reset por software
**  Parametros:     Ninguno
**  Retorno:        Retorna un true si es reset por software
****************************************************************************************/
bool resetSoftware(void)
{
    if (valorRCCcsr & RCC_CSR_SFTRSTF)
        return true;
    else
        return false;
}
