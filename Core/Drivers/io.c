/***************************************************************************************
**  io.c - Funciones para las IOs
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 06/05/2019
**  Fecha de modificacion: 03/08/2020
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
#include "io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define PUERTO_GPIO(tag)      ((tag & 0xF0) >> 4)


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    GPIO_A = 0,
    GPIO_B,
    GPIO_C,
    GPIO_D,
    GPIO_E,
    GPIO_F,
    GPIO_G,
} gpio_e;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static uint8_t relojHabilitado[8] = {0};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void habilitarRelojIO(gpio_e gpio);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void configurarIO(uint8_t tag, uint16_t cfg, uint8_t af)
**  Descripcion:    Configura el pin dada una configuracion
**  Parametros:     Tag del pin a configurar, configuracion, funcion especial
**  Retorno:        Ninguno
****************************************************************************************/
void configurarIO(uint8_t tag, uint16_t cfg, uint8_t af)
{
    if (TAG_VACIO(tag))
        return;

    // Habilitacion del reloj
    gpio_e puertoGPIO = PUERTO_GPIO(tag) - 1;

    if (!relojHabilitado[puertoGPIO])
        habilitarRelojIO(puertoGPIO);

    // Configuracion del GPIO
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = DEFIO_PIN(tag),
        .Mode = cfg & 0x00FF,
        .Speed = (cfg >> 8) & 0x0003,
        .Pull = (cfg >> 11) & 0x0003,
        .Alternate = af
    };

    HAL_GPIO_Init(DEFIO_GPIO(tag), &GPIO_InitStruct);
}


/***************************************************************************************
**  Nombre:         void habilitarRelojIO(gpio_e gpio)
**  Descripcion:    Habilita el reloj de un GPIO
**  Parametros:     GPIO
**  Retorno:        Ninguno
****************************************************************************************/
void habilitarRelojIO(gpio_e gpio)
{
    switch (gpio) {
        case GPIO_A:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            relojHabilitado[GPIO_A] = true;
            break;

        case GPIO_B:
            __HAL_RCC_GPIOB_CLK_ENABLE();
            relojHabilitado[GPIO_B] = true;
            break;

        case GPIO_C:
            __HAL_RCC_GPIOC_CLK_ENABLE();
            relojHabilitado[GPIO_C] = true;
            break;

        case GPIO_D:
            __HAL_RCC_GPIOD_CLK_ENABLE();
            relojHabilitado[GPIO_D] = true;
            break;

        case GPIO_E:
            __HAL_RCC_GPIOE_CLK_ENABLE();
            relojHabilitado[GPIO_E] = true;
            break;

        case GPIO_F:
            __HAL_RCC_GPIOF_CLK_ENABLE();
            relojHabilitado[GPIO_F] = true;
            break;

        case GPIO_G:
            __HAL_RCC_GPIOG_CLK_ENABLE();
            relojHabilitado[GPIO_G] = true;
            break;
    }
}


/***************************************************************************************
**  Nombre:         void escribirIO(uint8_t tag, bool estado)
**  Descripcion:    Escribe el estado del pin
**  Parametros:     Tag del pin, estado
**  Retorno:        Ninguno
****************************************************************************************/
void escribirIO(uint8_t tag, bool estado)
{
    if (!tag)
        return;

    HAL_GPIO_WritePin(DEFIO_GPIO(tag), DEFIO_PIN(tag), estado ? 1 : 0);
}


/***************************************************************************************
**  Nombre:         bool leerIO(uint8_t tago)
**  Descripcion:    Lee el estado del pin
**  Parametros:     Tag del pin
**  Retorno:        Estado
****************************************************************************************/
bool leerIO(uint8_t tag)
{
    if (!tag)
        return false;

    return HAL_GPIO_ReadPin(DEFIO_GPIO(tag), DEFIO_PIN(tag));
}


/***************************************************************************************
**  Nombre:         void invertirIO(uint8_t tag)
**  Descripcion:    Invierte el estado del pin
**  Parametros:     Tag del pin
**  Retorno:        Ninguno
****************************************************************************************/
void invertirIO(uint8_t tag)
{
    if (!tag)
        return;

    HAL_GPIO_TogglePin(DEFIO_GPIO(tag), DEFIO_PIN(tag));
}
