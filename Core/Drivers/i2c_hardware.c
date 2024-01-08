/***************************************************************************************
**  i2c_hardware.c - Hardware del I2C
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 25/07/2020
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
#include "i2c.h"

#ifdef USAR_I2C
#include "GP/gp_i2c.h"
#include "io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NUM_MAX_PIN_SEL_I2C   3


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    numI2C_e numI2C;
    I2C_TypeDef *reg;
    pin_t pinSCL[NUM_MAX_PIN_SEL_I2C];
    pin_t pinSDA[NUM_MAX_PIN_SEL_I2C];
} hardwareI2C_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static const hardwareI2C_t hardwareI2C[] = {
    {
        .numI2C = I2C_1,
        .reg = I2C1,
        .pinSCL = {
            { DEFIO_TAG(PB6), GPIO_AF4_I2C1  },
            { DEFIO_TAG(PB8), GPIO_AF4_I2C1  },
        },
        .pinSDA = {
            { DEFIO_TAG(PB7), GPIO_AF4_I2C1  },
            { DEFIO_TAG(PB9), GPIO_AF4_I2C1  },
        },
    },
    {
        .numI2C = I2C_2,
        .reg = I2C2,
        .pinSCL = {
            { DEFIO_TAG(PB10), GPIO_AF4_I2C2  },
            { DEFIO_TAG(PF1),  GPIO_AF4_I2C2  },
        },
        .pinSDA = {
            { DEFIO_TAG(PB11), GPIO_AF4_I2C2  },
            { DEFIO_TAG(PF0),  GPIO_AF4_I2C2  },
        },
    },
    {
        .numI2C = I2C_3,
        .reg = I2C3,
        .pinSCL = {
            { DEFIO_TAG(PA8),  GPIO_AF4_I2C3  },
        },
        .pinSDA = {
            { DEFIO_TAG(PC9),  GPIO_AF4_I2C3  },
        },
    },
#if defined(STM32F767xx)
    {
        .numI2C = I2C_4,
        .reg = I2C4,
        .pinSCL = {
            { DEFIO_TAG(PB6),  GPIO_AF11_I2C4 },
            { DEFIO_TAG(PB8),  GPIO_AF1_I2C4  },
            { DEFIO_TAG(PF14), GPIO_AF4_I2C4  },
        },
        .pinSDA = {
            { DEFIO_TAG(PB7),  GPIO_AF11_I2C4 },
            { DEFIO_TAG(PB9),  GPIO_AF1_I2C4  },
            { DEFIO_TAG(PD13), GPIO_AF4_I2C4  },
        },
    },
#endif
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool pinI2C(numI2C_e numI2C, uint8_t pinBusqueda, pin_t *pinDriver);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool asignarHALi2c(numI2C_e numI2C)
**  Descripcion:    Asigna el HAL a la estructura del driver
**  Parametros:     Numero del driver
**  Retorno:        True si ok
****************************************************************************************/
bool asignarHALi2c(numI2C_e numI2C)
{
    i2c_t *driver = punteroI2C(numI2C);

    // Asignamos los pines
    if (!pinI2C(numI2C, configI2C(numI2C)->pinSCL, &driver->hal.pinSCL))
        return false;

    if (!pinI2C(numI2C, configI2C(numI2C)->pinSDA, &driver->hal.pinSDA))
        return false;

    // Asignamos la instancia
    driver->hal.hi2c.Instance = hardwareI2C[numI2C].reg;
    return true;
}


/***************************************************************************************
**  Nombre:         bool pinI2C(numI2C_e numI2C, uint8_t pinBusqueda, pin_t *pinDriver)
**  Descripcion:    Encuentra el pin de la tabla de hardware
**  Parametros:     Numero del I2C, pin a buscar, pin del driver
**  Retorno:        True si OK
****************************************************************************************/
bool pinI2C(numI2C_e numI2C, uint8_t pinBusqueda, pin_t *pinDriver)
{
    for (uint8_t i = 0; i < NUM_MAX_PIN_SEL_I2C; i++) {
        if (pinBusqueda == hardwareI2C[numI2C].pinSCL[i].pin) {
        	pinDriver->pin = hardwareI2C[numI2C].pinSCL[i].pin;
        	pinDriver->af = hardwareI2C[numI2C].pinSCL[i].af;
            return true;
        }

        if (pinBusqueda == hardwareI2C[numI2C].pinSDA[i].pin) {
        	pinDriver->pin = hardwareI2C[numI2C].pinSDA[i].pin;
        	pinDriver->af = hardwareI2C[numI2C].pinSDA[i].af;
            return true;
        }
    }

    return false;
}


#endif // USAR_I2C
