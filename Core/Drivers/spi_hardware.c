/***************************************************************************************
**  spi_hardware.c - Hardware del SPI
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
#include "spi.h"

#ifdef USAR_SPI
#include "GP/gp_spi.h"
#include "io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NUM_MAX_PIN_SEL_SPI     5


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    numSPI_e numSPI;
    SPI_TypeDef *reg;
    pin_t pinSCK[NUM_MAX_PIN_SEL_SPI];
    pin_t pinMISO[NUM_MAX_PIN_SEL_SPI];
    pin_t pinMOSI[NUM_MAX_PIN_SEL_SPI];
} hardwareSPI_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static const hardwareSPI_t hardwareSPI[] = {
    {
        .numSPI = SPI_1,
        .reg = SPI1,
        .pinSCK = {
            { DEFIO_TAG(PA5), GPIO_AF5_SPI1  },
            { DEFIO_TAG(PB3), GPIO_AF5_SPI1  },
            { DEFIO_TAG(PG11), GPIO_AF5_SPI1 },
        },
        .pinMISO = {
            { DEFIO_TAG(PA6), GPIO_AF5_SPI1 },
            { DEFIO_TAG(PB4), GPIO_AF5_SPI1 },
            { DEFIO_TAG(PG9), GPIO_AF5_SPI1 },
        },
        .pinMOSI = {
            { DEFIO_TAG(PA7), GPIO_AF5_SPI1 },
            { DEFIO_TAG(PB5), GPIO_AF5_SPI1 },
            { DEFIO_TAG(PD7), GPIO_AF5_SPI1 },
        },
    },
    {
        .numSPI = SPI_2,
        .reg = SPI2,
        .pinSCK = {
            { DEFIO_TAG(PA9), GPIO_AF5_SPI2  },
            { DEFIO_TAG(PA12), GPIO_AF5_SPI2 },
            { DEFIO_TAG(PB10), GPIO_AF5_SPI2 },
            { DEFIO_TAG(PB13), GPIO_AF5_SPI2 },
            { DEFIO_TAG(PD3), GPIO_AF5_SPI2  },
        },
        .pinMISO = {
            { DEFIO_TAG(PB14), GPIO_AF5_SPI2 },
            { DEFIO_TAG(PC2), GPIO_AF5_SPI2  },
        },
        .pinMOSI = {
            { DEFIO_TAG(PB15), GPIO_AF5_SPI2 },
            { DEFIO_TAG(PC1), GPIO_AF5_SPI2  },
            { DEFIO_TAG(PC3), GPIO_AF5_SPI2  },
        },
    },
    {
        .numSPI = SPI_3,
        .reg = SPI3,
        .pinSCK = {
            { DEFIO_TAG(PB3), GPIO_AF6_SPI3  },
            { DEFIO_TAG(PC10), GPIO_AF6_SPI3 },
        },
        .pinMISO = {
            { DEFIO_TAG(PB4), GPIO_AF6_SPI3  },
            { DEFIO_TAG(PC11), GPIO_AF6_SPI3 },
        },
        .pinMOSI = {
            { DEFIO_TAG(PB2), GPIO_AF7_SPI3  },
            { DEFIO_TAG(PB5), GPIO_AF6_SPI3  },
            { DEFIO_TAG(PC12), GPIO_AF6_SPI3 },
            { DEFIO_TAG(PD6), GPIO_AF5_SPI3  },
        },
    },
#if defined(STM32F767xx)
    {
        .numSPI = SPI_4,
        .reg = SPI4,
        .pinSCK = {
            { DEFIO_TAG(PE2), GPIO_AF5_SPI4  },
            { DEFIO_TAG(PE12), GPIO_AF5_SPI4 },
        },
        .pinMISO = {
            { DEFIO_TAG(PE5), GPIO_AF5_SPI4  },
            { DEFIO_TAG(PE13), GPIO_AF5_SPI4 },
        },
        .pinMOSI = {
            { DEFIO_TAG(PE6), GPIO_AF5_SPI4  },
            { DEFIO_TAG(PE14), GPIO_AF5_SPI4 },
        },
    },
    {
        .numSPI = SPI_5,
        .reg = SPI5,
        .pinSCK = {
            { DEFIO_TAG(PF7), GPIO_AF5_SPI5 },
        },
        .pinMISO = {
            { DEFIO_TAG(PF8), GPIO_AF5_SPI5 },
        },
        .pinMOSI = {
            { DEFIO_TAG(PF9), GPIO_AF5_SPI5  },
            { DEFIO_TAG(PF11), GPIO_AF5_SPI5 },
        },
    },
    {
        .numSPI = SPI_6,
        .reg = SPI6,
        .pinSCK = {
            { DEFIO_TAG(PA5), GPIO_AF8_SPI6  },
            { DEFIO_TAG(PB3), GPIO_AF8_SPI6  },
            { DEFIO_TAG(PG13), GPIO_AF5_SPI6 },
        },
        .pinMISO = {
            { DEFIO_TAG(PA6), GPIO_AF8_SPI6  },
            { DEFIO_TAG(PB4), GPIO_AF8_SPI6  },
            { DEFIO_TAG(PG12), GPIO_AF5_SPI6 },
        },
        .pinMOSI = {
            { DEFIO_TAG(PA7), GPIO_AF8_SPI6  },
            { DEFIO_TAG(PB5), GPIO_AF8_SPI6  },
            { DEFIO_TAG(PG14), GPIO_AF5_SPI6 },
        },
    },
#endif
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool pinSPI(numSPI_e numSPI, uint8_t pinBusqueda, pin_t *pinDriver);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool asignarHALspi(numSPI_e numSPI)
**  Descripcion:    Asigna el HAL a la estructura del driver
**  Parametros:     Numero del driver
**  Retorno:        True si ok
****************************************************************************************/
bool asignarHALspi(numSPI_e numSPI)
{
    spi_t *driver = punteroSPI(numSPI);

    // Asignamos los pines
    if (!pinSPI(numSPI, configSPI(numSPI)->pinSCK, &driver->hal.pinSCK))
        return false;

    if (!pinSPI(numSPI, configSPI(numSPI)->pinMISO, &driver->hal.pinMISO))
        return false;

    if (!pinSPI(numSPI, configSPI(numSPI)->pinMOSI, &driver->hal.pinMOSI))
        return false;

    // Asignamos la instancia
    driver->hal.hspi.Instance = hardwareSPI[numSPI].reg;
    return true;
}


/***************************************************************************************
**  Nombre:         bool pinSPI(numSPI_e numSPI, uint8_t pinBusqueda, pin_t *pinDriver)
**  Descripcion:    Encuentra el pin de la tabla de hardware
**  Parametros:     Numero del SPI, pin a buscar, pin del driver
**  Retorno:        True si OK
****************************************************************************************/
bool pinSPI(numSPI_e numSPI, uint8_t pinBusqueda, pin_t *pinDriver)
{
    for (uint8_t i = 0; i < NUM_MAX_PIN_SEL_SPI; i++) {
        if (pinBusqueda == hardwareSPI[numSPI].pinSCK[i].pin) {
        	pinDriver->pin = hardwareSPI[numSPI].pinSCK[i].pin;
        	pinDriver->af = hardwareSPI[numSPI].pinSCK[i].af;
            return true;
        }

        if (pinBusqueda == hardwareSPI[numSPI].pinMISO[i].pin) {
        	pinDriver->pin = hardwareSPI[numSPI].pinMISO[i].pin;
        	pinDriver->af = hardwareSPI[numSPI].pinMISO[i].af;
            return true;
        }

        if (pinBusqueda == hardwareSPI[numSPI].pinMOSI[i].pin) {
        	pinDriver->pin = hardwareSPI[numSPI].pinMOSI[i].pin;
        	pinDriver->af = hardwareSPI[numSPI].pinMOSI[i].af;
            return true;
        }
    }

    return false;
}

#endif // USAR_SPI
