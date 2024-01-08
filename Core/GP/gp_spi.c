/***************************************************************************************
**  gp_spi.c - Funciones y grupo de parametros del SPI
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
#include "gp_spi.h"

#ifdef USAR_SPI
#include "Drivers/io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#ifndef PIN_SCK_SPI_1
  #define PIN_SCK_SPI_1         NINGUNO
#endif

#ifndef PIN_MISO_SPI_1
  #define PIN_MISO_SPI_1        NINGUNO
#endif

#ifndef PIN_MOSI_SPI_1
  #define PIN_MOSI_SPI_1        NINGUNO
#endif

#ifndef USAR_LEADING_EDGE_SPI_1
  #define LEADING_EDGE_SPI_1    false
#endif

#ifndef PIN_SCK_SPI_2
  #define PIN_SCK_SPI_2         NINGUNO
#endif

#ifndef PIN_MISO_SPI_2
  #define PIN_MISO_SPI_2        NINGUNO
#endif

#ifndef PIN_MOSI_SPI_2
  #define PIN_MOSI_SPI_2        NINGUNO
#endif

#ifndef USAR_LEADING_EDGE_SPI_2
  #define LEADING_EDGE_SPI_2    false
#endif

#ifndef PIN_SCK_SPI_3
  #define PIN_SCK_SPI_3         NINGUNO
#endif

#ifndef PIN_MISO_SPI_3
  #define PIN_MISO_SPI_3        NINGUNO
#endif

#ifndef PIN_MOSI_SPI_3
  #define PIN_MOSI_SPI_3        NINGUNO
#endif

#ifndef USAR_LEADING_EDGE_SPI_3
  #define LEADING_EDGE_SPI_3    false
#endif

#ifndef PIN_SCK_SPI_4
  #define PIN_SCK_SPI_4         NINGUNO
#endif

#ifndef PIN_MISO_SPI_4
  #define PIN_MISO_SPI_4        NINGUNO
#endif

#ifndef PIN_MOSI_SPI_4
  #define PIN_MOSI_SPI_4        NINGUNO
#endif

#ifndef USAR_LEADING_EDGE_SPI_4
  #define LEADING_EDGE_SPI_4    false
#endif

#ifndef PIN_SCK_SPI_5
  #define PIN_SCK_SPI_5         NINGUNO
#endif

#ifndef PIN_MISO_SPI_5
  #define PIN_MISO_SPI_5        NINGUNO
#endif

#ifndef PIN_MOSI_SPI_5
  #define PIN_MOSI_SPI_5        NINGUNO
#endif

#ifndef USAR_LEADING_EDGE_SPI_5
  #define LEADING_EDGE_SPI_5    false
#endif

#ifndef PIN_SCK_SPI_6
  #define PIN_SCK_SPI_6         NINGUNO
#endif

#ifndef PIN_MISO_SPI_6
  #define PIN_MISO_SPI_6        NINGUNO
#endif

#ifndef PIN_MOSI_SPI_6
  #define PIN_MOSI_SPI_6        NINGUNO
#endif

#ifndef USAR_LEADING_EDGE_SPI_6
  #define LEADING_EDGE_SPI_6    false
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_ARRAY_GP_CON_FN_RESET(configSPI_t, NUM_MAX_SPI, configSPI, GP_CONFIGURACION_SPI, 1);

static const configSPI_t configSPIdefecto[] = {
    { DEFIO_TAG(PIN_SCK_SPI_1), DEFIO_TAG(PIN_MISO_SPI_1), DEFIO_TAG(PIN_MOSI_SPI_1), LEADING_EDGE_SPI_1},
    { DEFIO_TAG(PIN_SCK_SPI_2), DEFIO_TAG(PIN_MISO_SPI_2), DEFIO_TAG(PIN_MOSI_SPI_2), LEADING_EDGE_SPI_2},
    { DEFIO_TAG(PIN_SCK_SPI_3), DEFIO_TAG(PIN_MISO_SPI_3), DEFIO_TAG(PIN_MOSI_SPI_3), LEADING_EDGE_SPI_3},
    { DEFIO_TAG(PIN_SCK_SPI_4), DEFIO_TAG(PIN_MISO_SPI_4), DEFIO_TAG(PIN_MOSI_SPI_4), LEADING_EDGE_SPI_4},
    { DEFIO_TAG(PIN_SCK_SPI_5), DEFIO_TAG(PIN_MISO_SPI_5), DEFIO_TAG(PIN_MOSI_SPI_5), LEADING_EDGE_SPI_5},
    { DEFIO_TAG(PIN_SCK_SPI_6), DEFIO_TAG(PIN_MISO_SPI_6), DEFIO_TAG(PIN_MOSI_SPI_6), LEADING_EDGE_SPI_6},
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/


/***************************************************************************************
**  Nombre:         void fnResetGP_configSPI(configSPI_t *configSPI)
**  Descripcion:    Funcion de reset de la configuracion del SPI
**  Parametros:     Configuracion a resetear
**  Retorno:        Ninguno
****************************************************************************************/
void fnResetGP_configSPI(configSPI_t *configSPI)
{
    for (uint8_t i = 0; i < NUM_MAX_SPI; i++) {
    	configSPI[i].pinSCK = configSPIdefecto[i].pinSCK;
    	configSPI[i].pinMISO = configSPIdefecto[i].pinMISO;
    	configSPI[i].pinMOSI = configSPIdefecto[i].pinMOSI;
    	configSPI[i].leadingEdge = configSPIdefecto[i].leadingEdge;
    }
}

#endif
