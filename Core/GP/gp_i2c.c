/***************************************************************************************
**  gp_i2c.c - Funciones y grupo de parametros del I2C
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
#include "gp_i2c.h"

#ifdef USAR_I2C
#include "Drivers/io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#ifndef PIN_SCL_I2C_1
  #define PIN_SCL_I2C_1          NINGUNO
#endif

#ifndef PIN_SDA_I2C_1
  #define PIN_SDA_I2C_1          NINGUNO
#endif

#ifndef USAR_PULLUP_I2C_1
  #define PULLUP_I2C_1           false
#else
  #define PULLUP_I2C_1           true
#endif

#ifndef USAR_OVERCLOCK_I2C_1
  #define OVERCLOCK_I2C_1        false
#else
  #define OVERCLOCK_I2C_1        true
#endif

#ifndef PIN_SCL_I2C_2
  #define PIN_SCL_I2C_2          NINGUNO
#endif

#ifndef PIN_SDA_I2C_2
  #define PIN_SDA_I2C_2          NINGUNO
#endif

#ifndef USAR_PULLUP_I2C_2
  #define PULLUP_I2C_2           false
#else
  #define PULLUP_I2C_2           true
#endif

#ifndef USAR_OVERCLOCK_I2C_2
  #define OVERCLOCK_I2C_2        false
#else
  #define OVERCLOCK_I2C_2        true
#endif

#ifndef PIN_SCL_I2C_3
  #define PIN_SCL_I2C_3          NINGUNO
#endif

#ifndef PIN_SDA_I2C_3
  #define PIN_SDA_I2C_3          NINGUNO
#endif

#ifndef USAR_PULLUP_I2C_3
  #define PULLUP_I2C_3           false
#else
  #define PULLUP_I2C_3           true
#endif

#ifndef USAR_OVERCLOCK_I2C_3
  #define OVERCLOCK_I2C_3        false
#else
  #define OVERCLOCK_I2C_3        true
#endif

#ifndef PIN_SCL_I2C_4
  #define PIN_SCL_I2C_4          NINGUNO
#endif

#ifndef PIN_SDA_I2C_4
  #define PIN_SDA_I2C_4          NINGUNO
#endif

#ifndef USAR_PULLUP_I2C_4
  #define PULLUP_I2C_4           false
#else
  #define PULLUP_I2C_4           true
#endif

#ifndef USAR_OVERCLOCK_I2C_4
  #define OVERCLOCK_I2C_4        false
#else
  #define OVERCLOCK_I2C_4        true
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_ARRAY_GP_CON_FN_RESET(configI2C_t, NUM_MAX_I2C, configI2C, GP_CONFIGURACION_I2C, 1);

static const configI2C_t configI2Cdefecto[] = {
    { DEFIO_TAG(PIN_SCL_I2C_1), DEFIO_TAG(PIN_SDA_I2C_1), OVERCLOCK_I2C_1, PULLUP_I2C_1},
    { DEFIO_TAG(PIN_SCL_I2C_2), DEFIO_TAG(PIN_SDA_I2C_2), OVERCLOCK_I2C_2, PULLUP_I2C_2},
    { DEFIO_TAG(PIN_SCL_I2C_3), DEFIO_TAG(PIN_SDA_I2C_3), OVERCLOCK_I2C_3, PULLUP_I2C_3},
    { DEFIO_TAG(PIN_SCL_I2C_4), DEFIO_TAG(PIN_SDA_I2C_4), OVERCLOCK_I2C_4, PULLUP_I2C_4},
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/


/***************************************************************************************
**  Nombre:         void fnResetGP_configI2C(configI2C_t *configI2C)
**  Descripcion:    Funcion de reset de la configuracion del I2C
**  Parametros:     Configuracion a resetear
**  Retorno:        Ninguno
****************************************************************************************/
void fnResetGP_configI2C(configI2C_t *configI2C)
{
    for (uint8_t i = 0; i < NUM_MAX_I2C; i++) {
    	configI2C[i].pinSCL = configI2Cdefecto[i].pinSCL;
    	configI2C[i].pinSDA = configI2Cdefecto[i].pinSDA;
    	configI2C[i].overclock = configI2Cdefecto[i].overclock;
    	configI2C[i].pullup = configI2Cdefecto[i].pullup;
    }
}

#endif
