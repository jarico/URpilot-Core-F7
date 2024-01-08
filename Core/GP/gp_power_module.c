/***************************************************************************************
**  gp_power_module.c - Funciones y grupo de parametros del Power Module
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 11/12/2020
**  Fecha de modificacion: 11/12/2020
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
#include "gp_power_module.h"

#ifdef USAR_POWER_MODULE
#include "Drivers/io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#ifndef TIPO_POWER_MODULE_1
  #define TIPO_POWER_MODULE_1          POWER_MODULE_NINGUNO
#endif

#ifndef MULT_V_POWER_MODULE_1
  #define MULT_V_POWER_MODULE_1        0
#endif

#ifndef MULT_I_POWER_MODULE_1
  #define MULT_I_POWER_MODULE_1        0
#endif

#ifndef DISP_BUS_POWER_MODULE_1
  #define DISP_BUS_POWER_MODULE_1      -1
#endif

#ifndef DIR_I2C_BUS_POWER_MODULE_1
  #define DIR_I2C_BUS_POWER_MODULE_1   0x00
#endif

#ifndef TIPO_POWER_MODULE_2
  #define TIPO_POWER_MODULE_2          POWER_MODULE_NINGUNO
#endif

#ifndef MULT_V_POWER_MODULE_2
  #define MULT_V_POWER_MODULE_2        0
#endif

#ifndef MULT_I_POWER_MODULE_2
  #define MULT_I_POWER_MODULE_2        0
#endif

#ifndef DISP_BUS_POWER_MODULE_2
  #define DISP_BUS_POWER_MODULE_2      -1
#endif

#ifndef DIR_I2C_BUS_POWER_MODULE_2
  #define DIR_I2C_BUS_POWER_MODULE_2   0x00
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_ARRAY_GP_CON_FN_RESET(configPowerModule_t, NUM_MAX_POWER_MODULE, configPowerModule, GP_CONFIGURACION_POWER_MODULE, 1);

static const configPowerModule_t configPowerModuleDefecto[] = {
    { TIPO_POWER_MODULE_1, MULT_V_POWER_MODULE_1, MULT_I_POWER_MODULE_1, DISP_BUS_POWER_MODULE_1, DIR_I2C_BUS_POWER_MODULE_1, FREC_LEER_POWER_MODULE_HZ, FREC_ACTUALIZAR_POWER_MODULE_HZ},
    { TIPO_POWER_MODULE_2, MULT_V_POWER_MODULE_2, MULT_I_POWER_MODULE_2, DISP_BUS_POWER_MODULE_2, DIR_I2C_BUS_POWER_MODULE_2, FREC_LEER_POWER_MODULE_HZ, FREC_ACTUALIZAR_POWER_MODULE_HZ},
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void fnResetGP_configPowerModule(configPowerModule_t *configPowerModule)
**  Descripcion:    Funcion de reset de la configuracion del Power Module
**  Parametros:     Configuracion a resetear
**  Retorno:        Ninguno
****************************************************************************************/
void fnResetGP_configPowerModule(configPowerModule_t *configPowerModule)
{
    for (uint8_t i = 0; i < NUM_MAX_POWER_MODULE; i++) {
        configPowerModule[i].tipo = configPowerModuleDefecto[i].tipo;

        switch (configPowerModuleDefecto[i].tipo) {
            case POWER_MODULE_ANALOGICO:
                configPowerModule[i].multV = configPowerModuleDefecto[i].multV;
                configPowerModule[i].multI = configPowerModuleDefecto[i].multI;
    	        break;

            case POWER_MODULE_I2C:
                configPowerModule[i].dispBus = configPowerModuleDefecto[i].dispBus;
                configPowerModule[i].dirI2C = configPowerModuleDefecto[i].dirI2C;
    	        break;

    	    default:
                break;
        }

        configPowerModule[i].frecLeer = configPowerModuleDefecto[i].frecLeer;
        configPowerModule[i].frecActualizar = configPowerModuleDefecto[i].frecActualizar;
    }
}

#endif
