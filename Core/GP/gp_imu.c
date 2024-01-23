/***************************************************************************************
**  gp_imu.c - Funciones y grupo de parametros de la IMU
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 17/06/2020
**  Fecha de modificacion: 13/09/2020
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
#include "gp_imu.h"

#ifdef USAR_IMU
#include "Drivers/io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define FREC_FILTRO_ACEL_IMU        100.0f
#define FREC_FILTRO_GIRO_IMU        100.0f

#ifndef TIPO_IMU_1
  #define TIPO_IMU_1                IMU_NINGUNO
#endif

#ifndef AUXILIAR_IMU_1
  #define AUX_IMU_1                 false
#else
  #define AUX_IMU_1                 true
#endif

#ifndef TIPO_BUS_IMU_1
  #define TIPO_BUS_IMU_1            BUS_NINGUNO
#endif

#ifndef DISP_BUS_IMU_1
  #define DISP_BUS_IMU_1            0
#endif

#ifndef CS_SPI_BUS_IMU_1
  #define CS_SPI_BUS_IMU_1          NINGUNO
#endif

#ifndef DIR_I2C_BUS_IMU_1
  #define DIR_I2C_BUS_IMU_1         0x00
#endif

#ifndef DRDY_IMU_1
  #define DRDY_IMU_1                NINGUNO
#endif

#ifndef ROTACION_IMU_1
  #define ROTACION_IMU_1            0
#endif

#ifndef VOLTEADO_IMU_1
  #define VOLTEADO_IMU_1            false
#endif

#ifndef TIPO_IMU_2
  #define TIPO_IMU_2                IMU_NINGUNO
#endif

#ifndef AUXILIAR_IMU_2
  #define AUX_IMU_2                 false
#else
  #define AUX_IMU_2                 true
#endif

#ifndef TIPO_BUS_IMU_2
  #define TIPO_BUS_IMU_2            BUS_NINGUNO
#endif

#ifndef DISP_BUS_IMU_2
  #define DISP_BUS_IMU_2            0
#endif

#ifndef CS_SPI_BUS_IMU_2
  #define CS_SPI_BUS_IMU_2          NINGUNO
#endif

#ifndef DIR_I2C_BUS_IMU_2
  #define DIR_I2C_BUS_IMU_2         0x00
#endif

#ifndef DRDY_IMU_2
  #define DRDY_IMU_2                NINGUNO
#endif

#ifndef ROTACION_IMU_2
  #define ROTACION_IMU_2            0
#endif

#ifndef VOLTEADO_IMU_2
  #define VOLTEADO_IMU_2            false
#endif

#ifndef TIPO_IMU_3
  #define TIPO_IMU_3                IMU_NINGUNO
#endif

#ifndef AUXILIAR_IMU_3
  #define AUX_IMU_3                 false
#else
  #define AUX_IMU_3                 true
#endif

#ifndef TIPO_BUS_IMU_3
  #define TIPO_BUS_IMU_3            BUS_NINGUNO
#endif

#ifndef DISP_BUS_IMU_3
  #define DISP_BUS_IMU_3            0
#endif

#ifndef CS_SPI_BUS_IMU_3
  #define CS_SPI_BUS_IMU_3          NINGUNO
#endif

#ifndef DIR_I2C_BUS_IMU_3
  #define DIR_I2C_BUS_IMU_3         0x00
#endif

#ifndef DRDY_IMU_3
  #define DRDY_IMU_3                NINGUNO
#endif

#ifndef ROTACION_IMU_3
  #define ROTACION_IMU_3            0
#endif

#ifndef VOLTEADO_IMU_3
  #define VOLTEADO_IMU_3            false
#endif

#ifndef TIPO_IMU_4
  #define TIPO_IMU_4                IMU_NINGUNO
#endif

#ifndef AUXILIAR_IMU_4
  #define AUX_IMU_4                 false
#else
  #define AUX_IMU_4                 true
#endif

#ifndef TIPO_BUS_IMU_4
  #define TIPO_BUS_IMU_4            BUS_NINGUNO
#endif

#ifndef DISP_BUS_IMU_4
  #define DISP_BUS_IMU_4            0
#endif

#ifndef CS_SPI_BUS_IMU_4
  #define CS_SPI_BUS_IMU_4          NINGUNO
#endif

#ifndef DIR_I2C_BUS_IMU_4
  #define DIR_I2C_BUS_IMU_4         0x00
#endif

#ifndef DRDY_IMU_4
  #define DRDY_IMU_4                NINGUNO
#endif

#ifndef ROTACION_IMU_4
  #define ROTACION_IMU_4            0
#endif

#ifndef VOLTEADO_IMU_4
  #define VOLTEADO_IMU_4            false
#endif

#ifndef TIPO_IMU_5
  #define TIPO_IMU_5                IMU_NINGUNO
#endif

#ifndef AUXILIAR_IMU_5
  #define AUX_IMU_5                 false
#else
  #define AUX_IMU_5                 true
#endif

#ifndef TIPO_BUS_IMU_5
  #define TIPO_BUS_IMU_5            BUS_NINGUNO
#endif

#ifndef DISP_BUS_IMU_5
  #define DISP_BUS_IMU_5            0
#endif

#ifndef CS_SPI_BUS_IMU_5
  #define CS_SPI_BUS_IMU_5          NINGUNO
#endif

#ifndef DIR_I2C_BUS_IMU_5
  #define DIR_I2C_BUS_IMU_5         0x00
#endif

#ifndef DRDY_IMU_5
  #define DRDY_IMU_5                NINGUNO
#endif

#ifndef ROTACION_IMU_5
  #define ROTACION_IMU_5            0
#endif

#ifndef VOLTEADO_IMU_5
  #define VOLTEADO_IMU_5            false
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_ARRAY_GP_CON_FN_RESET(configIMU_t, NUM_MAX_IMU, configIMU, GP_CONFIGURACION_IMU, 2);

static const configIMU_t configIMUdefecto[] = {
    { TIPO_IMU_1, AUX_IMU_1, TIPO_BUS_IMU_1, DISP_BUS_IMU_1, DEFIO_TAG(CS_SPI_BUS_IMU_1), DIR_I2C_BUS_IMU_1, DEFIO_TAG(DRDY_IMU_1), FREC_FILTRO_ACEL_IMU, FREC_FILTRO_GIRO_IMU, {ROTACION_IMU_1, VOLTEADO_IMU_1}, FREC_ACTUALIZAR_IMU_HZ, FREC_LEER_IMU_HZ},
    { TIPO_IMU_2, AUX_IMU_2, TIPO_BUS_IMU_2, DISP_BUS_IMU_2, DEFIO_TAG(CS_SPI_BUS_IMU_2), DIR_I2C_BUS_IMU_2, DEFIO_TAG(DRDY_IMU_2), FREC_FILTRO_ACEL_IMU, FREC_FILTRO_GIRO_IMU, {ROTACION_IMU_2, VOLTEADO_IMU_2}, FREC_ACTUALIZAR_IMU_HZ, FREC_LEER_IMU_HZ},
    { TIPO_IMU_3, AUX_IMU_3, TIPO_BUS_IMU_3, DISP_BUS_IMU_3, DEFIO_TAG(CS_SPI_BUS_IMU_3), DIR_I2C_BUS_IMU_3, DEFIO_TAG(DRDY_IMU_3), FREC_FILTRO_ACEL_IMU, FREC_FILTRO_GIRO_IMU, {ROTACION_IMU_3, VOLTEADO_IMU_3}, FREC_ACTUALIZAR_IMU_HZ, FREC_LEER_IMU_HZ},
    { TIPO_IMU_4, AUX_IMU_4, TIPO_BUS_IMU_4, DISP_BUS_IMU_4, DEFIO_TAG(CS_SPI_BUS_IMU_4), DIR_I2C_BUS_IMU_4, DEFIO_TAG(DRDY_IMU_4), FREC_FILTRO_ACEL_IMU, FREC_FILTRO_GIRO_IMU, {ROTACION_IMU_4, VOLTEADO_IMU_4}, FREC_ACTUALIZAR_IMU_HZ, FREC_LEER_IMU_HZ},
    { TIPO_IMU_5, AUX_IMU_5, TIPO_BUS_IMU_5, DISP_BUS_IMU_5, DEFIO_TAG(CS_SPI_BUS_IMU_5), DIR_I2C_BUS_IMU_5, DEFIO_TAG(DRDY_IMU_5), FREC_FILTRO_ACEL_IMU, FREC_FILTRO_GIRO_IMU, {ROTACION_IMU_5, VOLTEADO_IMU_5}, FREC_ACTUALIZAR_IMU_HZ, FREC_LEER_IMU_HZ},
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/


/***************************************************************************************
**  Nombre:         void fnResetGP_configIMU(configIMU_t *configIMU)
**  Descripcion:    Funcion de reset de la configuracion de la IMU
**  Parametros:     Configuracion a resetear
**  Retorno:        Ninguno
****************************************************************************************/
void fnResetGP_configIMU(configIMU_t *configIMU)
{
    for (uint8_t i = 0; i < NUM_MAX_IMU; i++) {
        configIMU[i].tipoIMU = configIMUdefecto[i].tipoIMU;
        configIMU[i].auxiliar = configIMUdefecto[i].auxiliar;
        configIMU[i].dispBus = configIMUdefecto[i].dispBus;
        configIMU[i].drdy = configIMUdefecto[i].drdy;
        configIMU[i].bus = configIMUdefecto[i].bus;
        configIMU[i].frecFiltroAcel = configIMUdefecto[i].frecFiltroAcel;
        configIMU[i].frecFiltroGiro = configIMUdefecto[i].frecFiltroGiro;
        configIMU[i].rotacion = configIMUdefecto[i].rotacion;
        configIMU[i].frecActualizar = configIMUdefecto[i].frecActualizar;
        configIMU[i].frecLeer = configIMUdefecto[i].frecLeer;

        switch (configIMU[i].bus) {
            case BUS_I2C:
                configIMU[i].dirI2C = configIMUdefecto[i].dirI2C;
        	    break;

            case BUS_SPI:
                configIMU[i].csSPI = configIMUdefecto[i].csSPI;
        	    break;

            default:
                break;
        }
    }
}

#endif
