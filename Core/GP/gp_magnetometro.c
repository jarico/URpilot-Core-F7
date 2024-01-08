/***************************************************************************************
**  gp_magnetometro.c - Funciones y grupo de parametros del magnetometro
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 13/06/2020
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
#include "gp_magnetometro.h"

#ifdef USAR_MAG
#include "Drivers/io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define RANGO_FILTRO_MAG            0           // La nueva muestra tiene que esar dentro de este porcentaje.
                                                // Este valor es un porcentaje de 0 a 100 sobre el valor medio

#ifndef TIPO_MAG_1
  #define TIPO_MAG_1                MAG_NINGUNO
#endif

#ifndef AUXILIAR_MAG_1
  #define AUX_MAG_1                 false
#else
  #define AUX_MAG_1                 true
#endif

#ifndef TIPO_BUS_MAG_1
  #define TIPO_BUS_MAG_1            BUS_NINGUNO
#endif

#ifndef DISP_BUS_MAG_1
  #define DISP_BUS_MAG_1            -1
#endif

#ifndef CS_SPI_BUS_MAG_1
  #define CS_SPI_BUS_MAG_1          NINGUNO
#endif

#ifndef DIR_I2C_BUS_MAG_1
  #define DIR_I2C_BUS_MAG_1         0x00
#endif

#ifndef DRDY_MAG_1
  #define DRDY_MAG_1                NINGUNO
#endif

#ifndef ROTACION_MAG_1
  #define ROTACION_MAG_1            0
#endif

#ifndef VOLTEADO_MAG_1
  #define VOLTEADO_MAG_1            false
#endif

#ifndef TIPO_MAG_2
  #define TIPO_MAG_2                MAG_NINGUNO
#endif

#ifndef AUXILIAR_MAG_2
  #define AUX_MAG_2                 false
#else
  #define AUX_MAG_2                 true
#endif

#ifndef TIPO_BUS_MAG_2
  #define TIPO_BUS_MAG_2            BUS_NINGUNO
#endif

#ifndef DISP_BUS_MAG_2
  #define DISP_BUS_MAG_2            -1
#endif

#ifndef CS_SPI_BUS_MAG_2
  #define CS_SPI_BUS_MAG_2          NINGUNO
#endif

#ifndef DIR_I2C_BUS_MAG_2
  #define DIR_I2C_BUS_MAG_2         0x00
#endif

#ifndef DRDY_MAG_2
  #define DRDY_MAG_2                NINGUNO
#endif

#ifndef ROTACION_MAG_2
  #define ROTACION_MAG_2            0
#endif

#ifndef VOLTEADO_MAG_2
  #define VOLTEADO_MAG_2            false
#endif

#ifndef TIPO_MAG_3
  #define TIPO_MAG_3                MAG_NINGUNO
#endif

#ifndef AUXILIAR_MAG_3
  #define AUX_MAG_3                 false
#else
  #define AUX_MAG_3                 true
#endif

#ifndef TIPO_BUS_MAG_3
  #define TIPO_BUS_MAG_3            BUS_NINGUNO
#endif

#ifndef DISP_BUS_MAG_3
  #define DISP_BUS_MAG_3            -1
#endif

#ifndef CS_SPI_BUS_MAG_3
  #define CS_SPI_BUS_MAG_3          NINGUNO
#endif

#ifndef DIR_I2C_BUS_MAG_3
  #define DIR_I2C_BUS_MAG_3         0x00
#endif

#ifndef DRDY_MAG_3
  #define DRDY_MAG_3                NINGUNO
#endif

#ifndef ROTACION_MAG_3
  #define ROTACION_MAG_3            0
#endif

#ifndef VOLTEADO_MAG_3
  #define VOLTEADO_MAG_3            false
#endif

#ifndef TIPO_MAG_4
  #define TIPO_MAG_4                MAG_NINGUNO
#endif

#ifndef AUXILIAR_MAG_4
  #define AUX_MAG_4                 false
#else
  #define AUX_MAG_4                 true
#endif

#ifndef TIPO_BUS_MAG_4
  #define TIPO_BUS_MAG_4            BUS_NINGUNO
#endif

#ifndef DISP_BUS_MAG_4
  #define DISP_BUS_MAG_4            -1
#endif

#ifndef CS_SPI_BUS_MAG_4
  #define CS_SPI_BUS_MAG_4          NINGUNO
#endif

#ifndef DIR_I2C_BUS_MAG_4
  #define DIR_I2C_BUS_MAG_4         0x00
#endif

#ifndef DRDY_MAG_4
  #define DRDY_MAG_4                NINGUNO
#endif

#ifndef ROTACION_MAG_4
  #define ROTACION_MAG_4            0
#endif

#ifndef VOLTEADO_MAG_4
  #define VOLTEADO_MAG_4            false
#endif

#ifndef TIPO_MAG_5
  #define TIPO_MAG_5                MAG_NINGUNO
#endif

#ifndef AUXILIAR_MAG_5
  #define AUX_MAG_5                 false
#else
  #define AUX_MAG_5                 true
#endif

#ifndef TIPO_BUS_MAG_5
  #define TIPO_BUS_MAG_5            BUS_NINGUNO
#endif

#ifndef DISP_BUS_MAG_5
  #define DISP_BUS_MAG_5            -1
#endif

#ifndef CS_SPI_BUS_MAG_5
  #define CS_SPI_BUS_MAG_5          NINGUNO
#endif

#ifndef DIR_I2C_BUS_MAG_5
  #define DIR_I2C_BUS_MAG_5         0x00
#endif

#ifndef DRDY_MAG_5
  #define DRDY_MAG_5                NINGUNO
#endif

#ifndef ROTACION_MAG_5
  #define ROTACION_MAG_5            0
#endif

#ifndef VOLTEADO_MAG_5
  #define VOLTEADO_MAG_5            false
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_ARRAY_GP_CON_FN_RESET(configMag_t, NUM_MAX_MAG, configMag, GP_CONFIGURACION_MAGNETOMETRO, 5);

static const configMag_t configMagDefecto[] = {
    { TIPO_MAG_1, AUX_MAG_1, TIPO_BUS_MAG_1, DISP_BUS_MAG_1, DEFIO_TAG(CS_SPI_BUS_MAG_1), DIR_I2C_BUS_MAG_1, DEFIO_TAG(DRDY_MAG_1), RANGO_FILTRO_MAG, {ROTACION_MAG_1, VOLTEADO_MAG_1}, FREC_ACTUALIZAR_MAG_HZ, FREC_LEER_MAG_HZ},
    { TIPO_MAG_2, AUX_MAG_2, TIPO_BUS_MAG_2, DISP_BUS_MAG_2, DEFIO_TAG(CS_SPI_BUS_MAG_2), DIR_I2C_BUS_MAG_2, DEFIO_TAG(DRDY_MAG_2), RANGO_FILTRO_MAG, {ROTACION_MAG_2, VOLTEADO_MAG_2}, FREC_ACTUALIZAR_MAG_HZ, FREC_LEER_MAG_HZ},
    { TIPO_MAG_3, AUX_MAG_3, TIPO_BUS_MAG_3, DISP_BUS_MAG_3, DEFIO_TAG(CS_SPI_BUS_MAG_3), DIR_I2C_BUS_MAG_3, DEFIO_TAG(DRDY_MAG_3), RANGO_FILTRO_MAG, {ROTACION_MAG_3, VOLTEADO_MAG_3}, FREC_ACTUALIZAR_MAG_HZ, FREC_LEER_MAG_HZ},
    { TIPO_MAG_4, AUX_MAG_4, TIPO_BUS_MAG_4, DISP_BUS_MAG_4, DEFIO_TAG(CS_SPI_BUS_MAG_4), DIR_I2C_BUS_MAG_4, DEFIO_TAG(DRDY_MAG_4), RANGO_FILTRO_MAG, {ROTACION_MAG_4, VOLTEADO_MAG_4}, FREC_ACTUALIZAR_MAG_HZ, FREC_LEER_MAG_HZ},
    { TIPO_MAG_5, AUX_MAG_5, TIPO_BUS_MAG_5, DISP_BUS_MAG_5, DEFIO_TAG(CS_SPI_BUS_MAG_5), DIR_I2C_BUS_MAG_5, DEFIO_TAG(DRDY_MAG_5), RANGO_FILTRO_MAG, {ROTACION_MAG_5, VOLTEADO_MAG_5}, FREC_ACTUALIZAR_MAG_HZ, FREC_LEER_MAG_HZ},
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/


/***************************************************************************************
**  Nombre:         void fnResetGP_configMag(configMag_t *configMag)
**  Descripcion:    Funcion de reset de la configuracion del magnetometro
**  Parametros:     Configuracion a resetear
**  Retorno:        Ninguno
****************************************************************************************/
void fnResetGP_configMag(configMag_t *configMag)
{
    for (uint8_t i = 0; i < NUM_MAX_MAG; i++) {
    	configMag[i].tipoMag = configMagDefecto[i].tipoMag;
        configMag[i].auxiliar = configMagDefecto[i].auxiliar;
    	configMag[i].dispBus = configMagDefecto[i].dispBus;
    	configMag[i].bus = configMagDefecto[i].bus;
    	configMag[i].drdy = configMagDefecto[i].drdy;
        configMag[i].rotacion = configMagDefecto[i].rotacion;
    	configMag[i].frecActualizar = configMagDefecto[i].frecActualizar;
    	configMag[i].frecLeer = configMagDefecto[i].frecLeer;

        switch (configMag[i].bus) {
            case BUS_I2C:
                configMag[i].dirI2C = configMagDefecto[i].dirI2C;
        	    break;

            case BUS_SPI:
                configMag[i].csSPI = configMagDefecto[i].csSPI;
        	    break;

            default:
                break;
        }
    }
}

#endif
