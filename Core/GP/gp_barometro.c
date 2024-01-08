/***************************************************************************************
**  gp_barometro.c - Funciones y grupo de parametros del barometro
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
#include "gp_barometro.h"

#ifdef USAR_BARO
#include "Drivers/io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define RANGO_FILTRO_BARO            0          // La nueva muestra tiene que esar dentro de este porcentaje.
                                                // Este valor es un porcentaje de 0 a 100 sobre el valor medio

#ifndef TIPO_BARO_1
  #define TIPO_BARO_1                BARO_NINGUNO
#endif

#ifndef AUXILIAR_BARO_1
  #define AUX_BARO_1                 false
#else
  #define AUX_BARO_1                 true
#endif

#ifndef TIPO_BUS_BARO_1
  #define TIPO_BUS_BARO_1            BUS_NINGUNO
#endif

#ifndef DISP_BUS_BARO_1
  #define DISP_BUS_BARO_1            -1
#endif

#ifndef CS_SPI_BUS_BARO_1
  #define CS_SPI_BUS_BARO_1          NINGUNO
#endif

#ifndef DIR_I2C_BUS_BARO_1
  #define DIR_I2C_BUS_BARO_1         0x00
#endif

#ifndef DRDY_BARO_1
  #define DRDY_BARO_1                NINGUNO
#endif

#ifndef TIPO_BARO_2
  #define TIPO_BARO_2                BARO_NINGUNO
#endif

#ifndef AUXILIAR_BARO_2
  #define AUX_BARO_2                 false
#else
  #define AUX_BARO_2                 true
#endif

#ifndef TIPO_BUS_BARO_2
  #define TIPO_BUS_BARO_2            BUS_NINGUNO
#endif

#ifndef DISP_BUS_BARO_2
  #define DISP_BUS_BARO_2            -1
#endif

#ifndef CS_SPI_BUS_BARO_2
  #define CS_SPI_BUS_BARO_2          NINGUNO
#endif

#ifndef DIR_I2C_BUS_BARO_2
  #define DIR_I2C_BUS_BARO_2         0x00
#endif

#ifndef DRDY_BARO_2
  #define DRDY_BARO_2                NINGUNO
#endif

#ifndef TIPO_BARO_3
  #define TIPO_BARO_3                BARO_NINGUNO
#endif

#ifndef AUXILIAR_BARO_3
  #define AUX_BARO_3                 false
#else
  #define AUX_BARO_3                 true
#endif

#ifndef TIPO_BUS_BARO_3
  #define TIPO_BUS_BARO_3            BUS_NINGUNO
#endif

#ifndef DISP_BUS_BARO_3
  #define DISP_BUS_BARO_3            -1
#endif

#ifndef CS_SPI_BUS_BARO_3
  #define CS_SPI_BUS_BARO_3          NINGUNO
#endif

#ifndef DIR_I2C_BUS_BARO_3
  #define DIR_I2C_BUS_BARO_3         0x00
#endif

#ifndef DRDY_BARO_3
  #define DRDY_BARO_3                NINGUNO
#endif

#ifndef TIPO_BARO_4
  #define TIPO_BARO_4                BARO_NINGUNO
#endif

#ifndef AUXILIAR_BARO_4
  #define AUX_BARO_4                 false
#else
  #define AUX_BARO_4                 true
#endif

#ifndef TIPO_BUS_BARO_4
  #define TIPO_BUS_BARO_4            BUS_NINGUNO
#endif

#ifndef DISP_BUS_BARO_4
  #define DISP_BUS_BARO_4            -1
#endif

#ifndef CS_SPI_BUS_BARO_4
  #define CS_SPI_BUS_BARO_4          NINGUNO
#endif

#ifndef DIR_I2C_BUS_BARO_4
  #define DIR_I2C_BUS_BARO_4         0x00
#endif

#ifndef DRDY_BARO_4
  #define DRDY_BARO_4                NINGUNO
#endif

#ifndef TIPO_BARO_5
  #define TIPO_BARO_5                BARO_NINGUNO
#endif

#ifndef AUXILIAR_BARO_5
  #define AUX_BARO_5                 false
#else
  #define AUX_BARO_5                 true
#endif

#ifndef TIPO_BUS_BARO_5
  #define TIPO_BUS_BARO_5            BUS_NINGUNO
#endif

#ifndef DISP_BUS_BARO_5
  #define DISP_BUS_BARO_5            -1
#endif

#ifndef CS_SPI_BUS_BARO_5
  #define CS_SPI_BUS_BARO_5          NINGUNO
#endif

#ifndef DIR_I2C_BUS_BARO_5
  #define DIR_I2C_BUS_BARO_5         0x00
#endif

#ifndef DRDY_BARO_5
  #define DRDY_BARO_5                NINGUNO
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
REGISTRAR_ARRAY_GP_CON_FN_RESET(configBaro_t, NUM_MAX_BARO, configBaro, GP_CONFIGURACION_BAROMETRO, 5);

static const configBaro_t configBaroDefecto[] = {
    { TIPO_BARO_1, AUX_BARO_1, TIPO_BUS_BARO_1, DISP_BUS_BARO_1, DEFIO_TAG(CS_SPI_BUS_BARO_1), DIR_I2C_BUS_BARO_1, DEFIO_TAG(DRDY_BARO_1), RANGO_FILTRO_BARO, FREC_ACTUALIZAR_BARO_HZ, FREC_LEER_BARO_HZ},
    { TIPO_BARO_2, AUX_BARO_2, TIPO_BUS_BARO_2, DISP_BUS_BARO_2, DEFIO_TAG(CS_SPI_BUS_BARO_2), DIR_I2C_BUS_BARO_2, DEFIO_TAG(DRDY_BARO_2), RANGO_FILTRO_BARO, FREC_ACTUALIZAR_BARO_HZ, FREC_LEER_BARO_HZ},
    { TIPO_BARO_3, AUX_BARO_3, TIPO_BUS_BARO_3, DISP_BUS_BARO_3, DEFIO_TAG(CS_SPI_BUS_BARO_3), DIR_I2C_BUS_BARO_3, DEFIO_TAG(DRDY_BARO_3), RANGO_FILTRO_BARO, FREC_ACTUALIZAR_BARO_HZ, FREC_LEER_BARO_HZ},
    { TIPO_BARO_4, AUX_BARO_4, TIPO_BUS_BARO_4, DISP_BUS_BARO_4, DEFIO_TAG(CS_SPI_BUS_BARO_4), DIR_I2C_BUS_BARO_4, DEFIO_TAG(DRDY_BARO_4), RANGO_FILTRO_BARO, FREC_ACTUALIZAR_BARO_HZ, FREC_LEER_BARO_HZ},
    { TIPO_BARO_5, AUX_BARO_5, TIPO_BUS_BARO_5, DISP_BUS_BARO_5, DEFIO_TAG(CS_SPI_BUS_BARO_5), DIR_I2C_BUS_BARO_5, DEFIO_TAG(DRDY_BARO_5), RANGO_FILTRO_BARO, FREC_ACTUALIZAR_BARO_HZ, FREC_LEER_BARO_HZ},
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/


/***************************************************************************************
**  Nombre:         void fnResetGP_configBaro(configBaro_t *configBaro)
**  Descripcion:    Funcion de reset de la configuracion del barometro
**  Parametros:     Configuracion a resetear
**  Retorno:        Ninguno
****************************************************************************************/
void fnResetGP_configBaro(configBaro_t *configBaro)
{
    for (uint8_t i = 0; i < NUM_MAX_BARO; i++) {
        configBaro[i].tipoBaro = configBaroDefecto[i].tipoBaro;
        configBaro[i].auxiliar = configBaroDefecto[i].auxiliar;
        configBaro[i].rangoFiltro = configBaroDefecto[i].rangoFiltro;
        configBaro[i].dispBus = configBaroDefecto[i].dispBus;
        configBaro[i].bus = configBaroDefecto[i].bus;
        configBaro[i].drdy = configBaroDefecto[i].drdy;
        configBaro[i].frecActualizar = configBaroDefecto[i].frecActualizar;
        configBaro[i].frecLeer = configBaroDefecto[i].frecLeer;

        switch (configBaro[i].bus) {
            case BUS_I2C:
                configBaro[i].dirI2C = configBaroDefecto[i].dirI2C;
        	    break;

            case BUS_SPI:
                configBaro[i].csSPI = configBaroDefecto[i].csSPI;
        	    break;

            default:
                break;
        }
    }
}

#endif
