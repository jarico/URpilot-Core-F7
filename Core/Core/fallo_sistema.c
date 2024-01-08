/***************************************************************************************
**  fallo_sistema.c - Funciones para fallos de sistema
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 10/05/2019
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
#include "fallo_sistema.h"
#include "Drivers/tiempo.h"
#include "Drivers/reset.h"
#include "led_estado.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define REPETICIONES_CODIGO_FALLO   5
#define REPETICIONES_FALLO          5
#define DURACION_CODIGO_FALLO_MS    250


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void indicarFallo(falloSistema_e fallo, uint8_t repeticiones);
void secuenciaFallo(uint8_t repeticiones, uint16_t duracion, uint8_t color1, uint8_t color2);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void falloSistema(falloSistema_e fallo)
**  Descripcion:    Ejecuta la secuencia del fallo correspondiente y resetea la placa
**  Parametros:     Tipo de fallo
**  Retorno:        Ninguno
****************************************************************************************/
void falloSistema(falloSistema_e fallo)
{
	indicarFallo(fallo, REPETICIONES_CODIGO_FALLO);
	resetSistema();
}


/***************************************************************************************
**  Nombre:         void indicarFallo(falloSistema_e fallo, uint8_t repeticiones)
**  Descripcion:    Ejecuta la secuencia del fallo correspondiente durante x repeticiones
**  Parametros:     Tipo de fallo, repeticiones
**  Retorno:        Ninguno
****************************************************************************************/
void indicarFallo(falloSistema_e fallo, uint8_t repeticiones)
{
    uint8_t color1;
    uint8_t color2;

    switch (fallo) {
        case FALLO_ESCRITURA_FLASH:
            color1 = ROJO;
            color2 = NEGRO;
            break;

#ifdef USAR_SD
        case FALLO_INICIAR_SD:
            color1 = ROJO;
            color2 = BLANCO;
            break;
#endif

#ifdef USAR_IMU
        case FALLO_INICIAR_IMU:
            color1 = ROJO;
            color2 = AMARILLO;
            break;
#endif

#ifdef USAR_BARO
        case FALLO_INICIAR_BARO:
            color1 = ROJO;
            color2 = AZUL;
            break;
#endif

#ifdef USAR_MAG
        case FALLO_INICIAR_MAG:
            color1 = ROJO;
            color2 = MORADO;
            break;
#endif

#ifdef USAR_GPS
        case FALLO_INICIAR_GPS:
            color1 = ROJO;
            color2 = CIAN;
            break;
#endif

#ifdef USAR_RADIO
        case FALLO_INICIAR_RADIO:
            color1 = BLANCO;
            color2 = AZUL;
            break;
#endif

#ifdef USAR_MOTORES
        case FALLO_INICIAR_MOTORES:
            color1 = BLANCO;
            color2 = VERDE;
            break;
#endif

        default:
            color1 = ROJO;
            color2 = NEGRO;
            break;
    }

    while (repeticiones--) {
        secuenciaFallo(fallo, DURACION_CODIGO_FALLO_MS, color1, color2);
        delay(1000);
    }
}


/***************************************************************************************
**  Nombre:         void secuenciaFallo(uint8_t repeticiones, uint16_t duracion)
**  Descripcion:    Ejecuta la secuencia de un fallo
**  Parametros:     Repeticiones, duracion, color 1, color 2
**  Retorno:        Ninguno
****************************************************************************************/
void secuenciaFallo(uint8_t repeticiones, uint16_t duracion, uint8_t color1, uint8_t color2)
{
    if (repeticiones) {
    	escribirLedEstado(color1);

        while (repeticiones--) {
        	delay(duracion);
        	escribirLedEstado(color2);
            delay(duracion);
            escribirLedEstado(color1);

        }
    }
}
