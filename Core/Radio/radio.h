/***************************************************************************************
**  rc.h - Funciones relativas al sistema receptor de radio
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 10/07/2019
**  Fecha de modificacion: 20/09/2020
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

#ifndef __RADIO_H_
#define __RADIO_H_

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "GP/gp.h"
#include "Drivers/uart.h"
#include "Drivers/timer.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NUM_MAX_CANALES_RADIO     18
#define NUM_MIN_CANALES_RADIO     5
#define CANALES_PWM_RADIO         8
#define VALOR_MAXIMO_RADIO        2000
#define VALOR_MINIMO_RADIO        1000
#define VALOR_MEDIO_RADIO         ((VALOR_MAXIMO_RADIO + VALOR_MINIMO_RADIO) / 2)


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    RX_ENTRADA_1 = 0,
    RX_ENTRADA_2,
    RX_ENTRADA_3,
    RX_ENTRADA_4,
    RX_ENTRADA_5,
    RX_ENTRADA_6,
    RX_ENTRADA_7,
    RX_ENTRADA_8,
} numEntradaRadio_e;

typedef enum {
    RX_NINGUNO = -1,
	RX_PPM     =  0,
    RX_IBUS,
    RX_SBUS,
} protocoloRadio_e;

typedef struct {
    uint16_t canales[NUM_MAX_CANALES_RADIO];
    uint8_t numCanales;
    uint32_t contadorEntradas;
    uint32_t ultimoContadorEntradas;
    bool failsafe;
    bool iniciada;
    bool nuevaEntrada;
    uint32_t ultimaMedida;
} radio_t;

typedef struct {
    bool (*iniciarRadio)(void);
    void (*leerRadio)(uint32_t tiempoActual);
} tablaFnRadio_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
extern tablaFnRadio_t tablaFnRadioPPM;
extern tablaFnRadio_t tablaFnRadioIBUS;
extern tablaFnRadio_t tablaFnRadioSBUS;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarRadio(void);
void leerRadio(uint32_t tiempoActual);
void anadirRecepcionRadio(uint8_t numValores, uint16_t *valores);
bool radioOperativa(void);
bool radioEnFailsafe(void);
void activarFailsafeRadio(void);
void desactivarFailsafeRadio(void);
bool nuevaEntradaRadioValida(void);
uint16_t canalRadio(uint8_t canal);
void canalesRadio(uint16_t *canales);

#endif // __RADIO_H_
