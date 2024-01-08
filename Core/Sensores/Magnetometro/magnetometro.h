/***************************************************************************************
**  magnetometro.h - Funciones y variables comunes a todos los magnetometros
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 01/06/2019
**  Fecha de modificacion: 21/09/2020
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

#ifndef __MAGNETOMETRO_H_
#define __MAGNETOMETRO_H_

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "Drivers/bus.h"
#include "GP/gp.h"
#include "Sensores/sensor.h"
#include "Sensores/Calibrador/calibrador_mag.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#if defined(STM32F767xx)
  #define NUM_MAX_MAG                5
#elif defined(STM32F722xx)
  #define NUM_MAX_MAG                2
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    MAG_1 = 0,
    MAG_2,
    MAG_3,
    MAG_4,
    MAG_5,
    MAG_6,
} numMag_e;

typedef enum {
    MAG_NINGUNO = -1,
    MAG_HMC5883 =  0,
    MAG_HMC5983,
    MAG_IST8310,
} tipoMag_e;

typedef struct {
    uint32_t ultimaActualizacion;        // Tiempo en us
    uint32_t ultimaMedida;               // Tiempo en us
    uint32_t ultimoCambio;               // Tiempo en us
} timingMag_t;

typedef struct {
	numMag_e numMag;
    bus_t bus;
    void *driver;
    uint8_t drdy;
    float campoMag[3];                   // Campo magnetico en mGa
    float escalado[3];
    float campoMagMedio;
    uint16_t cntErrorFiltro;
    bool iniciado;
    bool operativo;
    bool calibrado;
    bool nuevaMedida;
    timingMag_t timing;
} mag_t;

typedef struct {
    bool (*iniciarMag)(mag_t *dMag);
    void (*leerMag)(mag_t *dMag);
    void (*actualizarMag)(mag_t *dMag);
    bool (*calibrarMag)(mag_t *dMag);
} tablaFnMag_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
extern tablaFnMag_t tablaFnMagHoneywell;
extern tablaFnMag_t tablaFnMagIsentek;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarMag(void);
void leerMag(uint32_t tiempoActual);
void actualizarMag(uint32_t tiempoActual);
bool magOperativo(numMag_e numMag);
bool magsOperativos(void);
bool campoMagOk(mag_t *dMag, float *m);
uint8_t numMagsConectados(void);
bool magGenOperativo(void);

void campoMag(float *m);
void campoNumMag(numMag_e numMag, float *m);

#endif // __MAGNETOMETRO_H_
