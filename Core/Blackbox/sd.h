/***************************************************************************************
**  sd.h - Funciones de gestion de la tarjeta SD
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 12/09/2019
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

#ifndef __SD_H
#define __SD_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "sd_estandar.h"
#include "GP/gp.h"
#include "drivers/bus.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define TIMEOUT_INICIALIZACION_MS_SD           200
#define NUM_MAX_FALLOS_CONSECUTIVOS_SD         8


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    SD_MODO_NONE = 0,
    SD_MODO_SPI,
    SD_MODO_SDIO,
} modoSD_e;

typedef enum {
    // En estos estados, corremos a la velocidad de reloj de 400kHz
    SD_ESTADO_NO_PRESENTE = 0,
    SD_ESTADO_RESET,
    SD_ESTADO_INICIALIZACION_EN_PROGRESO,
    SD_ESTADO_INICIALIZACION_RECEPCION_CID,

    // En estos estados corremos a toda velocidad
    SD_ESTADO_READY,
    SD_ESTADO_LEYENDO,
    SD_ESTADO_ENVIANDO_ESCRITURA,
    SD_ESTADO_ESPERANDO_PARA_ESCRIBIR,
    SD_ESTADO_ESCRIBIENDO_MULTIPLES_BLOQUES,
    SD_ESTADO_PARANDO_ESCRITURA_MULTIPLES_BLOQUES,
} estadoSD_e;

typedef enum {
    SD_OPERACION_EN_PROGRESO,
    SD_OPERACION_OCUPADO,
    SD_OPERACION_EXITO,
    SD_OPERACION_FALLO,
} estadoOperacionSD_e;

typedef enum {
    SD_OPERACION_BLOQUE_LEER,
    SD_OPERACION_BLOQUE_ESCRIBIR,
    SD_OPERACION_BLOQUE_BORRAR,
} operacionBloqueSD_e;

typedef enum {
    SD_RECEPCION_EXITOSA,
    SD_RECEPCION_BLOQUE_EN_PROGRESO,
	SD_RECEPCION_ERROR,
} estadoRecepcionBloqueSD_e;

typedef struct {
    uint32_t numBloques;                // Capacidad Tarjeta en bloques de 512-bytes
    uint16_t idOEM;
    uint8_t idFabricante;
    char nombreProducto[5];
    uint32_t numSerie;
    uint8_t revisionProductoMayor;
    uint8_t revisionProductoMenor;
    uint16_t anoProduccion;
    uint8_t mesProduccion;
} metadatosSD_t;

typedef void(*callbackOpCompletaSD_c)(operacionBloqueSD_e operacion, uint32_t indice, uint8_t *buffer, uint32_t datoCallback);

typedef struct {
    bool (*iniciarSD)(void);
    bool (*sondearSD)(void);
    bool (*leerBloqueSD)(uint32_t indice, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback);
    estadoOperacionSD_e (*iniciarEscrituraBloquesSD)(uint32_t indice, uint32_t numBloques);
    estadoOperacionSD_e (*escribirBloqueSD)(uint32_t indice, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback);
} tablaFnSD_t;

typedef struct {
	bool iniciada;
	bool habilitada;
    bool usarCache;
    estadoSD_e estado;
    metadatosSD_t metadatos;
    uint32_t tiempoOpIniciada;
    uint8_t contadorFallos;
    uint8_t version;
    bool altaCapacidad;
    uint32_t siguienteBloqueMultiEscritura;
    uint32_t bloquesRestantesMultiEscritura;
    struct {
        uint8_t *buffer;
        uint32_t indiceBloque;
        uint8_t indiceTrozo;
        callbackOpCompletaSD_c callback;
        uint32_t datoCallback;
    } operacionPendiente;
} sd_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
#ifdef USAR_SD_SPI
extern tablaFnSD_t tablaFnSDspi;
#endif
#ifdef USAR_SD_SDIO
extern tablaFnSD_t tablaFnSDsdio;
#endif


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
sd_t *punteroSD(void);
void iniciarSD(void);
bool tarjetaSDinicializada(void);
bool tarjetaSDinsertada(void);
bool tarjetaSDfuncional(void);
const metadatosSD_t* leerMetadatosSD(void);
bool sondearSD(void);
bool leerBloqueSD(uint32_t indice, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback);
estadoOperacionSD_e iniciarEscrituraBloquesSD(uint32_t indice, uint32_t numBloques);
estadoOperacionSD_e escribirBloqueSD(uint32_t indice, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback);

#endif // __SD_H
