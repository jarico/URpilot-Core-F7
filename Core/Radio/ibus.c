/***************************************************************************************
**  ibus.c - Driver de la comunicacion iBUS de la emisora RC
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 10/07/2019
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
#include <string.h>

#include "radio.h"

#ifdef USAR_RADIO_UART
#include "GP/gp_radio.h"
#include "Drivers/uart.h"
#include "Drivers/tiempo.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define TAMANIO_FRAME_IBUS          32
#define TAMANIO_FRAME_IBUS_32       8
#define NUM_CANALES_ENTRADA_IBUS    14


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    bool tramaRecibida;
    uint8_t buffer[TAMANIO_FRAME_IBUS];
    uint32_t trama[TAMANIO_FRAME_IBUS_32];  // Buffer 4 veces mas pequeno para optimizacion
    uint8_t offset;
    uint32_t tiempoAnterior;
} ibus_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static ibus_t ibus;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarIBUS(void);
void procesarByteIBUS(uint8_t RxByte);
bool decodificarTramaIBUS(const uint8_t *trama, uint16_t *valores);
void leerIBUS(uint32_t tiempoActual);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarIBUS(void)
**  Descripcion:    Reseteamos las variables del IBUS y abrimos el puerto serie
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarIBUS(void)
{
    memset(&ibus, 0, sizeof(ibus_t));

    // Arrancamos la UART
    configIniUART_t config;
    config.baudrate = 115200;
    config.lWord = UART_LONGITUD_WORD_8;
    config.paridad = UART_NO_PARIDAD;
    config.stop = UART_BIT_STOP_1;
    if (!iniciarUART(configRadio()->dispUART, config, procesarByteIBUS))
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         void procesarByteIBUS(uint8_t RxByte)
**  Descripcion:    Recibe los bytes de la trama IBUs
**  Parametros:     Byte recibido
**  Retorno:        Ninguno
****************************************************************************************/
void procesarByteIBUS(uint8_t RxByte)
{
    ibus_t *driver = &ibus;

    uint32_t tiempoActual = microsISR();
    const bool separacionTrama = (tiempoActual - driver->tiempoAnterior >= 2000U);
    driver->tiempoAnterior = tiempoActual;

    if (separacionTrama)                         // Si tenemos separacion reseteamos el offset para recibir nueva trama
    	driver->offset = 0;

    if (RxByte != 0x20 && driver->offset == 0)      // Para ser IBUS el primer byte es 0x20
        return;

    if (driver->offset == 0 && !separacionTrama) // Debe haber una separacion entre tramas para empezar un nuevo frame
        return;

    driver->buffer[driver->offset++] = RxByte;

    if (driver->offset == sizeof(driver->buffer)) {
    	driver->offset = 0;
    	driver->tramaRecibida = true;

        uint32_t *p = (uint32_t *)driver->buffer;
        for (uint8_t i = 0; i < TAMANIO_FRAME_IBUS_32; i++) {
            driver->trama[i] = *p;
            p++;
        }
    }
}


/***************************************************************************************
**  Nombre:         bool decodificarTramaIBUS(const uint8_t *trama, uint16_t *valores
**  Descripcion:    Obtiene los valores de los canales dada una trama
**  Parametros:     Trama, valores de los canales
**  Retorno:        True si ok, failsafe
****************************************************************************************/
bool decodificarTramaIBUS(const uint8_t *trama, uint16_t *valores)
{
    uint32_t checksum = 96;

    // El inicio de la trama comienza con 0x20, 0x40
    if ((trama[0] != 0x20) || (trama[1] != 0x40))
        return false;

    // Extraemos los datos de la trama
    for (uint8_t canal = 0, i = 2; canal < NUM_CANALES_ENTRADA_IBUS; canal++, i += 2) {
        valores[canal] = trama[i] | (trama[i + 1] & 0x0F) << 8;
        checksum += trama[i] + trama[i + 1];
    }

    checksum += trama[TAMANIO_FRAME_IBUS - 2] | trama[TAMANIO_FRAME_IBUS - 1] << 8;

    if (checksum != 0xFFFF)
        return false;

    if ((trama[3] & 0xF0) || (trama[9] & 0xF0)) { // Failsafe
        activarFailsafeRadio();
        return false;
    }

    return true;
}


/***************************************************************************************
**  Nombre:         void leerIBUS(uint32_t tiempoActual)
**  Descripcion:    Lee la trama
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void leerIBUS(uint32_t tiempoActual)
{
    UNUSED(tiempoActual);
    ibus_t *driver = &ibus;

    if (!driver->tramaRecibida)
        return;

    uint16_t valores[NUM_CANALES_ENTRADA_IBUS];

    if (decodificarTramaIBUS((uint8_t *)driver->trama, valores)) {
    	driver->tramaRecibida = false;
        anadirRecepcionRadio(NUM_CANALES_ENTRADA_IBUS, valores);
    }
}


/***************************************************************************************
**  Nombre:         tablaFnRadio_t tablaFnRadioIBUS
**  Descripcion:    Tabla de funciones del protocolo IBUS
****************************************************************************************/
tablaFnRadio_t tablaFnRadioIBUS = {
    iniciarIBUS,
    leerIBUS,
};

#endif
