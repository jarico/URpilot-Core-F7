/***************************************************************************************
**  sbus.c - Driver de la comunicacion SBUS de la emisora RC
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 14/08/2019
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
#define TAMANIO_FRAME_SBUS          25
#define TAMANIO_FRAME_SBUS_32       7
#define NUM_CANALES_ENTRADA_SBUS    16

#define BYTE_FLAGS_SBUS             23
#define BIT_FAILSAFE_SBUS           3
#define BIT_TRAMA_PERDIDA_SBUS      2


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
/*
 * Matriz de decodificación SBUS
 *
 * Cada valor de los canales viene determinado por 3 bytes de entrada. Cada fila en la matriz
 * describe hasta 3 bytes y cada entrada nos da:
 *
 * - byte offset en la porcion de datos de la trama
 * - right shift aplicado al byte de datos
 * - mascara para el byte de datos
 * - left shift aplicado al resultado
 */
typedef struct {
    uint8_t byte;
    uint8_t rshift;
    uint8_t mascara;
    uint8_t lshift;
} decodificadorSBUS_t;

typedef struct {
	bool tramaRecibida;
    uint8_t buffer[TAMANIO_FRAME_SBUS];
    uint32_t trama[TAMANIO_FRAME_SBUS_32];  // Buffer 4 veces mas pequeno para optimizacion
    uint8_t offset;
    uint32_t tiempoAnterior;
} sbus_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static sbus_t sbus;

static const decodificadorSBUS_t decodificadorSBUS[NUM_CANALES_ENTRADA_SBUS][3] = {
    /*  0 */ { { 0, 0, 0xFF, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
    /*  1 */ { { 1, 3, 0x1F, 0}, { 2, 0, 0x3F, 5}, { 0, 0, 0x00,  0} },
    /*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xFF, 2}, { 4, 0, 0x01, 10} },
    /*  3 */ { { 4, 1, 0x7F, 0}, { 5, 0, 0x0F, 7}, { 0, 0, 0x00,  0} },
    /*  4 */ { { 5, 4, 0x0F, 0}, { 6, 0, 0x7F, 4}, { 0, 0, 0x00,  0} },
    /*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xFF, 1}, { 8, 0, 0x03,  9} },
    /*  6 */ { { 8, 2, 0x3F, 0}, { 9, 0, 0x1F, 6}, { 0, 0, 0x00,  0} },
    /*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xFF, 3}, { 0, 0, 0x00,  0} },
    /*  8 */ { {11, 0, 0xFF, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
    /*  9 */ { {12, 3, 0x1F, 0}, {13, 0, 0x3F, 5}, { 0, 0, 0x00,  0} },
    /* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xFF, 2}, {15, 0, 0x01, 10} },
    /* 11 */ { {15, 1, 0x7F, 0}, {16, 0, 0x0F, 7}, { 0, 0, 0x00,  0} },
    /* 12 */ { {16, 4, 0x0F, 0}, {17, 0, 0x7F, 4}, { 0, 0, 0x00,  0} },
    /* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xFF, 1}, {19, 0, 0x03,  9} },
    /* 14 */ { {19, 2, 0x3F, 0}, {20, 0, 0x1F, 6}, { 0, 0, 0x00,  0} },
    /* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xFF, 3}, { 0, 0, 0x00,  0} }
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarSBUS(void);
void procesarByteSBUS(uint8_t RxByte);
bool decodificarTramaSBUS(const uint8_t *trama, uint16_t *valores);
void leerSBUS(uint32_t tiempoActual);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarSBUS(void)
**  Descripcion:    Reseteamos las variables del SBUS y abrimos el puerto serie
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarSBUS(void)
{
    memset(&sbus, 0, sizeof(sbus_t));

    // Arrancamos la UART
    configIniUART_t config;
    config.baudrate = 100000;
    config.lWord = UART_LONGITUD_WORD_8;
    config.paridad = UART_NO_PARIDAD;
    config.stop = UART_BIT_STOP_2;
    if (!iniciarUART(configRadio()->dispUART, config, procesarByteSBUS))
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         void procesarByteSBUS(uint32_t tiempo, uint8_t RxByte)
**  Descripcion:    Recibe los bytes de la trama SBUs y la decodifica
**  Parametros:     Tiempo actual, byte recibido
**  Retorno:        Ninguno
****************************************************************************************/
void procesarByteSBUS(uint8_t RxByte)
{
	sbus_t *driver = &sbus;

	uint32_t tiempoActual = microsISR();
    const bool separacionTrama = (tiempoActual - driver->tiempoAnterior >= 2000U);
    driver->tiempoAnterior = tiempoActual;

    if (separacionTrama)                         // Si tenemos separacion reseteamos el offset para recibir nueva trama
    	driver->offset = 0;

    if (RxByte != 0x0F && driver->offset == 0)   // Para ser IBUS el primer byte es 0x20
        return;

    if (driver->offset == 0 && !separacionTrama) // Debe haber una separacion entre tramas para empezar un nuevo frame
        return;

    driver->buffer[driver->offset++] = RxByte;

    if (driver->offset == sizeof(driver->buffer)) {
    	driver->offset = 0;
    	driver->tramaRecibida = true;

        uint32_t *p = (uint32_t *)driver->buffer;
        for (uint8_t i = 0; i < TAMANIO_FRAME_SBUS_32; i++) {
        	driver->trama[i] = *p;
        	p++;
        }
    }
}


/***************************************************************************************
**  Nombre:         bool decodificarTramaSBUS(const uint8_t *trama, uint16_t *valores)
**  Descripcion:    Obtiene los valores de los canales dada una trama
**  Parametros:     Trama, valores de los canales
**  Retorno:        True si ok
****************************************************************************************/
bool decodificarTramaSBUS(const uint8_t *trama, uint16_t *valores)
{
    // El inicio de la trama comienza con 0x0F
    if (trama[0] != 0x0F)
        return false;

    switch (trama[24]) {
        case 0x00:
            // Esto es SBUS 1
            break;

        case 0x03:
            // SBUS 2 SLOT0: RX bateria y voltaje externo
            break;

        case 0x83:
            // SBUS 2 SLOT1
            break;

        case 0x43:
        case 0xC3:
        case 0x23:
        case 0xA3:
        case 0x63:
        case 0xE3:
            break;

        default:
            // Se espera uno de los bytes de arriba
            break;
    }

    // Extraemos los datos de la trama usando la matriz
    for (uint8_t canal = 0; canal < NUM_CANALES_ENTRADA_SBUS; canal++) {
        uint16_t valor = 0;

        for (uint8_t i = 0; i < 3; i++) {
            const decodificadorSBUS_t *decodificador = &decodificadorSBUS[canal][i];

            if (decodificador->mascara != 0) {
                uint8_t pieza = trama[1 + decodificador->byte];
                pieza >>= decodificador->rshift;
                pieza &= decodificador->mascara;
                pieza <<= decodificador->lshift;

                valor |= pieza;
            }
        }

        // Convierte de 0-2048 a 1000-2000
        valores[canal] = (valor * 1000 / 2048) + 1000;
    }

    // Decodifica el failsafe y la perdida de trama
    if (trama[BYTE_FLAGS_SBUS] & (1 << BIT_FAILSAFE_SBUS)) {
    	activarFailsafeRadio();
    	return false;
    }

    // Decodifica la perdida de trama
    if (trama[BYTE_FLAGS_SBUS] & (1 << BIT_TRAMA_PERDIDA_SBUS))
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         bool leerSBUS(uint32_t tiempoActual)
**  Descripcion:    Lee la trama
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void leerSBUS(uint32_t tiempoActual)
{
	sbus_t *driver = &sbus;

    if (!driver->tramaRecibida)
        return;

    uint16_t valores[NUM_CANALES_ENTRADA_SBUS];

    if (decodificarTramaSBUS((uint8_t *)driver->buffer, valores)) {
        driver->tramaRecibida = false;
        anadirRecepcionRadio(NUM_CANALES_ENTRADA_SBUS, valores);
    }
}


/***************************************************************************************
**  Nombre:         tablaFnRadio_t tablaFnRadioSBUS
**  Descripcion:    Tabla de funciones del protocolo SBUS
****************************************************************************************/
tablaFnRadio_t tablaFnRadioSBUS = {
    iniciarSBUS,
    leerSBUS,
};


#endif
