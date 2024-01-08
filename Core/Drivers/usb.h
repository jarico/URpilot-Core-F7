/***************************************************************************************
**  usb.h - Funciones de gestion del USB
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 01/05/2021
**  Fecha de modificacion: 01/05/2021
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

#ifndef __USB_H
#define __USB_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"
#include "io.h"
#include "usbd_def.h"
#include "usbd_cdc.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#ifndef TAMANIO_BUFFER_RX_USB
  #define TAMANIO_BUFFER_RX_USB     2048
#endif
#ifndef TAMANIO_BUFFER_TX_USB
  #define TAMANIO_BUFFER_TX_USB     2048
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    bool asignado;
    USBD_HandleTypeDef hUSB;
    PCD_HandleTypeDef hPCD;
    uint8_t IRQ;
    uint8_t prioridadIRQ;
    pin_t pinDP;
    pin_t pinDM;
} halUSB_t;

typedef struct {
	bool iniciado;
	halUSB_t hal;
    bool puertoAbierto;
    uint8_t recepcion[TAMANIO_BUFFER_RX_USB];
    volatile uint8_t rxBuffer[TAMANIO_BUFFER_RX_USB];
    volatile uint8_t txBuffer[TAMANIO_BUFFER_TX_USB];
    volatile uint32_t cabezaTxBuffer;
    volatile uint32_t colaTxBuffer;
    uint32_t cabezaRxBuffer;
    uint32_t colaRxBuffer;
} usb_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
usb_t *punteroUSB(void);
USBD_DescriptorsTypeDef *punteroDescriptorUSB(void);
USBD_CDC_ItfTypeDef *punteroUSBInterfazCDC(void);
bool asignarHALusb(void);
bool iniciarUSB(void);
bool iniciarDriverUSB(void);
bool usbAbierto(void);
bool usbConfigurado(void);
bool usbConectado(void);

void escribirUSB(uint8_t byteTx);
void escribirBufferUSB(uint8_t *datoTx, uint32_t longitud);
int16_t leerUSB(void);
void leerBufferUSB(int16_t *datoRx, uint16_t longitud);
void flushUSB(void);
uint32_t bytesRecibidosUSB(void);
bool bufferTxVacioUSB(void);
uint32_t bytesLibresBufferTxUSB(void);

#endif // __USB_H
