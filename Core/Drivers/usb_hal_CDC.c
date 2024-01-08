/***************************************************************************************
**  usb_hal.c - Funciones HAL del USB
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 03/05/2020
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
#include "usb.h"

#ifdef USAR_USB
#include "nvic.h"
#include "atomico.h"
#include "tiempo.h"
#include "timer.h"
#include "GP/gp_usb.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
int8_t iniciarCDC(void);
int8_t deIniciarCDC(void);
int8_t controlCDC(uint8_t cmd, uint8_t *buff, uint16_t lon);
int8_t recibirCDC(uint8_t* buff, uint32_t *lon);
int8_t transmisionCompletaCDC(uint8_t *buff, uint32_t *lon, uint8_t epNum);
void enviarDatoUSB(void);
USBD_StatusTypeDef estadoUSB(HAL_StatusTypeDef estado);


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
USBD_CDC_ItfTypeDef interfazCDC =
{
    iniciarCDC,
    deIniciarCDC,
    controlCDC,
    recibirCDC,
    transmisionCompletaCDC
};


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         USBD_CDC_ItfTypeDef *punteroUSBInterfazCDC(void)
**  Descripcion:    Devuelve el puntero a la interfaz CDC
**  Parametros:     Ninguno
**  Retorno:        Puntero a la interfaz
****************************************************************************************/
USBD_CDC_ItfTypeDef *punteroUSBInterfazCDC(void)
{
    return &interfazCDC;
}


/***************************************************************************************
**  Nombre:         int8_t iniciarCDC(void)
**  Descripcion:    Inicia el timer y los buffers del USB
**  Parametros:     Ninguno
**  Retorno:        USBD_OK si ok
****************************************************************************************/
int8_t iniciarCDC(void)
{
    // Iniciamos el timer
    if (!iniciarTimer(configUSB()->timer, 0, TIM_USO_USB))
        return (USBD_FAIL);

    usb_t *driver = punteroUSB();

    // Se configura el timer
    const uint16_t prescaler = ((SystemCoreClock / configUSB()->frecEnvio) - 1) / 0x10000;
    const uint32_t hz = SystemCoreClock / (prescaler + 1);
    const uint16_t periodo = hz / configUSB()->frecEnvio;

    configurarBaseTiempoTimer(configUSB()->timer, true, periodo, hz);
    asignarCallbackPeriodoTimer(configUSB()->timer, enviarDatoUSB);

    USBD_CDC_SetTxBuffer(&driver->hal.hUSB, (uint8_t *)driver->txBuffer, 0);
    USBD_CDC_SetRxBuffer(&driver->hal.hUSB, (uint8_t *)driver->recepcion);

    return USBD_OK;
}


/***************************************************************************************
**  Nombre:         int8_t deIniciarCDC(void)
**  Descripcion:    De inicializa el CDC
**  Parametros:     Ninguno
**  Retorno:        USBD_OK si ok
****************************************************************************************/
int8_t deIniciarCDC(void)
{
    return USBD_OK;
}


/***************************************************************************************
**  Nombre:         int8_t controlCDC(uint8_t cmd, uint8_t* buff, uint16_t lon)
**  Descripcion:    Controla los comandos del USB
**  Parametros:     Codigo de comando, buffer de comandos, longitud de datos a enviar
**  Retorno:        USBD_OK si ok
****************************************************************************************/
int8_t controlCDC(uint8_t cmd, uint8_t *buff, uint16_t lon)
{
    uint8_t bufTemp[7] = {0, 0, 0, 0, 0, 0, 0};
    USBD_SetupReqTypedef *req;

    switch (cmd) {
        case CDC_SEND_ENCAPSULATED_COMMAND:
            break;

        case CDC_GET_ENCAPSULATED_RESPONSE:
            break;

        case CDC_SET_COMM_FEATURE:
            break;

        case CDC_GET_COMM_FEATURE:
            break;

        case CDC_CLEAR_COMM_FEATURE:
            break;

        case CDC_SET_LINE_CODING:
        	bufTemp[0] = buff[0];
        	bufTemp[1] = buff[1];
        	bufTemp[2] = buff[2];
        	bufTemp[3] = buff[3];
        	bufTemp[4] = buff[4];
        	bufTemp[5] = buff[5];
        	bufTemp[6] = buff[6];
            break;

        case CDC_GET_LINE_CODING:
        	buff[0] = bufTemp[0];
        	buff[1] = bufTemp[1];
        	buff[2] = bufTemp[2];
        	buff[3] = bufTemp[3];
        	buff[4] = bufTemp[4];
        	buff[5] = bufTemp[5];
        	buff[6] = bufTemp[6];
            break;

        case CDC_SET_CONTROL_LINE_STATE:
        	req = (USBD_SetupReqTypedef *)buff;
            usb_t *driver = punteroUSB();

        	if((req->wValue & 0x0001) != 0)
                driver->puertoAbierto = true;
        	else
                driver->puertoAbierto = false;

        	break;

        case CDC_SEND_BREAK:
            break;

        default:
            break;
    }

    return USBD_OK;
}

uint16_t rxAvailable;
/***************************************************************************************
**  Nombre:         int8_t recibirCDC(uint8_t* buff, uint32_t *lon)
**  Descripcion:    Recibe datos del USB
**  Parametros:     Buffer de datos para ser recibidos, numero de datos recibidos
**  Retorno:        USBD_OK si ok
****************************************************************************************/
int8_t recibirCDC(uint8_t* buff, uint32_t *lon)
{
	usb_t *driver = punteroUSB();
	uint8_t tam = (uint8_t)*lon;

    if (tam > 0) {
        for (uint16_t i = 0; i < tam; i++) {
        	driver->rxBuffer[driver->cabezaRxBuffer] = buff[i];
        	driver->cabezaRxBuffer = (driver->cabezaRxBuffer + 1) % TAMANIO_BUFFER_RX_USB;
        }

        USBD_CDC_ReceivePacket(&driver->hal.hUSB);
    }

    return (USBD_OK);
}


/***************************************************************************************
**  Nombre:         int8_t transmisionCompletaCDC(uint8_t *buff, uint32_t *lon, uint8_t epNum)
**  Descripcion:    Funcion que se ejecuta al transmitir un dato
**  Parametros:     Buffer de envio, longitud del buffer, numero de endpoint
**  Retorno:        USBD_OK si ok
****************************************************************************************/
int8_t transmisionCompletaCDC(uint8_t *buff, uint32_t *lon, uint8_t epNum)
{
    UNUSED(buff);
    UNUSED(lon);
    UNUSED(epNum);

    return USBD_OK;
}


/***************************************************************************************
**  Nombre:         void escribirUSB(uint8_t byteTx)
**  Descripcion:    Envia un dato por el USB
**  Parametros:     Dato a enviar
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void escribirUSB(uint8_t byteTx)
{
	usb_t *driver = punteroUSB();

    if (!(usbConectado() && usbConfigurado() && usbAbierto()))
        return;

    BLOQUE_ATOMICO(NVIC_PRIO_USB) {
        driver->txBuffer[driver->cabezaTxBuffer] = byteTx;
        driver->cabezaTxBuffer = (driver->cabezaTxBuffer + 1) % TAMANIO_BUFFER_TX_USB;
    }
}


/***************************************************************************************
**  Nombre:         void escribirBufferUSB(uint8_t *datoTx, uint32_t longitud)
**  Descripcion:    Envia un buffer por el USB
**  Parametros:     Datos a enviar, longitud de los datos
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void escribirBufferUSB(uint8_t *datoTx, uint32_t longitud)
{
    for (uint16_t i = 0; i < longitud; i++)
        escribirUSB(datoTx[i]);
}


/***************************************************************************************
**  Nombre:         int16_t leerUSB(void)
**  Descripcion:    Lee un dato del USB
**  Parametros:     Ninguno
**  Retorno:        Dato leido
****************************************************************************************/
CODIGO_RAPIDO int16_t leerUSB(void)
{
	usb_t *driver = punteroUSB();
    int16_t byteRx;

    if (driver->cabezaRxBuffer != driver->colaRxBuffer) {
        byteRx = driver->rxBuffer[driver->colaRxBuffer];

        if (driver->colaRxBuffer + 1 >= TAMANIO_BUFFER_RX_USB)
        	driver->colaRxBuffer = 0;
        else
        	driver->colaRxBuffer++;

        return byteRx;
    }
    else
      return -1;
}


/***************************************************************************************
**  Nombre:         void leerBufferUSB(int16_t *datoRx, uint16_t longitud)
**  Descripcion:    Lee un buffer del USB
**  Parametros:     Dispositivo, buffer, longitud del buffer
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void leerBufferUSB(int16_t *datoRx, uint16_t longitud)
{
    for (uint16_t i = 0; i < longitud; i++)
        datoRx[i] = leerUSB();
}


/***************************************************************************************
**  Nombre:         void flushUSB(void)
**  Descripcion:    Borra el Buffer de recepcion del USB
**  Parametros:     Dispositivo
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void flushUSB(void)
{
    usb_t *driver = punteroUSB();

    driver->colaRxBuffer = 0;
    driver->cabezaRxBuffer = 0;

    for (uint16_t i = 0; i < TAMANIO_BUFFER_RX_USB; i++)
    	driver->rxBuffer[i] = 0;
}


/***************************************************************************************
**  Nombre:         uint32_t bytesRecibidosUSB(void)
**  Descripcion:    Devuelve el numero de bytes recibidos por el USB
**  Parametros:     Ninguno
**  Retorno:        Numero de bytes
****************************************************************************************/
CODIGO_RAPIDO uint32_t bytesRecibidosUSB(void)
{
    usb_t *driver = punteroUSB();
    uint32_t numBytes;

    BLOQUE_ATOMICO(NVIC_PRIO_USB) {
        if (driver->cabezaRxBuffer >= driver->colaRxBuffer)
        	numBytes = driver->cabezaRxBuffer - driver->colaRxBuffer;
        else
        	numBytes = TAMANIO_BUFFER_RX_USB + driver->cabezaRxBuffer - driver->colaRxBuffer;
    }

    return numBytes;
}


/***************************************************************************************
**  Nombre:         bool bufferTxVacioUSB(void)
**  Descripcion:    Comprueba si el buffer de transmision esta vacio
**  Parametros:     Ninguno
**  Retorno:        True si vacio
****************************************************************************************/
CODIGO_RAPIDO bool bufferTxVacioUSB(void)
{
	usb_t *driver = punteroUSB();
    bool bufVacio;

	BLOQUE_ATOMICO(NVIC_PRIO_USB) {
		bufVacio = driver->colaTxBuffer == driver->cabezaTxBuffer;
	}

    return bufVacio;
}


/***************************************************************************************
**  Nombre:         uint32_t bytesLibresBufferTxUSB(void)
**  Descripcion:    Retorna el numero de bytes libres en el buffer de transmision
**  Parametros:     Dispositivo
**  Retorno:        Numero de bytes libres
****************************************************************************************/
CODIGO_RAPIDO uint32_t bytesLibresBufferTxUSB(void)
{
	usb_t *driver = punteroUSB();

    if (driver->cabezaTxBuffer >= driver->colaTxBuffer)
        return TAMANIO_BUFFER_TX_USB - 1 - driver->cabezaTxBuffer + driver->colaTxBuffer;

    return driver->colaTxBuffer - driver->cabezaTxBuffer - 1;
}


/***************************************************************************************
**  Nombre:         void enviarDatoUSB(void)
**  Descripcion:    Envia datos por el USB
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
CODIGO_RAPIDO void enviarDatoUSB(void)
{
    uint32_t tamBuff;
    static uint32_t ultimoTamBuff = 0;

    usb_t *driver = punteroUSB();
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)driver->hal.hUSB.pClassData;

    if (hcdc->TxState == 0) {
        // El endpoint ha terminado transmitiendo el bloque previo
        if (ultimoTamBuff) {
            bool needZeroLengthPacket = ultimoTamBuff % 64 == 0;

            // Mueve la cola del bufer de anillo en funcion de la transmision exitosa anterior
            driver->colaTxBuffer += ultimoTamBuff;
            if (driver->colaTxBuffer == TAMANIO_BUFFER_TX_USB)
            	driver->colaTxBuffer = 0;

            ultimoTamBuff = 0;

            if (needZeroLengthPacket) {
                USBD_CDC_SetTxBuffer(&driver->hal.hUSB, (uint8_t*)&driver->txBuffer[driver->colaTxBuffer], 0);
                return;
            }
        }

        if (driver->colaTxBuffer != driver->cabezaTxBuffer) {
            if (driver->colaTxBuffer > driver->cabezaTxBuffer)
            	tamBuff = TAMANIO_BUFFER_TX_USB - driver->colaTxBuffer;
            else
            	tamBuff = driver->cabezaTxBuffer - driver->colaTxBuffer;

            if (tamBuff > TAMANIO_BUFFER_TX_USB)
            	tamBuff = TAMANIO_BUFFER_TX_USB;

            USBD_CDC_SetTxBuffer(&driver->hal.hUSB, (uint8_t*)&driver->txBuffer[driver->colaTxBuffer], tamBuff);

            if (USBD_CDC_TransmitPacket(&driver->hal.hUSB) == USBD_OK)
            	ultimoTamBuff = tamBuff;
        }
    }
}

#endif

