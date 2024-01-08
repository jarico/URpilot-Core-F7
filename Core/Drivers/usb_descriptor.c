/***************************************************************************************
**  usb_descriptor.c - Funciones generales del descriptor del USB
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 06/05/2021
**  Fecha de modificacion: 06/05/2021
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
#include "Version/version.h"
#include "usbd_core.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define DISPOSITIVO_ID1              (UID_BASE)
#define DISPOSITIVO_ID2              (UID_BASE + 0x4)
#define DISPOSITIVO_ID3              (UID_BASE + 0x8)

#define USB_TAM_STRING_SERIAL        0x1A
#define USB_TAM_BOS_DESCRIPTOR       0x0C

#define USB_VID                      1155
#define USB_STRING_ID_LENGUAJE       1033
#define USB_STRING_FABRICANTE        NOMBRE_FIRMWARE
#define USB_PID_FS                   22336
#define USB_STRING_PRODUCTO_FS       "STM32 Virtual ComPort"
#define USB_STRING_CONFIGURACION_FS  "CDC Config"
#define USB_STRING_INTERFAZ_FS       "CDC Interface"
#define USB_TAM_STRING_DESCRIPTOR    512U


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void generarNumeroSerieUSB(void);
void convertirNumeroSerie(uint32_t valor, uint8_t * buf, uint8_t lon);
uint8_t *descriptorDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon);
uint8_t *descriptorIdLenguajeDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon);
uint8_t *descriptorFabricanteDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon);
uint8_t *descriptorProductoDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon);
uint8_t *descriptorNumSerieDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon);
uint8_t *descriptorConfigDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon);
uint8_t *descriptorInterfazDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon);
#if (USBD_LPM_ENABLED == 1)
uint8_t *descriptorBOSDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon);
#endif


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
USBD_DescriptorsTypeDef descriptorUSB =
{
	descriptorDispositivoUSB
   ,descriptorIdLenguajeDispositivoUSB
   ,descriptorFabricanteDispositivoUSB
   ,descriptorProductoDispositivoUSB
   ,descriptorNumSerieDispositivoUSB
   ,descriptorConfigDispositivoUSB
   ,descriptorInterfazDispositivoUSB
#if (USBD_LPM_ENABLED == 1)
   ,descriptorBOSDispositivoUSB
#endif
};

#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif

__ALIGN_BEGIN uint8_t descriptorDispUSB[USB_LEN_DEV_DESC] __ALIGN_END =
{
  0x12,                       // bLength
  USB_DESC_TYPE_DEVICE,       // bDescriptorType
#if (USBD_LPM_ENABLED == 1)
  0x01,                       // bcdUSB changed to USB version 2.01 in order to support LPM L1 suspend resume test of USBCV3.0
#else
  0x00,                       // bcdUSB
#endif
  0x02,
  0x02,                       // bDeviceClass
  0x02,                       // bDeviceSubClass
  0x00,                       // bDeviceProtocol
  USB_MAX_EP0_SIZE,           // bMaxPacketSize
  LOBYTE(USB_VID),            // idVendor
  HIBYTE(USB_VID),            // idVendor
  LOBYTE(USB_PID_FS),         // idProduct
  HIBYTE(USB_PID_FS),         // idProduct
  0x00,                       // bcdDevice rel. 2.00
  0x02,
  USBD_IDX_MFC_STR,           // Index of manufacturer  string
  USBD_IDX_PRODUCT_STR,       // Index of product string
  USBD_IDX_SERIAL_STR,        // Index of serial number string
  USBD_MAX_NUM_CONFIGURATION  // bNumConfigurations
};


#if (USBD_LPM_ENABLED == 1)
#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif

__ALIGN_BEGIN uint8_t descriptorBOSdispUSB[USB_TAM_BOS_DESCRIPTOR] __ALIGN_END =
{
  0x5,
  USB_DESC_TYPE_BOS,
  0xC,
  0x0,
  0x1,                        // 1 device capability
                              // device capability
  0x7,
  USB_DEVICE_CAPABITY_TYPE,
  0x2,
  0x2,                        // LPM capability bit set
  0x0,
  0x0,
  0x0
};
#endif


#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif

__ALIGN_BEGIN uint8_t descriptorIdLenguajeUSB[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
     USB_LEN_LANGID_STR_DESC,
     USB_DESC_TYPE_STRING,
     LOBYTE(USB_STRING_ID_LENGUAJE),
     HIBYTE(USB_STRING_ID_LENGUAJE)
};

#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif


__ALIGN_BEGIN uint8_t stringDescriptorUSB[USB_TAM_STRING_DESCRIPTOR] __ALIGN_END;


#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif

__ALIGN_BEGIN uint8_t stringSerieUSB[USB_TAM_STRING_SERIAL] __ALIGN_END = {
    USB_TAM_STRING_SERIAL,
    USB_DESC_TYPE_STRING,
};


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void generarNumeroSerieUSB(void)
**  Descripcion:    Crea el string del numero de serie del descriptor
**  Parametros:     Ninugno
**  Retorno:        Ninugno
****************************************************************************************/
void generarNumeroSerieUSB(void)
{
    uint32_t dispSerie0, dispSerie1, dispSerie2;

    dispSerie0 = *(uint32_t *)DISPOSITIVO_ID1;
    dispSerie1 = *(uint32_t *)DISPOSITIVO_ID2;
    dispSerie2 = *(uint32_t *)DISPOSITIVO_ID3;

    dispSerie0 += dispSerie2;

    if (dispSerie0 != 0) {
    	convertirNumeroSerie(dispSerie0, &stringSerieUSB[2], 8);
    	convertirNumeroSerie(dispSerie1, &stringSerieUSB[18], 4);
    }
}


/***************************************************************************************
**  Nombre:         void convertirNumeroSerie(uint32_t valor, uint8_t * buf, uint8_t lon)
**  Descripcion:    Convierte el numero de serie en chars
**  Parametros:     Valor a convertir, puntero al buffer, longitud del buffer
**  Retorno:        Ninugno
****************************************************************************************/
void convertirNumeroSerie(uint32_t valor, uint8_t * buf, uint8_t lon)
{
    for (uint8_t i = 0; i < lon; i++) {
        if (((valor >> 28)) < 0xA)
            buf[2 * i] = (valor >> 28) + '0';
        else
            buf[2 * i] = (valor >> 28) + 'A' - 10;

        valor = valor << 4;
        buf[2 * i + 1] = 0;
    }
}


/***************************************************************************************
**  Nombre:         USBD_DescriptorsTypeDef *punteroDescriptorUSB(void)
**  Descripcion:    Devuelve el puntero al descriptor
**  Parametros:     Ninugno
**  Retorno:        Descriptor
****************************************************************************************/
USBD_DescriptorsTypeDef *punteroDescriptorUSB(void)
{
	return &descriptorUSB;
}


/***************************************************************************************
**  Nombre:         uint8_t *descriptorDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon)
**  Descripcion:    Devuelve el descriptor del dispositivo USB
**  Parametros:     Velocidad del dispositivo, longitud de los datos del puntero
**  Retorno:        Descriptor
****************************************************************************************/
uint8_t *descriptorDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon)
{
    UNUSED(velocidad);
    *lon = sizeof(descriptorDispUSB);
    return descriptorDispUSB;
}


/***************************************************************************************
**  Nombre:         uint8_t *descriptorIdLenguajeDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon)
**  Descripcion:    Devuelve el descriptor del lenguaje del dispositivo USB
**  Parametros:     Velocidad del dispositivo, longitud de los datos del puntero
**  Retorno:        Descriptor
****************************************************************************************/
uint8_t *descriptorIdLenguajeDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon)
{
    UNUSED(velocidad);
    *lon = sizeof(descriptorIdLenguajeUSB);
    return descriptorIdLenguajeUSB;
}


/***************************************************************************************
**  Nombre:         uint8_t *descriptorFabricanteDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon)
**  Descripcion:    Devuelve el descriptor del fabricante del dispositivo USB
**  Parametros:     Velocidad del dispositivo, longitud de los datos del puntero
**  Retorno:        Descriptor
****************************************************************************************/
uint8_t *descriptorFabricanteDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon)
{
    UNUSED(velocidad);
    USBD_GetString((uint8_t *)USB_STRING_FABRICANTE, stringDescriptorUSB, lon);
    return stringDescriptorUSB;
}


/***************************************************************************************
**  Nombre:         uint8_t *descriptorProductoDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon)
**  Descripcion:    Devuelve el descriptor del producto del dispositivo USB
**  Parametros:     Velocidad del dispositivo, longitud de los datos del puntero
**  Retorno:        Descriptor
****************************************************************************************/
uint8_t *descriptorProductoDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon)
{
    if (velocidad == 0)
        USBD_GetString((uint8_t *)USB_STRING_PRODUCTO_FS, stringDescriptorUSB, lon);
    else
        USBD_GetString((uint8_t *)USB_STRING_PRODUCTO_FS, stringDescriptorUSB, lon);

    return stringDescriptorUSB;
}


/***************************************************************************************
**  Nombre:         uint8_t *descriptorNumSerieDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon)
**  Descripcion:    Devuelve el descriptor del numero de serie del dispositivo USB
**  Parametros:     Velocidad del dispositivo, longitud de los datos del puntero
**  Retorno:        Descriptor
****************************************************************************************/
uint8_t *descriptorNumSerieDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon)
{
    UNUSED(velocidad);
    *lon = USB_TAM_STRING_SERIAL;

    // Actualiza el descriptor del numero de serie con los datos del ID unico
    generarNumeroSerieUSB();
    return (uint8_t *)stringSerieUSB;
}


/***************************************************************************************
**  Nombre:         uint8_t *descriptorConfigDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon)
**  Descripcion:    Devuelve el descriptor de la configuracion del dispositivo USB
**  Parametros:     Velocidad del dispositivo, longitud de los datos del puntero
**  Retorno:        Descriptor
****************************************************************************************/
uint8_t *descriptorConfigDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon)
{
    if (velocidad == USBD_SPEED_HIGH)
        USBD_GetString((uint8_t *)USB_STRING_CONFIGURACION_FS, stringDescriptorUSB, lon);
    else
        USBD_GetString((uint8_t *)USB_STRING_CONFIGURACION_FS, stringDescriptorUSB, lon);

    return stringDescriptorUSB;
}


/***************************************************************************************
**  Nombre:         uint8_t *descriptorInterfazDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon)
**  Descripcion:    Devuelve el descriptor de la interfaz del dispositivo USB
**  Parametros:     Velocidad del dispositivo, longitud de los datos del puntero
**  Retorno:        Descriptor
****************************************************************************************/
uint8_t *descriptorInterfazDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon)
{
    if (velocidad == 0)
        USBD_GetString((uint8_t *)USB_STRING_INTERFAZ_FS, stringDescriptorUSB, lon);
    else
        USBD_GetString((uint8_t *)USB_STRING_INTERFAZ_FS, stringDescriptorUSB, lon);

    return stringDescriptorUSB;
}


#if (USBD_LPM_ENABLED == 1)
/***************************************************************************************
**  Nombre:         uint8_t *descriptorBOSDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon)
**  Descripcion:    Devuelve el descriptor BOS del dispositivo USB
**  Parametros:     Velocidad del dispositivo, longitud de los datos del puntero
**  Retorno:        Descriptor
****************************************************************************************/
uint8_t *descriptorBOSDispositivoUSB(USBD_SpeedTypeDef velocidad, uint16_t *lon)
{
    UNUSED(velocidad);
    *lon = sizeof(descriptorBOSdispUSB);
    return (uint8_t*)descriptorBOSdispUSB;
}
#endif


