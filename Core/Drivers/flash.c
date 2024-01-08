/***************************************************************************************
**  flash.c - Funciones para la gestion de la flash y en concreto las funciones para
**            grabar la configuracion en la flash
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 08/05/2019
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

#include "flash.h"
#include "Core/fallo_sistema.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#if !defined(FLASH_PAGE_SIZE)
  #if defined(STM32F767xx)
    #define FLASH_PAGE_SIZE                 ((uint32_t)0x8000)  // 32k
  #elif defined(STM32F722xx)
    #define FLASH_PAGE_SIZE                 ((uint32_t)0x4000)  // 16k
  #else
    #error "Tamanio de pag de la Flash no definido."
  #endif
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
extern uint8_t inicioRegionConfig;       // Definido en el Linker


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
uint32_t sectorParaConfigGrabadorFlash(void);
int32_t escribirWordGrabadorFlash(grabadorFlash_t *grabador, uint32_t valor);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void resetearGrabadorFlash(grabadorFlash_t *grabador)
**  Descripcion:    Resetea el grabador
**  Parametros:     Grabador
**  Retorno:        Ninguno
****************************************************************************************/
void resetearGrabadorFlash(grabadorFlash_t *grabador)
{
    memset(grabador, 0, sizeof(*grabador));
}


/***************************************************************************************
**  Nombre:         int32_t estadoGrabadorFlash(grabadorFlash_t *grabador)
**  Descripcion:    Devuelve el estado del grabador
**  Parametros:     grabador
**  Retorno:        Codigo de error: 0 no error, -1 fallo en el borrado, -2 Fallo al grabar
****************************************************************************************/
int32_t estadoGrabadorFlash(grabadorFlash_t *grabador)
{
    return grabador->err;
}


/***************************************************************************************
**  Nombre:         void desbloquearGrabadorFlash(grabadorFlash_t *grabador, uintptr_t base, int32_t tam)
**  Descripcion:    Inicia el grabador ajustando la direccion y el tamanio
**  Parametros:     Grabador, direccion base, tamanio
**  Retorno:        Ninguno
****************************************************************************************/
void desbloquearGrabadorFlash(grabadorFlash_t *grabador, uintptr_t base, int32_t tam)
{
    grabador->dir = base;
    grabador->tam = tam;
    if (!grabador->desbloqueado) {
        HAL_FLASH_Unlock();
        grabador->desbloqueado = true;
    }

    grabador->err = 0;
}


/***************************************************************************************
**  Nombre:         int32_t bloquearGrabadorFlash(grabadorFlash_t *grabador)
**  Descripcion:    Bloquea el grabador y la flash
**  Parametros:     grabador
**  Retorno:        Codigo de error: 0 no error, -1 fallo en el borrado, -2 Fallo al grabar
****************************************************************************************/
int32_t bloquearGrabadorFlash(grabadorFlash_t *grabador)
{
    if (grabador->desbloqueado) {
        HAL_FLASH_Lock();
        grabador->desbloqueado = false;
    }
    return grabador->err;
}


/***************************************************************************************
**  Nombre:         int32_t escribirGrabadorFlash(grabadorFlash_t *grabador, const uint8_t *p, uint32_t tam)
**  Descripcion:    Graba un buffer en la flash
**  Parametros:     grabador, buffer a grabar, numero de datos
**  Retorno:        Codigo de error: 0 no error, -1 fallo en el borrado, -2 Fallo al grabar
****************************************************************************************/
int32_t escribirGrabadorFlash(grabadorFlash_t *grabador, const uint8_t *p, uint32_t tam)
{
    for (const uint8_t *pat = p; pat != (uint8_t*)p + tam; pat++) {
        grabador->buffer.b[grabador->at++] = *pat;

        if (grabador->at == sizeof(grabador->buffer)) {
            grabador->err = escribirWordGrabadorFlash(grabador, grabador->buffer.w);
            grabador->at = 0;
        }
    }
    return grabador->err;
}


/***************************************************************************************
**  Nombre:         int32_t flushGrabadorFlash(grabadorFlash_t *grabador)
**  Descripcion:    Hace un flush en la flash
**  Parametros:     grabador
**  Retorno:        Codigo de error: 0 no error, -1 fallo en el borrado, -2 Fallo al grabar
****************************************************************************************/
int32_t flushGrabadorFlash(grabadorFlash_t *grabador)
{
    if (grabador->at != 0) {
        memset(grabador->buffer.b + grabador->at, 0, sizeof(grabador->buffer) - grabador->at);
        grabador->err = escribirWordGrabadorFlash(grabador, grabador->buffer.w);
        grabador->at = 0;
    }
    return grabador-> err;
}


#if defined(STM32F767xx)
/*
Sector 0    0x08000000 - 0x08007FFF 32 Kbytes
Sector 1    0x08008000 - 0x0800FFFF 32 Kbytes
Sector 2    0x08010000 - 0x08017FFF 32 Kbytes
Sector 3    0x08018000 - 0x0801FFFF 32 Kbytes
Sector 4    0x08020000 - 0x0803FFFF 128 Kbytes
Sector 5    0x08040000 - 0x0807FFFF 256 Kbytes
Sector 6    0x08080000 - 0x080BFFFF 256 Kbytes
Sector 7    0x080C0000 - 0x080FFFFF 256 Kbytes

2M flash
Sector 8    0x08100000 - 0x0813FFFF 256 Kbytes
Sector 9    0x08140000 - 0x0817FFFF 256 Kbytes
Sector 10   0x08180000 - 0x081BFFFF 256 Kbytes
Sector 11   0x081C0000 - 0x081FFFFF 256 Kbytes
*/
/***************************************************************************************
**  Nombre:         uint32_t sectorParaConfigGrabadorFlash(void)
**  Descripcion:    Obtiene el sector de la flash
**  Parametros:     Ninguno
**  Retorno:        Sector de la flash
****************************************************************************************/
uint32_t sectorParaConfigGrabadorFlash(void)
{
    if ((uint32_t)&inicioRegionConfig <= 0x08007FFF)
        return FLASH_SECTOR_0;
    if ((uint32_t)&inicioRegionConfig <= 0x0800FFFF)
        return FLASH_SECTOR_1;
    if ((uint32_t)&inicioRegionConfig <= 0x08017FFF)
        return FLASH_SECTOR_2;
    if ((uint32_t)&inicioRegionConfig <= 0x0801FFFF)
        return FLASH_SECTOR_3;
    if ((uint32_t)&inicioRegionConfig <= 0x0803FFFF)
        return FLASH_SECTOR_4;
    if ((uint32_t)&inicioRegionConfig <= 0x0807FFFF)
        return FLASH_SECTOR_5;
    if ((uint32_t)&inicioRegionConfig <= 0x080BFFFF)
        return FLASH_SECTOR_6;
    if ((uint32_t)&inicioRegionConfig <= 0x080FFFFF)
        return FLASH_SECTOR_7;
    if ((uint32_t)&inicioRegionConfig <= 0x0813FFFF)
        return FLASH_SECTOR_8;
    if ((uint32_t)&inicioRegionConfig <= 0x0817FFFF)
        return FLASH_SECTOR_9;
    if ((uint32_t)&inicioRegionConfig <= 0x081BFFFF)
        return FLASH_SECTOR_10;
    if ((uint32_t)&inicioRegionConfig <= 0x081FFFFF)
        return FLASH_SECTOR_11;


    while (1)
        falloSistema(FALLO_ESCRITURA_FLASH);

    return FLASH_SECTOR_0;
}
#elif defined(STM32F722xx)
/*
Sector 0    0x08000000 - 0x08003FFF 16 Kbytes
Sector 1    0x08004000 - 0x08007FFF 16 Kbytes
Sector 2    0x08008000 - 0x0800BFFF 16 Kbytes
Sector 3    0x0800C000 - 0x0800FFFF 16 Kbytes
Sector 4    0x08010000 - 0x0801FFFF 64 Kbytes
Sector 5    0x08020000 - 0x0803FFFF 128 Kbytes
Sector 6    0x08040000 - 0x0805FFFF 128 Kbytes
Sector 7    0x08060000 - 0x0807FFFF 128 Kbytes
*/
/***************************************************************************************
**  Nombre:         uint32_t sectorParaConfigGrabadorFlash(void)
**  Descripcion:    Obtiene el sector de la flash
**  Parametros:     Ninguno
**  Retorno:        Sector de la flash
****************************************************************************************/
uint32_t sectorParaConfigGrabadorFlash(void)
{
    if ((uint32_t)&inicioRegionConfig <= 0x08003FFF)
        return FLASH_SECTOR_0;
    if ((uint32_t)&inicioRegionConfig <= 0x08007FFF)
        return FLASH_SECTOR_1;
    if ((uint32_t)&inicioRegionConfig <= 0x0800BFFF)
        return FLASH_SECTOR_2;
    if ((uint32_t)&inicioRegionConfig <= 0x0800FFFF)
        return FLASH_SECTOR_3;
    if ((uint32_t)&inicioRegionConfig <= 0x0801FFFF)
        return FLASH_SECTOR_4;
    if ((uint32_t)&inicioRegionConfig <= 0x0803FFFF)
        return FLASH_SECTOR_5;
    if ((uint32_t)&inicioRegionConfig <= 0x0805FFFF)
        return FLASH_SECTOR_6;
    if ((uint32_t)&inicioRegionConfig <= 0x0807FFFF)
        return FLASH_SECTOR_7;


    while (1)
        falloSistema(FALLO_ESCRITURA_FLASH);

    return FLASH_SECTOR_0;
}
#endif


/***************************************************************************************
**  Nombre:         int32_t escribirWordGrabadorFlash(grabadorFlash_t *grabador, uint32_t valor)
**  Descripcion:    Escribe una word en la flash
**  Parametros:     grabador, valor a grabar
**  Retorno:        Codigo de error: 0 no error, -1 fallo en el borrado, -2 Fallo al grabar
****************************************************************************************/
int32_t escribirWordGrabadorFlash(grabadorFlash_t *grabador, uint32_t valor)
{
    if (grabador->err != 0)
        return grabador->err;

    if (grabador->dir % FLASH_PAGE_SIZE == 0) {
        FLASH_EraseInitTypeDef inicioBorrado = {
            .TypeErase = FLASH_TYPEERASE_SECTORS,
            .VoltageRange = FLASH_VOLTAGE_RANGE_3,     // 2.7-3.6V
            .NbSectors = 1
        };
        inicioBorrado.Sector = sectorParaConfigGrabadorFlash();

        uint32_t errorSector;
        const HAL_StatusTypeDef estado = HAL_FLASHEx_Erase(&inicioBorrado, &errorSector);
        if (estado != HAL_OK)
            return -1;
    }

    const HAL_StatusTypeDef estado = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, grabador->dir, valor);
    if (estado != HAL_OK)
        return -2;

    grabador->dir += sizeof(valor);
    return 0;
}

