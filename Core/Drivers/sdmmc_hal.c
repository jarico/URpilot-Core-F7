/***************************************************************************************
**  sdmmc_hal.c - Funciones HAL de gestion del driver SDMMC
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 02/05/2020
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
#include "sdmmc.h"

#ifdef USAR_SDMMC
#include "GP/gp_sdmmc.h"
#include "dma.h"
#include "io.h"
#include "nvic.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    uint32_t *buffer;
    uint32_t tamBloque;
    uint32_t numBloques;
} parametrosLecturaSDMMC_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
parametrosLecturaSDMMC_t parametrosLecturaSDMMC;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void habilitarRelojSDMMC(void);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarDriverSDMMC(void)
**  Descripcion:    Inicia el driver SDMMC
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarDriverSDMMC(void)
{
    sdmmc_t *driver = punteroSDMMC();

    if (!driver->hal.asignado) {
        if (!asignarHALsdmmc())
            return false;
        else {
            // Configuracion los pines
            configurarIO(driver->hal.pinCK.pin, CONFIG_IO(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL), driver->hal.pinCK.af);
            configurarIO(driver->hal.pinCMD.pin, CONFIG_IO(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL), driver->hal.pinCMD.af);
            configurarIO(driver->hal.pinD0.pin, CONFIG_IO(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL), driver->hal.pinD0.af);

            if (configSDMMC_Sistema.ancho4bits == true) {
                configurarIO(driver->hal.pinD1.pin, CONFIG_IO(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL), driver->hal.pinD1.af);
                configurarIO(driver->hal.pinD2.pin, CONFIG_IO(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL), driver->hal.pinD2.af);
                configurarIO(driver->hal.pinD3.pin, CONFIG_IO(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL), driver->hal.pinD3.af);
            }

            driver->hal.asignado = true;
        }
    }

    if (!driver->hal.relojHabilitado) {
        habilitarRelojSDMMC();
        driver->hal.relojHabilitado = true;
    }

    // Configuracion del SDMMC
    driver->hal.hsdmmc.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    driver->hal.hsdmmc.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
    driver->hal.hsdmmc.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_ENABLE;
    if (configSDMMC()->ancho4bits == true)
        driver->hal.hsdmmc.Init.BusWide = SDMMC_BUS_WIDE_4B;
    else
        driver->hal.hsdmmc.Init.BusWide = SDMMC_BUS_WIDE_1B;

    driver->hal.hsdmmc.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
    driver->hal.hsdmmc.Init.ClockDiv = 0;

#ifdef USAR_DMA_SDMMC
    // Configuracion del DMA para el envio
    iniciarDMA(identificadorDMA(driver->hal.hdmaTx.Instance));

    driver->hal.hdmaTx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    driver->hal.hdmaTx.Init.PeriphInc = DMA_PINC_DISABLE;
    driver->hal.hdmaTx.Init.MemInc = DMA_MINC_ENABLE;
    driver->hal.hdmaTx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    driver->hal.hdmaTx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    driver->hal.hdmaTx.Init.Mode = DMA_PFCTRL;
    driver->hal.hdmaTx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    driver->hal.hdmaTx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    driver->hal.hdmaTx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    driver->hal.hdmaTx.Init.MemBurst = DMA_MBURST_INC4;
    driver->hal.hdmaTx.Init.PeriphBurst = DMA_PBURST_INC4;

    if (HAL_DMA_Init(&driver->hal.hdmaTx) != HAL_OK)
        return false;

    __HAL_LINKDMA(&driver->hal.hsdmmc, hdmatx, driver->hal.hdmaTx);

    // Configuracion del DMA para la recepcion
    iniciarDMA(identificadorDMA(driver->hal.hdmaRx.Instance));

    driver->hal.hdmaRx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    driver->hal.hdmaRx.Init.PeriphInc = DMA_PINC_DISABLE;
    driver->hal.hdmaRx.Init.MemInc = DMA_MINC_ENABLE;
    driver->hal.hdmaRx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    driver->hal.hdmaRx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    driver->hal.hdmaRx.Init.Mode = DMA_PFCTRL;
    driver->hal.hdmaRx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    driver->hal.hdmaRx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    driver->hal.hdmaRx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    driver->hal.hdmaRx.Init.MemBurst = DMA_MBURST_INC4;
    driver->hal.hdmaRx.Init.PeriphBurst = DMA_PBURST_INC4;

    if (HAL_DMA_Init(&driver->hal.hdmaRx) != HAL_OK)
      return false;

    __HAL_LINKDMA(&driver->hal.hsdmmc, hdmarx, driver->hal.hdmaRx);
#endif

    // Configuracion de las interrupciones
    HAL_NVIC_SetPriority(driver->hal.IRQ, PRIORIDAD_BASE_NVIC(driver->hal.prioridadIRQ), PRIORIDAD_SUB_NVIC(driver->hal.prioridadIRQ));
    HAL_NVIC_EnableIRQ(driver->hal.IRQ);

    return true;
}


/***************************************************************************************
**  Nombre:         bool iniciarTarjetaSDMMC(void)
**  Descripcion:    Inicia la tarjeta SD
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarTarjetaSDMMC(void)
{
	sdmmc_t *driver = punteroSDMMC();

    if (HAL_SD_Init(&driver->hal.hsdmmc) != HAL_OK)
        return false;

    switch(driver->hal.hsdmmc.SdCard.CardType) {
        case CARD_SDSC:
            switch (driver->hal.hsdmmc.SdCard.CardVersion) {
                case CARD_V1_X:
                    driver->tipo = SD_STD_CAPACIDAD_V1_1;
                    break;

                case CARD_V2_X:
                    driver->tipo = SD_STD_CAPACIDAD_V2_0;
                    break;

                default:
                    return false;
            }
            break;

        case CARD_SDHC_SDXC:
            driver->tipo = SD_ALTA_CAPACIDAD;
            break;

        default:
            return false;
    }

    return true;
}


/***************************************************************************************
**  Nombre:         errorSD_e leerInfoSDMMC(void)
**  Descripcion:    Obtiene la estructura infoSD_t de la tarjeta
**  Parametros:     Ninguno
**  Retorno:        Codigo del error
****************************************************************************************/
errorSD_e leerInfoSDMMC(void)
{
    sdmmc_t *driver = punteroSDMMC();
    errorSD_e error = SD_OK;
    uint32_t temp = 0;

    // Byte 0
    temp = (driver->hal.hsdmmc.CSD[0] & 0xFF000000) >> 24;
    driver->info.csd.estructuraCSD = (uint8_t)((temp & 0xC0) >> 6);
    driver->info.csd.versionEspecSistema = (uint8_t)((temp & 0x3C) >> 2);
    driver->info.csd.reservado1 = temp & 0x03;

    // Byte 1
    temp = (driver->hal.hsdmmc.CSD[0] & 0x00FF0000) >> 16;
    driver->info.csd.taac = (uint8_t)temp;

    // Byte 2
    temp = (driver->hal.hsdmmc.CSD[0] & 0x0000FF00) >> 8;
    driver->info.csd.nsac = (uint8_t)temp;

    // Byte 3
    temp = driver->hal.hsdmmc.CSD[0] & 0x000000FF;
    driver->info.csd.frecMaxBus = (uint8_t)temp;

    // Byte 4
    temp = (driver->hal.hsdmmc.CSD[1] & 0xFF000000) >> 24;
    driver->info.csd.clasesComandos = (uint16_t)(temp << 4);

    // Byte 5
    temp = (driver->hal.hsdmmc.CSD[1] & 0x00FF0000) >> 16;
    driver->info.csd.clasesComandos |= (uint16_t)((temp & 0xF0) >> 4);
    driver->info.csd.maxLonBloqueLect = (uint8_t)(temp & 0x0F);

    // Byte 6
    temp = (driver->hal.hsdmmc.CSD[1] & 0x0000FF00) >> 8;
    driver->info.csd.bloquesParcLecHab = (uint8_t)((temp & 0x80) >> 7);
    driver->info.csd.desalinBloqueEsc = (uint8_t)((temp & 0x40) >> 6);
    driver->info.csd.desalinBloqueLec = (uint8_t)((temp & 0x20) >> 5);
    driver->info.csd.dsrImplementado = (uint8_t)((temp & 0x10) >> 4);
    driver->info.csd.reservado2 = 0;

    if ((driver->tipo == SD_STD_CAPACIDAD_V1_1) || (driver->tipo == SD_STD_CAPACIDAD_V2_0)) {
    	driver->info.csd.tamanioDispositivo = (temp & 0x03) << 10;

        // Byte 7
        temp = (uint8_t)(driver->hal.hsdmmc.CSD[1] & 0x000000FF);
        driver->info.csd.tamanioDispositivo |= (temp) << 2;

        // Byte 8
        temp = (uint8_t)((driver->hal.hsdmmc.CSD[2] & 0xFF000000) >> 24);
        driver->info.csd.tamanioDispositivo |= (temp & 0xC0) >> 6;

        driver->info.csd.maxCorrienteLectVDDmin = (temp & 0x38) >> 3;
        driver->info.csd.maxCorrienteLectVDDmax = (temp & 0x07);

        // Byte 9
        temp = (uint8_t)((driver->hal.hsdmmc.CSD[2] & 0x00FF0000) >> 16);
        driver->info.csd.maxCorrienteEscVDDmin = (temp & 0xE0) >> 5;
        driver->info.csd.maxCorrienteEscVDDmax = (temp & 0x1C) >> 2;
        driver->info.csd.multTamanioDisp = (temp & 0x03) << 1;

        // Byte 10
        temp = (uint8_t)((driver->hal.hsdmmc.CSD[2] & 0x0000FF00) >> 8);
        driver->info.csd.multTamanioDisp |= (temp & 0x80) >> 7;

        driver->info.capacidad = (driver->info.csd.tamanioDispositivo + 1) ;
        driver->info.capacidad *= (1 << (driver->info.csd.multTamanioDisp + 2));
        driver->info.tamanioBloque = 1 << (driver->info.csd.maxLonBloqueLect);
        driver->info.capacidad = driver->info.capacidad * driver->info.tamanioBloque / 512;
    }
    else if (driver->tipo == SD_ALTA_CAPACIDAD) {
        // Byte 7
        temp = (uint8_t)(driver->hal.hsdmmc.CSD[1] & 0x000000FF);
        driver->info.csd.tamanioDispositivo = (temp & 0x3F) << 16;

        // Byte 8
        temp = (uint8_t)((driver->hal.hsdmmc.CSD[2] & 0xFF000000) >> 24);
        driver->info.csd.tamanioDispositivo |= (temp << 8);

        // Byte 9
        temp = (uint8_t)((driver->hal.hsdmmc.CSD[2] & 0x00FF0000) >> 16);
        driver->info.csd.tamanioDispositivo |= (temp);

        // Byte 10
        temp = (uint8_t)((driver->hal.hsdmmc.CSD[2] & 0x0000FF00) >> 8);
        driver->info.capacidad = ((uint64_t)driver->info.csd.tamanioDispositivo + 1) * 1024;
        driver->info.tamanioBloque = 512;
    }
    else {
        // Tipo no soportado
        error = SD_ERROR;
    }

    driver->info.csd.tamanioGrupoBorrado = (temp & 0x40) >> 6;
    driver->info.csd.multTamanioGrupoBorrado = (temp & 0x3F) << 1;

    // Byte 11
    temp = (uint8_t)(driver->hal.hsdmmc.CSD[2] & 0x000000FF);
    driver->info.csd.multTamanioGrupoBorrado |= (temp & 0x80) >> 7;
    driver->info.csd.tamGrupoProtecEsc = (temp & 0x7F);

    // Byte 12
    temp = (uint8_t)((driver->hal.hsdmmc.CSD[3] & 0xFF000000) >> 24);
    driver->info.csd.habGrupoProtecEsc = (temp & 0x80) >> 7;
    driver->info.csd.eccDefectoFabricante = (temp & 0x60) >> 5;
    driver->info.csd.factorVelEscritura = (temp & 0x1C) >> 2;
    driver->info.csd.lonMaxBloqueEscDatos = (temp & 0x03) << 2;

    // Byte 13
    temp = (uint8_t)((driver->hal.hsdmmc.CSD[3] & 0x00FF0000) >> 16);
    driver->info.csd.lonMaxBloqueEscDatos |= (temp & 0xC0) >> 6;
    driver->info.csd.bloquesParcialesEscHabil = (temp & 0x20) >> 5;
    driver->info.csd.reservado3 = 0;
    driver->info.csd.proteccionContAplicacion = (temp & 0x01);

    // Byte 14
    temp = (uint8_t)((driver->hal.hsdmmc.CSD[3] & 0x0000FF00) >> 8);
    driver->info.csd.formatoGrupoFicheros = (temp & 0x80) >> 7;
    driver->info.csd.flagCopia = (temp & 0x40) >> 6;
    driver->info.csd.proteccionEscrituraPerm = (temp & 0x20) >> 5;
    driver->info.csd.proteccionEscrituraTemp = (temp & 0x10) >> 4;
    driver->info.csd.formatoFichero = (temp & 0x0C) >> 2;
    driver->info.csd.ecc = (temp & 0x03);

    // Byte 15
    temp = (uint8_t)(driver->hal.hsdmmc.CSD[3] & 0x000000FF);
    driver->info.csd.csdCRC = (temp & 0xFE) >> 1;
    driver->info.csd.reservado4 = 1;

    // Byte 0
    temp = (uint8_t)((driver->hal.hsdmmc.CID[0] & 0xFF000000) >> 24);
    driver->info.cid.idFabricante = temp;

    // Byte 1
    temp = (uint8_t)((driver->hal.hsdmmc.CID[0] & 0x00FF0000) >> 16);
    driver->info.cid.idOEM = temp << 8;

    // Byte 2
    temp = (uint8_t)((driver->hal.hsdmmc.CID[0] & 0x000000FF00) >> 8);
    driver->info.cid.idOEM |= temp;

    // Byte 3
    temp = (uint8_t)(driver->hal.hsdmmc.CID[0] & 0x000000FF);
    driver->info.cid.nombreProducto1 = temp << 24;

    // Byte 4
    temp = (uint8_t)((driver->hal.hsdmmc.CID[1] & 0xFF000000) >> 24);
    driver->info.cid.nombreProducto1 |= temp << 16;

    // Byte 5
    temp = (uint8_t)((driver->hal.hsdmmc.CID[1] & 0x00FF0000) >> 16);
    driver->info.cid.nombreProducto1 |= temp << 8;

    // Byte 6
    temp = (uint8_t)((driver->hal.hsdmmc.CID[1] & 0x0000FF00) >> 8);
    driver->info.cid.nombreProducto1 |= temp;

    // Byte 7
    temp = (uint8_t)(driver->hal.hsdmmc.CID[1] & 0x000000FF);
    driver->info.cid.nombreProducto2 = temp;

    // Byte 8
    temp = (uint8_t)((driver->hal.hsdmmc.CID[2] & 0xFF000000) >> 24);
    driver->info.cid.revisionProducto = temp;

    // Byte 9
    temp = (uint8_t)((driver->hal.hsdmmc.CID[2] & 0x00FF0000) >> 16);
    driver->info.cid.numeroSerie = temp << 24;

    // Byte 10
    temp = (uint8_t)((driver->hal.hsdmmc.CID[2] & 0x0000FF00) >> 8);
    driver->info.cid.numeroSerie |= temp << 16;

    // Byte 11
    temp = (uint8_t)(driver->hal.hsdmmc.CID[2] & 0x000000FF);
    driver->info.cid.numeroSerie |= temp << 8;

    // Byte 12
    temp = (uint8_t)((driver->hal.hsdmmc.CID[3] & 0xFF000000) >> 24);
    driver->info.cid.numeroSerie |= temp;

    // Byte 13
    temp = (uint8_t)((driver->hal.hsdmmc.CID[3] & 0x00FF0000) >> 16);
    driver->info.cid.reservado1 |= (temp & 0xF0) >> 4;
    driver->info.cid.fechaProduccion = (temp & 0x0F) << 8;

    // Byte 14
    temp = (uint8_t)((driver->hal.hsdmmc.CID[3] & 0x0000FF00) >> 8);
    driver->info.cid.fechaProduccion |= temp;

    // Byte 15
    temp = (uint8_t)(driver->hal.hsdmmc.CID[3] & 0x000000FF);
    driver->info.cid.cidCRC = (temp & 0xFE) >> 1;
    driver->info.cid.reservado2 = 1;

    return error;
}


/***************************************************************************************
**  Nombre:         bool estadoSDsdmmc(void)
**  Descripcion:    Obtiene el Estado de la SD
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool estadoSDMMC(void)
{
    sdmmc_t *driver = punteroSDMMC();

    HAL_SD_CardStateTypeDef estado = HAL_SD_GetCardState(&driver->hal.hsdmmc);
    return estado == HAL_SD_CARD_TRANSFER;
}


/***************************************************************************************
**  Nombre:         uint32_t chequearEscrituraSDMMC(void)
**  Descripcion:    Comprueba si la escritura se ha completado
**  Parametros:     Ninguno
**  Retorno:        Codigo de error
****************************************************************************************/
uint32_t chequearEscrituraSDMMC(void)
{
    sdmmc_t *driver = punteroSDMMC();

    if (driver->escribiendo)
        return HAL_SD_ERROR_BUSY;

    return HAL_SD_ERROR_NONE;
}


/***************************************************************************************
**  Nombre:         uint32_t chequearLecturaSDMMC(void)
**  Descripcion:    Comprueba si la lectura se ha completado
**  Parametros:     Ninguno
**  Retorno:        Codigo de error
****************************************************************************************/
uint32_t chequearLecturaSDMMC(void)
{
    sdmmc_t *driver = punteroSDMMC();

    if (driver->leyendo)
        return HAL_SD_ERROR_BUSY;

    return HAL_SD_ERROR_NONE;
}


/***************************************************************************************
**  Nombre:         errorSD_e escribirBloquesSDMMC(uint64_t dir, uint32_t *buffer, uint32_t tamBloque, uint32_t numBloques)
**  Descripcion:    Escribe bloques en la SD
**  Parametros:     Direccion, buffer a escribir, tamanio del bloque, numero de bloques a escribir
**  Retorno:        Codigo de error
****************************************************************************************/
CODIGO_RAPIDO errorSD_e escribirBloquesSDMMC(uint64_t dir, uint32_t *buffer, uint32_t tamBloque, uint32_t numBloques)
{
    sdmmc_t *driver = punteroSDMMC();

    driver->escribiendo = true;

    if (tamBloque != 512)
        return SD_ERROR;

#ifdef USAR_DMA_SDMMC
    uint32_t dirAlineada = (uint32_t)buffer & ~0x1F;
    SCB_CleanDCache_by_Addr((uint32_t*)dirAlineada, numBloques * tamBloque + ((uint32_t)buffer - dirAlineada));

    if (HAL_SD_WriteBlocks_DMA(&driver->hal.hsdmmc, (uint8_t *)buffer, dir, numBloques) != HAL_OK)
        return SD_ERROR;

#else
    if (HAL_SD_WriteBlocks_IT(&driver->hal.hsdmmc, (uint8_t *)buffer, dir, numBloques) != HAL_OK)
        return SD_ERROR;
#endif

    return SD_OK;
}


/***************************************************************************************
**  Nombre:         errorSD_e leerBloquesSDMMC(uint64_t dir, uint32_t *buffer, uint32_t tamBloque, uint32_t numBloques)
**  Descripcion:    Lee bloques de la SD
**  Parametros:     Direccion, buffer a leer, tamanio del bloque, numero de bloques a leer
**  Retorno:        Codigo de error
****************************************************************************************/
CODIGO_RAPIDO errorSD_e leerBloquesSDMMC(uint64_t dir, uint32_t *buffer, uint32_t tamBloque, uint32_t numBloques)
{
    sdmmc_t *driver = punteroSDMMC();

    if (tamBloque != 512)
        return SD_ERROR;

    driver->leyendo = true;

    parametrosLecturaSDMMC.buffer = buffer;
    parametrosLecturaSDMMC.tamBloque = tamBloque;
    parametrosLecturaSDMMC.numBloques = numBloques;

#ifdef USAR_DMA_SDMMC
    if (HAL_SD_ReadBlocks_DMA(&driver->hal.hsdmmc, (uint8_t *)buffer, dir, numBloques) != HAL_OK)
        return SD_ERROR;

#else
    if (HAL_SD_ReadBlocks_IT(&driver->hal.hsdmmc, (uint8_t *)buffer, dir, numBloques) != HAL_OK)
        return SD_ERROR;
#endif

    return SD_OK;
}


/***************************************************************************************
**  Nombre:         void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsdmmc)
**  Descripcion:    Callback de la transferencia completa
**  Parametros:     Handler de la sdmmc
**  Retorno:        Ninguno
****************************************************************************************/
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsdmmc)
{
    UNUSED(hsdmmc);

    sdmmc_t *driver = punteroSDMMC();
    driver->escribiendo = false;
}


/***************************************************************************************
**  Nombre:         void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsdmmc)
**  Descripcion:    Callback de la recepcion completa
**  Parametros:     Handler de la sdmmc
**  Retorno:        Ninguno
****************************************************************************************/
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsdmmc)
{
    UNUSED(hsdmmc);

    sdmmc_t *driver = punteroSDMMC();
    driver->leyendo = false;

    uint32_t dirAlineada = (uint32_t)parametrosLecturaSDMMC.buffer & ~0x1F;
    SCB_InvalidateDCache_by_Addr((uint32_t*)dirAlineada, parametrosLecturaSDMMC.numBloques * parametrosLecturaSDMMC.tamBloque + ((uint32_t)parametrosLecturaSDMMC.buffer - dirAlineada));
}


/***************************************************************************************
**  Nombre:         void HAL_SD_AbortCallback(SD_HandleTypeDef *hsdmmc)
**  Descripcion:    Abortar callback
**  Parametros:     Handler de la sdmmc
**  Retorno:        Ninguno
****************************************************************************************/
void HAL_SD_AbortCallback(SD_HandleTypeDef *hsdmmc)
{
    UNUSED(hsdmmc);

    sdmmc_t *driver = punteroSDMMC();

    driver->escribiendo = false;
    driver->leyendo = false;
}


/***************************************************************************************
**  Nombre:         void SDMMC1_IRQHandler(void)
**  Descripcion:    Interrupcion general del SDMMC1
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void SDMMC1_IRQHandler(void)
{
    sdmmc_t *driver = punteroSDMMC();
    HAL_SD_IRQHandler(&driver->hal.hsdmmc);
}


/***************************************************************************************
**  Nombre:         void SDMMC2_IRQHandler(void)
**  Descripcion:    Interrupcion general del SDMMC2
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void SDMMC2_IRQHandler(void)
{
    sdmmc_t *driver = punteroSDMMC();
    HAL_SD_IRQHandler(&driver->hal.hsdmmc);
}


/***************************************************************************************
**  Nombre:         void habilitarRelojSDMMC(void)
**  Descripcion:    Habilita el reloj del SDMMC
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void habilitarRelojSDMMC(void)
{
    switch (configSDMMC()->numSDMMC) {
        case SDMMC_1:
        	__HAL_RCC_SDMMC1_CLK_ENABLE();
        	break;

        case SDMMC_2:
        	__HAL_RCC_SDMMC2_CLK_ENABLE();
        	break;

        default:
            break;
    }
}


#endif // USAR_SDMMC

