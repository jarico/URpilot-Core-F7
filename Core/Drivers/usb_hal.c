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
#include "usbd_core.h"
#include "nvic.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef estadoHAL);


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarDriverUSB(void)
**  Descripcion:    Inicia el USB
**  Parametros:     Ninguno
**  Retorno:        True si OK
****************************************************************************************/
bool iniciarDriverUSB(void)
{
    usb_t *driver = punteroUSB();
    USBD_DescriptorsTypeDef *descriptor = punteroDescriptorUSB();
    USBD_CDC_ItfTypeDef *interfazCDC = punteroUSBInterfazCDC();

    if (!asignarHALusb())
        return false;

    if (USBD_Init(&driver->hal.hUSB, descriptor, DEVICE_FS) != USBD_OK)
        return false;

    if (USBD_RegisterClass(&driver->hal.hUSB, &USBD_CDC) != USBD_OK)
	    return false;

    if (USBD_CDC_RegisterInterface(&driver->hal.hUSB, interfazCDC) != USBD_OK)
	    return false;

    if (USBD_Start(&driver->hal.hUSB) != USBD_OK)
	    return false;

    return true;
}


/***************************************************************************************
**  Nombre:         void OTG_FS_IRQHandler(void)
**  Descripcion:    Interrupcion del USB
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void OTG_FS_IRQHandler(void)
{
	usb_t *driver = punteroUSB();
    HAL_PCD_IRQHandler(&driver->hal.hPCD);
}


/***************************************************************************************
**  Nombre:         void HAL_PCD_MspInit(PCD_HandleTypeDef *pcdHandle)
**  Descripcion:    Inicializa el PCD MSP
**  Parametros:     Handler PCD
**  Retorno:        Ninguno
****************************************************************************************/
void HAL_PCD_MspInit(PCD_HandleTypeDef* pcdHandle)
{
    if(pcdHandle->Instance==USB_OTG_FS) {
  	    usb_t *driver = punteroUSB();

  	    configurarIO(driver->hal.pinDM.pin, CONFIG_IO(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL), driver->hal.pinDM.af);
  	    configurarIO(driver->hal.pinDP.pin, CONFIG_IO(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL), driver->hal.pinDP.af);

        // Habilitacion del reloj del USB
        __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

        // Configuracion de las interrupciones
        HAL_NVIC_SetPriority(driver->hal.IRQ, PRIORIDAD_BASE_NVIC(driver->hal.prioridadIRQ), PRIORIDAD_SUB_NVIC(driver->hal.prioridadIRQ));
        HAL_NVIC_EnableIRQ(driver->hal.IRQ);
    }
}


/***************************************************************************************
**  Nombre:         void HAL_PCD_MspDeInit(PCD_HandleTypeDef *pcdHandle)
**  Descripcion:    De inicializacion del PCD MSP
**  Parametros:     Handler PCD
**  Retorno:        Ninguno
****************************************************************************************/
void HAL_PCD_MspDeInit(PCD_HandleTypeDef* pcdHandle)
{
    if (pcdHandle->Instance == USB_OTG_FS) {
        __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
    }
}


/***************************************************************************************
**  Nombre:         void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
**  Descripcion:    Configuracion de la etapa callback
**  Parametros:     Handler PCD
**  Retorno:        Ninguno
****************************************************************************************/
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
#endif
{
    USBD_LL_SetupStage((USBD_HandleTypeDef*)hpcd->pData, (uint8_t *)hpcd->Setup);
}


/***************************************************************************************
**  Nombre:         void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epNum)
**  Descripcion:    Callback de la etapa de salida
**  Parametros:     Handler PCD, numero de endpoint
**  Retorno:        Ninguno
****************************************************************************************/
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epNum)
#else
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epNum)
#endif
{
    USBD_LL_DataOutStage((USBD_HandleTypeDef*)hpcd->pData, epNum, hpcd->OUT_ep[epNum].xfer_buff);
}


/***************************************************************************************
**  Nombre:         void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epNum)
**  Descripcion:    Callback de la etapa de entrada
**  Parametros:     Handler PCD, numero de endpoint
**  Retorno:        Ninguno
****************************************************************************************/
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epNum)
#else
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epNum)
#endif
{
    USBD_LL_DataInStage((USBD_HandleTypeDef*)hpcd->pData, epNum, hpcd->IN_ep[epNum].xfer_buff);
}


/***************************************************************************************
**  Nombre:         void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
**  Descripcion:    Callback SOF
**  Parametros:     Handler PCD
**  Retorno:        Ninguno
****************************************************************************************/
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
#endif
{
    USBD_LL_SOF((USBD_HandleTypeDef*)hpcd->pData);
}


/***************************************************************************************
**  Nombre:         void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
**  Descripcion:    Callback reset
**  Parametros:     Handler PCD
**  Retorno:        Ninguno
****************************************************************************************/
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
#endif
{
    USBD_SpeedTypeDef velocidad = USBD_SPEED_FULL;

    switch (hpcd->Init.speed) {
        case PCD_SPEED_HIGH:
        	velocidad = USBD_SPEED_HIGH;
            break;

        case PCD_SPEED_FULL:
        	velocidad = USBD_SPEED_FULL;
            break;

        default:
        	velocidad = USBD_SPEED_FULL;
            break;
    }

    USBD_LL_SetSpeed((USBD_HandleTypeDef*)hpcd->pData, velocidad);
    USBD_LL_Reset((USBD_HandleTypeDef*)hpcd->pData);
}


/***************************************************************************************
**  Nombre:         void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
**  Descripcion:    Callbck suspender. Cuando el modo de bajo consumo esta activado el
**                  debug no se puede usar.
**  Parametros:     Handler PCD
**  Retorno:        Ninguno
****************************************************************************************/
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
#endif
{
    USBD_LL_Suspend((USBD_HandleTypeDef*)hpcd->pData);
    __HAL_PCD_GATE_PHYCLOCK(hpcd);

    // Modo stop
    if (hpcd->Init.low_power_enable)
        SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));      // Activa el bit SLEEPDEEP y SleepOnExit del SCR

}


/***************************************************************************************
**  Nombre:         void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
**  Descripcion:    Callback reanudar. Cuando el modo de bajo consumo esta activado el
**                  debug no se puede usar
**  Parametros:     Handler PCD
**  Retorno:        Ninguno
****************************************************************************************/
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
#endif
{
    USBD_LL_Resume((USBD_HandleTypeDef*)hpcd->pData);
}


/***************************************************************************************
**  Nombre:         void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epNum)
**  Descripcion:    Callback ISOOUTIncomplete
**  Parametros:     Handler PCD, numero de endpoint
**  Retorno:        Ninguno
****************************************************************************************/
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epNum)
#else
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epNum)
#endif
{
    USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef*)hpcd->pData, epNum);
}


/***************************************************************************************
**  Nombre:         void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epNum)
**  Descripcion:    Callback ISOINIncomplete
**  Parametros:     Handler PCD, numero de endpoint
**  Retorno:        Ninguno
****************************************************************************************/
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epNum)
#else
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epNum)
#endif
{
    USBD_LL_IsoINIncomplete((USBD_HandleTypeDef*)hpcd->pData, epNum);
}


/***************************************************************************************
**  Nombre:         void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
**  Descripcion:    Callback conectar
**  Parametros:     Handler PCD
**  Retorno:        Ninguno
****************************************************************************************/
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
#endif
{
    USBD_LL_DevConnected((USBD_HandleTypeDef*)hpcd->pData);
}


/***************************************************************************************
**  Nombre:         void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
**  Descripcion:    Callback desconectar
**  Parametros:     Handler PCD
**  Retorno:        Ninguno
****************************************************************************************/
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
#endif
{
    USBD_LL_DevDisconnected((USBD_HandleTypeDef*)hpcd->pData);
}


/***************************************************************************************
**  Nombre:         USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
**  Descripcion:    Inicializa la parte de bajo nivel del controlador de dispositivo
**  Parametros:     Handler del dispositivo
**  Retorno:        Estado
****************************************************************************************/
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
    if (pdev->id == DEVICE_FS) {
        usb_t *driver = punteroUSB();
    	PCD_HandleTypeDef *pcd = &driver->hal.hPCD;

    	pcd->pData = pdev;
        pdev->pData = pcd;

        pcd->Init.dev_endpoints = 6;
        pcd->Init.speed = PCD_SPEED_FULL;
        pcd->Init.dma_enable = DISABLE;
        pcd->Init.phy_itface = PCD_PHY_EMBEDDED;
        pcd->Init.Sof_enable = DISABLE;
        pcd->Init.low_power_enable = DISABLE;
        pcd->Init.lpm_enable = DISABLE;
        pcd->Init.vbus_sensing_enable = DISABLE;
        pcd->Init.use_dedicated_ep1 = DISABLE;

        HAL_PCD_Init(pcd);

        if (HAL_PCD_Init(pcd) != HAL_OK)
        	return USBD_FAIL;

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
        HAL_PCD_RegisterCallback(pcd, HAL_PCD_SOF_CB_ID, PCD_SOFCallback);
        HAL_PCD_RegisterCallback(pcd, HAL_PCD_SETUPSTAGE_CB_ID, PCD_SetupStageCallback);
        HAL_PCD_RegisterCallback(pcd, HAL_PCD_RESET_CB_ID, PCD_ResetCallback);
        HAL_PCD_RegisterCallback(pcd, HAL_PCD_SUSPEND_CB_ID, PCD_SuspendCallback);
        HAL_PCD_RegisterCallback(pcd, HAL_PCD_RESUME_CB_ID, PCD_ResumeCallback);
        HAL_PCD_RegisterCallback(pcd, HAL_PCD_CONNECT_CB_ID, PCD_ConnectCallback);
        HAL_PCD_RegisterCallback(pcd, HAL_PCD_DISCONNECT_CB_ID, PCD_DisconnectCallback);

        HAL_PCD_RegisterDataOutStageCallback(pcd, PCD_DataOutStageCallback);
        HAL_PCD_RegisterDataInStageCallback(pcd, PCD_DataInStageCallback);
        HAL_PCD_RegisterIsoOutIncpltCallback(pcd, PCD_ISOOUTIncompleteCallback);
        HAL_PCD_RegisterIsoInIncpltCallback(pcd, PCD_ISOINIncompleteCallback);
#endif

        HAL_PCDEx_SetRxFiFo(pcd, 0x80);
        HAL_PCDEx_SetTxFiFo(pcd, 0, 0x40);
        HAL_PCDEx_SetTxFiFo(pcd, 1, 0x80);
    }

    return USBD_OK;
}


/***************************************************************************************
**  Nombre:         USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
**  Descripcion:    De-inicializa la parte de bajo nivel del controlador de dispositivo
**  Parametros:     Handler del dispositivo
**  Retorno:        Estado
****************************************************************************************/
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
    HAL_StatusTypeDef estadoHAL = HAL_OK;
    USBD_StatusTypeDef estadoUSB = USBD_OK;

    estadoHAL = HAL_PCD_DeInit(pdev->pData);
    estadoUSB = USBD_Get_USB_Status(estadoHAL);

    return estadoUSB;
}


/***************************************************************************************
**  Nombre:         USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
**  Descripcion:    Inicia la parte de bajo nivel del controlador de dispositivo
**  Parametros:     Handler del dispositivo
**  Retorno:        Estado
****************************************************************************************/
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
    HAL_StatusTypeDef estadoHAL = HAL_OK;
    USBD_StatusTypeDef estadoUSB = USBD_OK;

    estadoHAL = HAL_PCD_Start(pdev->pData);
    estadoUSB = USBD_Get_USB_Status(estadoHAL);

    return estadoUSB;
}


/***************************************************************************************
**  Nombre:         USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
**  Descripcion:    Detiene la parte de bajo nivel del controlador de dispositivo
**  Parametros:     Handler del dispositivo
**  Retorno:        Estado
****************************************************************************************/
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
    HAL_StatusTypeDef estadoHAL = HAL_OK;
    USBD_StatusTypeDef estadoUSB = USBD_OK;

    estadoHAL = HAL_PCD_Stop(pdev->pData);
    estadoUSB = USBD_Get_USB_Status(estadoHAL);

    return estadoUSB;
}


/***************************************************************************************
**  Nombre:         USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t epDir, uint8_t epTipo, uint16_t epMaxTam)
**  Descripcion:    Abre un endpoint del controlador de bajo nivel
**  Parametros:     Handler del dispositivo, numero de endpoint, tipo de endpoint, tamanio maximo paquetes endpoint
**  Retorno:        Estado
****************************************************************************************/
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t epDir, uint8_t epTipo, uint16_t epMaxTam)
{
    HAL_StatusTypeDef estadoHAL = HAL_OK;
    USBD_StatusTypeDef estadoUSB = USBD_OK;

    estadoHAL = HAL_PCD_EP_Open(pdev->pData, epDir, epMaxTam, epTipo);
    estadoUSB = USBD_Get_USB_Status(estadoHAL);

    return estadoUSB;
}


/***************************************************************************************
**  Nombre:         USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t epDir)
**  Descripcion:    Cierra un endpoint del controlador de bajo nivel
**  Parametros:     Handler del dispositivo, numero de endpoint
**  Retorno:        Estado
****************************************************************************************/
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t epDir)
{
    HAL_StatusTypeDef estadoHAL = HAL_OK;
    USBD_StatusTypeDef estadoUSB = USBD_OK;

    estadoHAL = HAL_PCD_EP_Close(pdev->pData, epDir);
    estadoUSB = USBD_Get_USB_Status(estadoHAL);

    return estadoUSB;
}


/***************************************************************************************
**  Nombre:         USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t epDir)
**  Descripcion:    Limpia un endpoint
**  Parametros:     Handler del dispositivo, numero de endpoint
**  Retorno:        Estado
****************************************************************************************/
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t epDir)
{
    HAL_StatusTypeDef estadoHAL = HAL_OK;
    USBD_StatusTypeDef estadoUSB = USBD_OK;

    estadoHAL = HAL_PCD_EP_Flush(pdev->pData, epDir);
    estadoUSB = USBD_Get_USB_Status(estadoHAL);

    return estadoUSB;
}


/***************************************************************************************
**  Nombre:         USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t epDir)
**  Descripcion:    Establece una condición de bloqueo en un punto final del controlador de nivel bajo
**  Parametros:     Handler del dispositivo, numero de endpoint
**  Retorno:        Estado
****************************************************************************************/
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t epDir)
{
    HAL_StatusTypeDef estadoHAL = HAL_OK;
    USBD_StatusTypeDef estadoUSB = USBD_OK;

    estadoHAL = HAL_PCD_EP_SetStall(pdev->pData, epDir);
    estadoUSB = USBD_Get_USB_Status(estadoHAL);

    return estadoUSB;
}


/***************************************************************************************
**  Nombre:         USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t epDir)
**  Descripcion:    Borra una condición de bloqueo en un punto final del controlador de nivel bajo
**  Parametros:     Handler del dispositivo, numero de endpoint
**  Retorno:        Estado
****************************************************************************************/
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t epDir)
{
    HAL_StatusTypeDef estadoHAL = HAL_OK;
    USBD_StatusTypeDef estadoUSB = USBD_OK;

    estadoHAL = HAL_PCD_EP_ClrStall(pdev->pData, epDir);
    estadoUSB = USBD_Get_USB_Status(estadoHAL);

    return estadoUSB;
}


/***************************************************************************************
**  Nombre:         uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t epDir)
**  Descripcion:    Devuelve si establoqueado
**  Parametros:     Handler del dispositivo, numero de endpoint
**  Retorno:        1 OK, 0 no OK
****************************************************************************************/
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t epDir)
{
    PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef*) pdev->pData;

    if((epDir & 0x80) == 0x80)
        return hpcd->IN_ep[epDir & 0x7F].is_stall;
    else
        return hpcd->OUT_ep[epDir & 0x7F].is_stall;
}


/***************************************************************************************
**  Nombre:         USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t epDir)
**  Descripcion:    Asigna una direccion de USB al dispositivo
**  Parametros:     Handler del dispositivo, numero de endpoint
**  Retorno:        Estado
****************************************************************************************/
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t epDir)
{
    HAL_StatusTypeDef estadoHAL = HAL_OK;
    USBD_StatusTypeDef estadoUSB = USBD_OK;

    estadoHAL = HAL_PCD_SetAddress(pdev->pData, epDir);
    estadoUSB = USBD_Get_USB_Status(estadoHAL);

    return estadoUSB;
}


/***************************************************************************************
**  Nombre:         USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t epDir, uint8_t *buf, uint32_t tam)
**  Descripcion:    Transmite datos sobre un endpoint
**  Parametros:     Handler del dispositivo, numero de endpoint, datos a enviar, tamanio de los datos
**  Retorno:        Estado
****************************************************************************************/
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t epDir, uint8_t *buf, uint32_t tam)
{
    HAL_StatusTypeDef estadoHAL = HAL_OK;
    USBD_StatusTypeDef estadoUSB = USBD_OK;

    estadoHAL = HAL_PCD_EP_Transmit(pdev->pData, epDir, buf, tam);
    estadoUSB =  USBD_Get_USB_Status(estadoHAL);

    return estadoUSB;
}


/***************************************************************************************
**  Nombre:         USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t epDir, uint8_t *buf, uint32_t tam)
**  Descripcion:    Prepara un endpoint para la recepcion
**  Parametros:     Handler del dispositivo, numero de endpoint, datos recibidos, numero de datos a recibir
**  Retorno:        Estado
****************************************************************************************/
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t epDir, uint8_t *buf, uint32_t tam)
{
    HAL_StatusTypeDef estadoHAL = HAL_OK;
    USBD_StatusTypeDef estadoUSB = USBD_OK;

    estadoHAL = HAL_PCD_EP_Receive(pdev->pData, epDir, buf, tam);
    estadoUSB =  USBD_Get_USB_Status(estadoHAL);

    return estadoUSB;
}


/***************************************************************************************
**  Nombre:         uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t epDir)
**  Descripcion:    Devuelve el ultimo tamanio de paquete transferido
**  Parametros:     Handler del dispositivo, numero de endpoint
**  Retorno:        Tamanio
****************************************************************************************/
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t epDir)
{
    return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef*) pdev->pData, epDir);
}


/***************************************************************************************
**  Nombre:         void HAL_PCDEx_LPM_Callback(PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef mensaje)
**  Descripcion:    Envia un mensaje LPM a la capa de usuario
**  Parametros:     Handler del dispositivo, mensaje
**  Retorno:        Tamanio
****************************************************************************************/
void HAL_PCDEx_LPM_Callback(PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef mensaje)
{
    switch (mensaje) {
        case PCD_LPM_L0_ACTIVE:
            if (hpcd->Init.low_power_enable) {
            	configurarRelojSistema();
                SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));  // Resetea el bit SLEEPDEEP del SCR
            }

            __HAL_PCD_UNGATE_PHYCLOCK(hpcd);
            USBD_LL_Resume(hpcd->pData);
            break;

        case PCD_LPM_L1_ACTIVE:
            __HAL_PCD_GATE_PHYCLOCK(hpcd);
            USBD_LL_Suspend(hpcd->pData);

            // Modo stop
            if (hpcd->Init.low_power_enable)
                SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));  // Activa el bit SLEEPDEEP y SleepOnExit del SCR

            break;
    }
}


/***************************************************************************************
**  Nombre:         USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef estadoHAL)
**  Descripcion:    Devuelve el estado del USB
**  Parametros:     Estado del handler
**  Retorno:        Estado del USB
****************************************************************************************/
USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef estadoHAL)
{
    USBD_StatusTypeDef estadoUSB = USBD_OK;

    switch (estadoHAL) {
        case HAL_OK :
    	    estadoUSB = USBD_OK;
            break;

        case HAL_ERROR:
    	    estadoUSB = USBD_FAIL;
            break;

        case HAL_BUSY:
    	    estadoUSB = USBD_BUSY;
            break;

        case HAL_TIMEOUT:
    	    estadoUSB = USBD_FAIL;
            break;

        default:
    	    estadoUSB = USBD_FAIL;
            break;
    }

    return estadoUSB;
}
#endif

