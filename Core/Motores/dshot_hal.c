/***************************************************************************************
**  dshot.c - Funciones relativas al protocolo DSHOT
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 27/08/2019
**  Fecha de modificacion: 09/09/2020
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
#include <math.h>

#include "dshot.h"

#ifdef USAR_DSHOT
#include "motor.h"
#include "Drivers/io.h"
#include "Drivers/dma.h"
#include "Drivers/nvic.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define PROSHOT1000_HZ                MHZ_A_HZ(24)
#define DSHOT1200_HZ                  MHZ_A_HZ(24)
#define DSHOT600_HZ                   MHZ_A_HZ(12)
#define DSHOT300_HZ                   MHZ_A_HZ(6)
#define DSHOT150_HZ                   MHZ_A_HZ(3)

#define BIT_0_MOTOR                   7
#define BIT_1_MOTOR                   14
#define LONGITUD_BIT_MOTOR            20

#define NUM_MAX_DMA_TIMERS_MOTOR      8


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static uint32_t bufferDshotDMA[NUM_MAX_MOTORES][TAMANIO_BUFFER_DMA_DSHOT];
static uint32_t bufferBurstDMA[NUM_MAX_DMA_TIMERS_MOTOR][TAMANIO_BUFFER_DMA_DSHOT * 4];
static uint8_t contadorMotorDshot = 0;
static motorDshotTimer_t motoresDshotTimer[NUM_MAX_MOTORES];
motorDshot_t motoresDshot[NUM_MAX_MOTORES];
bool usarBurstDshot = false;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
uint32_t frecDshot(protocoloMotor_e protocolo);
uint8_t indiceTimer(TIM_TypeDef *timer);
void iniciarPWMcanalDMA(TIM_HandleTypeDef *htim, uint32_t canal, uint32_t *pDato, uint16_t lon);
void pararPWMcanalDMA(TIM_HandleTypeDef *htim, uint32_t canal);
void iniciarPWMburstDMA(TIM_HandleTypeDef *htim, uint32_t dirBaseBurst, uint32_t fuenteSolicitudBurst, uint32_t unidadBurst, uint32_t* bufferBurst, uint32_t lonBurst);
uint16_t prepararPaqueteDshot(motorDshot_t *const motor);
void motor_DMA_IRQHandler(descriptorCanalDMA_t* descriptor);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void configurarHardwareDshot(tim_t *tim, uint8_t indice, protocoloMotor_e protocolo, uint8_t tipoCanal, uint8_t inversion)
**  Descripcion:    Configura el hardware para el protocolo dshot
**  Parametros:     Puntero al timer, indice del motor, protocolo dshot, tipo de canal usado, salida invertida o no
**  Retorno:        Ninguno
****************************************************************************************/
void configurarHardwareDshot(tim_t *tim, uint8_t indice, protocoloMotor_e protocolo, uint8_t tipoCanal, uint8_t inversion)
{
    DMA_Stream_TypeDef *sDMA;

    if (usarBurstDshot)
        sDMA = tim->halTimUP.DMAy_Streamx;
    else
        sDMA = tim->halTim.DMAy_Streamx;

    if (sDMA == NULL)
        return;

    motorDshot_t * const motor = &motoresDshot[indice];
    motor->salida = tipoCanal;
    motor->timer = tim;

    const uint8_t indiceTimer = indiceTimer(tim->hal.htim.Instance);
    const bool confTimer = (indiceTimer == contadorMotorDshot - 1);

    // Configuramos el GPIO
    configurarIO(tim->hal.pin.pin, CONFIG_IO(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN), tim->hal.pin.af);

    // Configuracion de la base de tiempo
    if (confTimer) {
        motor->htim.Instance = tim->hal.htim.Instance;
        motor->htim.Init.Prescaler = (uint16_t)(lrintf((float)SystemCoreClock / frecDshot(protocolo) + 0.01f) - 1);
        motor->htim.Init.Period = protocolo == PWM_TIPO_PROSHOT1000 ? MOTOR_LONGITUD_NIBBLE_PROSHOT : LONGITUD_BIT_MOTOR;
        motor->htim.Init.RepetitionCounter = 0;
        motor->htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        motor->htim.Init.CounterMode = TIM_COUNTERMODE_UP;
        motor->htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

        if (HAL_TIM_PWM_Init(&tim->hal.htim) != HAL_OK)
            return;
    }

    // Configuracion del canal
    TIM_OC_InitTypeDef configOC;
    configOC.OCMode = TIM_OCMODE_PWM1;
    if (tipoCanal & TIMER_CANAL_N) {
        configOC.OCIdleState = TIM_OCIDLESTATE_RESET;
        configOC.OCPolarity = (inversion & TIMER_SALIDA_INVERTIDA) ? TIM_OCPOLARITY_HIGH : TIM_OCPOLARITY_LOW;
        configOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
        configOC.OCNPolarity = (inversion & TIMER_SALIDA_INVERTIDA) ? TIM_OCNPOLARITY_HIGH : TIM_OCNPOLARITY_LOW;
    }
    else {
        configOC.OCIdleState = TIM_OCIDLESTATE_SET;
        configOC.OCPolarity = (inversion & TIMER_SALIDA_INVERTIDA) ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH;
        configOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
        configOC.OCNPolarity = (inversion & TIMER_SALIDA_INVERTIDA) ? TIM_OCNPOLARITY_LOW : TIM_OCNPOLARITY_HIGH;
    }

    configOC.OCFastMode = TIM_OCFAST_DISABLE;
    configOC.Pulse = 0;

    if (HAL_TIM_PWM_ConfigChannel(&tim->hal.htim, &configOC, tim->canal) != HAL_OK)
        return;

    // Configuracion del DMA
    motor->motorTimer = &motoresDshotTimer[indiceTimer];

    if (usarBurstDshot) {
        motor->motorTimer->dmaBurst = sDMA;

        if (!confTimer) {
            motor->configurado = true;
            return;
        }
    }
    else {
        motor->fuenteDMAtimer = fuenteDMAtimer(tim->canal);
        motor->motorTimer->fuentesDMAtimer |= motor->fuenteDMAtimer;
        motor->indiceDMAtimer = indiceDMAtimer(tim->canal);
    }

    if (usarBurstDshot) {
        motor->motorTimer->hdma.Init.Direction = DMA_MEMORY_TO_PERIPH;
        motor->motorTimer->hdma.Init.PeriphInc = DMA_PINC_DISABLE;
        motor->motorTimer->hdma.Init.MemInc = DMA_MINC_ENABLE;
        motor->motorTimer->hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD ;
        motor->motorTimer->hdma.Init.MemDataAlignment = DMA_MDATAALIGN_WORD ;
        motor->motorTimer->hdma.Init.Mode = DMA_NORMAL;
        motor->motorTimer->hdma.Init.Priority = DMA_PRIORITY_HIGH;
        motor->motorTimer->hdma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        motor->motorTimer->hdma.Init.PeriphBurst = DMA_PBURST_SINGLE;
        motor->motorTimer->hdma.Init.MemBurst = DMA_MBURST_SINGLE;
        motor->motorTimer->hdma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;

        motor->motorTimer->bufferBurstDMA = &bufferBurstDMA[indiceTimer][0];
        memset(motor->motorTimer->bufferBurstDMA, 0, TAMANIO_BUFFER_DMA_DSHOT * 4 * sizeof(uint32_t));

        motor->motorTimer->htim = motor->htim;
        motor->motorTimer->hdma.Instance = tim->halTimUP.DMAy_Streamx;

        __HAL_LINKDMA(&motor->motorTimer->htim, hdma[TIM_DMA_ID_UPDATE], motor->motorTimer->hdma);
        if (HAL_DMA_Init(motor->motorTimer->htim.hdma[TIM_DMA_ID_UPDATE]) != HAL_OK)
            return;
    }
    else {
        motor->hdma.Init.Direction = DMA_MEMORY_TO_PERIPH;
        motor->hdma.Init.PeriphInc = DMA_PINC_DISABLE;
        motor->hdma.Init.MemInc = DMA_MINC_ENABLE;
        motor->hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD ;
        motor->hdma.Init.MemDataAlignment = DMA_MDATAALIGN_WORD ;
        motor->hdma.Init.Mode = DMA_NORMAL;
        motor->hdma.Init.Priority = DMA_PRIORITY_HIGH;
        motor->hdma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        motor->hdma.Init.PeriphBurst = DMA_PBURST_SINGLE;
        motor->hdma.Init.MemBurst = DMA_MBURST_SINGLE;
        motor->hdma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;

        motor->bufferDMA = &bufferDshotDMA[indice][0];
        motor->bufferDMA[TAMANIO_BUFFER_DMA_DSHOT - 2] = 0;
        motor->bufferDMA[TAMANIO_BUFFER_DMA_DSHOT - 1] = 0;

        motor->hdma.Instance = tim->halTim.DMAy_Streamx;

        __HAL_LINKDMA(&motor->htim, hdma[motor->indiceDMAtimer], motor->hdma);
        if (HAL_DMA_Init(motor->htim.hdma[motor->indiceDMAtimer]) != HAL_OK)
            return;
    }

    if (usarBurstDshot) {
    	iniciarDMA(tim->halTimUP.dmaTimIrqHandler);
    	ajustarHandlerDMA(tim->halTimUP.dmaTimIrqHandler, motor_DMA_IRQHandler, CONSTRUIR_PRIORIDAD_NVIC(1, 2), indiceTimer);
    }
    else {
    	iniciarDMA(tim->halTim.dmaTimIrqHandler);
    	ajustarHandlerDMA(tim->halTim.dmaTimIrqHandler, motor_DMA_IRQHandler, CONSTRUIR_PRIORIDAD_NVIC(1, 2), indice);
    }

    // Iniciamos el canal del timer
    // Habilitando y deshabilitando el DMA request se puede reiniciar un nuevo ciclo sin PWM start/stop
    if (tipoCanal == TIMER_CANAL_N) {
        if (HAL_TIMEx_PWMN_Start(&motor->htim, tim->canal) != HAL_OK)
            return;
    }
    else {
        if (HAL_TIM_PWM_Start(&motor->htim, tim->canal) != HAL_OK)
            return;
    }

    motor->configurado = true;
}


/***************************************************************************************
**  Nombre:         motorDshot_t *motorDshot(uint8_t indice)
**  Descripcion:    Obtiene la estructura motoresDshot
**  Parametros:     Indice
**  Retorno:        Puntero a la estructura dshot
****************************************************************************************/
motorDshot_t *motorDshot(uint8_t indice)
{
    return &motoresDshot[indice];
}


/***************************************************************************************
**  Nombre:         uint32_t frecDshot(protocoloMotor_e protocolo)
**  Descripcion:    Obtiene la frecuencia del protocolo
**  Parametros:     Protocolo
**  Retorno:        Frecuencia
****************************************************************************************/
uint32_t frecDshot(protocoloMotor_e protocolo)
{
    switch (protocolo) {
        case(PWM_TIPO_PROSHOT1000):
            return PROSHOT1000_HZ;

        case(PWM_TIPO_DSHOT1200):
            return DSHOT1200_HZ;

        case(PWM_TIPO_DSHOT600):
            return DSHOT600_HZ;

        case(PWM_TIPO_DSHOT300):
            return DSHOT300_HZ;

        default:
        case(PWM_TIPO_DSHOT150):
            return DSHOT150_HZ;
    }
}


/***************************************************************************************
**  Nombre:         uint8_t indiceTimer(TIM_TypeDef *tim)
**  Descripcion:    Obtiene el indice del timer
**  Parametros:     Puntero al timer
**  Retorno:        Indice
****************************************************************************************/
uint8_t indiceTimer(TIM_TypeDef *tim)
{
    for (uint8_t i = 0; i < contadorMotorDshot; i++) {
        if (motoresDshotTimer[i].tim == tim)
            return i;
    }

    motoresDshotTimer[contadorMotorDshot++].tim = tim;
    return contadorMotorDshot - 1;
}


/***************************************************************************************
**  Nombre:         void escribirPWMdshot(uint8_t indice, float valor)
**  Descripcion:    Escribe el valor del PWM en dshot
**  Parametros:     Indice del motor, valor a escribir
**  Retorno:        Ninguno
****************************************************************************************/
void escribirPWMdshot(uint8_t indice, float valor)
{
    motorDshot_t *const motor = &motoresDshot[indice];

    if (!motor->configurado)
        return;

    // Si hay un comando listo para enviar, sobreescribe el valor y envia
    if (comandoDshotSiendoProcesado()) {
        valor = comandoDshot(indice);
        if (valor)
            motor->solicitarTelemetria = true;
    }

    if (!motor->timer || !motor->timer->halTim.DMAy_Streamx)
        return;

    motor->valor = valor;

    uint16_t paquete = prepararPaqueteDshot(motor);
    uint8_t tamBuffer;

    if (usarBurstDshot) {
        tamBuffer = cargarBufferDMA(&motor->motorTimer->bufferBurstDMA[motor->timer->canal >> 2], 4, paquete);
        motor->motorTimer->longBurstDMA = tamBuffer * 4;
    }
    else {
        tamBuffer = cargarBufferDMA(motor->bufferDMA, 1, paquete);
        iniciarPWMcanalDMA(&motor->htim, motor->timer->canal, motor->bufferDMA, tamBuffer);
    }
}


/***************************************************************************************
**  Nombre:         void actualizarPWMdshot(uint8_t numMotores)
**  Descripcion:    Actualizacion completa del PWM
**  Parametros:     Numero de motores
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarPWMdshot(uint8_t numMotores)
{
    if (hayComandosDshotEnCola()) {
        if (!comandoSalidaDshotHabilitado(numMotores))
            return;
    }

    if (usarBurstDshot) {
        for (uint8_t i = 0; i < contadorMotorDshot; i++) {
            motorDshotTimer_t *burstDMAtimer = &motoresDshotTimer[i];

            // Transfiere CCR1 a CCR4 por cada Burst
            iniciarPWMburstDMA(&burstDMAtimer->htim, TIM_DMABASE_CCR1, TIM_DMA_UPDATE, TIM_DMABURSTLENGTH_4TRANSFERS,
            (uint32_t*)burstDMAtimer->bufferBurstDMA, burstDMAtimer->longBurstDMA);
        }
    }
}


/***************************************************************************************
**  Nombre:         void iniciarPWMcanalDMA(TIM_HandleTypeDef *htim, uint32_t canal, uint32_t *pDato, uint16_t lon)
**  Descripcion:    TIM_CHANNEL_x TIM_CHANNEL_x/4 TIM_DMA_ID_CCx TIM_DMA_CCx
**                  0x0           0               1              0x0200
**                  0x4           1               2              0x0400
**                  0x8           2               3              0x0800
**                  0xC           3               4              0x1000
**
**                  TIM_CHANNEL_TO_TIM_DMA_ID_CC = (TIM_CHANNEL_x / 4) + 1
**                  TIM_CHANNEL_TO_TIM_DMA_CC = 0x200 << (TIM_CHANNEL_x / 4)
**
**                  Una variante de HAL_TIM_PWM_Start_DMA/HAL_TIMEx_PWMN_Start_DMA que solo habilita el DMA en un canal
**                      * Configura y habilita el DMA Stream
**                      * Habilita el DMA request en un canal del timer
**  Parametros:     Handler timer, canal, puntero al buffer, longitud del buffer
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarPWMcanalDMA(TIM_HandleTypeDef *htim, uint32_t canal, uint32_t *pDato, uint16_t lon)
{
    switch (canal) {
        case TIM_CHANNEL_1:
            HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC1], (uint32_t)pDato, (uint32_t)&htim->Instance->CCR1, lon);
            __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC1);
            break;

        case TIM_CHANNEL_2:
            HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC2], (uint32_t)pDato, (uint32_t)&htim->Instance->CCR2, lon);
            __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC2);
            break;

        case TIM_CHANNEL_3:
            HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC3], (uint32_t)pDato, (uint32_t)&htim->Instance->CCR3, lon);
            __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC3);
            break;

        case TIM_CHANNEL_4:
            HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC4], (uint32_t)pDato, (uint32_t)&htim->Instance->CCR4, lon);
            __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC4);
            break;
    }
}


/***************************************************************************************
**  Nombre:         void pararPWMcanalDMA(TIM_HandleTypeDef *htim, uint32_t canal)
**  Descripcion:    Una variante de HAL_TIM_PWM_Stop_DMA/HAL_TIMEx_PWMN_Stop_DMA que solo deshabilita
**                  el DMA en el canal del timer. Deshabilita el TIM Capture / Compare 1 DMA request
**  Parametros:     Handler timer, canal
**  Retorno:        Ninguno
****************************************************************************************/
void pararPWMcanalDMA(TIM_HandleTypeDef *htim, uint32_t canal)
{
    switch (canal) {
        case TIM_CHANNEL_1:
            __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
            break;

        case TIM_CHANNEL_2:
            __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
            break;

        case TIM_CHANNEL_3:
            __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
            break;

        case TIM_CHANNEL_4:
            __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
            break;
    }
}


/***************************************************************************************
**  Nombre:         void iniciarPWMburstDMA(TIM_HandleTypeDef *htim, uint32_t dirBaseBurst, uint32_t fuenteSolicitudBurst, uint32_t unidadBurst, uint32_t* bufferBurst, uint32_t lonBurst)
**  Descripcion:    Esta funcion maneja multiples burst no como HAL_TIM_DMABurst_WriteStart
**  Parametros:     Handler timer, direccion base burst, solicitud DMA, unidad burst, puntero al buffer, numero de datos a enviar
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarPWMburstDMA(TIM_HandleTypeDef *htim, uint32_t dirBaseBurst, uint32_t fuenteSolicitudBurst, uint32_t unidadBurst, uint32_t* bufferBurst, uint32_t lonBurst)
{
    HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_UPDATE], (uint32_t)bufferBurst, (uint32_t)&htim->Instance->DMAR, lonBurst);

    htim->Instance->DCR = dirBaseBurst | unidadBurst;

    __HAL_TIM_ENABLE_DMA(htim, fuenteSolicitudBurst);
}


/***************************************************************************************
**  Nombre:         uint8_t cargarBufferDMAdshot(uint32_t *bufferDMA, uint8_t paso, uint16_t paquete)
**  Descripcion:    Carga en el buffer del DMA los valores
**  Parametros:     Buffer, paso, paquete
**  Retorno:        Tamanio del buffer
****************************************************************************************/
CODIGO_RAPIDO uint8_t cargarBufferDMAdshot(uint32_t *bufferDMA, uint8_t paso, uint16_t paquete)
{
    for (uint8_t i = 0; i < 16; i++) {
        bufferDMA[i * paso] = (paquete & 0x8000) ? BIT_1_MOTOR : BIT_0_MOTOR;  // MSB primero
        paquete <<= 1;
    }
    bufferDMA[16 * paso] = 0;
    bufferDMA[17 * paso] = 0;

    return TAMANIO_BUFFER_DMA_DSHOT;
}


/***************************************************************************************
**  Nombre:         uint8_t cargarBufferDMAproshot(uint32_t *bufferDMA, uint8_t paso, uint16_t paquete)
**  Descripcion:    Carga en el buffer del DMA los valores
**  Parametros:     Buffer, paso, paquete
**  Retorno:        Tamanio del buffer
****************************************************************************************/
uint8_t cargarBufferDMAproshot(uint32_t *bufferDMA, uint8_t paso, uint16_t paquete)
{
    for (uint8_t i = 0; i < 4; i++) {
        bufferDMA[i * paso] = SIMBOLO_BASE_PROSHOT + ((paquete & 0xF000) >> 12) * ANCHO_BIT_PROSHOT;  // Primer cuarteto mas relevante
        paquete <<= 4;   // Desplazamos 4 bits
    }
    bufferDMA[4 * paso] = 0;
    bufferDMA[5 * paso] = 0;

    return TAMANIO_BUFFER_DMA_PROSHOT;
}


/***************************************************************************************
**  Nombre:         uint16_t prepararPaqueteDshot(motorDshot_t *const motor)
**  Descripcion:    Prepara el paquete de la trama
**  Parametros:     Puntero a la estructura Dshot
**  Retorno:        Paquete
****************************************************************************************/
CODIGO_RAPIDO uint16_t prepararPaqueteDshot(motorDshot_t *const motor)
{
    uint16_t paquete = (motor->valor << 1) | (motor->solicitarTelemetria ? 1 : 0);
    motor->solicitarTelemetria = false;

    // Calculamos el checksum
    int32_t csum = 0;
    int32_t datoCsum = paquete;

    for (uint8_t i = 0; i < 3; i++) {
        csum ^=  datoCsum;   // XOR por cuartetos
        datoCsum >>= 4;
    }

    csum &= 0xf;
    paquete = (paquete << 4) | csum;

    return paquete;
}


/***************************************************************************************
**  Nombre:         void motor_DMA_IRQHandler(descriptorCanalDMA_t* descriptor)
**  Descripcion:    Interrupcion del DMA
**  Parametros:     Descriptor del DMA
**  Retorno:        Ninguno
****************************************************************************************/
void motor_DMA_IRQHandler(descriptorCanalDMA_t* descriptor)
{
    if (OBTENER_FLAG_STATUS_DMA(descriptor, DMA_IT_TCIF)) {
        if (usarBurstDshot) {
            motorDshotTimer_t *burstDMAtimer = &motoresDshotTimer[descriptor->paramUsuario];

            HAL_TIM_DMABurst_WriteStop(&burstDMAtimer->htim, TIM_DMA_UPDATE);
            HAL_DMA_IRQHandler(&burstDMAtimer->hdma);
        }
        else {
            motorDshot_t * const motor = &motoresDshot[descriptor->paramUsuario];

            pararPWMcanalDMA(&motor->htim, motor->timer->canal);
            HAL_DMA_IRQHandler(motor->htim.hdma[motor->indiceDMAtimer]);
        }

        LIMPIAR_FLAG_DMA(descriptor, DMA_IT_TCIF);
    }
}

#endif
