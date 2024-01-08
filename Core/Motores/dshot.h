/***************************************************************************************
**  dshot.h - Funciones relativas al protocolo DSHOT
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 27/08/2019
**  Fecha de modificacion: 13/09/2020
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

#ifndef __DSHOT_H_
#define __DSHOT_H_

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "motor.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define COMANDO_MAX_DSHOT                   47
#define MIN_THROTTLE_DSHOT                  48
#define MAX_THROTTLE_DSHOT                  2047

#define TAMANIO_BUFFER_DMA_DSHOT            18    // resolucion + reset trama (2us)
#define TAMANIO_BUFFER_DMA_PROSHOT          6     // resolucion + reset trama (2us)

#define SIMBOLO_BASE_PROSHOT                24    // 1uS
#define MOTOR_LONGITUD_NIBBLE_PROSHOT       96    // 4uS
#define ANCHO_BIT_PROSHOT                   3


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
/*
  DshotSettingRequest (KISS24). Direccion de giro, 3d y guardado de configuracion requieren 10 solicitudes..
  y el byte TLM debe ser siempre alto si 1-47 son usados para enviar configuracion

  Modo 3D:
  0 = stop
  48   (bajo) - 1047 (alto) -> direccion negativa
  1048 (bajo) - 2047 (alto) -> direccion positiva
 */
typedef enum {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO,                                   // V2 includes settings
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST,                           // Actualmente no implementado
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON,                                    // Solo BLHeli32
    DSHOT_CMD_LED1_ON,                                    // Solo BLHeli32
    DSHOT_CMD_LED2_ON,                                    // Solo BLHeli32
    DSHOT_CMD_LED3_ON,                                    // Solo BLHeli32
    DSHOT_CMD_LED0_OFF,                                   // Solo BLHeli32
    DSHOT_CMD_LED1_OFF,                                   // Solo BLHeli32
    DSHOT_CMD_LED2_OFF,                                   // Solo BLHeli32
    DSHOT_CMD_LED3_OFF,                                   // Solo BLHeli32
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30,              // KISS audio Stream mode on/Off
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31,                    // KISS silent Mode on/Off
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 33,
    DSHOT_CMD_MAX = 47,
} comandosDshot_e;

typedef struct {
    TIM_TypeDef *tim;
    TIM_HandleTypeDef htim;
    DMA_Stream_TypeDef *dmaBurst;
    DMA_HandleTypeDef hdma;
    uint16_t longBurstDMA;
    uint32_t *bufferBurstDMA;
    uint16_t fuentesDMAtimer;
    uint32_t direccionEntrada;
} motorDshotTimer_t;

typedef struct {
    uint8_t pin;
    tim_t *timer;
    TIM_HandleTypeDef htim;
    DMA_HandleTypeDef hdma;
    uint16_t fuenteDMAtimer;
    uint8_t indiceDMAtimer;
    uint16_t valor;
    bool configurado;
    tipoCanal_e salida;
    uint8_t indice;
    motorDshotTimer_t *motorTimer;
    volatile bool solicitarTelemetria;
    uint32_t *bufferDMA;
} motorDshot_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
extern motorDshot_t motoresDshot[NUM_MAX_MOTORES];
extern bool usarBurstDshot;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void configurarHardwareDshot(tim_t *tim, uint8_t indice, protocoloMotor_e protocolo, uint8_t tipoCanal, uint8_t inversion);
motorDshot_t *motorDshot(uint8_t indice);
void escribirPWMdshot(uint8_t indice, float valor);
void actualizarPWMdshot(uint8_t numMotores);
uint8_t cargarBufferDMAdshot(uint32_t *bufferDMA, uint8_t paso, uint16_t paquete);
uint8_t cargarBufferDMAproshot(uint32_t *bufferDMA, uint8_t paso, uint16_t paquete);

bool comandoDshotSiendoProcesado(void);
uint8_t comandoDshot(uint8_t indice);
bool hayComandosDshotEnCola(void);
bool comandoSalidaDshotHabilitado(uint8_t numMotores);

#endif // __DSHOT_H_
