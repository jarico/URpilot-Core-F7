/***************************************************************************************
**  scheduler.h - Funciones del scheduler que gestiona las tareas
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 12/05/2019
**  Fecha de modificacion: 21/08/2020
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

#ifndef __SCHEDULER_H
#define __SCHEDULER_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define INTERVALO_GUARDA_TIEMPO_REAL        50                     // Tiempo en us

#define PERIODO_TAREA_HZ_SCHEDULER(hz)      (1000000 / (hz))
#define PERIODO_TAREA_MS_SCHEDULER(ms)      ((ms) * 1000)
#define PERIODO_TAREA_US_SCHEDULER(us)      (us)


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    PRIORIDAD_DESHABILITADA = 0,       // La tarea solo es ejecutada si no hay otra tarea activa en ese ciclo
    PRIORIDAD_BAJA          = 1,
    PRIORIDAD_MEDIA         = 3,
    PRIORIDAD_MEDIA_ALTA    = 4,
    PRIORIDAD_ALTA          = 5,
    PRIORIDAD_TIEMPO_REAL   = 6,
    PRIORIDAD_MAXIMA        = 255,
} prioridadTarea_e;

typedef struct {
    const char *nombreTarea;
    const char *subNombreTarea;
    bool habilitado;
    uint8_t prioridadEstatica;
    int32_t periodo;
    int32_t ultimoPeriodo;
    uint32_t tiempoMaxEjecucion;
    uint32_t tiempoEjecucionTotal;
    uint32_t tiempoEjecucionMedio;
} infoTarea_t;

typedef enum {
	TAREA_SCHEDULER = 0,
    TAREA_STACK,
#ifdef USAR_ADC_INTERNO
    TAREA_ADC_INTERNO,
#endif
#ifdef USAR_POWER_MODULE
    TAREA_LEER_POWER_MODULE,
    TAREA_ACTUALIZAR_POWER_MODULE,
#endif
#ifdef USAR_BARO
    TAREA_ACTUALIZAR_BARO,
    TAREA_LEER_BARO,
#endif
#ifdef USAR_MAG
    TAREA_ACTUALIZAR_MAG,
    TAREA_LEER_MAG,
    TAREA_ACTUALIZAR_CALIBRADOR_MAGNETOMETRO,
#endif
#ifdef USAR_IMU
    TAREA_ACTUALIZAR_IMU,
    TAREA_LEER_IMU,
    TAREA_ACTUALIZAR_CALIBRADOR_ACELEROMETRO,
    TAREA_ACTUALIZAR_CALIBRADOR_GIROSCOPIO,
#endif
#ifdef USAR_GPS
    TAREA_LEER_GPS,
#endif
#ifdef USAR_RADIO
    TAREA_LEER_RADIO,
#endif
    TAREA_ACTUALIZAR_RC,
    TAREA_ACTUALIZAR_VEL_ANGULAR_FC,
    TAREA_ACTUALIZAR_ACTITUD_FC,
    TAREA_ACTUALIZAR_POSICION_FC,
	TAREA_ACTUALIZAR_TELEMETRIA,
    TAREA_CONTADOR,
    TAREA_NINGUNA = TAREA_CONTADOR,
    TASK_SELF,
} idTarea_e;

typedef struct {
    // Configuracion
    const char *nombreTarea;
    const char *subNombreTarea;
    void (*funTarea)(uint32_t tiempoActual);
    int32_t periodo;
    const uint8_t prioridadEstatica;     // La prioridad dinamica crece en pasos de este valor, no deberia ser 0

    // Scheduling
    uint16_t prioridadDinamica;          // Medida de como de vieja es la tarea desde la ultima ejecucion
    uint16_t edadCiclosTarea;
    int32_t ultimoPeriodoEjec;           // Ultima periodo de ejecucion
    uint32_t ultimoTiempoEjec;           // Ultimo tiempo de invocacion

#if defined(USAR_ESTADISTICAS_TAREAS)
    // Estadisticas
    uint64_t sumaMovTiempoEjec;          // Suma sobre 32 muestras
    uint64_t tiempoMaxEjecucion;
    uint64_t tiempoEjecucionTotal;       // tiempo total consumido por la tarea desde el encendido
#endif

    int32_t periodoHist[100];
    uint32_t cntHist;
    float tau;
    bool calc;
} tarea_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
extern tarea_t tareas[TAREA_CONTADOR];


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void iniciarScheduler(void);
bool anadirTareaEnCola(tarea_t *tarea);
bool quitarTareaDeCola(tarea_t *tarea);
void ajustarFrecuenciaEjecucionTarea(idTarea_e idTarea, uint32_t periodo);
void calcularCargaScheduler(uint32_t tiempoActual);
uint8_t cargaScheduler(void);
void habilitarEstadisticasScheduler(void);
void deshabilitarEstadisticasScheduler(void);
void infoTarea(idTarea_e idTarea, infoTarea_t *infoTarea);
void resetearEstadisticasTarea(idTarea_e idTarea);
void resetearTiempoMaxEjecTarea(idTarea_e idTarea);
void scheduler(void);

#endif // __SCHEDULER_H
