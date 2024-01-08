/***************************************************************************************
**  tareas.c - Funciones generales de las tareas del scheduler
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

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include "tareas.h"
#include "Core/stack.h"
#include "Drivers/adc.h"
#include "GP/gp_adc.h"
#include "Sensores/PM/power_module.h"
#include "GP/gp_power_module.h"
#include "Sensores/Barometro/barometro.h"
#include "GP/gp_barometro.h"
#include "Sensores/Magnetometro/magnetometro.h"
#include "GP/gp_magnetometro.h"
#include "Sensores/IMU/imu.h"
#include "GP/gp_imu.h"
#include "Sensores/GPS/gps.h"
#include "GP/gp_gps.h"
#include "Radio/radio.h"
#include "GP/gp_radio.h"
#include "FC/rc.h"
#include "GP/gp_rc.h"
#include "FC/fc.h"
#include "GP/gp_fc.h"
#include "Blackbox/blackbox.h"
#include "Sensores/Calibrador/calibrador_imu.h"
#include "Sensores/Calibrador/calibrador_mag.h"
#include "GP/gp_calibrador.h"
#include "Telemetria/telemetria.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
tarea_t tareas[TAREA_CONTADOR] = {
    [TAREA_SCHEDULER] = {
        .nombreTarea = "SCHEDULER",
        .subNombreTarea = "CARGA",
        .funTarea = calcularCargaScheduler,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(10),
        .prioridadEstatica = PRIORIDAD_BAJA,
    },
    [TAREA_STACK] = {
        .nombreTarea = "STACK",
        .subNombreTarea = "",
        .funTarea = chequearStack,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(10),
        .prioridadEstatica = PRIORIDAD_DESHABILITADA,
    },
#ifdef USAR_ADC_INTERNO
    [TAREA_ADC_INTERNO] = {
        .nombreTarea = "ADC_INTERNO",
        .subNombreTarea = "ADC",
        .funTarea = actualizarADCinterno,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(FREC_ACTUALIZAR_ADC_INT_HZ),
        .prioridadEstatica = PRIORIDAD_DESHABILITADA,
    },
#endif
#ifdef USAR_POWER_MODULE
    [TAREA_ACTUALIZAR_POWER_MODULE] = {
        .nombreTarea = "ACTUALIZAR_POWER_MODULE",
        .subNombreTarea = "",
        .funTarea = actualizarPowerModule,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(FREC_ACTUALIZAR_POWER_MODULE_HZ),
        .prioridadEstatica = PRIORIDAD_BAJA,
    },
    [TAREA_LEER_POWER_MODULE] = {
        .nombreTarea = "LEER_POWER_MODULE",
        .subNombreTarea = "",
        .funTarea = leerPowerModule,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(FREC_LEER_POWER_MODULE_HZ),
        .prioridadEstatica = PRIORIDAD_BAJA,
    },
#endif
#ifdef USAR_BARO
    [TAREA_ACTUALIZAR_BARO] = {
        .nombreTarea = "ACTUALIZAR BARO",
        .subNombreTarea = "SENSORES",
        .funTarea = actualizarBaro,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(FREC_ACTUALIZAR_BARO_HZ),
        .prioridadEstatica = PRIORIDAD_ALTA,
    },
    [TAREA_LEER_BARO] = {
        .nombreTarea = "LEER BARO",
        .subNombreTarea = "SENSORES",
        .funTarea = leerBaro,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(FREC_LEER_BARO_HZ),
        .prioridadEstatica = PRIORIDAD_MEDIA_ALTA,
    },
#endif
#ifdef USAR_MAG
    [TAREA_ACTUALIZAR_MAG] = {
        .nombreTarea = "ACTUALIZAR MAG",
        .subNombreTarea = "SENSORES",
        .funTarea = actualizarMag,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(FREC_ACTUALIZAR_MAG_HZ),
        .prioridadEstatica = PRIORIDAD_ALTA,
    },
    [TAREA_LEER_MAG] = {
        .nombreTarea = "LEER MAG",
        .subNombreTarea = "SENSORES",
        .funTarea = leerMag,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(FREC_LEER_MAG_HZ),
        .prioridadEstatica = PRIORIDAD_MEDIA_ALTA,
    },
    [TAREA_ACTUALIZAR_CALIBRADOR_MAGNETOMETRO] = {
        .nombreTarea = "ACTUALIZAR CALIBRADOR MAGNETOMETRO",
        .subNombreTarea = "CALIBRADOR MAGNETOMETRO",
        .funTarea = actualizarCalMag,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(FREC_ACTUALIZAR_CALIBRADOR_MAG_HZ),
        .prioridadEstatica = PRIORIDAD_BAJA,
    },
#endif
#ifdef USAR_IMU
    [TAREA_ACTUALIZAR_IMU] = {
        .nombreTarea = "ACTUALIZAR IMU",
        .subNombreTarea = "SENSORES",
        .funTarea = actualizarIMU,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(FREC_ACTUALIZAR_IMU_HZ),
        .prioridadEstatica = PRIORIDAD_TIEMPO_REAL,
    },
    [TAREA_LEER_IMU] = {
        .nombreTarea = "LEER IMU",
        .subNombreTarea = "SENSORES",
        .funTarea = leerIMU,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(FREC_LEER_IMU_HZ),
        .prioridadEstatica = PRIORIDAD_TIEMPO_REAL,
    },
    [TAREA_ACTUALIZAR_CALIBRADOR_ACELEROMETRO] = {
        .nombreTarea = "ACTUALIZAR CALIBRADOR ACELEROMETRO",
        .subNombreTarea = "CALIBRADOR ACELEROMETRO",
        .funTarea = actualizarCalAcel,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(FREC_ACTUALIZAR_CALIBRADOR_IMU_HZ),
        .prioridadEstatica = PRIORIDAD_BAJA,
    },
    [TAREA_ACTUALIZAR_CALIBRADOR_GIROSCOPIO] = {
        .nombreTarea = "ACTUALIZAR CALIBRADOR GIROSCOPIO",
        .subNombreTarea = "CALIBRADOR GIROSCOPIO",
        .funTarea = actualizarCalGir,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(FREC_ACTUALIZAR_CALIBRADOR_IMU_HZ),
        .prioridadEstatica = PRIORIDAD_BAJA,
    },
#endif
#ifdef USAR_GPS
    [TAREA_LEER_GPS] = {
        .nombreTarea = "LEER GPS",
        .subNombreTarea = "SENSORES",
        .funTarea = leerGPS,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(FREC_LEER_GPS_HZ),
        .prioridadEstatica = PRIORIDAD_MEDIA,
    },
#endif
#ifdef USAR_RADIO
    [TAREA_LEER_RADIO] = {
        .nombreTarea = "LEER RADIO",
        .subNombreTarea = "RADIO",
        .funTarea = leerRadio,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(FREC_LEER_RADIO_HZ),
        .prioridadEstatica = PRIORIDAD_MEDIA,
    },
#endif
    [TAREA_ACTUALIZAR_RC] = {
        .nombreTarea = "ACTUALIZAR RC",
        .subNombreTarea = "RC",
        .funTarea = actualizarRC,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(FREC_ACTUALIZAR_RC_HZ),
        .prioridadEstatica = PRIORIDAD_MEDIA,
    },
    [TAREA_ACTUALIZAR_VEL_ANGULAR_FC] = {
        .nombreTarea = "ACTUALIZAR VEL ANGULAR FC",
        .subNombreTarea = "FC",
        .funTarea = actualizarLazoVelAngularFC,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(FREC_ACTUALIZAR_VEL_ANGULAR_FC_HZ),
        .prioridadEstatica = PRIORIDAD_TIEMPO_REAL,
    },
    [TAREA_ACTUALIZAR_ACTITUD_FC] = {
        .nombreTarea = "ACTUALIZAR ACTITUD FC",
        .subNombreTarea = "FC",
        .funTarea = actualizarLazoActitudFC,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(FREC_ACTUALIZAR_ACTITUD_FC_HZ),
        .prioridadEstatica = PRIORIDAD_TIEMPO_REAL,
    },
    [TAREA_ACTUALIZAR_POSICION_FC] = {
        .nombreTarea = "ACTUALIZAR POSICION FC",
        .subNombreTarea = "FC",
        .funTarea = actualizarLazoPosicionFC,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(FREC_ACTUALIZAR_POSICION_FC_HZ),
        .prioridadEstatica = PRIORIDAD_ALTA,
    },
    [TAREA_ACTUALIZAR_TELEMETRIA] = {
        .nombreTarea = "ACTUALIZAR TELEMETRIA",
        .subNombreTarea = "TELEMETRIA",
        .funTarea = actualizarTelemetria,
        .periodo = PERIODO_TAREA_HZ_SCHEDULER(200),
        .prioridadEstatica = PRIORIDAD_MEDIA_ALTA,
    },
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void iniciarTareas(void)
**  Descripcion:    Inicia el scheduler y habilita las tareas
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarTareas(void)
{
    iniciarScheduler();

    anadirTareaEnCola(&tareas[TAREA_SCHEDULER]);
    anadirTareaEnCola(&tareas[TAREA_STACK]);

#ifdef USAR_ADC_INTERNO
    anadirTareaEnCola(&tareas[TAREA_ADC_INTERNO]);
#endif

#ifdef USAR_POWER_MODULE
    anadirTareaEnCola(&tareas[TAREA_LEER_POWER_MODULE]);
    anadirTareaEnCola(&tareas[TAREA_ACTUALIZAR_POWER_MODULE]);
#endif

#ifdef USAR_IMU
    anadirTareaEnCola(&tareas[TAREA_ACTUALIZAR_IMU]);
  #ifdef LEER_IMU_SCHEDULER
    anadirTareaEnCola(&tareas[TAREA_LEER_IMU]);
  #endif
#endif

#ifdef USAR_BARO
    anadirTareaEnCola(&tareas[TAREA_ACTUALIZAR_BARO]);
  #ifdef LEER_BARO_SCHEDULER
    anadirTareaEnCola(&tareas[TAREA_LEER_BARO]);
  #endif
#endif

#ifdef USAR_MAG
    anadirTareaEnCola(&tareas[TAREA_ACTUALIZAR_MAG]);
  #ifdef LEER_MAG_SCHEDULER
    anadirTareaEnCola(&tareas[TAREA_LEER_MAG]);
  #endif
#endif

#ifdef USAR_GPS
  #ifdef LEER_GPS_SCHEDULER
    anadirTareaEnCola(&tareas[TAREA_LEER_GPS]);
  #endif
#endif

#ifdef USAR_RADIO
    anadirTareaEnCola(&tareas[TAREA_LEER_RADIO]);
#endif

    anadirTareaEnCola(&tareas[TAREA_ACTUALIZAR_RC]);

    anadirTareaEnCola(&tareas[TAREA_ACTUALIZAR_VEL_ANGULAR_FC]);
    anadirTareaEnCola(&tareas[TAREA_ACTUALIZAR_ACTITUD_FC]);
    anadirTareaEnCola(&tareas[TAREA_ACTUALIZAR_POSICION_FC]);

    anadirTareaEnCola(&tareas[TAREA_ACTUALIZAR_TELEMETRIA]);
}


