/***************************************************************************************
**  inicializacion.c - Funcion de inicializacion de la placa
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 11/05/2019
**  Fecha de modificacion: 15/09/2020
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
#include <stdio.h>
#include <string.h>

#include "inicializacion.h"
#include "fallo_sistema.h"
#include "Drivers/tiempo.h"
#include "Drivers/rtc.h"
#include "led_estado.h"
#include "fallo_sistema.h"
#include "Scheduler/tareas.h"
#include "GP/config_flash.h"
#include "GP/gp_sistema.h"
#include "Version/version.h"
#include "Sensores/PM/power_module.h"
#include "Sensores/Barometro/barometro.h"
#include "Sensores/Magnetometro/magnetometro.h"
#include "Sensores/IMU/imu.h"
#include "Sensores/GPS/gps.h"
#include "Radio/radio.h"
#include "Motores/motor.h"
#include "FC/rc.h"
#include "FC/fc.h"
#include "AHRS/ahrs.h"
#include "FC/mixer.h"
#include "Drivers/usb.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
uint8_t estadoSistema = ESTADO_SIS_INICIALIZANDO;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void iniciarPlaca(void)
**  Descripcion:    Realiza todas las inicializaciones de la placa
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarPlaca(void)
{
#ifdef DEBUG
    printf("Bienvenido al modo debug de URpilot\n");
#endif

    iniciarLedEstado();


    // Configuracion -------------------------------------------------------------
    // Se inicia la configuracion de la flash
    iniciarConfigFlash();

    bool estado = cargarConfigFlash();
    if (!estado || !versionValidaConfigFlash() || strncasecmp(configSistema()->identificadorPlaca, NOMBRE_PLACA, sizeof(NOMBRE_PLACA)))
    	resetearConfigFlash();

    estadoSistema |= ESTADO_SIS_CONFIG_CARGADA;


    // Drivers ------------------------------------------------------------------
    // Overclock
#ifdef USAR_OVERCLOCK
    resetearSiOverclock(configSistema()->overclock);
#endif
    delay(100);

#ifdef DEBUG
    printf("System clock ajustado a: %lu Hz\n", SystemCoreClock);
#endif

    if (!iniciarVersion()) {
#ifdef DEBUG
    printf("Fallo al iniciar el calculo de la version HW\n");
#endif
    }

    // RTC
#ifdef USAR_RTC
    iniciarRTC();
#endif

    // USB
#ifdef USAR_USB
    iniciarUSB();
#endif

    estadoSistema |= ESTADO_SIS_DRIVERS_READY;


    // Perifericos --------------------------------------------------------------
    // Power Module
#ifdef USAR_POWER_MODULE
    if (!iniciarPowerModule())
         falloSistema(FALLO_INICIAR_POWER_MODULE);
#endif

    // Sensores
#ifdef USAR_IMU
    if (!iniciarIMU())
        falloSistema(FALLO_INICIAR_IMU);
#endif

#ifdef USAR_BARO
    if (!iniciarBaro())
        falloSistema(FALLO_INICIAR_BARO);
#endif

#ifdef USAR_MAG
    if (!iniciarMag())
        falloSistema(FALLO_INICIAR_MAG);
#endif

#ifdef USAR_GPS
    if (!iniciarGPS())
        falloSistema(FALLO_INICIAR_GPS);
#endif

    // Radiocontrol
#ifdef USAR_RADIO
    if (!iniciarRadio())
        falloSistema(FALLO_INICIAR_RADIO);
#endif

    // Motores
#ifdef USAR_MOTORES
    if (!iniciarMotores()) {
        falloSistema(FALLO_INICIAR_MOTORES);
    }
#endif

    estadoSistema |= ESTADO_SIS_PERIFERICOS_READY;


    // FC ----------------------------------------------------------------------
    iniciarRC();
    iniciarAHRS();
    iniciarFC();
    iniciarMixer();


    // Scheduler ---------------------------------------------------------------
    // Iniciar las tareas del scheduler
    iniciarTareas();
    estadoSistema |= ESTADO_SIS_SCHEDULER_READY;


    // Fin ini -----------------------------------------------------------------
    escribirLedEstado(VERDE);
    estadoSistema |= ESTADO_SIS_READY;

#ifdef DEBUG
    printf("Sistema arrancado\n");
#endif
}
