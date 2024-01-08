/***************************************************************************************
**  ppm.h - Driver de la comunicacion PPM de la emisora RC
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 10/07/2019
**  Fecha de modificacion: 20/09/2020
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
#include "radio.h"

#ifdef USAR_RADIO_PPM
#include "GP/gp_radio.h"
#include "Drivers/timer.h"
#include "Motores/motor.h"
#include "Drivers/io.h"
#include "Comun/util.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NUM_CAPTURAS_PPM                      12
#define SINCRONISMO_US_PPM                    2700
#define PULSO_MIN_CANAL_PPM                   750
#define PULSO_MAX_CANAL_PPM                   2250
#define MIN_CANALES_PPM                       4
#define NUM_CANALES_PPM                       NUM_CAPTURAS_PPM
#define NUM_FRAMES_ESTABLES_REQUERIDOS_PPM    25
#define TIMER_PERIODO_PPM                     0x10000
#define TIMER_1MHZ                            MHZ_A_HZ(1)


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    uint32_t contadorLargo;
    uint32_t capturaActual;
    uint32_t tiempoActual;
    uint32_t incrementoTiempo;
    uint8_t indicePulso;
    int8_t numCanales;
    int8_t numCanalesPrevFrame;
    uint8_t contadorFramesEstables;
    bool tracking;
    bool overflowed;
    uint16_t capturas[NUM_CAPTURAS_PPM];
    timerCCHandlerRec_t edgeCb;
    timerOvrHandlerRec_t overflowCb;
} ppm_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static ppm_t ppm;
static uint16_t capturasPPM[NUM_CAPTURAS_PPM];


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarPPM(void);
void overflowCallbackPPM(timerOvrHandlerRec_t* cbRec, uint16_t captura);
void edgeCallbackPPM(timerCCHandlerRec_t* cbRec, uint16_t captura);
uint16_t leerCanalPPM(uint8_t canal);
void leerPPM(uint32_t tiempoActual);
void resetearPPM(void);
bool datosPPMsiendoRecibidos(void);
void resetearEstadoDatosRecibidosPPM(void);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarPPM(void)
**  Descripcion:    Inicia el PPM
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarPPM(void)
{
    resetearPPM();

    // Iniciamos el timer
    if (!iniciarTimer(configRadio()->pinPPM.numTimer, configRadio()->pinPPM.pin, TIM_USO_PPM))
        return false;

    // Obtenemos el Timer para hacer las lecturas PPM
    bool encontrado;
    tim_t *dTim = timerPorTag(configRadio()->pinPPM.pin, &encontrado);

    if (!encontrado || dTim == NULL)
        return false;

    // Configuramos el GPIO
    configurarIO(dTim->pin.pin, CONFIG_IO(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL), dTim->pin.af);

    // Configuramos el Timer
    configurarCapturaEntradaTimer(dTim, (uint16_t)TIMER_PERIODO_PPM, TIMER_1MHZ, true);

    // Asignamos las funciones
    asignarCallbackCCtimer(&ppm.edgeCb, edgeCallbackPPM);
    asignarCallbackOvrTimer(&ppm.overflowCb, overflowCallbackPPM);
    configurarCallbacksTimer(dTim, &ppm.edgeCb, &ppm.overflowCb);
    return true;
}


/***************************************************************************************
**  Nombre:         void overflowCallbackPPM(timerOvrHandlerRec_t* cbRec, uint16_t captura)
**  Descripcion:    Funcion callback overflow del Timer que gestiona el PPM
**  Parametros:     Puntero al handler Overflow, captura
**  Retorno:        Ninguno
****************************************************************************************/
void overflowCallbackPPM(timerOvrHandlerRec_t* cbRec, uint16_t captura)
{
    UNUSED(cbRec);

    ppm.contadorLargo += captura + 1;
    if (captura == TIMER_PERIODO_PPM - 1)
        ppm.overflowed = true;
}


/***************************************************************************************
**  Nombre:         void edgeCallbackPPM(timerCCHandlerRec_t* cbRec, uint16_t captura)
**  Descripcion:    Funcion callback Edge del Timer que gestiona el PPM
**  Parametros:     Puntero al handler Capture Compare, captura
**  Retorno:        Ninguno
****************************************************************************************/
void edgeCallbackPPM(timerCCHandlerRec_t* cbRec, uint16_t captura)
{
    UNUSED(cbRec);
    int32_t i;
    uint32_t tiempoAnterior = ppm.tiempoActual;
    uint32_t capturaPrevia = ppm.capturaActual;

    uint32_t tiempoActual = captura;
    tiempoActual += ppm.contadorLargo;

    if (captura < capturaPrevia) {
        if (ppm.overflowed)
            tiempoActual += TIMER_PERIODO_PPM;
    }

    if (tiempoActual > tiempoAnterior)
        ppm.incrementoTiempo = tiempoActual - (tiempoAnterior + (ppm.overflowed ? TIMER_PERIODO_PPM : 0));
    else
        ppm.incrementoTiempo = TIMER_PERIODO_PPM + tiempoActual - tiempoAnterior;

    ppm.overflowed = false;

    // Guardamos las medidas
    ppm.tiempoActual = tiempoActual;
    ppm.capturaActual = captura;

    // Pulso de sincronismo detectado
    if (ppm.incrementoTiempo > SINCRONISMO_US_PPM) {
        if (ppm.indicePulso == ppm.numCanalesPrevFrame && ppm.indicePulso >= MIN_CANALES_PPM && ppm.indicePulso <= NUM_CANALES_PPM) {
            // Si vemos n tramas simultaneas del mismo numero de canales lo guardamos como el tamanio de trama
            if (ppm.contadorFramesEstables < NUM_FRAMES_ESTABLES_REQUERIDOS_PPM)
                ppm.contadorFramesEstables++;
            else
                ppm.numCanales = ppm.indicePulso;
        }
        else
            ppm.contadorFramesEstables = 0;

        // Comprobamos si la ultima trama estaba bien formateada
        if (ppm.indicePulso == ppm.numCanales && ppm.tracking) {
            // La ultima trama estaba bien formateada
            for (i = 0; i < ppm.numCanales; i++)
                capturasPPM[i] = ppm.capturas[i];

            for (i = ppm.numCanales; i < NUM_CANALES_PPM; i++)
                capturasPPM[i] = 0;

            anadirRecepcionRadio(ppm.numCanales, capturasPPM);
        }

        ppm.tracking = true;
        ppm.numCanalesPrevFrame = ppm.indicePulso;
        ppm.indicePulso = 0;
    }
    else if (ppm.tracking) {
        //Duracion de pulso valida 0.75 a 2.5 ms
        if (ppm.incrementoTiempo > PULSO_MIN_CANAL_PPM && ppm.incrementoTiempo < PULSO_MAX_CANAL_PPM && ppm.indicePulso < NUM_CANALES_PPM) {
            ppm.capturas[ppm.indicePulso] = ppm.incrementoTiempo;
            ppm.indicePulso++;
        }
        else {
            // Duracion de pulso no valida
            ppm.tracking = false;
            for (i = 0; i < NUM_CAPTURAS_PPM; i++)
                ppm.capturas[i] = 0;
        }
    }
}


/***************************************************************************************
**  Nombre:         uint16_t leerCanalPPM(uint8_t canal)
**  Descripcion:    Lee el valor de un canal
**  Parametros:     Canal a leer
**  Retorno:        Valor del canal
****************************************************************************************/
uint16_t leerCanalPPM(uint8_t canal)
{
    return capturasPPM[canal];
}


/***************************************************************************************
**  Nombre:         void leerPPM(uint32_t tiempoActual)
**  Descripcion:    Lee el valor de todos los canales
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
void leerPPM(uint32_t tiempoActual)
{
    UNUSED(tiempoActual);
}


/***************************************************************************************
**  Nombre:         void resetearPPM(void)
**  Descripcion:    Resetea la estructura PPM
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void resetearPPM(void)
{
    ppm.indicePulso = 0;
    ppm.capturaActual = 0;
    ppm.tiempoActual = 0;
    ppm.incrementoTiempo = 0;
    ppm.contadorLargo = 0;
    ppm.numCanales = -1;
    ppm.numCanalesPrevFrame = -1;
    ppm.contadorFramesEstables = 0;
    ppm.tracking = false;
    ppm.overflowed = false;
}


/***************************************************************************************
**  Nombre:         tablaFnRadio_t tablaFnRadioPPM
**  Descripcion:    Tabla de funciones del protocolo PPM
****************************************************************************************/
tablaFnRadio_t tablaFnRadioPPM = {
    iniciarPPM,
    leerPPM,
};

#endif
