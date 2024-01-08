/***************************************************************************************
**  led_estado.c - Funciones para manejar los led de estado de la placa
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 06/12/2020
**  Fecha de modificacion: 06/12/2020
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
#include "led_estado.h"
#include "Drivers/io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NUM_LEDS_RGB             3

#ifndef PIN_LEDR
  #define PIN_LEDR               NINGUNO
#endif

#ifndef PIN_LEDG
  #define PIN_LEDG               NINGUNO
#endif

#ifndef PIN_LEDB
  #define PIN_LEDB               NINGUNO
#endif


#define LEDR_INVERTIR            invertirLed(0)
#define LEDR_OFF                 escribirLed(0, false)
#define LEDR_ON                  escribirLed(0, true)

#define LEDG_INVERTIR            invertirLed(1)
#define LEDG_OFF                 escribirLed(1, false)
#define LEDG_ON                  escribirLed(1, true)

#define LEDB_INVERTIR            invertirLed(2)
#define LEDB_OFF                 escribirLed(2, false)
#define LEDB_ON                  escribirLed(2, true)


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    uint8_t pin[NUM_LEDS_RGB];
    bool inversion;
} configLedEstado_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static configLedEstado_t configLedEstado;
static uint8_t ledEstado[NUM_LEDS_RGB];


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void configurarLedEstado(configLedEstado_t *configLedEstado);
void escribirLed(uint8_t numLed, bool estado);
void invertirLed(uint8_t numLed);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void configurarLedEstado(configLedEstado_t *configLedEstado)
**  Descripcion:    Asigna los pines a los leds de estado y comprueba si son invertidos
**  Parametros:     Configuracion de los leds de estado
**  Retorno:        Ninguno
****************************************************************************************/
void configurarLedEstado(configLedEstado_t *configLedEstado)
{
	configLedEstado->pin[0] = DEFIO_TAG(PIN_LEDR);
	configLedEstado->pin[1] = DEFIO_TAG(PIN_LEDG);
	configLedEstado->pin[2] = DEFIO_TAG(PIN_LEDB);

	configLedEstado->inversion = false
#ifdef LED_ESTADO_INVERTIDO
    | true
#endif
    ;
}


/***************************************************************************************
**  Nombre:         void iniciarLedEstado(void)
**  Descripcion:    Configura las GPIOs de los leds y los apaga
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarLedEstado(void)
{
    // Asigna los pines a los leds
	configurarLedEstado(&configLedEstado);

    for (uint8_t i = 0; i < NUM_LEDS_RGB; i++) {
        if (configLedEstado.pin[i]) {
        	ledEstado[i] = configLedEstado.pin[i];
            configurarIO(ledEstado[i], CONFIG_IO(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL), 0);
        }
        else
        	ledEstado[i] = DEFIO_TAG__NINGUNO;
    }

    LEDR_ON;
    LEDG_ON;
    LEDB_ON;
}


/***************************************************************************************
**  Nombre:         void escribirLedEstado(colorRGB_e color)
**  Descripcion:    Escribe un color en el led de estado
**  Parametros:     Color
**  Retorno:        Ninguno
****************************************************************************************/
void escribirLedEstado(colorRGB_e color)
{
    switch (color) {
        case NEGRO:
            LEDR_OFF;
            LEDG_OFF;
            LEDB_OFF;
        	break;

        case BLANCO:
            LEDR_ON;
            LEDG_ON;
            LEDB_ON;
        	break;

        case ROJO:
            LEDR_ON;
            LEDG_OFF;
            LEDB_OFF;
        	break;

        case AZUL:
            LEDR_OFF;
            LEDG_OFF;
            LEDB_ON;
        	break;

        case VERDE:
            LEDR_OFF;
            LEDG_ON;
            LEDB_OFF;
        	break;

        case AMARILLO:
            LEDR_ON;
            LEDG_ON;
            LEDB_OFF;
        	break;

        case MORADO:
            LEDR_ON;
            LEDG_OFF;
            LEDB_ON;
        	break;

        case CIAN:
            LEDR_OFF;
            LEDG_ON;
            LEDB_ON;
        	break;

        default:
            LEDR_OFF;
            LEDG_OFF;
            LEDB_OFF;
		    break;
    }
}


/***************************************************************************************
**  Nombre:         void escribirLed(uint8_t numLed, bool estado)
**  Descripcion:    Enciende o apaga un led
**  Parametros:     Led, encender o apagar
**  Retorno:        Ninguno
****************************************************************************************/
void escribirLed(uint8_t numLed, bool estado)
{
    if (configLedEstado.inversion)
        escribirIO(ledEstado[numLed], !estado);
    else
        escribirIO(ledEstado[numLed], estado);
}


/***************************************************************************************
**  Nombre:         void invertirLed(uint8_t numLed)
**  Descripcion:    Invierte el estado de un led
**  Parametros:     Led
**  Retorno:        Ninguno
****************************************************************************************/
void invertirLed(uint8_t numLed)
{
    invertirIO(ledEstado[numLed]);
}
