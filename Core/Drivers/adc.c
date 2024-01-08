/***************************************************************************************
**  adc.c - Funciones generales del ADC
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 11/05/2019
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
#include <stdio.h>

#include "adc.h"

#ifdef USAR_ADC
#include "io.h"
#include "Drivers/tiempo.h"
#include "Comun/util.h"

#ifdef USAR_ADC_INTERNO
#include "Filtros/filtro_media_movil.h"
#endif


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define TENSION_REFERENCIA_ADC         3.3

#ifdef USAR_ADC_INTERNO
#define TAMANIO_FILTRO_ADC_INTERNO     8
#endif


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static adc_t adc[NUM_MAX_ADC];
static puertoADC_t puertosADC[NUM_MAX_PUERTOS_ADC];
static bool adcIniciado = false;

#ifdef USAR_ADC_INTERNO
static filtroMediaMovil_t filtroTemp;
static filtroMediaMovil_t filtroVref;

static float tempCore;
static float tensionRef;
#endif


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
uint16_t leerValorCanalADC(numADC_e numADC, uint8_t canal);

#ifdef USAR_ADC_INTERNO
void iniciarAdcInterno(void);
#endif


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         adc_t *punteroADC(numADC_e numADC)
**  Descripcion:    Devuelve el puntero a un ADC seleccionado
**  Parametros:     Dispositivo a devolver
**  Retorno:        Puntero
****************************************************************************************/
adc_t *punteroADC(numADC_e numADC)
{
    return &adc[numADC];
}


/***************************************************************************************
**  Nombre:         bool iniciarADC(void)
**  Descripcion:    Configura e inicia el ADC
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarADC(void)
{
    // Reset puertos
	for (uint8_t i = 0; i < NUM_MAX_PUERTOS_ADC; i++) {
	    memset(&puertosADC[i], 0, sizeof(puertosADC[i]));
	}

    for (uint8_t i = 0; i < NUM_MAX_ADC; i++) {
        adc_t *driver = &adc[i];

        // Reset driver ADC
        memset(driver, 0, sizeof(adc_t));

        if (iniciarDriverADC(i))
            driver->iniciado = true;
        else
            driver->iniciado = false;
    }

    for (uint8_t i = 0; i < NUM_MAX_ADC; i++) {
        if (!adc[i].iniciado) {
#ifdef DEBUG
            printf("Fallo en la inicializacion del ADC %u\n", i + 1);
#endif
            adcIniciado = false;
        	return false;
        }
    }

    adcIniciado = true;

#ifdef USAR_ADC_INTERNO
    // Se inicia el ADC interno
    iniciarAdcInterno();
#endif

    return true;
}


/***************************************************************************************
**  Nombre:         bool iniciadoADC(void)
**  Descripcion:    Devuelve si los ADC estan iniciados
**  Parametros:     Ninguno
**  Retorno:        ADC iniciado
****************************************************************************************/
bool iniciadoADC(void)
{
	return adcIniciado;
}


/***************************************************************************************
**  Nombre:         puertoADC_t *puertoADC(uint8_t puerto)
**  Descripcion:    Devuelve el puerto ADC
**  Parametros:     Numero del puerto
**  Retorno:        Puerto
****************************************************************************************/
puertoADC_t *puertoADC(uint8_t puerto)
{
	return &puertosADC[puerto];
}


/***************************************************************************************
**  Nombre:         void asignarPuertoADC(uint8_t numPuerto, puertoADC_t puerto)
**  Descripcion:    Asigns el puerto ADC
**  Parametros:     Numero de puerto, datos a asignar
**  Retorno:        Ninguno
****************************************************************************************/
void asignarPuertoADC(uint8_t numPuerto, puertoADC_t puerto)
{
    puertosADC[numPuerto] = puerto;
}


/***************************************************************************************
**  Nombre:         uint16_t leerValorCanalADC(numADC_e numADC, uint8_t canal)
**  Descripcion:    Devuelve el valor analogico de un canal
**  Parametros:     Numero del driver, canal a leer
**  Retorno:        Valor del canal
****************************************************************************************/
uint16_t leerValorCanalADC(numADC_e numADC, uint8_t canal)
{
    return adc[numADC].valores[canal];
}


/***************************************************************************************
**  Nombre:         uint16_t leerCanalADC(uint8_t canal)
**  Descripcion:    Devuelve el valor analogico de un canal
**  Parametros:     Canal a leer
**  Retorno:        Valor del canal
****************************************************************************************/
uint16_t leerCanalADC(uint8_t canal)
{
    puertoADC_t *puerto = puertoADC(canal);

    if (puerto->pinAsignado)
	    return leerValorCanalADC(puerto->numADC, puerto->canalADC);
    else
        return 65535;
}


#ifdef USAR_ADC_INTERNO
/***************************************************************************************
**  Nombre:         void iniciarAdcInterno(void)
**  Descripcion:    Rellena el filtro de media movil
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarAdcInterno(void)
{
	ajustarFiltroMediaMovil(&filtroTemp, TAMANIO_FILTRO_ADC_INTERNO);
	ajustarFiltroMediaMovil(&filtroVref, TAMANIO_FILTRO_ADC_INTERNO);

    // Llamamos a la funcion "actualizarAdcInterno" para rellenar el filtro de media movil
    for (uint8_t i = 0 ; i < TAMANIO_FILTRO_ADC_INTERNO ; i++) {
        actualizarADCinterno(0);
        delay(20);
    }
}


/***************************************************************************************
**  Nombre:         void actualizarAdcInterno(uint32_t tiempoActual)
**  Descripcion:    Actualiza el filtro de media movil y manda una nueva conversion del ADC
**  Parametros:     Tiempo actual (no usado)
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarADCinterno(uint32_t tiempoActual)
{
    UNUSED(tiempoActual);

    uint16_t muestraVref;
	int16_t muestraTempCore;

    leerADCinterno(&muestraVref, &muestraTempCore);
    tensionRef = actualizarFiltroMediaMovil(&filtroVref, (float)muestraVref) / 1000;
    tempCore = actualizarFiltroMediaMovil(&filtroTemp, (float)muestraTempCore);
}


/***************************************************************************************
**  Nombre:         float temperaturaCore(void)
**  Descripcion:    Devuelve la temperatura del core
**  Parametros:     Ninguno
**  Retorno:        Temperatura del core
****************************************************************************************/
float temperaturaCore(void)
{
    return tempCore;
}
#endif


/***************************************************************************************
**  Nombre:         float tensionReferencia(void)
**  Descripcion:    Devuelve la tension de referencia del micro
**  Parametros:     Ninguno
**  Retorno:        Tension de referencia
****************************************************************************************/
float tensionReferencia(void)
{
#ifdef USAR_ADC_INTERNO
    return tensionRef;
#else
    return TENSION_REFERENCIA_ADC;
#endif
}


#endif
