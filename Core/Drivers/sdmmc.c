/***************************************************************************************
**  sdmmc.c - Funciones generales del driver SDMMC
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
#include <stdio.h>
#include <string.h>

#include "sdmmc.h"

#ifdef USAR_SDMMC
#include "GP/gp_sdmmc.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static sdmmc_t sdmmc;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         sdmmc_t *punteroSDMMC(void)
**  Descripcion:    Devuelve el puntero a un SDMMC seleccionado
**  Parametros:     Ninguno
**  Retorno:        Puntero
****************************************************************************************/
sdmmc_t *punteroSDMMC(void)
{
    return &sdmmc;
}


/***************************************************************************************
**  Nombre:         bool iniciarSDMMC(void)
**  Descripcion:    Inicia el SDMMC
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarSDMMC(void)
{
    if (configSDMMC()->numSDMMC == SDMMC_NINGUNO) {
#ifdef DEBUG
        printf("Fallo en la definicion del SDMMC\n");
#endif
        return false;
    }

    memset(&sdmmc, 0, sizeof(sdmmc));
    resetearContadorErrorSDMMC();
    sdmmc.iniciado = false;

    if (iniciarDriverSDMMC()) {
        sdmmc.iniciado = true;
        return true;
    }
    else {
#ifdef DEBUG
        printf("Fallo en la inicializacion del SDMMC\n");
#endif
        return false;
    }
}


/***************************************************************************************
**  Nombre:         bool sdmmcIniciado(void)
**  Descripcion:    Comprueba si el SDMMC esta iniciado
**  Parametros:     Ninguno
**  Retorno:        True si iniciada
****************************************************************************************/
bool sdmmcIniciado(void)
{
    return sdmmc.iniciado;
}


/***************************************************************************************
**  Nombre:         void errorCallbackSDMMC(void)
**  Descripcion:    Incrementa el contador de errores
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void errorCallbackSDMMC(void)
{
    sdmmc.numErrores++;
}


/***************************************************************************************
**  Nombre:         uint16_t contadorErrorSDMMC(void)
**  Descripcion:    Devuelve el contador de errores
**  Parametros:     Ninguno
**  Retorno:        Retorna el numero de errores
****************************************************************************************/
uint16_t contadorErrorSDMMC(void)
{
    return sdmmc.numErrores;
}


/***************************************************************************************
**  Nombre:         void resetearContadorErrorSDMMC(void)
**  Descripcion:    Resetea el contador de errores
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void resetearContadorErrorSDMMC(void)
{
    sdmmc.numErrores = 0;
}


/***************************************************************************************
**  Nombre:         tipoSD_e tipoSDMMC(void)
**  Descripcion:    Devuelve el tipo de SDMMC
**  Parametros:     Ninguno
**  Retorno:        Tipo
****************************************************************************************/
tipoSD_e tipoSDMMC(void)
{
    return sdmmc.tipo;
}

/***************************************************************************************
**  Nombre:         infoSD_t infoSDMMC(void)
**  Descripcion:    Devuelve la info del SDMMC
**  Parametros:     Ninguno
**  Retorno:        Info
****************************************************************************************/
infoSD_t infoSDMMC(void)
{
    return sdmmc.info;
}

#endif // USAR_SDMMC

