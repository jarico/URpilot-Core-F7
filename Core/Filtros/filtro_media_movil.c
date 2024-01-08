/***************************************************************************************
**  filtro_media_movil.c - Funciones del filtro de media movil
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 23/05/2020
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
#include "filtro_media_movil.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void ajustarFiltroMediaMovil(filtroMediaMovil_t *filtro, uint8_t tamFiltro)
**  Descripcion:    Ajusta el filtro
**  Parametros:     Puntero al filtro, tamanio del filtro
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarFiltroMediaMovil(filtroMediaMovil_t *filtro, uint8_t tamFiltro)
{
    filtro->tamFiltro = tamFiltro;
    filtro->indiceMuestra = 0;
    filtro->numMuestras = 0;
    resetearFiltroMediaMovil(filtro);
}


/***************************************************************************************
**  Nombre:         void resetearFiltroMediaMovil(filtroMediaMovil_t *filtro)
**  Descripcion:    Resetea el filtro
**  Parametros:     Puntero al filtro
**  Retorno:        Ninguno
****************************************************************************************/
void resetearFiltroMediaMovil(filtroMediaMovil_t *filtro)
{
    for (uint8_t i = 0; i < TAM_MAX_FILTRO_MEDIA_MOVIL; i++)
        filtro->muestras[i] = 0;

    filtro->indiceMuestra = 0;
    filtro->numMuestras = 0;
}


/***************************************************************************************
**  Nombre:         float actualizarFiltroMediaMovil(filtroMediaMovil_t *filtro, float muestra)
**  Descripcion:    Actualiza el filtro
**  Parametros:     Puntero al filtro, muestra
**  Retorno:        Valor filtrado
****************************************************************************************/
float actualizarFiltroMediaMovil(filtroMediaMovil_t *filtro, float muestra)
{
    float sumValores = 0;

    // Anadimos una muestra al array
    filtro->muestras[filtro->indiceMuestra] = muestra;
    filtro->indiceMuestra++;
    if (filtro->indiceMuestra >= filtro->tamFiltro)
        filtro->indiceMuestra = 0;

    filtro->numMuestras++;
    if (filtro->numMuestras >= filtro->tamFiltro)
        filtro->numMuestras = filtro->tamFiltro;

    // Calculo del valor filtrado
    for (uint8_t i = 0; i < filtro->tamFiltro; i++)
        sumValores += filtro->muestras[i];

    return sumValores / filtro->numMuestras;
}
