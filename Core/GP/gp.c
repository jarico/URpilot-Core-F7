/***************************************************************************************
**  gp.c - Funciones de gestion de los grupos de parametros
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 07/05/2019
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

#include "gp.h"
#include "Comun/matematicas.h"


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
uint8_t *offsetGP(const registroGP_t* reg);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         registroGP_t* encontrarGP(uint16_t gpn)
**  Descripcion:    Encuenta un registro
**  Parametros:     Numero del registro a encontrar
**  Retorno:        Registro encontrado
****************************************************************************************/
const registroGP_t* encontrarGP(uint16_t gpn)
{
    POR_CADA_GP(reg) {
        if (numeroGP(reg) == gpn)
            return reg;
    }
    return NULL;
}


/***************************************************************************************
**  Nombre:         bool cargarGP(const registroGP_t* reg, const void *origen, uint16_t tam, uint8_t version)
**  Descripcion:    Carga un registro
**  Parametros:     Registro a cargar, direccion de origen, tamanio, version
**  Retorno:        True si lo carga
****************************************************************************************/
bool cargarGP(const registroGP_t* reg, const void *origen, uint16_t tam, uint8_t version)
{
    resetearInstanciaGP(reg, offsetGP(reg));

    if (version == versionGP(reg)) {
        const uint16_t num = MIN(tam, tamanioGP(reg));
        memcpy(offsetGP(reg), origen, num);
        return true;
    }

    return false;
}


/***************************************************************************************
**  Nombre:         uint16_t guardarGP(const registroGP_t* reg, void *to, uint16_t tam)
**  Descripcion:    Guarda un registro
**  Parametros:     Registro a guardar, direccion de destino, tamanio
**  Retorno:        Numero de datos guardados
****************************************************************************************/
uint16_t guardarGP(const registroGP_t* reg, void *to, uint16_t tam)
{
    const uint16_t num = MIN(tam, tamanioGP(reg));
    memcpy(to, offsetGP(reg), num);
    return num;
}


/***************************************************************************************
**  Nombre:         void resetearTodosGP(void)
**  Descripcion:    Resetea todos los GP
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void resetearTodosGP(void)
{
    POR_CADA_GP(reg) {
        resetearGP(reg);
    }
}


/***************************************************************************************
**  Nombre:         void resetearGP(const registroGP_t* reg)
**  Descripcion:    Resetea el GP
**  Parametros:     Registro a resetear
**  Retorno:        Ninguno
****************************************************************************************/
void resetearGP(const registroGP_t* reg)
{
    resetearInstanciaGP(reg, offsetGP(reg));
}


/***************************************************************************************
**  Nombre:         void resetearInstanciaGP(const registroGP_t *reg, uint8_t *base)
**  Descripcion:    Resetea la instancia del GP
**  Parametros:     Registro a resetear, direccion base
**  Retorno:        Ninguno
****************************************************************************************/
void resetearInstanciaGP(const registroGP_t *reg, uint8_t *base)
{
    const uint16_t tamReg = tamanioGP(reg);

    memset(base, 0, tamReg);
    if (reg->reset.ptr >= (void*)_sresetGP && reg->reset.ptr < (void*)_eresetGP)
        memcpy(base, reg->reset.ptr, tamReg);
    else if (reg->reset.fn)
        reg->reset.fn(base, tamReg);
}


/***************************************************************************************
**  Nombre:         uint8_t *offsetGP(const registroGP_t* reg)
**  Descripcion:    Obtiene el offset del registro
**  Parametros:     Registro
**  Retorno:        Offset
****************************************************************************************/
uint8_t *offsetGP(const registroGP_t* reg)
{
    return reg->dir;
}

