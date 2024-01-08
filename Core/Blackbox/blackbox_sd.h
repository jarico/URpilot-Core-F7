/***************************************************************************************
**  blackbox_sd.h - Funciones de comunicacion de la blackbox con la SD
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 14/05/2020
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

#ifndef __BLACKBOX_SD_H
#define __BLACKBOX_SD_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Sistema/plataforma.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
// Idealmente cada iteracion escribe una cantidad de datos similar. De esta manera no saturamos la CPU
#define BYTES_LIBRES_CAB_POR_ITERACION_BLACKBOX           64

#define ESCRIBIR_LINEA_CAB_BLACKBOX(nombre, formato, ...) case __COUNTER__: \
                                                              escribirLineaCabeceraBlackbox(nombre, formato, __VA_ARGS__); \
                                                              break;


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
extern int32_t bytesLibresCabBlackbox;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
// Funciones del driver
bool abrirBlackbox(void);
bool iniciarLogBlackbox(void);
bool finalizarLogBlackbox(bool logEmpezado);
void escribirBlackbox(uint8_t valor);
uint32_t escribirStringBlackbox(const char *s);
uint32_t printfBlackbox(const char *fmt, ...);
void escribirLineaCabeceraBlackbox(const char *nombre, const char *fmt, ...);
void calcularBytesLibresCabBlackbox(void);
bool blackboxLlena(void);
bool forzarFlushCompletoBlackbox(void);
bool forzarFlushBlackbox(void);
bool trabajandoBlackbox(void);
int32_t numeroLogBlackbox(void);
bool comprobarEspacioBlackbox(int32_t numBytes);


#endif // __BLACKBOX_SD_H
