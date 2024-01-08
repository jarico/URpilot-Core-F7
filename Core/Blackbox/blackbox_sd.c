/***************************************************************************************
**  blackbox_sd.c - Funciones de comunicacion de la blackbox con la SD
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 16/05/2020
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
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include "blackbox_sd.h"

#ifdef USAR_BLACKBOX
#include "sd.h"
#include "asyncfatfs/asyncfatfs.h"
#include "Comun/matematicas.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define FICHERO_LOG_PREFIJO_BLACKBOX              "LOG"
#define FICHERO_LOG_SUFIJO_BLACKBOX               "URP"

#define NUM_MAX_BYTES_LIBRES_BLACKBOX             256


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    BLACKBOX_SD_INICIAL,
    BLACKBOX_SD_ESPERANDO,
    BLACKBOX_SD_ENUMERAR_FICHEROS,
    BLACKBOX_SD_CAMBIAR_DIRECTORIO_LOG,
    BLACKBOX_SD_LISTO_PARA_CREAR_LOG,
    BLACKBOX_SD_LISTO_PARA_LOG,
} estadoBlackboxSD;

typedef struct {
    afatfsFilePtr_t ficheroLog;
    afatfsFilePtr_t directorioLog;
    afatfsFinder_t buscadorDirectorioLog;
    int32_t numFicheroLog;
    estadoBlackboxSD estado;
} blackboxSD_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static blackboxSD_t blackboxSD;
int32_t bytesLibresCabBlackbox;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void directorioLogCreadoBlackbox(afatfsFilePtr_t directorio);
void ficheroLogCreadoBlackbox(afatfsFilePtr_t fichero);
void crearFicheroLogBlackbox(void);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool abrirBlackbox(void)
**  Descripcion:    Abre el driver blackbox
**  Parametros:     Ninguno
**  Retorno:        True si Ok
****************************************************************************************/
bool abrirBlackbox(void)
{
    if (afatfs_getFilesystemState() == AFATFS_FILESYSTEM_STATE_FATAL || afatfs_getFilesystemState() == AFATFS_FILESYSTEM_STATE_UNKNOWN || afatfs_isFull())
        return false;

    return true;
}


/***************************************************************************************
**  Nombre:         bool iniciarLogBlackbox(void)
**  Descripcion:    Inicia un log en la SD
**  Parametros:     Ninguno
**  Retorno:        True si Ok
****************************************************************************************/
bool iniciarLogBlackbox(void)
{
    fatDirectoryEntry_t *entradaDirectorio;

  nuevoInt:
    switch (blackboxSD.estado) {
        case BLACKBOX_SD_INICIAL:
            if (afatfs_getFilesystemState() == AFATFS_FILESYSTEM_STATE_READY) {
                blackboxSD.estado = BLACKBOX_SD_ESPERANDO;
                afatfs_mkdir("logs", directorioLogCreadoBlackbox);
            }
            break;

        case BLACKBOX_SD_ESPERANDO:
            break;

        case BLACKBOX_SD_ENUMERAR_FICHEROS:
            while (afatfs_findNext(blackboxSD.directorioLog, &blackboxSD.buscadorDirectorioLog, &entradaDirectorio) == AFATFS_OPERATION_SUCCESS) {
                if (entradaDirectorio && !fat_isDirectoryEntryTerminator(entradaDirectorio)) {
                    // Si es un fichero log, analizamos el numero del fichero
                    if (strncmp(entradaDirectorio->filename, FICHERO_LOG_PREFIJO_BLACKBOX, strlen(FICHERO_LOG_PREFIJO_BLACKBOX)) == 0
                        && strncmp(entradaDirectorio->filename + 8, FICHERO_LOG_SUFIJO_BLACKBOX, strlen(FICHERO_LOG_SUFIJO_BLACKBOX)) == 0) {
                        char numLog[6];

                        memcpy(numLog, entradaDirectorio->filename + 3, 5);
                        numLog[5] = '\0';

                        blackboxSD.numFicheroLog = MAX((int32_t)atoi(numLog), blackboxSD.numFicheroLog);
                    }
                }
                else {
                    // Se han analizado todos los ficheros de la SD, ahora se puede crear uno nuevo
                    afatfs_findLast(blackboxSD.directorioLog);

                    blackboxSD.estado = BLACKBOX_SD_CAMBIAR_DIRECTORIO_LOG;
                    goto nuevoInt;
                }
            }
            break;

        case BLACKBOX_SD_CAMBIAR_DIRECTORIO_LOG:
            if (afatfs_chdir(blackboxSD.directorioLog)) {
                // Ya no se necesita el handler abierto
                afatfs_fclose(blackboxSD.directorioLog, NULL);
                blackboxSD.directorioLog = NULL;

                blackboxSD.estado = BLACKBOX_SD_LISTO_PARA_CREAR_LOG;
                goto nuevoInt;
            }
            break;

        case BLACKBOX_SD_LISTO_PARA_CREAR_LOG:
        	crearFicheroLogBlackbox();
            break;

        case BLACKBOX_SD_LISTO_PARA_LOG:
            return true;
    }

    return false;
}


/***************************************************************************************
**  Nombre:         bool finalizarLogBlackbox(bool logEmpezado)
**  Descripcion:    Finaliza el log en la blackbox
**  Parametros:     Log empezado
**  Retorno:        True si OK
****************************************************************************************/
bool finalizarLogBlackbox(bool logEmpezado)
{
    if ((logEmpezado && afatfs_fclose(blackboxSD.ficheroLog, NULL)) || (!logEmpezado && afatfs_funlink(blackboxSD.ficheroLog, NULL))) {
        blackboxSD.ficheroLog = NULL;
        blackboxSD.estado = BLACKBOX_SD_LISTO_PARA_CREAR_LOG;
        return true;
    }

    return false;
}


/***************************************************************************************
**  Nombre:         void escribirBlackbox(uint8_t valor)
**  Descripcion:    Escribe un dato en la blackbox
**  Parametros:     Dato a escribir
**  Retorno:        Ninguno
****************************************************************************************/
void escribirBlackbox(uint8_t valor)
{
    afatfs_fputc(blackboxSD.ficheroLog, valor);
}


/***************************************************************************************
**  Nombre:         uint32_t escribirStringBlackbox(const char *s)
**  Descripcion:    Escribe un string en la blackbox
**  Parametros:     String a escribir
**  Retorno:        Numero de bytes escritos
****************************************************************************************/
uint32_t escribirStringBlackbox(const char *s)
{
    uint32_t longitud;

    longitud = strlen(s);
    afatfs_fwrite(blackboxSD.ficheroLog, (const uint8_t*) s, longitud);

    return longitud;
}


/***************************************************************************************
**  Nombre:         void printfBlackbox(const char *fmt, ...)
**  Descripcion:    Escribe un string con datos variables en la blackbox
**  Parametros:     Argumentos genericos
**  Retorno:        Numero de bytes escritos
****************************************************************************************/
uint32_t printfBlackbox(const char *fmt, ...)
{
    va_list va;
    char stringEscritura[64];

    va_start(va, fmt);

    sprintf(stringEscritura, fmt, va);
    uint32_t bytesEscritos = escribirStringBlackbox(stringEscritura);

    va_end(va);

    return bytesEscritos;
}


/***************************************************************************************
**  Nombre:         void escribirLineaCabeceraBlackbox(const char *nombre, const char *fmt, ...)
**  Descripcion:    Escribe una linea de informacion del sistema
**  Parametros:     String a escribir, argumentos genericos
**  Retorno:        Ninguno
****************************************************************************************/
void escribirLineaCabeceraBlackbox(const char *nombre, const char *fmt, ...)
{
    va_list va;

    escribirBlackbox('C');
    escribirBlackbox(' ');
    escribirStringBlackbox(nombre);
    escribirBlackbox(':');

    va_start(va, fmt);

    uint32_t bytesEscritos = printfBlackbox(fmt, va);

    va_end(va);

    escribirBlackbox('\n');
    bytesLibresCabBlackbox -= bytesEscritos + 3;
}


/***************************************************************************************
**  Nombre:         void calcularBytesLibresCabBlackbox(void)
**  Descripcion:    Calcula los bytes libres para escribir en la blackbox
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void calcularBytesLibresCabBlackbox(void)
{
    int32_t espacioLibre = afatfs_getFreeBufferSpace();

    bytesLibresCabBlackbox = MIN(MIN(espacioLibre, bytesLibresCabBlackbox + BYTES_LIBRES_CAB_POR_ITERACION_BLACKBOX), NUM_MAX_BYTES_LIBRES_BLACKBOX);
}


/***************************************************************************************
**  Nombre:         bool blackboxLlena(void)
**  Descripcion:    Comprueba si la blackbox esta llena
**  Parametros:     Ninguno
**  Retorno:        True si OK
****************************************************************************************/
bool blackboxLlena(void)
{
    return afatfs_isFull();
}


/***************************************************************************************
**  Nombre:         bool forzarFlushCompletoBlackbox(void)
**  Descripcion:    Hace unflush en la SD y retorna true cuando el sync es completo
**  Parametros:     Ninguno
**  Retorno:        True si OK
****************************************************************************************/
bool forzarFlushCompletoBlackbox(void)
{
    if (afatfs_sectorCacheInSync()) {
        return true;
	}
    else {
    	forzarFlushBlackbox();
        return false;
    }
}


/***************************************************************************************
**  Nombre:         bool forzarFlushBlackbox(void)
**  Descripcion:    Fuerza un flush en la blackbox
**  Parametros:     Ninguno
**  Retorno:        True si OK
****************************************************************************************/
bool forzarFlushBlackbox(void)
{
    return afatfs_flush();
}


/***************************************************************************************
**  Nombre:         bool trabajandoBlackbox(void)
**  Descripcion:    Comprueba si la blackbox esta trabajando
**  Parametros:     Ninguno
**  Retorno:        True si OK
****************************************************************************************/
bool trabajandoBlackbox(void)
{
    return tarjetaSDinsertada() && tarjetaSDfuncional() && (afatfs_getFilesystemState() == AFATFS_FILESYSTEM_STATE_READY);
}


/***************************************************************************************
**  Nombre:         int32_t blackboxNumeroLog(void)
**  Descripcion:    Devuelve el numero del fichero log
**  Parametros:     Ninguno
**  Retorno:        Numero de log
****************************************************************************************/
int32_t blackboxNumeroLog(void)
{
    return blackboxSD.numFicheroLog;
}


/***************************************************************************************
**  Nombre:         bool comprobarEspacioBlackbox(int32_t numBytes)
**  Descripcion:    Comprueba si hay bytes libres para la caabecera
**  Parametros:     Ninguno
**  Retorno:        True si OK
****************************************************************************************/
bool comprobarEspacioBlackbox(int32_t numBytes)
{
    if (numBytes <= bytesLibresCabBlackbox)
        return true;

    return false;
}


/***************************************************************************************
**  Nombre:         void directorioLogCreadoBlackbox(afatfsFilePtr_t directorio)
**  Descripcion:    Comprueba si el directorio se ha creado
**  Parametros:     Directorio a comprobar
**  Retorno:        Ninguno
****************************************************************************************/
void directorioLogCreadoBlackbox(afatfsFilePtr_t directorio)
{
    if (directorio) {
        blackboxSD.directorioLog = directorio;
        afatfs_findFirst(blackboxSD.directorioLog, &blackboxSD.buscadorDirectorioLog);

        blackboxSD.estado = BLACKBOX_SD_ENUMERAR_FICHEROS;
    }
    else
        blackboxSD.estado = BLACKBOX_SD_INICIAL;
}


/***************************************************************************************
**  Nombre:         void ficheroLogCreadoBlackbox(afatfsFilePtr_t fichero)
**  Descripcion:    Comprueba si el fichero se ha creado
**  Parametros:     Fichero a comprobar
**  Retorno:        Ninguno
****************************************************************************************/
void ficheroLogCreadoBlackbox(afatfsFilePtr_t fichero)
{
    if (fichero) {
        blackboxSD.ficheroLog = fichero;
        blackboxSD.numFicheroLog++;

        blackboxSD.estado = BLACKBOX_SD_LISTO_PARA_LOG;
    }
    else
        blackboxSD.estado = BLACKBOX_SD_LISTO_PARA_CREAR_LOG;
}


/***************************************************************************************
**  Nombre:         void crearFicheroLogBlackbox(void)
**  Descripcion:    Crea un fichero log
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void crearFicheroLogBlackbox(void)
{
    int32_t numRegistro = blackboxSD.numFicheroLog + 1;
    char nombreFichero[] = FICHERO_LOG_PREFIJO_BLACKBOX "00000." FICHERO_LOG_SUFIJO_BLACKBOX;

    for (uint8_t i = 7; i >= 3; i--) {
        nombreFichero[i] = (numRegistro % 10) + '0';
        numRegistro /= 10;
    }

    blackboxSD.estado = BLACKBOX_SD_ESPERANDO;
    afatfs_fopen(nombreFichero, "as", ficheroLogCreadoBlackbox);
}

#endif
