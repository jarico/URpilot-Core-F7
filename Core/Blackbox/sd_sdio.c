/***************************************************************************************
**  sd_sdio.c - Funciones de gestion de la tarjeta SD en modo SDIO
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 02/05/2020
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

#include "sd.h"

#ifdef USAR_SD_SDIO
#include "Drivers/sdmmc.h"
#include "GP/gp_sd.h"
#include "Drivers/tiempo.h"
#include "Comun/util.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
// Esto se usa para acelerar la escritura en la SD. Asyncfatfs tiene soporte limitado para escritura multibloque
#define TAM_BLOQUE_CACHE_FATFS_SD_SDIO      16


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
uint8_t escrituraCacheSDsdio[512 * TAM_BLOQUE_CACHE_FATFS_SD_SDIO] __attribute__ ((aligned (4)));
uint32_t contadorCacheSDsdio = 0;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
// Funciones de la tabla generica
bool iniciarSDsdio(void);
bool sondearSDsdio(void);
bool leerBloqueSDsdio(uint32_t indiceBloque, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback);
estadoOperacionSD_e iniciarEscrituraBloquesSDsdio(uint32_t indiceBloque, uint32_t numBloques);
estadoOperacionSD_e escribirBloqueSDsdio(uint32_t indiceBloque, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback);

// Funciones auxiliares
bool inicializacionCompletaSDsdio(void);
void resetearSDsdio(void);
bool tarjetaSDsdioReady(void);
bool leerRegistroCSDsdSDIO(void);
bool leerRegistroCIDsdSDIO(void);
estadoOperacionSD_e finalizarEscrituraBloquesSDsdio(void);
estadoRecepcionBloqueSD_e recibirBloqueDatosSDsdio(uint8_t *buffer, uint16_t tam);
void resetearCacheSDsdio(void);
void escribirCacheSDsdio(uint8_t *buffer);
uint16_t obtenerContadorCacheSDsdio(void);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarSDsdio(void)
**  Descripcion:    Inicia la tarjeta SD en modo SDIO
**  Parametros:     Ninguno
**  Retorno:        True si OK
****************************************************************************************/
bool iniciarSDsdio(void)
{
    sd_t *driver = punteroSD();

    driver->habilitada = configSD()->modo == SD_MODO_SDIO;
    if (!driver->habilitada) {
    	driver->estado = SD_ESTADO_NO_PRESENTE;
        return false;
    }

    driver->usarCache = configSD()->usarCache;

    iniciarDriverSDMMC();

    if (tarjetaSDinsertada()) {
        if (!iniciarTarjetaSDMMC()) {
        	driver->estado = SD_ESTADO_NO_PRESENTE;
        	driver->contadorFallos++;
            return false;
        }
    }
    else {
    	driver->estado = SD_ESTADO_NO_PRESENTE;
    	driver->contadorFallos++;
        return false;
    }

    driver->tiempoOpIniciada = millis();
    driver->estado = SD_ESTADO_RESET;
    driver->contadorFallos = 0;
    return true;
}


/***************************************************************************************
**  Nombre:         bool sondearSDsdio(void)
**  Descripcion:    Esta funcion se debe llamar periodicamente para que la tarjeta SD realice
**                  transferencias en curso
**  Parametros:     Ninguno
**  Retorno:        Devuelve True si la tarjeta esta lista para aceptar comandos
****************************************************************************************/
bool sondearSDsdio(void)
{
    sd_t *driver = punteroSD();

    if (!driver->habilitada) {
    	driver->estado = SD_ESTADO_NO_PRESENTE;
        return false;
    }

  nuevoInt:
    switch (driver->estado) {
        case SD_ESTADO_RESET:
            // El HAL se encarga del voltaje
        	driver->estado = SD_ESTADO_INICIALIZACION_EN_PROGRESO;
            goto nuevoInt;
            break;

        case SD_ESTADO_INICIALIZACION_EN_PROGRESO:
            if (inicializacionCompletaSDsdio()) {
                // Leemos los registros CSD y CID
                if (leerRegistroCSDsdSDIO()) {
                	driver->estado = SD_ESTADO_INICIALIZACION_RECEPCION_CID;
                    goto nuevoInt;
                }
                else {
                	resetearSDsdio();
                    goto nuevoInt;
                }
            }
            break;

        case SD_ESTADO_INICIALIZACION_RECEPCION_CID:
            if (leerRegistroCIDsdSDIO()) {
            	driver->bloquesRestantesMultiEscritura = 0;
            	driver->estado = SD_ESTADO_READY;
                goto nuevoInt;
            }
            break;

        case SD_ESTADO_ENVIANDO_ESCRITURA:
            // ¿Hemos terminado de enviar la escritura?
            if (chequearEscrituraSDMMC() == SD_OK) {
                // La SD ahora esta ocupada comprometiendo la escritura en la tarjeta
            	driver->estado = SD_ESTADO_ESPERANDO_PARA_ESCRIBIR;
            	driver->tiempoOpIniciada = millis();

                // Como se ha transmitido el buffer, ya se puede avisar de que la operacion ha sido un exito
                if (driver->operacionPendiente.callback)
                	driver->operacionPendiente.callback(SD_OPERACION_BLOQUE_ESCRIBIR, driver->operacionPendiente.indiceBloque, driver->operacionPendiente.buffer, driver->operacionPendiente.datoCallback);
            }
            break;

        case SD_ESTADO_ESPERANDO_PARA_ESCRIBIR:
            if (estadoSDMMC()) {
            	driver->contadorFallos = 0; // Se entiende que la SD esta OK si puede completar una escritura

                // Aun quedan mas bloques para escribir en una cadena de bloques multiples?
                if (driver->bloquesRestantesMultiEscritura > 1) {
                	driver->bloquesRestantesMultiEscritura--;
                	driver->siguienteBloqueMultiEscritura++;
                    if (driver->usarCache)
                    	resetearCacheSDsdio();

                    driver->estado = SD_ESTADO_ESCRIBIENDO_MULTIPLES_BLOQUES;
                }
                else if (driver->bloquesRestantesMultiEscritura == 1)
                	finalizarEscrituraBloquesSDsdio();     // Esta función cambia el estado de la tarjeta SD, ya sea inmediatamente exitoso o retrasado
                else
                	driver->estado = SD_ESTADO_READY;
            }
            else if (millis() > driver->tiempoOpIniciada + SD_TIMEOUT_ESCRITURA_MS) {
            	resetearSDsdio();
                goto nuevoInt;
            }
            break;

        case SD_ESTADO_LEYENDO:
            switch (recibirBloqueDatosSDsdio(driver->operacionPendiente.buffer, TAMANIO_BLOQUE_SD)) {
                case SD_RECEPCION_EXITOSA:
                	driver->estado = SD_ESTADO_READY;
                	driver->contadorFallos = 0;                   // Asumimos que la SD esta ok si puede completar una lectura

                    if (driver->operacionPendiente.callback) {
                    	driver->operacionPendiente.callback(SD_OPERACION_BLOQUE_LEER, driver->operacionPendiente.indiceBloque,
                    			driver->operacionPendiente.buffer, driver->operacionPendiente.datoCallback);
                    }
                    break;

                case SD_RECEPCION_BLOQUE_EN_PROGRESO:
                    if (millis() <= driver->tiempoOpIniciada + SD_TIMEOUT_LECTURA_MS)
                        break;
                    FALLTHROUGH;

                case SD_RECEPCION_ERROR:
                	resetearSDsdio();
                    goto nuevoInt;
                    break;
            }
            break;

        case SD_ESTADO_PARANDO_ESCRITURA_MULTIPLES_BLOQUES:
            if (estadoSDMMC())
            	driver->estado = SD_ESTADO_READY;
            else if (millis() > driver->tiempoOpIniciada + SD_TIMEOUT_ESCRITURA_MS) {
            	resetearSDsdio();
                goto nuevoInt;
            }
            break;

        case SD_ESTADO_NO_PRESENTE:
        default:
            break;
    }

    // Comprobamos cuanto tarda en inicializarse
    if (driver->estado >= SD_ESTADO_RESET && driver->estado < SD_ESTADO_READY && millis() - driver->tiempoOpIniciada > TIMEOUT_INICIALIZACION_MS_SD)
    	resetearSDsdio();

    return tarjetaSDsdioReady();
}


/***************************************************************************************
**  Nombre:         bool leerBloqueSDsdio(uint32_t indiceBloque, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback)
**  Descripcion:    Lee el bloque de 512 bytes con el indice dado
**
**                  Cuando se complete la lectura, se llamara al callback.
**                  Si la lectura es exitosa, el puntero del buffer sera el mismo buffer que ingreso
**                  originalmente, de ​​lo contrario, el buffer se establecera en NULL
**
**                  IMPORTANTE ¡Se debe mantener el puntero al buffer valido hasta que se complete la operacion!
**  Parametros:     Indice del bloque, buffer de recepcion, callback, dato callback
**  Retorno:            True  - La operación se puso en cola con exito para su posterior finalizacion, su callback será llamada despues
**                      False - No se ha podido iniciar la operación debido a que la tarjeta esta ocupada
****************************************************************************************/
bool leerBloqueSDsdio(uint32_t indiceBloque, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback)
{
    sd_t *driver = punteroSD();

    if (driver->estado != SD_ESTADO_READY) {
        if (driver->estado == SD_ESTADO_ESCRIBIENDO_MULTIPLES_BLOQUES) {
            if (finalizarEscrituraBloquesSDsdio() != SD_OPERACION_EXITO)
                return false;
        }
        else
            return false;
    }

    // Las tarjetas de tamanio estandar usan direccionamiento de bytes, las tarjetas de alta capacidad usan direccionamiento de bloque
    uint8_t estado = leerBloquesSDMMC(indiceBloque, (uint32_t*) buffer, 512, 1);

    if (estado == SD_OK) {
    	driver->operacionPendiente.buffer = buffer;
    	driver->operacionPendiente.indiceBloque = indiceBloque;
    	driver->operacionPendiente.callback = callback;
    	driver->operacionPendiente.datoCallback = datoCallback;
    	driver->estado = SD_ESTADO_LEYENDO;
    	driver->tiempoOpIniciada = millis();
        return true;
    }
    else {
    	resetearSDsdio();
        if (driver->operacionPendiente.callback) {
        	driver->operacionPendiente.callback(SD_OPERACION_BLOQUE_LEER, driver->operacionPendiente.indiceBloque,
            NULL, driver->operacionPendiente.datoCallback);
        }
        return false;
    }
}


/***************************************************************************************
**  Nombre:         estadoOperacionSD_e iniciarEscrituraBloquesSDsdio(uint32_t indiceBloque, uint32_t numBloques)
**  Descripcion:    Comienza a escribir una serie de bloques consecutivos que comienzan en el indice de bloque dado
**                  Esto permitira (pero no requerira) la tarjeta SD para borrar previamente el numero de bloques que
**                  especifique, lo que puede permitir que las escrituras se completen mas rapido
**
**                  Luego, simplemente se llama a escribirBloqueSDsdio() como de costumbre para escribir esos bloques consecutivamente
**
**                  Esta bien abortar la escritura de bloques multiples en cualquier momento escribiendo a una dirección no consecutiva o realizando una lectura.
**  Parametros:     Indice de bloque, numero de bloques a escribir
**  Retorno:            SD_OPERACION_EXITO   - La escritura de bloques multiples se ha puesto en cola
**                      SD_OPERACION_OCUPADO - La tarjeta ya esta ocupada y no puede aceptar su escritura
**                      SD_OPERACION_FALLO   - Se ha producido un error fatal, la tarjeta se reiniciara
****************************************************************************************/
estadoOperacionSD_e iniciarEscrituraBloquesSDsdio(uint32_t indiceBloque, uint32_t numBloques)
{
    sd_t *driver = punteroSD();

    if (driver->estado != SD_ESTADO_READY) {
        if (driver->estado == SD_ESTADO_ESCRIBIENDO_MULTIPLES_BLOQUES) {
            if (indiceBloque == driver->siguienteBloqueMultiEscritura)
                return SD_OPERACION_EXITO;         // Se asume que se desea continuar con la escritura de bloques multiples que ya esta en progreso
            else if (finalizarEscrituraBloquesSDsdio() != SD_OPERACION_EXITO)
                return SD_OPERACION_OCUPADO;
        }
        else
            return SD_OPERACION_OCUPADO;
    }

    driver->estado = SD_ESTADO_ESCRIBIENDO_MULTIPLES_BLOQUES;
    driver->bloquesRestantesMultiEscritura = numBloques;
    driver->siguienteBloqueMultiEscritura = indiceBloque;
    return SD_OPERACION_EXITO;
}


/***************************************************************************************
**  Nombre:         estadoOperacionSD_e escribirBloqueSDsdio(uint32_t indiceBloque, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback)
**  Descripcion:    Escribe el bloque de 512 bytes del buffer dado en el bloque con el indice dado. Si la escritura no se completa de inmediato, su devolucion
**                  de llamada se llamara mas tarde. Si la escritura es exitosa, el puntero del buffer sera el mismo buffer que ingreso originalmente;
**                  de ​​lo contrario, el buffer se establecera en NULL
**  Parametros:     Indice de bloque, buffer, callback dato del callback
**  Retorno:            SD_OPERACION_EN_PROGRESO - Su buffer se esta transmitiendo actualmente a la tarjeta y su devolucion de llamada sera
**                                                 llamado mas tarde para informar la finalizacion. El puntero del buffer debe permanecer valido hasta ese momento
**                      SD_OPERACION_EXITO       - El buffer ha sido transmitido
**                      SD_OPERACION_OCUPADO     - La tarjeta esta ocupada y no puede aceptar su escritura
**                      SD_OPERACION_FALLO       - Su escritura fue rechazada por la tarjeta, la tarjeta se restablecera
****************************************************************************************/
estadoOperacionSD_e escribirBloqueSDsdio(uint32_t indiceBloque, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback)
{
    sd_t *driver = punteroSD();

  nuevoInt:
    switch (driver->estado) {
        case SD_ESTADO_ESCRIBIENDO_MULTIPLES_BLOQUES:
            // ¿Necesitamos cancelar la escritura multibloque anterior?
            if (indiceBloque != driver->siguienteBloqueMultiEscritura) {
                if (finalizarEscrituraBloquesSDsdio() == SD_OPERACION_EXITO)
                    goto nuevoInt;           // Ahora que hemos entrado en el estado listo, podemos intentarlo de nuevo
                else
                    return SD_OPERACION_OCUPADO;
            }
            break;

        case SD_ESTADO_READY:
            break;

        default:
            return SD_OPERACION_OCUPADO;
    }

    driver->operacionPendiente.buffer = buffer;
    driver->operacionPendiente.indiceBloque = indiceBloque;

    uint16_t contadorBloque = 1;
    if ((obtenerContadorCacheSDsdio() < TAM_BLOQUE_CACHE_FATFS_SD_SDIO) && (driver->bloquesRestantesMultiEscritura != 0) && driver->usarCache) {
    	escribirCacheSDsdio(buffer);
        if (obtenerContadorCacheSDsdio() == TAM_BLOQUE_CACHE_FATFS_SD_SDIO || driver->bloquesRestantesMultiEscritura == 1) {
            // Reasignamos el buffer
            buffer = (uint8_t*)escrituraCacheSDsdio;
            // Recalculamos el indice de bloque
            indiceBloque -= obtenerContadorCacheSDsdio() - 1;
            contadorBloque = obtenerContadorCacheSDsdio();
        }
        else {
        	driver->bloquesRestantesMultiEscritura--;
        	driver->siguienteBloqueMultiEscritura++;
        	driver->estado = SD_ESTADO_READY;
            return SD_OPERACION_EXITO;
        }
    }

    driver->operacionPendiente.callback = callback;
    driver->operacionPendiente.datoCallback = datoCallback;
    driver->operacionPendiente.indiceTrozo = 1;              // (para transferencias que no son DMA) ya hemos enviado el fragmento #0
    driver->estado = SD_ESTADO_ENVIANDO_ESCRITURA;

    if (escribirBloquesSDMMC(indiceBloque, (uint32_t*) buffer, 512, contadorBloque) != SD_OK) {
    	resetearSDsdio();

        // Se anuncia un fallo de escritura:
        if (driver->operacionPendiente.callback) {
        	driver->operacionPendiente.callback(SD_OPERACION_BLOQUE_ESCRIBIR, driver->operacionPendiente.indiceBloque,
            NULL, driver->operacionPendiente.datoCallback);
        }

        return SD_OPERACION_FALLO;
    }

    return SD_OPERACION_EN_PROGRESO;
}


/***************************************************************************************
**  Nombre:         bool inicializacionCompletaSDsdio(void)
**  Descripcion:    Comprueba si la tarjeta SD ha completado su secuencia de inicio
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool inicializacionCompletaSDsdio(void)
{
    sd_t *driver = punteroSD();

    if (estadoSDMMC()) {
        if (leerInfoSDMMC() != SD_OK)
            return false;

        driver->version = (tipoSDMMC()) ? 2 : 1;
        driver->altaCapacidad = (tipoSDMMC() == 2) ? 1 : 0;
        return true;
    }

    // Cuando la inicializacion se completa, el bit inactivo en la respuesta se vuelve cero
    return false;
}


/***************************************************************************************
**  Nombre:         void resetearSDsdio(void)
**  Descripcion:    Maneja un fallo de una operacion de tarjeta SD restableciendo la tarjeta a su
**                  fase de inicialización.
**
**                  Incrementa el contador de fallos, y cuando se alcanza el umbral de fallos, deshabilita
**                  la tarjeta hasta la proxima llamada a iniciarSDsdio().
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void resetearSDsdio(void)
{
    sd_t *driver = punteroSD();

    if (iniciarTarjetaSDMMC() != 0) {
    	driver->contadorFallos++;
        if (driver->contadorFallos >= NUM_MAX_FALLOS_CONSECUTIVOS_SD || !tarjetaSDinsertada())
        	driver->estado = SD_ESTADO_NO_PRESENTE;
        else {
        	driver->tiempoOpIniciada = millis();
            driver->estado = SD_ESTADO_RESET;
        }
    }
}


/***************************************************************************************
**  Nombre:         void tarjetaSDsdioReady(void)
**  Descripcion:    Comprueba si la SD esta lista para aceptar comandos de lectura o escritura
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool tarjetaSDsdioReady(void)
{
    sd_t *driver = punteroSD();
    return driver->estado == SD_ESTADO_READY || driver->estado == SD_ESTADO_ESCRIBIENDO_MULTIPLES_BLOQUES;
}


/***************************************************************************************
**  Nombre:         bool leerRegistroCSDsdSDIO(void)
**  Descripcion:    Extrae el CSD
**  Parametros:     Ninguno
**  Retorno:        True si OK
****************************************************************************************/
bool leerRegistroCSDsdSDIO(void)
{
    sd_t *driver = punteroSD();

    // El bloque de datos del comando CSD siempre debe llegar dentro de 8 ciclos de reloj inactivo
    // porque la informacion sobre la latencia de la tarjeta se almacena en el propio registro CSD, por lo que aun no se puede usar
    if (leerInfoSDMMC())
        return false;

    infoSD_t info = infoSDMMC();
    driver->metadatos.numBloques = info.capacidad;
    return true;
}


/***************************************************************************************
**  Nombre:         bool leerRegistroCIDsdSDIO(void)
**  Descripcion:    Extrae el CID
**  Parametros:     Ninguno
**  Retorno:        True si OK
****************************************************************************************/
bool leerRegistroCIDsdSDIO(void)
{
    sd_t *driver = punteroSD();

    if (leerInfoSDMMC())
         return false;

    infoSD_t info = infoSDMMC();

    driver->metadatos.idFabricante = info.cid.idFabricante;
    driver->metadatos.idOEM = info.cid.idOEM;
    driver->metadatos.nombreProducto[0] = (info.cid.nombreProducto1 & 0xFF000000) >> 24;
    driver->metadatos.nombreProducto[1] = (info.cid.nombreProducto1 & 0x00FF0000) >> 16;
    driver->metadatos.nombreProducto[2] = (info.cid.nombreProducto1 & 0x0000FF00) >> 8;
    driver->metadatos.nombreProducto[3] = (info.cid.nombreProducto1 & 0x000000FF) >> 0;
    driver->metadatos.nombreProducto[4] = info.cid.nombreProducto2;
    driver->metadatos.revisionProductoMayor = info.cid.revisionProducto >> 4;
    driver->metadatos.revisionProductoMenor = info.cid.revisionProducto & 0x0F;
    driver->metadatos.numSerie = info.cid.numeroSerie;
    driver->metadatos.anoProduccion = (((info.cid.fechaProduccion & 0x0F00) >> 8) | ((info.cid.fechaProduccion & 0xFF) >> 4)) + 2000;
    driver->metadatos.mesProduccion = info.cid.fechaProduccion & 0x000F;

    return true;
}


/***************************************************************************************
**  Nombre:         estadoOperacionSD_e finalizarEscrituraBloquesSDsdio(void)
**  Descripcion:    Envia el token de detencion de transmision para completar una escritura de bloques multiples
**  Parametros:     Ninguno
**  Retorno:            OPERACION_SD_EN_PROGRESO  - Ahora estamos esperando que se complete esa parada la tarjeta
**                                                  se pondra en ESTADO_SD_PARANDO_ESCRITURA_MULTIPLES_BLOQUES
**                      OPERACION_SD_EXITO        - La escritura de bloques multiples finalizo inmediatamente, la tarjeta
**                                                  se pondra en ESTADO_SD_READY
****************************************************************************************/
estadoOperacionSD_e finalizarEscrituraBloquesSDsdio(void)
{
    sd_t *driver = punteroSD();

	driver->bloquesRestantesMultiEscritura = 0;
    if (driver->usarCache)
    	resetearCacheSDsdio();

    // La tarjeta puede optar por generar una señal de ocupado (no-0xFF) despues de un retraso maximo de N_BR (1 byte)
    if (estadoSDMMC()) {
    	driver->estado = SD_ESTADO_READY;
        return SD_OPERACION_EXITO;
    }
    else {
    	driver->estado = SD_ESTADO_PARANDO_ESCRITURA_MULTIPLES_BLOQUES;
    	driver->tiempoOpIniciada = millis();

        return SD_OPERACION_EN_PROGRESO;
    }
}


/***************************************************************************************
**  Nombre:         estadoRecepcionBloqueSD_e recibirBloqueDatosSDsdio(uint8_t *buffer, uint16_t num)
**  Descripcion:    Intenta recibir un bloque de datos de la tarjeta SD.
**  Parametros:     Puntero al buffer de recepcion, numero de datos
**  Retorno:        Estado de la recepcion
****************************************************************************************/
estadoRecepcionBloqueSD_e recibirBloqueDatosSDsdio(uint8_t *buffer, uint16_t tam)
{
    UNUSED(buffer);
    UNUSED(tam);

    if (chequearLecturaSDMMC() == SD_OCUPADA)
        return SD_RECEPCION_BLOQUE_EN_PROGRESO;

    if (!estadoSDMMC())
        return SD_RECEPCION_ERROR;

    return SD_RECEPCION_EXITOSA;
}


/***************************************************************************************
**  Nombre:         void resetearCacheSDsdio(void)
**  Descripcion:    Resetea el contador de la cache
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void resetearCacheSDsdio(void)
{
	contadorCacheSDsdio = 0;
}


/***************************************************************************************
**  Nombre:         void escribirCacheSDsdio(uint8_t *buffer)
**  Descripcion:    Escribe en la cache un buffer
**  Parametros:     Buffer a escribir
**  Retorno:        Ninguno
****************************************************************************************/
void escribirCacheSDsdio(uint8_t *buffer)
{
    if (contadorCacheSDsdio == sizeof(escrituraCacheSDsdio))
        return;

    memcpy(&escrituraCacheSDsdio[contadorCacheSDsdio], buffer, 512);
    contadorCacheSDsdio += 512;
}


/***************************************************************************************
**  Nombre:         uint16_t obtenerContadorCacheSDsdio(void)
**  Descripcion:    Devuelve el contador de la cache
**  Parametros:     Ninguno
**  Retorno:        Contador de la cache
****************************************************************************************/
uint16_t obtenerContadorCacheSDsdio(void)
{
    return (contadorCacheSDsdio / 512);
}


/***************************************************************************************
**  Nombre:         tablaFnSD_t tablaFnSDsdio
**  Descripcion:    Tabla de funciones de la SD en modo SDIO
****************************************************************************************/
tablaFnSD_t tablaFnSDsdio = {
    iniciarSDsdio,
    sondearSDsdio,
    leerBloqueSDsdio,
    iniciarEscrituraBloquesSDsdio,
    escribirBloqueSDsdio,
};


#endif
