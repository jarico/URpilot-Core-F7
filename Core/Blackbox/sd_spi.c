/***************************************************************************************
**  sd_spi.c - Funciones de gestion de la tarjeta SD en modo SPI
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 13/09/2019
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
#include "sd.h"

#ifdef USAR_SD_SPI
#include "Drivers/tiempo.h"
#include "Drivers/spi_bus.h"
#include "GP/gp_sd.h"
#include "Comun/util.h"
#include "Drivers/io.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define NUM_BYTES_BASURA_INICIALIZACION_SD_SPI            10
#define NUM_MAX_BYTES_RETARDO_RESPUESTA_CMD_SD_SPI        8

// Elegido para que CMD8 tenga el mismo CRC que CMD0:
#define PATRON_CHEQUEO_IF_COND_SD_SPI                     0xAB

// Divide los sectores de la tarjeta SD de 512 bytes en trozos de este tamanio cuando escriba sin DMA para reducir la sobrecarga maxima por llamada a sondearSD().
#define TAMANIO_TROZO_NO_DMA_SD_SPI                       256


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
	bus_t bus;
	registroCSDsd_t csd;
} sdSPI_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static sdSPI_t sdSPI;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/

// Funciones de la tabla generica
bool iniciarSDspi(void);
bool sondearSDspi(void);
bool leerBloqueSDspi(uint32_t indiceBloque, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback);
estadoOperacionSD_e iniciarEscrituraBloquesSDspi(uint32_t indiceBloque, uint32_t numBloques);
estadoOperacionSD_e escribirBloqueSDspi(uint32_t indiceBloque, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback);

// Funciones auxiliares
bool inicializacionCompletaSDspi(void);
void resetearSDspi(void);
bool tarjetaReadySDspi(void);
void seleccionarSDspi(void);
void deseleccionarSDspi(void);
bool validarVersionInterfaceSDspi(void);
bool leerRegistroOCRsdSPI(uint32_t *registro);
bool leerRegistroCSDsdSPI(void);
bool leerRegistroCIDsdSPI(void);
bool ajustarLonBloqueSDspi(uint32_t lonBloque);
bool esperarReposoSDspi(uint8_t bytesMaxEspera);
uint8_t esperarRespuestaNoResposoSDspi(uint8_t retardoMax);
uint8_t enviarComandoSDspi(uint8_t codComando, uint32_t argComando);
uint8_t enviarComandoAppSDspi(uint8_t codComando, uint32_t argComando);
void iniciarEnvioBloqueDatosSDspi(uint8_t *buffer, bool escribirMultBloques);
bool finalizarEnvioBloqueDatosSDspi(void);
estadoRecepcionBloqueSD_e recibirBloqueDatosSDspi(uint8_t *buffer, uint16_t tam);
estadoOperacionSD_e finalizarEscrituraBloquesSDspi(void);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarSDspi(void)
**  Descripcion:    Inicia la tarjeta SD en modo SPI
**  Parametros:     Ninguno
**  Retorno:        True si OK
****************************************************************************************/
bool iniciarSDspi(void)
{
    sd_t *driver = punteroSD();

    driver->habilitada = configSD()->modo == SD_MODO_SPI;
    if (!driver->habilitada) {
    	driver->estado = SD_ESTADO_NO_PRESENTE;
        return false;
    }

    if (configSD()->pinCS) {
        sdSPI.bus.tipo = BUS_SPI;
        sdSPI.bus.bus_u.spi.numSPI = configSD()->numSPI;
        sdSPI.bus.bus_u.spi.pinCS = configSD()->pinCS;
        configurarIO(sdSPI.bus.bus_u.spi.pinCS, CONFIG_IO(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL), 0);

        // Iniciamos el bus si es necesario
        if (!spiIniciado(sdSPI.bus.bus_u.spi.numSPI)) {
        	if (!iniciarSPI(sdSPI.bus.bus_u.spi.numSPI))
                return false;
        }
    }

    // Frecuencia maxima de inicializacion a 400kHz
    ajustarRelojSPI(sdSPI.bus.bus_u.spi.numSPI, SPI_RELOJ_INICIALIZACION);

    // La tarjeta SD necesita de un retardo despues de alimentarse
    delay(1000);

    // Se transmiten una serie de bytes basura ficticios con CS alto para que la tarjeta SD pueda iniciarse
    escribirIO(sdSPI.bus.bus_u.spi.pinCS, true);
    transferirRawBufferBusSPI(&sdSPI.bus, NULL, NULL, NUM_BYTES_BASURA_INICIALIZACION_SD_SPI);

    // Espera a que termine la transmision antes de habilitar la tarjeta SD
    uint32_t tiempo = 100000;
    while (ocupadoBusSPI(&sdSPI.bus)) {
        if (tiempo-- == 0) {
        	driver->estado = SD_ESTADO_NO_PRESENTE;
        	driver->contadorFallos++;
            return false;
        }
    }

    driver->tiempoOpIniciada = millis();
    driver->estado = SD_ESTADO_RESET;
    driver->contadorFallos = 0;
    return true;
}


/***************************************************************************************
**  Nombre:         bool sondearSDspi(void)
**  Descripcion:    Esta funcion se debe llamar periodicamente para que la tarjeta SD realice
**                  transferencias en curso
**  Parametros:     Ninguno
**  Retorno:        Devuelve True si la tarjeta esta lista para aceptar comandos
****************************************************************************************/
bool sondearSDspi(void)
{
    sd_t *driver = punteroSD();

    if (!driver->habilitada) {
    	driver->estado = SD_ESTADO_NO_PRESENTE;
        return false;
    }

    uint8_t estadoInicial;
    bool envioCompleto;

  nuevoInt:
    switch (driver->estado) {
        case SD_ESTADO_RESET:
        	seleccionarSDspi();
            estadoInicial = enviarComandoSDspi(SD_COMANDO_GO_IDLE_STATE, 0);
            deseleccionarSDspi();

            if (estadoInicial == SD_R1_BIT_ESTADO_REPOSO) {
                // Chequea el voltaje y la version
                if (validarVersionInterfaceSDspi()) {
                	driver->estado = SD_ESTADO_INICIALIZACION_EN_PROGRESO;
                    goto nuevoInt;
                }
                else
                	driver->estado = SD_ESTADO_NO_PRESENTE;        // Mala respuesta/voltaje, debemos abstenernos de acceder a la tarjeta.
            }
            break;

        case SD_ESTADO_INICIALIZACION_EN_PROGRESO:
            if (inicializacionCompletaSDspi()) {
                if (driver->version == 2) {
                    // Chequeo para tarjetas de alta capacidad
                    uint32_t ocr;

                    if (!leerRegistroOCRsdSPI(&ocr)) {
                    	resetearSDspi();
                        goto nuevoInt;
                    }

                    driver->altaCapacidad = (ocr & (1 << 30)) != 0;
                }
                else
                	driver->altaCapacidad = false;        // Las tarjetas con version 1 son siempre de baja capacidad

                // Ahora buscamos los registros CSD y CID
                if (leerRegistroCSDsdSPI()) {
                	seleccionarSDspi();
                    uint8_t estado = enviarComandoSDspi(SD_COMANDO_SEND_CID, 0);

                    if (estado == 0) {
                        // Mantiene la tarjeta seleccionada para recibir el bloque de respuesta
                    	driver->estado = SD_ESTADO_INICIALIZACION_RECEPCION_CID;
                        goto nuevoInt;
                    }
                    else {
                    	deseleccionarSDspi();
                    	resetearSDspi();
                        goto nuevoInt;
                    }
                }
            }
            break;

        case SD_ESTADO_INICIALIZACION_RECEPCION_CID:
            if (leerRegistroCIDsdSPI()) {
            	deseleccionarSDspi();

                // La especificacion es dudosa sobre cual es el tamanio de bloque predeterminado para las tarjetas estandar (se puede cambiar en
                // tarjetas de tamanio estandar), asi que se configura en 512 explicitamente para no tener ningun problema.
                if (!driver->altaCapacidad && !ajustarLonBloqueSDspi(TAMANIO_BLOQUE_SD)) {
                	resetearSDspi();
                    goto nuevoInt;
                }

                // Ya se puede cambiar la velocidad del reloj a < 25MHz
                ajustarRelojSPI(sdSPI.bus.bus_u.spi.numSPI, SPI_RELOJ_RAPIDO);

                driver->bloquesRestantesMultiEscritura = 0;
                driver->estado = SD_ESTADO_READY;
                goto nuevoInt;
            }   // Sino, sigue esperando a que llegue el CID
            break;

        case SD_ESTADO_ENVIANDO_ESCRITURA:
            // ¿Hemos terminado de enviar la escritura?
            envioCompleto = false;

            // Enviar otro trozo
            transferirRawBufferBusSPI(&sdSPI.bus, driver->operacionPendiente.buffer + TAMANIO_TROZO_NO_DMA_SD_SPI * driver->operacionPendiente.indiceTrozo, NULL, TAMANIO_TROZO_NO_DMA_SD_SPI);
            driver->operacionPendiente.indiceTrozo++;
            envioCompleto = driver->operacionPendiente.indiceTrozo == TAMANIO_BLOQUE_SD / TAMANIO_TROZO_NO_DMA_SD_SPI;

            if (envioCompleto) {
                // Termina enviando el CRC y verificando la aceptación / rechazo de la tarjeta SD
                if (finalizarEnvioBloqueDatosSDspi()) {
                    // La SD ahora esta ocupada comprometiendo la escritura en la tarjeta
                	driver->estado = SD_ESTADO_ESPERANDO_PARA_ESCRIBIR;
                	driver->tiempoOpIniciada = millis();

                    // Como se ha transmitido el buffer, ya se puede avisar de que la operacion ha sido un exito
                    if (driver->operacionPendiente.callback)
                    	driver->operacionPendiente.callback(SD_OPERACION_BLOQUE_ESCRIBIR, driver->operacionPendiente.indiceBloque, driver->operacionPendiente.buffer, driver->operacionPendiente.datoCallback);
                }
                else {
                    // La escritura ha sido rechazada! Esto podria deberse a una mala direccion, pero se entiende que no es eso, asi que se asume que la tarjeta esta rota y necesita reiniciarse
                	resetearSDspi();

                    // Fallo de escritura
                    if (driver->operacionPendiente.callback)
                    	driver->operacionPendiente.callback(SD_OPERACION_BLOQUE_ESCRIBIR, driver->operacionPendiente.indiceBloque, NULL, driver->operacionPendiente.datoCallback);

                    goto nuevoInt;
                }
            }
            break;

        case SD_ESTADO_ESPERANDO_PARA_ESCRIBIR:
            if (esperarReposoSDspi(NUM_MAX_BYTES_RETARDO_RESPUESTA_CMD_SD_SPI)) {
            	driver->contadorFallos = 0; // Se entiende que la SD esta OK si puede completar una escritura

                // Aun quedan mas bloques para escribir en una cadena de bloques multiples?
                if (driver->bloquesRestantesMultiEscritura > 1) {
                	driver->bloquesRestantesMultiEscritura--;
                	driver->siguienteBloqueMultiEscritura++;
                	driver->estado = SD_ESTADO_ESCRIBIENDO_MULTIPLES_BLOQUES;
                }
                else if (driver->bloquesRestantesMultiEscritura == 1) {
                    // Esta función cambia el estado de la tarjeta SD, ya sea inmediatamente exitoso o retrasado:
                    if (finalizarEscrituraBloquesSDspi() == SD_OPERACION_EXITO)
                    	deseleccionarSDspi();
                }
                else {
                	driver->estado = SD_ESTADO_READY;
                    deseleccionarSDspi();
                }
            }
            else if (millis() > driver->tiempoOpIniciada + SD_TIMEOUT_ESCRITURA_MS) {
            	resetearSDspi();
                goto nuevoInt;
            }
            break;

        case SD_ESTADO_LEYENDO:
            switch (recibirBloqueDatosSDspi(driver->operacionPendiente.buffer, TAMANIO_BLOQUE_SD)) {
                case SD_RECEPCION_EXITOSA:
                	deseleccionarSDspi();

                	driver->estado = SD_ESTADO_READY;
                	driver->contadorFallos = 0;           // Asumimos que la SD esta ok si puede completar una lectura

                    if (driver->operacionPendiente.callback) {
                    	driver->operacionPendiente.callback(SD_OPERACION_BLOQUE_LEER, driver->operacionPendiente.indiceBloque,
                        driver->operacionPendiente.buffer, driver->operacionPendiente.datoCallback);
                    }
                    break;

                case SD_RECEPCION_BLOQUE_EN_PROGRESO:
                    if (millis() <= driver->tiempoOpIniciada + SD_TIMEOUT_LECTURA_MS)
                        break;
                    // El timeout a expirado y se convierte en un error fatal
                    FALLTHROUGH;

                case SD_RECEPCION_ERROR:
                	deseleccionarSDspi();
                	resetearSDspi();
                    goto nuevoInt;
                    break;
            }
            break;

        case SD_ESTADO_PARANDO_ESCRITURA_MULTIPLES_BLOQUES:
            if (esperarReposoSDspi(NUM_MAX_BYTES_RETARDO_RESPUESTA_CMD_SD_SPI)) {
            	deseleccionarSDspi();
            	driver->estado = SD_ESTADO_READY;
            }
            else if (millis() > driver->tiempoOpIniciada + SD_TIMEOUT_ESCRITURA_MS) {
            	resetearSDspi();
                goto nuevoInt;
            }
            break;

        case SD_ESTADO_NO_PRESENTE:
        default:
            break;
    }

    // Comprobamos cuanto tarda en inicializarse
    if (driver->estado >= SD_ESTADO_RESET && driver->estado < SD_ESTADO_READY && millis() - driver->tiempoOpIniciada > TIMEOUT_INICIALIZACION_MS_SD)
    	resetearSDspi();

    return tarjetaReadySDspi();
}


/***************************************************************************************
**  Nombre:         bool leerBloqueSDspi(uint32_t indiceBloque, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback)
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
bool leerBloqueSDspi(uint32_t indiceBloque, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback)
{
    sd_t *driver = punteroSD();

    if (driver->estado != SD_ESTADO_READY) {
        if (driver->estado == SD_ESTADO_ESCRIBIENDO_MULTIPLES_BLOQUES) {
            if (finalizarEscrituraBloquesSDspi() != SD_OPERACION_EXITO)
                return false;
        }
        else
            return false;
    }

    seleccionarSDspi();

    // Las tarjetas de tamanio estandar usan direccionamiento de bytes, las tarjetas de alta capacidad usan direccionamiento de bloque
    uint8_t estado = enviarComandoSDspi(SD_COMANDO_READ_SINGLE_BLOCK, driver->altaCapacidad ? indiceBloque : indiceBloque * TAMANIO_BLOQUE_SD);

    if (estado == 0) {
    	driver->operacionPendiente.buffer = buffer;
    	driver->operacionPendiente.indiceBloque = indiceBloque;
    	driver->operacionPendiente.callback = callback;
    	driver->operacionPendiente.datoCallback = datoCallback;

    	driver->estado = SD_ESTADO_LEYENDO;
    	driver->tiempoOpIniciada = millis();

        // Se deja la tarjeta seleccionada para toda la transaccion
        return true;
    }
    else {
    	deseleccionarSDspi();
        return false;
    }
}


/***************************************************************************************
**  Nombre:         estadoOperacionSD_e iniciarEscrituraBloquesSDspi(uint32_t indiceBloque, uint32_t numBloques)
**  Descripcion:    Comienza a escribir una serie de bloques consecutivos que comienzan en el indice de bloque dado
**                  Esto permitira (pero no requerira) la tarjeta SD para borrar previamente el numero de bloques que
**                  especifique, lo que puede permitir que las escrituras se completen mas rapido
**
**                  Luego, simplemente se llama a escribirBloqueSDspi() como de costumbre para escribir esos bloques consecutivamente
**
**                  Esta bien abortar la escritura de bloques multiples en cualquier momento escribiendo a una dirección no consecutiva o realizando una lectura.
**  Parametros:     Indice de bloque, numero de bloques a escribir
**  Retorno:            SD_OPERACION_EXITO   - La escritura de bloques multiples se ha puesto en cola
**                      SD_OPERACION_OCUPADO - La tarjeta ya esta ocupada y no puede aceptar su escritura
**                      SD_OPERACION_FALLO   - Se ha producido un error fatal, la tarjeta se reiniciara
****************************************************************************************/
estadoOperacionSD_e iniciarEscrituraBloquesSDspi(uint32_t indiceBloque, uint32_t numBloques)
{
    sd_t *driver = punteroSD();

    if (driver->estado != SD_ESTADO_READY) {
        if (driver->estado == SD_ESTADO_ESCRIBIENDO_MULTIPLES_BLOQUES) {
            if (indiceBloque == driver->siguienteBloqueMultiEscritura)
                return SD_OPERACION_EXITO;           // Se asume que se desea continuar con la escritura de bloques multiples que ya esta en progreso
            else if (finalizarEscrituraBloquesSDspi() != SD_OPERACION_EXITO)
                return SD_OPERACION_OCUPADO;
        }
        else
            return SD_OPERACION_OCUPADO;
    }

    seleccionarSDspi();

    if (enviarComandoAppSDspi(SD_ACOMANDO_SET_WR_BLK_ERASE_COUNT, numBloques) == 0
        && enviarComandoSDspi(SD_COMANDO_WRITE_MULTIPLE_BLOCK, driver->altaCapacidad ? indiceBloque : indiceBloque * TAMANIO_BLOQUE_SD) == 0) {
    	driver->estado = SD_ESTADO_ESCRIBIENDO_MULTIPLES_BLOQUES;
    	driver->bloquesRestantesMultiEscritura = numBloques;
    	driver->siguienteBloqueMultiEscritura = indiceBloque;

        // Se deja la SD seleccionada
        return SD_OPERACION_EXITO;
    }
    else {
    	deseleccionarSDspi();
    	resetearSDspi();
        return SD_OPERACION_FALLO;
    }
}


/***************************************************************************************
**  Nombre:         estadoOperacionSD_e escribirBloqueSDspi(uint32_t indiceBloque, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback)
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
estadoOperacionSD_e escribirBloqueSDspi(uint32_t indiceBloque, uint8_t *buffer, callbackOpCompletaSD_c callback, uint32_t datoCallback)
{
    sd_t *driver = punteroSD();
    uint8_t estado;

  nuevoInt:
    switch (driver->estado) {
        case SD_ESTADO_ESCRIBIENDO_MULTIPLES_BLOQUES:
            // ¿Necesitamos cancelar la escritura multibloque anterior?
            if (indiceBloque != driver->siguienteBloqueMultiEscritura) {
                if (finalizarEscrituraBloquesSDspi() == SD_OPERACION_EXITO)
                    goto nuevoInt;         // Ahora que hemos entrado en el estado listo, podemos intentarlo de nuevo
                else
                    return SD_OPERACION_OCUPADO;
            }

            // Continuamos una escritura de bloques multiples
            break;

        case SD_ESTADO_READY:
            // No continuamos una escritura de bloques multiples, por lo que debemos enviar un comando de escritura de bloque unico
        	seleccionarSDspi();

            // Las tarjetas de tamanio estandar usan direccionamiento de bytes, las tarjetas de alta capacidad usan direccionamiento de bloque
            estado = enviarComandoSDspi(SD_COMANDO_WRITE_BLOCK, driver->altaCapacidad ? indiceBloque : indiceBloque * TAMANIO_BLOQUE_SD);

            if (estado != 0) {
            	deseleccionarSDspi();
            	resetearSDspi();
                return SD_OPERACION_FALLO;
            }
            break;

        default:
            return SD_OPERACION_OCUPADO;
    }

    iniciarEnvioBloqueDatosSDspi(buffer, driver->estado == SD_ESTADO_ESCRIBIENDO_MULTIPLES_BLOQUES);

    driver->operacionPendiente.buffer = buffer;
    driver->operacionPendiente.indiceBloque = indiceBloque;
    driver->operacionPendiente.callback = callback;
    driver->operacionPendiente.datoCallback = datoCallback;
    driver->operacionPendiente.indiceTrozo = 1;    // (para transferencias que no son DMA) ya hemos enviado el fragmento #0
    driver->estado = SD_ESTADO_ENVIANDO_ESCRITURA;

    return SD_OPERACION_EN_PROGRESO;
}


/***************************************************************************************
**  Nombre:         bool inicializacionCompletaSDspi(void)
**  Descripcion:    Comprueba si la tarjeta SD ha completado su secuencia de inicio
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool inicializacionCompletaSDspi(void)
{
    sd_t *driver = punteroSD();

	seleccionarSDspi();
    uint8_t estado = enviarComandoAppSDspi(SD_ACOMANDO_SEND_OP_COND, driver->version == 2 ? 1 << 30 : 0);
    deseleccionarSDspi();

    // Cuando la inicializacion se completa, el bit inactivo en la respuesta se vuelve cero
    return estado == 0x00;
}


/***************************************************************************************
**  Nombre:         void resetearSDspi(void)
**  Descripcion:    Maneja un fallo de una operacion de tarjeta SD restableciendo la tarjeta a su
**                  fase de inicialización.
**
**                  Incrementa el contador de fallos, y cuando se alcanza el umbral de fallos, deshabilita
**                  la tarjeta hasta la proxima llamada a iniciarSDspi().
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void resetearSDspi(void)
{
    sd_t *driver = punteroSD();

    if (!tarjetaSDinsertada()) {
	    driver->estado = SD_ESTADO_NO_PRESENTE;
        return;
    }

    if (driver->estado >= SD_ESTADO_READY)
        ajustarRelojSPI(sdSPI.bus.bus_u.spi.numSPI, SPI_RELOJ_INICIALIZACION);

    driver->contadorFallos++;
    if (driver->contadorFallos >= NUM_MAX_FALLOS_CONSECUTIVOS_SD)
    	driver->estado = SD_ESTADO_NO_PRESENTE;
    else {
    	driver->tiempoOpIniciada = millis();
    	driver->estado = SD_ESTADO_RESET;
    }
}


/***************************************************************************************
**  Nombre:         void tarjetaReadySDspi(void)
**  Descripcion:    Comprueba si la SD esta lista para aceptar comandos de lectura o escritura
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool tarjetaReadySDspi(void)
{
    sd_t *driver = punteroSD();
    return driver->estado == SD_ESTADO_READY || driver->estado == SD_ESTADO_ESCRIBIENDO_MULTIPLES_BLOQUES;
}


/***************************************************************************************
**  Nombre:         void seleccionarSDspi(void)
**  Descripcion:    Selecciona la SD en el bus SPI
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void seleccionarSDspi(void)
{
    escribirIO(sdSPI.bus.bus_u.spi.pinCS, false);
}


/***************************************************************************************
**  Nombre:         void deseleccionarSDspi(void)
**  Descripcion:    Quita la seleccion de la SD en el bus SPI
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void deseleccionarSDspi(void)
{
    while (ocupadoBusSPI(&sdSPI.bus));
    delayMicroseconds(10);
    escribirIO(sdSPI.bus.bus_u.spi.pinCS, true);
}


/***************************************************************************************
**  Nombre:         bool validarVersionInterfaceSDspi(void)
**  Descripcion:    Envia un mensaje IF_COND a la tarjeta para verificar su version y validar
**                  sus requisitos de voltaje. Establece la version detectada (0, 1 o 2)
**  Parametros:     Ninguno
**  Retorno:        True si es compatible
****************************************************************************************/
bool validarVersionInterfaceSDspi(void)
{
    uint8_t respuesta[4];
    sd_t *driver = punteroSD();

    driver->version = 0;

    seleccionarSDspi();
    uint8_t estado = enviarComandoSDspi(SD_COMANDO_SEND_IF_COND, (SD_VOLTAJE_ACEPTADO_2_7_a_3_6 << 8) | PATRON_CHEQUEO_IF_COND_SD_SPI);

    // No anula la seleccion de la tarjeta de inmediato, porque queremos leer el resto de su respuesta si es una tarjeta V2
    if (estado == (SD_R1_BIT_ESTADO_COMANDO_ILEGAL | SD_R1_BIT_ESTADO_REPOSO))
    	driver->version = 1;             // Las tarjetas V1 no soportan este comando
    else if (estado == SD_R1_BIT_ESTADO_REPOSO) {
        transferirRawBufferBusSPI(&sdSPI.bus, NULL, respuesta, sizeof(respuesta));

        // No nos molestamos en validar el rango de voltaje de funcionamiento de la tarjeta SD ya que la especificacion requiere que acepte 3.3V
        // pero se verifica que repita nuestro patron de verificacion correctamente
        if (respuesta[3] == PATRON_CHEQUEO_IF_COND_SD_SPI)
        	driver->version = 2;
    }

    deseleccionarSDspi();
    return driver->version > 0;
}


/***************************************************************************************
**  Nombre:         bool leerRegistroOCRsdSPI(uint32_t *registro)
**  Descripcion:    Lee el registro OCR
**  Parametros:     Puntero al valor del registro
**  Retorno:        True si ok
****************************************************************************************/
bool leerRegistroOCRsdSPI(uint32_t *registro)
{
    uint8_t respuesta[4];

    seleccionarSDspi();
    uint8_t estado = enviarComandoSDspi(SD_COMANDO_READ_OCR, 0);

    transferirRawBufferBusSPI(&sdSPI.bus, NULL, respuesta, sizeof(respuesta));

    if (estado == 0) {
    	deseleccionarSDspi();
        *registro = (respuesta[0] << 24) | (respuesta[1] << 16) | (respuesta[2] << 8) | respuesta[3];
        return true;
    }
    else {
    	deseleccionarSDspi();
        return false;
    }
}


/***************************************************************************************
**  Nombre:         bool leerRegistroCSDsdSPI(void)
**  Descripcion:    Extrae el CSD
**  Parametros:     Ninguno
**  Retorno:        True si OK
****************************************************************************************/
bool leerRegistroCSDsdSPI(void)
{
    uint32_t lonBloqueLectura, numBloques, numBloquesMult;
    uint64_t bytesCapacidad;
    sd_t *driver = punteroSD();

    seleccionarSDspi();
    /*
     * El bloque de datos del comando CSD siempre debe llegar dentro de 8 ciclos de reloj inactivo
     * Esto es porque la informacion sobre la latencia de la tarjeta se almacena en el propio registro CSD,
     * por lo que aun no podemos usarla.
     */
    bool exito = enviarComandoSDspi(SD_COMANDO_SEND_CSD, 0) == 0 && recibirBloqueDatosSDspi((uint8_t*) &sdSPI.csd, sizeof(sdSPI.csd)) == SD_RECEPCION_EXITOSA
                 && OBTENER_CAMPO_CSD_SD(sdSPI.csd, 1, TRAILER) == 1;

    if (exito) {
        switch (OBTENER_CAMPO_CSD_SD(sdSPI.csd, 1, CSD_STRUCTURE_VER)) {
            case SD_CSD_STRUCTURA_VERSION_1:
                // Tamanio del bloque en bytes (no tiene que ser 512)
                lonBloqueLectura = 1 << OBTENER_CAMPO_CSD_SD(sdSPI.csd, 1, READ_BLOCK_LEN);
                numBloquesMult = 1 << (OBTENER_CAMPO_CSD_SD(sdSPI.csd, 1, CSIZE_MULT) + 2);
                numBloques = (OBTENER_CAMPO_CSD_SD(sdSPI.csd, 1, CSIZE) + 1) * numBloquesMult;

                // Podriamos hacer esto en 32 bits, pero hace que el caso de 2GB sea incomodo
                bytesCapacidad = (uint64_t) numBloques * lonBloqueLectura;

                // Vuelve a expresar esa capacidad (maximo 2 GB) en nuestro tamanio de bloque estandar de 512 bytes
                driver->metadatos.numBloques = bytesCapacidad / TAMANIO_BLOQUE_SD;
                break;

            case SD_CSD_STRUCTURA_VERSION_2:
            	driver->metadatos.numBloques = (OBTENER_CAMPO_CSD_SD(sdSPI.csd, 2, CSIZE) + 1) * 1024;
                break;

            default:
                exito = false;
                break;
        }
    }

    deseleccionarSDspi();
    return exito;
}


/***************************************************************************************
**  Nombre:         bool leerRegistroCIDsdSPI(void)
**  Descripcion:    Obtiene el CID
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool leerRegistroCIDsdSPI(void)
{
    uint8_t cid[16];
    sd_t *driver = punteroSD();

    if (recibirBloqueDatosSDspi(cid, sizeof(cid)) != SD_RECEPCION_EXITOSA)
        return false;

    driver->metadatos.idFabricante = cid[0];
    driver->metadatos.idOEM = (cid[1] << 8) | cid[2];
    driver->metadatos.nombreProducto[0] = cid[3];
    driver->metadatos.nombreProducto[1] = cid[4];
    driver->metadatos.nombreProducto[2] = cid[5];
    driver->metadatos.nombreProducto[3] = cid[6];
    driver->metadatos.nombreProducto[4] = cid[7];
    driver->metadatos.revisionProductoMayor = cid[8] >> 4;
    driver->metadatos.revisionProductoMenor = cid[8] & 0x0F;
    driver->metadatos.numSerie = (cid[9] << 24) | (cid[10] << 16) | (cid[11] << 8) | cid[12];
    driver->metadatos.anoProduccion = (((cid[13] & 0x0F) << 4) | (cid[14] >> 4)) + 2000;
    driver->metadatos.mesProduccion = cid[14] & 0x0F;

    return true;
}


/***************************************************************************************
**  Nombre:         bool ajustarLonBloqueSDspi(uint32_t lonBloque)
**  Descripcion:    Ajusta la longitud de los bloques
**  Parametros:     Longitud de bloque
**  Retorno:        True si ok
****************************************************************************************/
bool ajustarLonBloqueSDspi(uint32_t lonBloque)
{
	seleccionarSDspi();
    uint8_t estado = enviarComandoSDspi(SD_COMANDO_SET_BLOCKLEN, lonBloque);
    deseleccionarSDspi();

    return estado == 0;
}


/***************************************************************************************
**  Nombre:         bool esperarReposoSDspi(uint8_t bytesMaxEspera)
**  Descripcion:    La especificacion de la tarjeta SD requiere que enviemos 8 ciclos de reloj en el bus
**                  despues de la mayoria de los comandos para que pueda finalizar su procesamiento.
**                  La forma más facil de hacer esto es esperar a que el bus este inactivo antes de
**                  transmitir un comando, enviando al menos 8 bytes al bus cuando lo hacemos.
**  Parametros:     Bytes de espera
**  Retorno:        True si SD en reposo
****************************************************************************************/
bool esperarReposoSDspi(uint8_t bytesMaxEspera)
{
    while (bytesMaxEspera > 0) {
        uint8_t respuesta;

        transferirRawBusSPI(&sdSPI.bus, 0xFF, &respuesta);
        if (respuesta == 0xFF)
            return true;

        bytesMaxEspera--;
    }

    return false;
}


/***************************************************************************************
**  Nombre:         uint8_t esperarRespuestaNoResposoSDspi(uint8_t retardoMax)
**  Descripcion:    Espera a que lleguen hasta retardoMax 0xFF bytes inactivos desde la tarjeta
                    devolviendo el primer byte no inactivo encontrado
**  Parametros:     Bytes de espera
**  Retorno:        0xFF si hay fallo, sino se devuelve la respuesta
****************************************************************************************/
uint8_t esperarRespuestaNoResposoSDspi(uint8_t retardoMax)
{
    for (uint8_t i = 0; i < retardoMax + 1; i++) { // + 1 para que podamos esperar retardoMax '0xFF' bytes antes de leer un byte de respuesta
        uint8_t respuesta;

        transferirRawBusSPI(&sdSPI.bus, 0xFF, &respuesta);
        if (respuesta != 0xFF)
            return respuesta;
    }

    return 0xFF;
}


/***************************************************************************************
**  Nombre:         uint8_t enviarComandoSDspi(uint8_t codComando, uint32_t argComando)
**  Descripcion:    Espera hasta MAXIMOS_BYTES_RETARDO_PARA_RESPUESTA_CMD_SD bytes para que la tarjeta este lista.
**                  Envia un comando a la tarjeta con el argumento dado, espera hasta MAXIMOS_BYTES_RETARDO_PARA_RESPUESTA_CMD_SD
**                  bytes para una respuesta, y devuelve el primer byte no 0xFF de la respuesta
**  Parametros:     Codigo del comando, argumento del comando
**  Retorno:        0xFF si hay fallo, sino se devuelve la respuesta
****************************************************************************************/
uint8_t enviarComandoSDspi(uint8_t codComando, uint32_t argComando)
{
    uint8_t comando[6] = {
        0x40 | codComando,
        argComando >> 24,
        argComando >> 16,
        argComando >> 8,
        argComando,
        0x95 // CRC estatico. Este CRC es valido parar CMD0 con argumento 0, y CMD8 con argumento 0x1AB, los cuales son comandos que requieren CRC
    };

    if (!esperarReposoSDspi(NUM_MAX_BYTES_RETARDO_RESPUESTA_CMD_SD_SPI) && codComando != SD_COMANDO_GO_IDLE_STATE)
        return 0xFF;

    transferirRawBufferBusSPI(&sdSPI.bus, comando, NULL, sizeof(comando));

    // La tarjeta puede tomar hasta NUM_MAX_BYTES_RETARDO_RESPUESTA_CMD_SD_SPI bytes para enviar la respuesta, mientras tanto transmitira 0xFF bytes de relleno.
    return esperarRespuestaNoResposoSDspi(NUM_MAX_BYTES_RETARDO_RESPUESTA_CMD_SD_SPI);
}


/***************************************************************************************
**  Nombre:         uint8_t enviarComandoAppSDspi(uint8_t codComando, uint32_t argComando)
**  Descripcion:    Envia un comando APP
**  Parametros:     Codigo del comando, argumento del comando
**  Retorno:        0xFF si hay fallo, sino se devuelve la respuesta
****************************************************************************************/
uint8_t enviarComandoAppSDspi(uint8_t codComando, uint32_t argComando)
{
	enviarComandoSDspi(SD_COMANDO_APP_CMD, 0);
    return enviarComandoSDspi(codComando, argComando);
}


/***************************************************************************************
**  Nombre:         void iniciarEnvioBloqueDatosSDspi(uint8_t *buffer, bool escribirMultBloques)
**  Descripcion:    Comienza a enviar un buffer de TAMANIO_BLOQUE_SD bytes a la tarjeta SD
**  Parametros:     Buffer de escritura, multiples bloques o no
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarEnvioBloqueDatosSDspi(uint8_t *buffer, bool escribirMultBloques)
{
    // La tarjeta requiere 8 ciclos de reloj ficticios entre la respuesta del comando de escritura y un bloque de datos que comienza
    transferirRawBusSPI(&sdSPI.bus, 0xFF, NULL);
    transferirRawBusSPI(&sdSPI.bus, escribirMultBloques ? TOKEN_INICIAR_ESCRITURA_MULTIPLES_BLOQUES_SD : TOKEN_INICIAR_ESCRITURA_BLOQUE_SD, NULL);

    // Envia el primer fragmento ahora
    transferirRawBufferBusSPI(&sdSPI.bus, buffer, NULL, TAMANIO_TROZO_NO_DMA_SD_SPI);
}


/***************************************************************************************
**  Nombre:         bool finalizarEnvioBloqueDatosSDspi(void)
**  Descripcion:    Funcion para finalizar el envio de bloques
**  Parametros:     Ninguno
**  Retorno:        True si los datos se han aceptado
****************************************************************************************/
bool finalizarEnvioBloqueDatosSDspi(void)
{
    uint8_t token;

    // Drenamos el buffer de lectura
    drenarBufferRecepcionSPI(sdSPI.bus.bus_u.spi.numSPI);

    // Enviamos un CRC
    transferirRawBusSPI(&sdSPI.bus, 0x00, NULL);
    transferirRawBusSPI(&sdSPI.bus, 0x00, NULL);

    transferirRawBusSPI(&sdSPI.bus, 0xFF, &token);

    /*
     * Comprobamos que la SD ha aceptado la escritura (no hay error de CRC / direccion)
     *
     * Los 5 bits mas bajosestan estructrurados asi:
     * | 0 | Estado  | 1 |
     * | 0 | x  x  x | 1 |
     *
     * Estados:
     * 010 - Dato aceptado
     * 101 - Error CRC
     * 110 - Error Escritura
     */
    return (token & 0x1F) == 0x05;
}


/***************************************************************************************
**  Nombre:         estadoRecepcionBloqueSD_e recibirBloqueDatosSDspi(uint8_t *buffer, uint16_t num)
**  Descripcion:    Intenta recibir un bloque de datos de la tarjeta SD.
**  Parametros:     Puntero al buffer de recepcion, numero de datos
**  Retorno:        Estado de la recepcion
****************************************************************************************/
estadoRecepcionBloqueSD_e recibirBloqueDatosSDspi(uint8_t *buffer, uint16_t tam)
{
    uint8_t token = esperarRespuestaNoResposoSDspi(8);

    if (token == 0xFF)
        return SD_RECEPCION_BLOQUE_EN_PROGRESO;

    if (token != TOKEN_INICIAR_LECTURA_BLOQUE_SD)
        return SD_RECEPCION_ERROR;

    transferirRawBufferBusSPI(&sdSPI.bus, NULL, buffer, tam);

    // Descarta el CRC
    transferirRawBusSPI(&sdSPI.bus, 0xFF, NULL);
    transferirRawBusSPI(&sdSPI.bus, 0xFF, NULL);

    return SD_RECEPCION_EXITOSA;
}


/***************************************************************************************
**  Nombre:         estadoOperacionSD_e finalizarEscrituraBloquesSDspi(void)
**  Descripcion:    Envia el token de detencion de transmision para completar una escritura de bloques multiples
**  Parametros:     Ninguno
**  Retorno:            OPERACION_SD_EN_PROGRESO  - Ahora estamos esperando que se complete esa parada la tarjeta
**                                                  se pondra en ESTADO_SD_PARANDO_ESCRITURA_MULTIPLES_BLOQUES
**                      OPERACION_SD_EXITO        - La escritura de bloques multiples finalizo inmediatamente, la tarjeta
**                                                  se pondra en ESTADO_SD_READY
****************************************************************************************/
estadoOperacionSD_e finalizarEscrituraBloquesSDspi(void)
{
    sd_t *driver = punteroSD();

	driver->bloquesRestantesMultiEscritura = 0;

    // 8 Ciclos de reloj para garantizar N_WR relojes entre la ultima respuesta de la tarjeta y este token
    transferirRawBusSPI(&sdSPI.bus, 0xFF, NULL);
    transferirRawBusSPI(&sdSPI.bus, TOKEN_PARAR_ESCRITURA_MULTIPLES_BLOQUES_SD, NULL);

    // La tarjeta puede optar por generar una señal de ocupado (no-0xFF) despues de un retraso maximo de N_BR (1 byte)
    if (esperarRespuestaNoResposoSDspi(1) == 0xFF) {
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
**  Nombre:         tablaFnSD_t tablaFnSDspi
**  Descripcion:    Tabla de funciones de la SD en modo SPI
****************************************************************************************/
tablaFnSD_t tablaFnSDspi = {
    iniciarSDspi,
    sondearSDspi,
    leerBloqueSDspi,
    iniciarEscrituraBloquesSDspi,
    escribirBloqueSDspi,
};

#endif

