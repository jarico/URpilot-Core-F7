/***************************************************************************************
**  gps_ublox.c - Gestion de los GPS del fabricante Ublox
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 14/06/2019
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
#include <string.h>
#include <math.h>

#include "gps_ublox.h"

#ifdef USAR_GPS
#include "GP/gp_gps.h"
#include "GP/gp_rtc.h"
#include "Drivers/rtc.h"
#include "Drivers/tiempo.h"
#include "Drivers/io.h"
#include "Drivers/uart.h"
#include "Comun/util.h"
#include "Comun/matematicas.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define UBX_RATE_POSLLH                        1
#define UBX_RATE_STATUS                        1
#define UBX_RATE_SOL                           1
#define UBX_RATE_TIMEGPS                       5
#define UBX_RATE_PVT                           1
#define UBX_RATE_VELNED                        1
#define UBX_RATE_DOP                           1
#define UBX_RATE_HW                            5
#define UBX_RATE_HW2                           5


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    PREAMBLE1            = 0xb5,
    PREAMBLE2            = 0x62,
    CLASS_NAV            = 0x01,
    CLASS_ACK            = 0x05,
    CLASS_CFG            = 0x06,
    CLASS_MON            = 0x0A,
    CLASS_RXM            = 0x02,
    MSG_ACK_NACK         = 0x00,
    MSG_ACK_ACK          = 0x01,
    MSG_POSLLH           = 0x2,
    MSG_STATUS           = 0x3,
    MSG_DOP              = 0x4,
    MSG_SOL              = 0x6,
    MSG_PVT              = 0x7,
    MSG_TIMEGPS          = 0x20,
    MSG_VELNED           = 0x12,
    MSG_CFG_CFG          = 0x09,
    MSG_CFG_RATE         = 0x08,
    MSG_CFG_MSG          = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24,
    MSG_CFG_PRT          = 0x00,
    MSG_CFG_SBAS         = 0x16,
    MSG_CFG_GNSS         = 0x3E,
    MSG_CFG_TP5          = 0x31,
    MSG_MON_HW           = 0x09,
    MSG_MON_HW2          = 0x0B,
    MSG_MON_VER          = 0x04,
    MSG_NAV_SVINFO       = 0x30,
    MSG_RXM_RAW          = 0x10,
    MSG_RXM_RAWX         = 0x15,
} bytesProtocoloUBX_e;

typedef enum {
    UBLOX_PASO_PVT = 0,
    UBLOX_PASO_PORT,
    UBLOX_PASO_NAV_RATE,
    UBLOX_PASO_SOL,
    UBLOX_PASO_STATUS,
    UBLOX_PASO_POSLLH,
    UBLOX_PASO_VELNED,
    UBLOX_PASO_POLL_SBAS,
    UBLOX_PASO_POLL_NAV,
    UBLOX_PASO_POLL_GNSS,
    UBLOX_PASO_DOP,
    UBLOX_PASO_VERSION,
    UBLOX_PASO_ULTIMO,
} configPasoUBX_e;

typedef enum {
    FIX_NONE               = 0,
    FIX_DEAD_RECKONING     = 1,
    FIX_2D                 = 2,
    FIX_3D                 = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME               = 5,
} navFixTipoUBX_e;

typedef enum {
    NAV_STATUS_FIX_VALID         = 1,
    NAV_STATUS_DGPS_USED         = 2,
    NAV_STATUS_TIME_WEEK_VALID   = 4,
    NAV_STATUS_TIME_SECOND_VALID = 8,
} navStatusBitsUBX_e;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static gpsUblox_t gpsUblox[NUM_MAX_GPS];


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool detectarUblox(gps_t *dGPS, uint8_t dato);
void solicitarPuertoGPSublox(gps_t *dGPS);
bool solicitarFrecuenciaMensajeGPSublox(gps_t *dGPS, uint8_t msgClass, uint8_t msgId);
bool enviarMensajeGPSublox(gps_t *dGPS, uint8_t msgClass, uint8_t msgId, void *msg, uint16_t tam);
void actualizarChecksumGPSublox(uint8_t *dato, uint16_t len, uint8_t *ck_a, uint8_t *ck_b);
bool analizarTramaGPSublox(gps_t *dGPS);
void verificarFrecuenciaMensajeGPSublox(gps_t *dGPS, uint8_t clase, uint8_t id, uint8_t frec);
bool configurarFrecuenciaMensajeGPSublox(gps_t *dGPS, uint8_t clase, uint8_t id, uint8_t frec);
void configurarFrecuenciaNavegacionGPSublox(gps_t *dGPS);
void mensajeInesperadoGPSublox(gps_t *dGPS);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void iniciarGPSublox(gps_t *dGPS)
**  Descripcion:    Inicia el sensor
**  Parametros:     Puntero al sensor
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarGPSublox(gps_t *dGPS)
{
    // Asignamos la posicion en el array de datos del driver especifico
    gpsUblox_t *driver = &gpsUblox[dGPS->numGPS];
    dGPS->driver = driver;

    // Reseteamos el driver
    memset(driver, 0, sizeof(*driver));
    driver->siguienteMensaje = UBLOX_PASO_PVT;
    driver->mensajesNoConfig = CONFIG_TODO_GPS;
    driver->siguienteFix = NO_FIX;
    driver->puertoUblox = 255;

    switch (configGPS(dGPS->numGPS)->tipoGPS) {
        case GPS_UBLOX_NEO_6M:
            // Solo 7 y posterior soporta CONFIG_GNSS_GPS
            driver->mensajesNoConfig &= ~CONFIG_GNSS_GPS;
            break;

        case GPS_UBLOX_NEO_7M:
        case GPS_UBLOX_NEO_M8:
        default:
            break;
    };
}


/***************************************************************************************
**  Nombre:   bool detectarGPSublox(gps_t dGPS, uint8_t data)
**  Función:  Determina si tenemos conectado un GPS ublox
**  Entradas: Puntero al GPS, dato para reconocer el GPS
**  Salidas:  True si ok
***************************************************************************************/
bool detectarGPSublox(gps_t *dGPS, uint8_t dato)
{
  reset:
    switch (dGPS->deteccion.ubloxDeteccion.step) {
        case 1:
            if (PREAMBLE2 == dato) {
            	dGPS->deteccion.ubloxDeteccion.step++;
                break;
            }
            dGPS->deteccion.ubloxDeteccion.step = 0;
            FALLTHROUGH;

        case 0:
            if (PREAMBLE1 == dato)
            	dGPS->deteccion.ubloxDeteccion.step++;
            break;

        case 2:
        	dGPS->deteccion.ubloxDeteccion.step++;
        	dGPS->deteccion.ubloxDeteccion.ck_b = dGPS->deteccion.ubloxDeteccion.ck_a = dato;
            break;

        case 3:
        	dGPS->deteccion.ubloxDeteccion.step++;
        	dGPS->deteccion.ubloxDeteccion.ck_b += (dGPS->deteccion.ubloxDeteccion.ck_a += dato);
            break;

        case 4:
        	dGPS->deteccion.ubloxDeteccion.step++;
        	dGPS->deteccion.ubloxDeteccion.ck_b += (dGPS->deteccion.ubloxDeteccion.ck_a += dato);
        	dGPS->deteccion.ubloxDeteccion.payloadLength = dato;
            break;

        case 5:
        	dGPS->deteccion.ubloxDeteccion.step++;
        	dGPS->deteccion.ubloxDeteccion.ck_b += (dGPS->deteccion.ubloxDeteccion.ck_a += dato);
        	dGPS->deteccion.ubloxDeteccion.payloadCounter = 0;
            break;

        case 6:
        	dGPS->deteccion.ubloxDeteccion.ck_b += (dGPS->deteccion.ubloxDeteccion.ck_a += dato);
            if (++dGPS->deteccion.ubloxDeteccion.payloadCounter == dGPS->deteccion.ubloxDeteccion.payloadLength)
            	dGPS->deteccion.ubloxDeteccion.step++;
            break;

        case 7:
        	dGPS->deteccion.ubloxDeteccion.step++;
            if (dGPS->deteccion.ubloxDeteccion.ck_a != dato) {
            	dGPS->deteccion.ubloxDeteccion.step = 0;
                goto reset;
            }
            break;

        case 8:
        	dGPS->deteccion.ubloxDeteccion.step = 0;
            if (dGPS->deteccion.ubloxDeteccion.ck_b == dato)
                return true;
            else
                goto reset;
    }

    return false;
}


/***************************************************************************************
**  Nombre:   void solcitarSiguienteConfigGPSublox(gps_t *dGPS)
**  Función:  Solicita la siguiente configuracion al GPS
**  Entradas: Puntero al GPS
**  Salidas:  Ninguno
***************************************************************************************/
void solcitarSiguienteConfigGPSublox(gps_t *dGPS)
{
    gpsUblox_t *driver = dGPS->driver;
    // Asegurarse de que haya suficiente espacio para el mayor mensaje de salida posible
    if (bytesLibresBufferTxUART(configGPS(dGPS->numGPS)->dispUART) < (int16_t)(sizeof(headerUBX_t) + sizeof(cfgNavRateUBX_t) + 2))
        return;

    switch (driver->siguienteMensaje) {
        case UBLOX_PASO_PVT:
            if (!solicitarFrecuenciaMensajeGPSublox(dGPS, CLASS_NAV, MSG_PVT))
                driver->siguienteMensaje--;
            driver->siguienteMensaje++;
            break;

        case UBLOX_PASO_PORT:
            solicitarPuertoGPSublox(dGPS);
            driver->siguienteMensaje++;
            break;

        case UBLOX_PASO_POLL_SBAS:
            if (configGPS(dGPS->numGPS)->modoSBAS != SBAS_SIN_CAMBIOS) {
                if (!enviarMensajeGPSublox(dGPS, CLASS_CFG, MSG_CFG_SBAS, NULL, 0))
                    driver->siguienteMensaje--;
            }
            else
                driver->mensajesNoConfig &= ~CONFIG_SBAS_GPS;
            driver->siguienteMensaje++;
            break;

        case UBLOX_PASO_POLL_NAV:
            if (!enviarMensajeGPSublox(dGPS, CLASS_CFG, MSG_CFG_NAV_SETTINGS, NULL, 0))
                driver->siguienteMensaje--;
            driver->siguienteMensaje++;
            break;

        case UBLOX_PASO_POLL_GNSS:
            if (!enviarMensajeGPSublox(dGPS, CLASS_CFG, MSG_CFG_GNSS, NULL, 0))
                driver->siguienteMensaje--;
            driver->siguienteMensaje++;
            break;

        case UBLOX_PASO_NAV_RATE:
            if (!enviarMensajeGPSublox(dGPS, CLASS_CFG, MSG_CFG_RATE, NULL, 0))
                driver->siguienteMensaje--;
            driver->siguienteMensaje++;
            break;

        case UBLOX_PASO_POSLLH:
            if (!solicitarFrecuenciaMensajeGPSublox(dGPS, CLASS_NAV, MSG_POSLLH))
                driver->siguienteMensaje--;
            driver->siguienteMensaje++;
            break;

        case UBLOX_PASO_STATUS:
            if (!solicitarFrecuenciaMensajeGPSublox(dGPS, CLASS_NAV, MSG_STATUS))
                driver->siguienteMensaje--;
            driver->siguienteMensaje++;
            break;

        case UBLOX_PASO_SOL:
            if (!solicitarFrecuenciaMensajeGPSublox(dGPS, CLASS_NAV, MSG_SOL))
                driver->siguienteMensaje--;
            driver->siguienteMensaje++;
            break;

        case UBLOX_PASO_VELNED:
            if (!solicitarFrecuenciaMensajeGPSublox(dGPS, CLASS_NAV, MSG_VELNED))
                driver->siguienteMensaje--;
            driver->siguienteMensaje++;
            break;

        case UBLOX_PASO_DOP:
            if (!solicitarFrecuenciaMensajeGPSublox(dGPS, CLASS_NAV, MSG_DOP))
                driver->siguienteMensaje--;
            driver->siguienteMensaje = UBLOX_PASO_PVT;
            break;

        default:
            // Nunca se debe llegar a este caso, se hace un reinicio completo
            driver->siguienteMensaje = UBLOX_PASO_PVT;
            break;
    }
}


/***************************************************************************************
**  Nombre:         void solicitarPuertoGPSublox(gps_t *dGPS)
**  Descripcion:    Solicita el puerto de conexion del GPS
**  Parametros:     Puntero al GPS
**  Retorno:        Ninguno
****************************************************************************************/
void solicitarPuertoGPSublox(gps_t *dGPS)
{
    if (bytesLibresBufferTxUART(configGPS(dGPS->numGPS)->dispUART) < (int16_t)(sizeof(headerUBX_t) + 2))
        return;

    enviarMensajeGPSublox(dGPS, CLASS_CFG, MSG_CFG_PRT, NULL, 0);
}


/***************************************************************************************
**  Nombre:         bool solicitarFrecuenciaMensajeGPSublox(gps_t *dGPS, uint8_t msgClass, uint8_t msgId)
**  Descripcion:    Solicita la frecuencia de mensaje dada para una clase de mensaje especifica y msgId
**  Parametros:     Clase de mensaje e id
**  Retorno:        Devuelve true si envio la solicitud, falso si espera conocer el puerto
****************************************************************************************/
bool solicitarFrecuenciaMensajeGPSublox(gps_t *dGPS, uint8_t msgClass, uint8_t msgId)
{
	cfgMsgUBX_t msg;

    msg.msgClass = msgClass;
    msg.msgId = msgId;
    return enviarMensajeGPSublox(dGPS, CLASS_CFG, MSG_CFG_MSG, &msg, sizeof(msg));
}


/***************************************************************************************
**  Nombre:         bool enviarMensajeGPSublox(gps_t *dGPS, uint8_t msgClass, uint8_t msgId, void *msg, uint16_t tam)
**  Descripcion:    Envia un mensaje al GPS
**  Parametros:     Puntero al driver, clase de mensaje, id, contenido y tamanio
**  Retorno:        Mensaje enviado
****************************************************************************************/
bool enviarMensajeGPSublox(gps_t *dGPS, uint8_t msgClass, uint8_t msgId, void *msg, uint16_t tam)
{
    if (bytesLibresBufferTxUART(configGPS(dGPS->numGPS)->dispUART) < (sizeof(headerUBX_t) + 2 + tam))
        return false;

    headerUBX_t header;
    uint8_t ck_a = 0, ck_b = 0;
    header.preamble1 = PREAMBLE1;
    header.preamble2 = PREAMBLE2;
    header.msgClass = msgClass;
    header.msgId = msgId;
    header.length = tam;

    actualizarChecksumGPSublox((uint8_t *)&header.msgClass, sizeof(header) - 2, &ck_a, &ck_b);
    actualizarChecksumGPSublox((uint8_t *)msg, tam, &ck_a, &ck_b);

    escribirBufferUART(configGPS(dGPS->numGPS)->dispUART, (uint8_t *)&header, sizeof(header));
    escribirBufferUART(configGPS(dGPS->numGPS)->dispUART, (uint8_t *)msg, tam);
    escribirBufferUART(configGPS(dGPS->numGPS)->dispUART, (uint8_t *)&ck_a, 1);
    escribirBufferUART(configGPS(dGPS->numGPS)->dispUART, (uint8_t *)&ck_b, 1);
    return true;
}


/***************************************************************************************
**  Nombre:         void actualizarChecksumGPSublox(uint8_t *dato, uint16_t tam, uint8_t &ck_a, uint8_t &ck_b)
**  Descripcion:    Actualiza el checksum de la trama
**  Parametros:     Datos, longitud, ck_a, ck_b
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarChecksumGPSublox(uint8_t *dato, uint16_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    while (len--) {
        *ck_a += *dato;
        *ck_b += *ck_a;
        dato++;
    }
}


/***************************************************************************************
**  Nombre:         bool leerGPSublox(gps_t *dGPS)
**  Descripcion:    Lee la trama recibida y actualiza los datos
**  Parametros:     Puntero al sensor
**  Retorno:        Trama leida
****************************************************************************************/
bool leerGPSublox(gps_t *dGPS)
{
    gpsUblox_t *driver = dGPS->driver;
    uint8_t dato;
    uint16_t numc;
    bool analizado = false;

    if (driver->mensajesNoConfig != 0) {
        uint32_t tiempo = millis();

        if (tiempo - driver->ultimoTiempoConfig >= TIEMPO_RETARDO_CONFIG_GPS_UBLOX) {
            solcitarSiguienteConfigGPSublox(dGPS);
            driver->ultimoTiempoConfig = tiempo;
        }
    }

    numc = bytesRecibidosUART(configGPS(dGPS->numGPS)->dispUART);

    for (uint16_t i = 0; i < numc; i++) {                          // Procesa los bytes recibidos
        dato = leerUART(configGPS(dGPS->numGPS)->dispUART);

      reset:
        switch (driver->estadoLectura) {
            /*
            Deteccion del preamble del mensaje

            Si no podemos hacer coincidir cualquiera de los bytes esperados, reiniciamos
            la maquina de estado y reconsidera el byte fallido como el primer byte del preamble.
            Esto mejora nuestras posibilidades de recuperarse de un desajuste y lo hace menos
            probable a que nos dejemos engañar por el pramble que aparece como datos en algun otro mensaje.
            */
            case 1:
                if (PREAMBLE2 == dato) {
                    driver->estadoLectura++;
                    break;
                }
                else
                    driver->estadoLectura = 0;
                FALLTHROUGH;

            case 0:
                if (PREAMBLE1 == dato)
                    driver->estadoLectura++;
                break;
            /*
            Procesamiento del encabezado del mensaje

            Detectamos la clase y el ID del mensaje para decidir si vamos a juntar los bytes del mensaje o simplemente descartarlos
            Siempre recolectamos la longitud para que podamos evitar ser engañados por los bytes del preamble en los mensajes.
            */
            case 2:
                driver->estadoLectura++;
                driver->claseMensaje = dato;
                driver->ckbMensaje = driver->ckaMensaje = dato;             // Resetea los acumuladores del checksum
                break;

            case 3:
                driver->estadoLectura++;
                driver->ckbMensaje += (driver->ckaMensaje += dato);         // Checksum byte
                driver->idMensaje = dato;
                break;

            case 4:
                driver->estadoLectura++;
                driver->ckbMensaje += (driver->ckaMensaje += dato);         // Checksum byte
                driver->longitudPayload = dato;                             // Byte bajo de longitud del payload
                break;

            case 5:
                driver->estadoLectura++;
                driver->ckbMensaje += (driver->ckaMensaje += dato);         // Checksum byte
                driver->longitudPayload += (uint16_t)(dato << 8);
                if (driver->longitudPayload > TAM_BUFFER_RECEPCION_GPS_UBLOX) {
                    driver->longitudPayload = 0;
                    driver->estadoLectura = 0;
                    goto reset;
                }
                if (driver->longitudPayload == 0)
                    driver->estadoLectura = 7;
                driver->contadorPayload = 0;                                // Prepara para recibir el payload
                break;

            // Recibe los datos del mensaje
            case 6:
                driver->ckbMensaje += (driver->ckaMensaje += dato);         // checksum byte
                if (driver->contadorPayload < TAM_BUFFER_RECEPCION_GPS_UBLOX)
                    driver->bufferRecepcion.buffer[driver->contadorPayload] = dato;

                driver->contadorPayload++;
                if (driver->contadorPayload == driver->longitudPayload)
                    driver->estadoLectura++;
                break;

            // Procesamiento del checksum y el mensaje
            case 7:
                driver->estadoLectura++;
                if (driver->ckaMensaje != dato) {
                    driver->estadoLectura = 0;
                    goto reset;
                }
                break;

            case 8:
                driver->estadoLectura = 0;
                if (driver->ckbMensaje != dato)
                    break;                                                  // Checksum incorrecto

                if (analizarTramaGPSublox(dGPS))
                    analizado = true;
                break;
        }
    }
    return analizado;
}


/***************************************************************************************
**  Nombre:         bool analizarTramaGPSublox(gps_t *dGPS)
**  Descripcion:    Analiza el mensaje recibido
**  Parametros:     Puntero al sensor
**  Retorno:        Trama analizada
****************************************************************************************/
bool analizarTramaGPSublox(gps_t *dGPS)
{
    gpsUblox_t *driver = dGPS->driver;

    if (driver->claseMensaje == CLASS_ACK) {
        if (driver->idMensaje == MSG_ACK_ACK) {
            switch (driver->bufferRecepcion.ack.clsID) {
                case CLASS_CFG:
                    switch (driver->bufferRecepcion.ack.msgID) {
                        case MSG_CFG_CFG:
                            driver->cfgGuardada = true;
                            driver->cfgNecesitaGuardar = false;
                            break;

                        case MSG_CFG_GNSS:
                            driver->mensajesNoConfig &= ~CONFIG_GNSS_GPS;
                            break;

                        case MSG_CFG_MSG:
                            /*
                            No hay forma de saber que configuración de MSG fue ack, suponemos que fue la ultima solicitada.
                            Para verificar que vuelva a solicitar la ultima configuración que enviamos.
                            Si nos olvidamos el ack lo atraparemos la proxima vez a traves del ciclo pool, pero eso
                            sera un buen pedazo de tiempo despues.
                            */
                            break;

                        case MSG_CFG_NAV_SETTINGS:
                            driver->mensajesNoConfig &= ~CONFIG_NAV_SETTINGS_GPS;
                            break;

                        case MSG_CFG_RATE:
                            /*
                            El GPS enviara ACK por una tasa de actualizacion que no es valida. para detectar esto solo acepta la
                            tasa configurada leyendo las configuraciones y validando que todos coinciden con los valores objetivo
                            */
                            break;

                        case MSG_CFG_SBAS:
                            driver->mensajesNoConfig &= ~CONFIG_SBAS_GPS;
                            break;

                        case MSG_CFG_TP5:
                            break;
                    }
                    break;
            }
        }
        return false;
    }

    if (driver->claseMensaje == CLASS_CFG) {
        switch (driver->idMensaje) {
            case MSG_CFG_NAV_SETTINGS:
                driver->bufferRecepcion.navSettings.mask = 0;
                if (configGPS(dGPS->numGPS)->modoConf != GPS_ENGINE_NINGUNO && driver->bufferRecepcion.navSettings.dynModel != configGPS(dGPS->numGPS)->modoConf) {
                    // Recibimos la configuracion de navegación actual, cambiamos el modelo de configuracion y lo enviamos de vuelta
                    driver->bufferRecepcion.navSettings.dynModel = configGPS(dGPS->numGPS)->modoConf;
                    driver->bufferRecepcion.navSettings.mask |= 1;
                }
                if (configGPS(dGPS->numGPS)->elevacionMin != -100 && driver->bufferRecepcion.navSettings.minElev != configGPS(dGPS->numGPS)->elevacionMin) {
                    driver->bufferRecepcion.navSettings.minElev = configGPS(dGPS->numGPS)->elevacionMin;
                    driver->bufferRecepcion.navSettings.mask |= 2;
                }
                if (driver->bufferRecepcion.navSettings.mask != 0) {
                    enviarMensajeGPSublox(dGPS, CLASS_CFG, MSG_CFG_NAV_SETTINGS, &driver->bufferRecepcion.navSettings, sizeof(driver->bufferRecepcion.navSettings));
                    driver->mensajesNoConfig |= CONFIG_NAV_SETTINGS_GPS;
                    driver->cfgNecesitaGuardar = true;
                }
                else
                    driver->mensajesNoConfig &= ~CONFIG_NAV_SETTINGS_GPS;
                return false;

            case MSG_CFG_GNSS:
                if (configGPS(dGPS->numGPS)->gnss != 0) {
                    cfgGNSSubx_t gnssInicial = driver->bufferRecepcion.gnss;
                    uint8_t gnssCnt = 0;

                    for (uint8_t i = 0; i < NUM_MAX_GNSS_CONFIG_BLOCKS_GPS_UBLOX; i++) {
                        if ((configGPS(dGPS->numGPS)->gnss & (1 << i)) && i != GNSS_SBAS)
                            gnssCnt++;
                    }

                    for (uint8_t i = 0; i < driver->bufferRecepcion.gnss.numConfigBlocks; i++) {
                        // Reserva una porcion igual de canales para todos los sistemas habilitados
                        if (configGPS(dGPS->numGPS)->gnss & (1 << driver->bufferRecepcion.gnss.configBlock[i].gnssId)) {
                            if (GNSS_SBAS != driver->bufferRecepcion.gnss.configBlock[i].gnssId) {
                                driver->bufferRecepcion.gnss.configBlock[i].resTrkCh = (driver->bufferRecepcion.gnss.numTrkChHw - 3) / (gnssCnt * 2);
                                driver->bufferRecepcion.gnss.configBlock[i].maxTrkCh = driver->bufferRecepcion.gnss.numTrkChHw;
                            }
                            else {
                                driver->bufferRecepcion.gnss.configBlock[i].resTrkCh = 1;
                                driver->bufferRecepcion.gnss.configBlock[i].maxTrkCh = 3;
                            }
                            driver->bufferRecepcion.gnss.configBlock[i].flags = driver->bufferRecepcion.gnss.configBlock[i].flags | 0x00000001;
                        }
                        else {
                            driver->bufferRecepcion.gnss.configBlock[i].resTrkCh = 0;
                            driver->bufferRecepcion.gnss.configBlock[i].maxTrkCh = 0;
                            driver->bufferRecepcion.gnss.configBlock[i].flags = driver->bufferRecepcion.gnss.configBlock[i].flags & 0xFFFFFFFE;
                        }
                    }
                    if (!memcmp(&gnssInicial, &driver->bufferRecepcion.gnss, sizeof(gnssInicial))) {
                        enviarMensajeGPSublox(dGPS, CLASS_CFG, MSG_CFG_GNSS, &driver->bufferRecepcion.gnss, 4 + (8 * driver->bufferRecepcion.gnss.numConfigBlocks));
                        driver->mensajesNoConfig |= CONFIG_GNSS_GPS;
                        driver->cfgNecesitaGuardar = true;
                    }
                    else
                        driver->mensajesNoConfig &= ~CONFIG_GNSS_GPS;
                }
                else
                    driver->mensajesNoConfig &= ~CONFIG_GNSS_GPS;
                return false;

            case MSG_CFG_SBAS:
                if (configGPS(dGPS->numGPS)->modoSBAS != SBAS_SIN_CAMBIOS) {
                    if (driver->bufferRecepcion.sbas.mode != configGPS(dGPS->numGPS)->modoSBAS) {
                        driver->bufferRecepcion.sbas.mode = configGPS(dGPS->numGPS)->modoSBAS;
                        enviarMensajeGPSublox(dGPS, CLASS_CFG, MSG_CFG_SBAS, &driver->bufferRecepcion.sbas, sizeof(driver->bufferRecepcion.sbas));
                        driver->mensajesNoConfig |= CONFIG_SBAS_GPS;
                        driver->cfgNecesitaGuardar = true;
                    }
                    else
                        driver->mensajesNoConfig &= ~CONFIG_SBAS_GPS;
                }
                else
                    driver->mensajesNoConfig &= ~CONFIG_SBAS_GPS;
                return false;

            case MSG_CFG_MSG:
                if (driver->longitudPayload == sizeof(cfgMsgRate6UBX_t)) {
                    if (driver->puertoUblox >= NUM_MAX_PUERTOS_GPS_UBLOX) {
                        solicitarPuertoGPSublox(dGPS);
                        return false;
                    }
                    verificarFrecuenciaMensajeGPSublox(dGPS, driver->bufferRecepcion.msgRate6.msgClass, driver->bufferRecepcion.msgRate6.msgId, driver->bufferRecepcion.msgRate6.rates[driver->puertoUblox]);
                }
                else
                    verificarFrecuenciaMensajeGPSublox(dGPS, driver->bufferRecepcion.msgRate.msgClass, driver->bufferRecepcion.msgRate.msgId, driver->bufferRecepcion.msgRate.rate);
                return false;

            case MSG_CFG_PRT:
                driver->puertoUblox = driver->bufferRecepcion.prt.portID;

            case MSG_CFG_RATE:
                if (driver->bufferRecepcion.navRate.measureRateMs != configGPS(dGPS->numGPS)->periodoMuestreo || driver->bufferRecepcion.navRate.navRate != 1 || driver->bufferRecepcion.navRate.timeRef != 0) {
                    configurarFrecuenciaNavegacionGPSublox(dGPS);
                    driver->mensajesNoConfig |= CONFIG_RATE_NAV_GPS;
                    driver->cfgNecesitaGuardar = true;
                }
                else
                    driver->mensajesNoConfig &= ~CONFIG_RATE_NAV_GPS;

                return false;
        }
    }

    if (driver->claseMensaje != CLASS_NAV) {
        mensajeInesperadoGPSublox(dGPS);
        return false;
    }

    switch (driver->idMensaje) {
        case MSG_POSLLH:
            if (driver->tienePVTmsg) {
                driver->mensajesNoConfig |= CONFIG_RATE_POSLLH_GPS;
                break;
            }
            driver->ultimoTiempoPos = driver->bufferRecepcion.posllh.itow;
            dGPS->localizacion.longitud = driver->bufferRecepcion.posllh.longitude;
            dGPS->localizacion.latitud = driver->bufferRecepcion.posllh.latitude;
            dGPS->localizacion.altitud = driver->bufferRecepcion.posllh.altitudeMsl / 10;
            dGPS->estado.status = driver->siguienteFix;
            driver->nuevaPosicion = true;
            dGPS->estado.precisionHorizontal = driver->bufferRecepcion.posllh.horizontalAccuracy * 1.0e-3f;
            dGPS->estado.precisionVertical = driver->bufferRecepcion.posllh.verticalAccuracy * 1.0e-3f;
            dGPS->estado.tienePrecisionHorizontal = true;
            dGPS->estado.tienePrecisionVertical = true;
            break;

        case MSG_STATUS:
            if (driver->tienePVTmsg) {
                driver->mensajesNoConfig |= CONFIG_RATE_STATUS_GPS;
                break;
            }
            if (driver->bufferRecepcion.status.fixStatus & NAV_STATUS_FIX_VALID) {
                if ((driver->bufferRecepcion.status.fixType == FIX_3D) && (driver->bufferRecepcion.status.fixStatus & NAV_STATUS_DGPS_USED))
                    driver->siguienteFix = GPS_OK_FIX_3D_DGPS;
                else if (driver->bufferRecepcion.status.fixType == FIX_3D)
                    driver->siguienteFix = GPS_OK_FIX_3D;
                else if (driver->bufferRecepcion.status.fixType == FIX_2D)
                    driver->siguienteFix = GPS_OK_FIX_2D;
                else {
                    driver->siguienteFix = NO_FIX;
                    dGPS->estado.status = NO_FIX;
                }
            }
            else {
                driver->siguienteFix = NO_FIX;
                dGPS->estado.status = NO_FIX;
            }
            break;

        case MSG_DOP:
            driver->hdopNoRecibido = false;
            dGPS->estado.hdop = driver->bufferRecepcion.dop.hDOP;
            dGPS->estado.vdop = driver->bufferRecepcion.dop.vDOP;
            break;

        case MSG_SOL:
            if (driver->tienePVTmsg) {
            	dGPS->estado.numSemana = driver->bufferRecepcion.solution.week;
                break;
            }
            if (driver->bufferRecepcion.solution.fixStatus & NAV_STATUS_FIX_VALID) {
                if ((driver->bufferRecepcion.solution.fixType == FIX_3D) && (driver->bufferRecepcion.solution.fixStatus & NAV_STATUS_DGPS_USED))
                    driver->siguienteFix = GPS_OK_FIX_3D_DGPS;
                else if (driver->bufferRecepcion.solution.fixType == FIX_3D)
                    driver->siguienteFix = GPS_OK_FIX_3D;
                else if (driver->bufferRecepcion.solution.fixType == FIX_2D)
                    driver->siguienteFix = GPS_OK_FIX_2D;
                else {
                    driver->siguienteFix = NO_FIX;
                    dGPS->estado.status = NO_FIX;
                }
            }
            else {
                driver->siguienteFix = NO_FIX;
                dGPS->estado.status = NO_FIX;
            }
            if (driver->hdopNoRecibido)
            	dGPS->estado.hdop = driver->bufferRecepcion.solution.positionDOP;

            dGPS->estado.numSats = driver->bufferRecepcion.solution.satellites;
            if (driver->siguienteFix >= GPS_OK_FIX_2D) {
            	dGPS->estado.ultimaHoraGPSms = millis();
            	dGPS->estado.horaSemana = driver->bufferRecepcion.solution.itow;
            	dGPS->estado.numSemana = driver->bufferRecepcion.solution.week;
#ifdef USAR_RTC
                // Ajustamos el RTC
                if (!tieneHoraRTC() && (driver->bufferRecepcion.solution.fixStatus & NAV_STATUS_TIME_SECOND_VALID) && (driver->bufferRecepcion.solution.fixStatus & NAV_STATUS_TIME_WEEK_VALID)) {
                    //Calculamos el tiempo Unix: numero de semana * ms en una semana + ms de la semana + fracciones de segundo + offset del UNIX - 18 segundos de desfase entre tiempo Unix y el GPS
                    int64_t tiempoUnix = (((int64_t) driver->bufferRecepcion.solution.week) * 7 * 24 * 60 * 60 * 1000) + driver->bufferRecepcion.solution.itow + (driver->bufferRecepcion.solution.timeNsec / 1000000) + 315964800000LL - configRTC()->offsetGPSutc * 1000;
                    ajustarUnixRTC(tiempoUnix);
                }
#endif
            }
            break;

        case MSG_PVT:
            driver->tienePVTmsg = true;

            // Posicion
            driver->ultimoTiempoPos = driver->bufferRecepcion.pvt.itow;
            dGPS->localizacion.longitud = driver->bufferRecepcion.pvt.lon;
            dGPS->localizacion.latitud = driver->bufferRecepcion.pvt.lat;
            dGPS->localizacion.altitud = driver->bufferRecepcion.pvt.h_msl / 10;
            switch (driver->bufferRecepcion.pvt.fixType) {
                case 0:
                	dGPS->estado.status = NO_FIX;
                    break;

                case 1:
                	dGPS->estado.status = NO_FIX;
                    break;

                case 2:
                	dGPS->estado.status = GPS_OK_FIX_2D;
                    break;

                case 3:
                	dGPS->estado.status = GPS_OK_FIX_3D;
                    if (driver->bufferRecepcion.pvt.flags & 0b00000010)  // Se han aplicado correcciones diferenciales
                    	dGPS->estado.status = GPS_OK_FIX_3D_DGPS;
                    if (driver->bufferRecepcion.pvt.flags & 0b01000000)  // carrsoln - float
                    	dGPS->estado.status = GPS_OK_FIX_3D_RTK_FLOAT;
                    if (driver->bufferRecepcion.pvt.flags & 0b10000000)  // carrsoln - fixed
                    	dGPS->estado.status = GPS_OK_FIX_3D_RTK_FIXED;
                    break;

                case 4:
                	dGPS->estado.status = GPS_OK_FIX_3D;
                    break;

                case 5:
                	dGPS->estado.status = NO_FIX;
                    break;

                default:
                	dGPS->estado.status = NO_FIX;
                    break;
            }
            driver->siguienteFix = dGPS->estado.status;
            driver->nuevaPosicion = true;
            dGPS->estado.precisionHorizontal = driver->bufferRecepcion.pvt.hAcc * 1.0e-3f;
            dGPS->estado.precisionVertical = driver->bufferRecepcion.pvt.vAcc * 1.0e-3f;
            dGPS->estado.tienePrecisionHorizontal = true;
            dGPS->estado.tienePrecisionVertical = true;

            // Satelites
            dGPS->estado.numSats = driver->bufferRecepcion.pvt.numSv;

            // Velocidad
            driver->ultimoTiempoVel = driver->bufferRecepcion.pvt.itow;
            dGPS->vel2d = driver->bufferRecepcion.pvt.gspeed * 0.001f;                          // m/s
            dGPS->velAngular = envolverInt360(driver->bufferRecepcion.pvt.head_mot * 1.0e-5f, 1);  // Heading 2D deg * 100000
            dGPS->estado.tieneVelVertical = true;
            dGPS->velocidad.norte = driver->bufferRecepcion.pvt.velN * 0.001f;
            dGPS->velocidad.este = driver->bufferRecepcion.pvt.velE * 0.001f;
            dGPS->velocidad.vertical = driver->bufferRecepcion.pvt.velD * 0.001f;
            dGPS->estado.tienePrecisionVel = true;
            dGPS->estado.precisionVel = driver->bufferRecepcion.pvt.sAcc * 0.001f;
            driver->nuevaVelocidad = true;

            // DOP
            if (driver->hdopNoRecibido) {
            	dGPS->estado.hdop = driver->bufferRecepcion.pvt.pDop;
            	dGPS->estado.vdop = driver->bufferRecepcion.pvt.pDop;
            }
            dGPS->estado.ultimaHoraGPSms = millis();

            // Tiempo
            dGPS->estado.horaSemana = driver->bufferRecepcion.pvt.itow;
            break;

        case MSG_VELNED:
            if (driver->tienePVTmsg) {
                driver->mensajesNoConfig |= CONFIG_RATE_VELNED_GPS;
                break;
            }
            driver->ultimoTiempoVel = driver->bufferRecepcion.velned.itow;
            dGPS->vel2d = driver->bufferRecepcion.velned.speed2d * 0.01f;                       // m/s
            dGPS->velAngular = envolverInt360(driver->bufferRecepcion.velned.heading2d * 1.0e-5f, 1);    // Heading 2D deg * 100000
            dGPS->estado.tieneVelVertical = true;
            dGPS->velocidad.norte = driver->bufferRecepcion.velned.nedNorth * 0.01f;
            dGPS->velocidad.este = driver->bufferRecepcion.velned.nedEast * 0.01f;
            dGPS->velocidad.vertical = driver->bufferRecepcion.velned.nedDown * 0.01f;
            dGPS->velAngular = envolverInt360(grados(atan2f(dGPS->velocidad.este, dGPS->velocidad.norte)), 1);
            dGPS->vel2d = sqrtf(powf((float)dGPS->velocidad.este, 2) + powf((float)dGPS->velocidad.norte, 2));
            dGPS->estado.tienePrecisionVel = true;
            dGPS->estado.precisionVel = driver->bufferRecepcion.velned.speedAccuracy * 0.01f;
            driver->nuevaVelocidad = true;
            break;

        default:
            if (++driver->contadorDeshabilitacion == 0)
                configurarFrecuenciaMensajeGPSublox(dGPS, CLASS_NAV, driver->idMensaje, 0);
            return false;
    }

    // solo devolvemos true cuando obtenemos nuevos datos de posición y velocidad
    if (driver->nuevaPosicion && driver->nuevaVelocidad && driver->ultimoTiempoVel == driver->ultimoTiempoPos) {
        driver->nuevaVelocidad = driver->nuevaPosicion = false;
        return true;
    }
    return false;
}


/***************************************************************************************
**  Nombre:         void verificarFrecuenciaMensajeGPSublox(gps_t *dGPS, uint8_t clase, uint8_t id, uint8_t frec)
**  Descripcion:    Verifica la frecuencia de envio del mensaje y si es incorrecta la corrige
**  Parametros:     Puntero al GPS, clase de mensaje, id, frecuencia
**  Retorno:        Ninguno
****************************************************************************************/
void verificarFrecuenciaMensajeGPSublox(gps_t *dGPS, uint8_t clase, uint8_t id, uint8_t frec)
{
    gpsUblox_t *driver = dGPS->driver;
    uint8_t frecuenciaDeseada;

    switch (clase) {
        case CLASS_NAV:
            switch (id) {
                case MSG_POSLLH:
                    frecuenciaDeseada = driver->tienePVTmsg ? 0 : UBX_RATE_POSLLH;
                    if (frec == frecuenciaDeseada)
                        driver->mensajesNoConfig &= ~CONFIG_RATE_POSLLH_GPS;
                    else {
                        configurarFrecuenciaMensajeGPSublox(dGPS, clase, id, frecuenciaDeseada);
                        driver->mensajesNoConfig |= CONFIG_RATE_POSLLH_GPS;
                        driver->cfgNecesitaGuardar = true;
                    }
                    break;

                case MSG_STATUS:
                    frecuenciaDeseada = driver->tienePVTmsg ? 0 : UBX_RATE_STATUS;
                    if (frec == frecuenciaDeseada)
                        driver->mensajesNoConfig &= ~CONFIG_RATE_STATUS_GPS;
                    else {
                        configurarFrecuenciaMensajeGPSublox(dGPS, clase, id, frecuenciaDeseada);
                        driver->mensajesNoConfig |= CONFIG_RATE_STATUS_GPS;
                        driver->cfgNecesitaGuardar = true;
                    }
                    break;

                case MSG_SOL:
                    if (frec == UBX_RATE_SOL)
                        driver->mensajesNoConfig &= ~CONFIG_RATE_SOL_GPS;
                    else {
                        configurarFrecuenciaMensajeGPSublox(dGPS, clase, id, UBX_RATE_SOL);
                        driver->mensajesNoConfig |= CONFIG_RATE_SOL_GPS;
                        driver->cfgNecesitaGuardar = true;
                    }
                    break;

                case MSG_PVT:
                    if (frec == UBX_RATE_PVT)
                        driver->mensajesNoConfig &= ~CONFIG_RATE_PVT_GPS;
                    else {
                        configurarFrecuenciaMensajeGPSublox(dGPS, clase, id, UBX_RATE_PVT);
                        driver->mensajesNoConfig |= CONFIG_RATE_PVT_GPS;
                        driver->cfgNecesitaGuardar = true;
                    }
                    break;

                case MSG_VELNED:
                    frecuenciaDeseada = driver->tienePVTmsg ? 0 : UBX_RATE_VELNED;
                    if (frec == frecuenciaDeseada)
                        driver->mensajesNoConfig &= ~CONFIG_RATE_VELNED_GPS;
                    else {
                        configurarFrecuenciaMensajeGPSublox(dGPS, clase, id, frecuenciaDeseada);
                        driver->mensajesNoConfig |= CONFIG_RATE_VELNED_GPS;
                        driver->cfgNecesitaGuardar = true;
                    }
                    break;

                case MSG_DOP:
                    if (frec == UBX_RATE_DOP)
                        driver->mensajesNoConfig &= ~CONFIG_RATE_DOP_GPS;
                    else {
                        configurarFrecuenciaMensajeGPSublox(dGPS, clase, id, UBX_RATE_DOP);
                        driver->mensajesNoConfig |= CONFIG_RATE_DOP_GPS;
                        driver->cfgNecesitaGuardar = true;
                    }
                    break;
            }
            break;
    }
}


/***************************************************************************************
**  Nombre:         bool configurarFrecuenciaMensajeGPSublox(gps_t *dGPS, uint8_t clase, uint8_t id, uint8_t frec)
**  Descripcion:    Configura la frecuencia de envio del mensaje
**  Parametros:     Puntero al GPS, clase de mensaje, id, frecuencia
**  Retorno:        Mensaje enviado
****************************************************************************************/
bool configurarFrecuenciaMensajeGPSublox(gps_t *dGPS, uint8_t clase, uint8_t id, uint8_t frec)
{
    if (bytesLibresBufferTxUART(configGPS(dGPS->numGPS)->dispUART) < (int16_t)(sizeof(headerUBX_t) + sizeof(cfgMsgRateUBX_t) + 2))
        return false;

    cfgMsgRateUBX_t msg;
    msg.msgClass = clase;
    msg.msgId = id;
    msg.rate = frec;
    return enviarMensajeGPSublox(dGPS, CLASS_CFG, MSG_CFG_MSG, &msg, sizeof(msg));
}


/***************************************************************************************
**  Nombre:         void configurarFrecuenciaNavegacionGPSublox(gps_t *dGPS)
**  Descripcion:    Configura la frecuencia de navegacion del GPS
**  Parametros:     Puntero al GPS
**  Retorno:        Ninguno
****************************************************************************************/
void configurarFrecuenciaNavegacionGPSublox(gps_t *dGPS)
{
	cfgNavRateUBX_t msg;

    msg.measureRateMs = configGPS(dGPS->numGPS)->periodoMuestreo;
    msg.navRate = 1;
    msg.timeRef = 0;
    enviarMensajeGPSublox(dGPS, CLASS_CFG, MSG_CFG_RATE, &msg, sizeof(msg));
}


/***************************************************************************************
**  Nombre:         void mensajeInesperadoGPSublox(gps_t *dGPS)
**  Descripcion:    Deshabilita el envio del mensaje si el contador llega a 256
**  Parametros:     Puntero al GPS
**  Retorno:        Ninguno
****************************************************************************************/
void mensajeInesperadoGPSublox(gps_t *dGPS)
{
    gpsUblox_t *driver = dGPS->driver;

    // Deshabilitar futuros envios de este mensaje, pero solo se hace esto cada 256 mensajes porque algunos de
    // los tipos de mensajes no pueden ser deshabilitados y no queremos entrar en una guerra de ack
    if (++driver->contadorDeshabilitacion == 0)
        configurarFrecuenciaMensajeGPSublox(dGPS, driver->claseMensaje, driver->idMensaje, 0);
}

#endif
