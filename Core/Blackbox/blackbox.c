/***************************************************************************************
**  blackbox.c - Funciones de gestion de la blackbox
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 14/05/2020
**  Fecha de modificacion: 21/09/2020
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

#include "blackbox.h"

#ifdef USAR_BLACKBOX
#include "blackbox_sd.h"
#include "GP/gp_blackbox.h"
#include "Drivers/tiempo.h"
#include "Drivers/rtc.h"
#include "Version/version.h"
#include "Radio/radio.h"
#include "Sensores/Barometro/barometro.h"
#include "Sensores/Magnetometro/magnetometro.h"
#include "Sensores/IMU/imu.h"
#include "Sensores/GPS/gps.h"
#include "Comun/util.h"
#include "sd.h"
#include "asyncfatfs/asyncfatfs.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define TIMEOUT_APAGAR_MS_BLACKBOX   200


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    BLACKBOX_ESTADO_DESHABILITADO = 0,
    BLACKBOX_ESTADO_PARADO,
    BLACKBOX_ESTADO_PREPARAR_FICHERO_LOG,
    BLACKBOX_ESTADO_ENVIAR_CABECERA,
    BLACKBOX_ESTADO_ENVIAR_CABECERA_CAMPO,
    BLACKBOX_ESTADO_ENVIAR_CABECERA_LENTA,
    BLACKBOX_ESTADO_ENVIAR_INFO_SISTEMA,
	BLACKBOX_ESTADO_FLUSH_CACHE,
    BLACKBOX_ESTADO_PAUSADO,
    BLACKBOX_ESTADO_CORRIENDO,
    BLACKBOX_ESTADO_APAGANDO,
} estadoBlackbox_e;

typedef enum {
    BLACKBOX_1_DRIVER = 1,
    BLACKBOX_NUM_DRIVERS_IMU,
    BLACKBOX_NUM_DRIVERS_BARO,
    BLACKBOX_NUM_DRIVERS_MAG,
    BLACKBOX_NUM_DRIVERS_GPS,
} numDriversBlackbox_e;

typedef struct {
    const char *nombre;
    int8_t indiceNombreCampo;
    numDriversBlackbox_e numDrivers;
} defCabCampoBlackbox_t;

typedef struct {
    uint32_t indiceCabecera;
    uint8_t indiceCampo;
    uint32_t tiempoInicio;
} datosTXblackbox_t;

typedef struct {
    bool arrancar;
    bool pausar;
    estadoBlackbox_e estado;
    uint16_t intervaloRapido;
    uint16_t intervaloLento;
    bool logEmpezado;
} blackbox_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static blackbox_t blackbox;
static datosTXblackbox_t datosTXblackbox;
static uint32_t iteradorBlackbox;
static int32_t iteradorRapidoBlackbox = 0;
static int32_t iteradorLentoBlackbox = 0;

static const char cabeceraBlackbox[] =
    "C Bienvenido al grabador de datos URpilot\n"
    "C Version Blackbox: 1\n";

static const char* const nombresCabCampoBlackbox[] = {
    "nombre",
    "drivers",
};

static const defCabCampoBlackbox_t camposRapidosBlackbox[] = {
    {"iteracion", -1,    BLACKBOX_1_DRIVER},
    {"tiempo",    -1,    BLACKBOX_1_DRIVER},
#ifdef USAR_IMU
    {"giro",       0,    BLACKBOX_NUM_DRIVERS_IMU},
    {"giro",       1,    BLACKBOX_NUM_DRIVERS_IMU},
    {"giro",       2,    BLACKBOX_NUM_DRIVERS_IMU},
    {"acel",       0,    BLACKBOX_NUM_DRIVERS_IMU},
    {"acel",       1,    BLACKBOX_NUM_DRIVERS_IMU},
    {"acel",       2,    BLACKBOX_NUM_DRIVERS_IMU},
#endif
#ifdef USAR_MAG
    {"mag",        0,    BLACKBOX_NUM_DRIVERS_MAG},
    {"mag",        1,    BLACKBOX_NUM_DRIVERS_MAG},
    {"mag",        2,    BLACKBOX_NUM_DRIVERS_MAG},
#endif
#ifdef USAR_BARO
    {"BaroP",      -1,   BLACKBOX_NUM_DRIVERS_BARO},
    {"BaroT",      -1,   BLACKBOX_NUM_DRIVERS_BARO},
#endif
#ifdef USAR_RADIO
    {"radio",      0,    BLACKBOX_1_DRIVER},
    {"radio",      1,    BLACKBOX_1_DRIVER},
    {"radio",      2,    BLACKBOX_1_DRIVER},
    {"radio",      3,    BLACKBOX_1_DRIVER},
    {"radio",      4,    BLACKBOX_1_DRIVER},
    {"radio",      5,    BLACKBOX_1_DRIVER},
    {"radio",      6,    BLACKBOX_1_DRIVER},
    {"radio",      7,    BLACKBOX_1_DRIVER},
#endif
#ifdef USAR_MOTORES
    {"motor",      0,    BLACKBOX_1_DRIVER},
    {"motor",      1,    BLACKBOX_1_DRIVER},
    {"motor",      2,    BLACKBOX_1_DRIVER},
    {"motor",      3,    BLACKBOX_1_DRIVER},
    {"motor",      4,    BLACKBOX_1_DRIVER},
    {"motor",      5,    BLACKBOX_1_DRIVER},
    {"motor",      6,    BLACKBOX_1_DRIVER},
    {"motor",      7,    BLACKBOX_1_DRIVER},
#endif
};

static const defCabCampoBlackbox_t camposLentosBlackbox[] = {
#ifdef USAR_GPS
    {"satelites", -1,    BLACKBOX_NUM_DRIVERS_GPS},
    {"latitud",   -1,    BLACKBOX_NUM_DRIVERS_GPS},
    {"longitud",  -1,    BLACKBOX_NUM_DRIVERS_GPS},
    {"altGPS",    -1,    BLACKBOX_NUM_DRIVERS_GPS},
    {"velGPS",    -1,    BLACKBOX_NUM_DRIVERS_GPS},
    {"velAngGPS", -1,    BLACKBOX_NUM_DRIVERS_GPS},
#endif
};


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void ajustarEstadoBlackbox(estadoBlackbox_e estado);
void resetearIteradoresBlackbox(void);
void lanzarBlackbox(void);
void actualizarIteradoresBlackbox(void);
bool enviarDefCampoBlackbox(char identificador, const void *defCampo, const void *segundaDefcampo, uint8_t numCampos, const uint8_t *numDrivers, const uint8_t *segundoNumDrivers);
bool escribirInfoSistemaBlackbox(void);
char *obtenerFechaHoraBlackbox(char *buf);
uint8_t numDriversBlackbox(numDriversBlackbox_e numDrivers);
void iterarLogBlackbox(uint32_t tiempoActual);
bool necesarioEscribirLogRapidoBlackbox(void);
bool necesarioEscribirLogLentoBlackbox(void);
void escribirLogRapidoBlackbox(uint32_t tiempoActual);
void escribirLogLentoBlackbox(void);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void iniciarBlackbox(void)
**  Descripcion:    Inicia la blackbox
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void iniciarBlackbox(void)
{
	iniciarSD();
	afatfs_init();

    // Resetea los iteradores encargados de ver cuando escribir en la blackbox
	resetearIteradoresBlackbox();

    // Se calculan los intervalos de ciclo para hacer un log
    blackbox.intervaloRapido = (uint16_t)(configBlackbox()->ratio * 1000);
    blackbox.intervaloLento  = (uint16_t)(configBlackbox()->ratioLento * 1000);

    blackbox.arrancar = false;
    blackbox.pausar = false;
    ajustarEstadoBlackbox(BLACKBOX_ESTADO_PARADO);

    arrancarBlackbox();
}


/***************************************************************************************
**  Nombre:         void finalizarBlackbox(void)
**  Descripcion:    Finaliza la blackbox
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void finalizarBlackbox(void)
{
    switch (blackbox.estado) {
        case BLACKBOX_ESTADO_DESHABILITADO:
        case BLACKBOX_ESTADO_PARADO:
        case BLACKBOX_ESTADO_APAGANDO:
            break;

        case BLACKBOX_ESTADO_CORRIENDO:
        case BLACKBOX_ESTADO_PAUSADO:
        	escribirLogEventoBlackbox(BLACKBOX_LOG_EVENTO_LOG_FIN, NULL);
            FALLTHROUGH;

        default:
        	ajustarEstadoBlackbox(BLACKBOX_ESTADO_APAGANDO);
    }
}


/***************************************************************************************
**  Nombre:         void arrancarBlackbox(void)
**  Descripcion:    Arranca la blackbox
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void arrancarBlackbox(void)
{
    blackbox.arrancar = true;
}


/***************************************************************************************
**  Nombre:         void pausarBlackbox(void)
**  Descripcion:    Pausa la blackbox
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void pausarBlackbox(void)
{
    blackbox.pausar = true;
}


/***************************************************************************************
**  Nombre:         void resetearIteradoresBlackbox(void)
**  Descripcion:    Resetea los iteradores de la blackbox
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void resetearIteradoresBlackbox(void)
{
	iteradorBlackbox = 0;
	iteradorRapidoBlackbox = 0;
	iteradorLentoBlackbox = 0;
}


/***************************************************************************************
**  Nombre:         void ajustarEstadoBlackbox(estadoBlackbox_e estado)
**  Descripcion:    Ajusta el estado de la blackbox
**  Parametros:     Estado a ajustar
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarEstadoBlackbox(estadoBlackbox_e estado)
{
    switch (estado) {
        case BLACKBOX_ESTADO_PARADO:
            blackbox.arrancar = false;
            break;

        case BLACKBOX_ESTADO_PREPARAR_FICHERO_LOG:
            blackbox.logEmpezado = false;
            break;

        case BLACKBOX_ESTADO_ENVIAR_CABECERA:
        	bytesLibresCabBlackbox = 0;
        	datosTXblackbox.indiceCabecera = 0;
        	datosTXblackbox.tiempoInicio = millis();
            break;

        case BLACKBOX_ESTADO_ENVIAR_CABECERA_CAMPO:
        case BLACKBOX_ESTADO_ENVIAR_CABECERA_LENTA:
        	datosTXblackbox.indiceCabecera = 0;
        	datosTXblackbox.indiceCampo = -1;
            break;

        case BLACKBOX_ESTADO_ENVIAR_INFO_SISTEMA:
        	datosTXblackbox.indiceCabecera = 0;
            break;

        case BLACKBOX_ESTADO_CORRIENDO:
        	iteradorLentoBlackbox = blackbox.intervaloLento; // Fuerza el iterador lento para escribirlo la primera vez
            break;

        case BLACKBOX_ESTADO_APAGANDO:
        	datosTXblackbox.tiempoInicio = millis();
            break;

        default:
            break;
    }

    blackbox.estado = estado;
}


/***************************************************************************************
**  Nombre:         void actualizarBlackbox(uint32_t tiempoActual)
**  Descripcion:    Actualiza la blackbox
**  Parametros:     Tiempo actual en us
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarBlackbox(uint32_t tiempoActual)
{
	afatfs_poll();

    switch (blackbox.estado) {
        case BLACKBOX_ESTADO_PARADO:
            if (blackbox.arrancar) {
                blackbox.arrancar = false;
                lanzarBlackbox();
            }
            break;

        case BLACKBOX_ESTADO_PREPARAR_FICHERO_LOG:
            if (iniciarLogBlackbox())
            	ajustarEstadoBlackbox(BLACKBOX_ESTADO_ENVIAR_CABECERA);
            break;

        case BLACKBOX_ESTADO_ENVIAR_CABECERA:
        	calcularBytesLibresCabBlackbox();

            if (millis() > datosTXblackbox.tiempoInicio + 100) {
                if (comprobarEspacioBlackbox(BYTES_LIBRES_CAB_POR_ITERACION_BLACKBOX)) {
                    for (uint16_t i = 0; i < BYTES_LIBRES_CAB_POR_ITERACION_BLACKBOX && cabeceraBlackbox[datosTXblackbox.indiceCabecera] != '\0'; i++, datosTXblackbox.indiceCabecera++) {
                    	escribirBlackbox(cabeceraBlackbox[datosTXblackbox.indiceCabecera]);
                    	bytesLibresCabBlackbox--;
                    }
                    if (cabeceraBlackbox[datosTXblackbox.indiceCabecera] == '\0')
                    	ajustarEstadoBlackbox(BLACKBOX_ESTADO_ENVIAR_CABECERA_CAMPO);
                }
            }
            break;

        case BLACKBOX_ESTADO_ENVIAR_CABECERA_CAMPO:
        	calcularBytesLibresCabBlackbox();

            if (!enviarDefCampoBlackbox('R', camposRapidosBlackbox, camposRapidosBlackbox + 1, LONG_ARRAY(camposRapidosBlackbox),
                &camposRapidosBlackbox[0].numDrivers, &camposRapidosBlackbox[1].numDrivers)) {
            	ajustarEstadoBlackbox(BLACKBOX_ESTADO_ENVIAR_CABECERA_LENTA);
            }
            break;

        case BLACKBOX_ESTADO_ENVIAR_CABECERA_LENTA:
        	calcularBytesLibresCabBlackbox();

            if (!enviarDefCampoBlackbox('L', camposLentosBlackbox, camposLentosBlackbox + 1, LONG_ARRAY(camposLentosBlackbox),
                &camposLentosBlackbox[0].numDrivers, &camposLentosBlackbox[1].numDrivers))
            	ajustarEstadoBlackbox(BLACKBOX_ESTADO_ENVIAR_INFO_SISTEMA);
            break;

        case BLACKBOX_ESTADO_ENVIAR_INFO_SISTEMA:
        	calcularBytesLibresCabBlackbox();

            if (escribirInfoSistemaBlackbox()) {
                // Se espera a hacer un flush para garantizar una escritura correcta
                ajustarEstadoBlackbox(BLACKBOX_ESTADO_FLUSH_CACHE);
            }
            break;

        case BLACKBOX_ESTADO_FLUSH_CACHE:
            if (forzarFlushCompletoBlackbox())
            	ajustarEstadoBlackbox(BLACKBOX_ESTADO_CORRIENDO);

            break;

        case BLACKBOX_ESTADO_PAUSADO:
            if (blackbox.arrancar) {
                blackbox.arrancar = false;
                logEventoReanudarLog_t reanudarLog;

                reanudarLog.logIteracion = iteradorBlackbox;
                reanudarLog.horaActual = tiempoActual;
                escribirLogEventoBlackbox(BLACKBOX_LOG_EVENTO_LOG_REANUDAR, (logEventoDatos_u *) &reanudarLog);

                ajustarEstadoBlackbox(BLACKBOX_ESTADO_CORRIENDO);
            }

            actualizarIteradoresBlackbox();
            break;

        case BLACKBOX_ESTADO_CORRIENDO:
            if (blackbox.pausar) {
                blackbox.pausar = false;
                ajustarEstadoBlackbox(BLACKBOX_ESTADO_PAUSADO);
            }
            else
                iterarLogBlackbox(tiempoActual);


            actualizarIteradoresBlackbox();
            break;

        case BLACKBOX_ESTADO_APAGANDO:
            if (finalizarLogBlackbox(blackbox.logEmpezado) && (millis() > datosTXblackbox.tiempoInicio + TIMEOUT_APAGAR_MS_BLACKBOX || forzarFlushBlackbox()))
            	ajustarEstadoBlackbox(BLACKBOX_ESTADO_PARADO);
            break;

        default:
            break;
    }

    if (blackboxLlena())
    	ajustarEstadoBlackbox(BLACKBOX_ESTADO_PARADO);
}


/***************************************************************************************
**  Nombre:         void lanzarBlackbox(void)
**  Descripcion:    Lanza el proceso de la blackbox
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void lanzarBlackbox(void)
{
    if (!abrirBlackbox()) {
    	ajustarEstadoBlackbox(BLACKBOX_ESTADO_DESHABILITADO);
        return;
    }

    resetearIteradoresBlackbox();
    ajustarEstadoBlackbox(BLACKBOX_ESTADO_PREPARAR_FICHERO_LOG);
}


/***************************************************************************************
**  Nombre:         void actualizarIteradoresBlackbox(void)
**  Descripcion:    Actualiza los iteradores
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarIteradoresBlackbox(void)
{
	iteradorBlackbox++;
	iteradorRapidoBlackbox++;
	iteradorLentoBlackbox++;
}


/***************************************************************************************
**  Nombre:         bool enviarDefCampoBlackbox(char identificador, const void *defCampo, const void *segundaDefcampo, uint8_t numCampos, const uint8_t *numDrivers, const uint8_t *segundoNumDrivers)
**  Descripcion:    Envia la definicion del campo de este estilo: C campo R nombre: a,b,c // C campo R drivers: 2,3,1
**  Parametros:     Letra identificador, definicion de campo, segunda definicion de campo, numero de campos, numero de drivers, segundo numero de drivers
**  Retorno:        True si todavia queda una cabecera por transmitir
****************************************************************************************/
bool enviarDefCampoBlackbox(char identificador, const void *defCampo, const void *segundaDefcampo, uint8_t numCampos, const uint8_t *numDrivers, const uint8_t *segundoNumDrivers)
{
    char stringEscritura[20];
    const defCabCampoBlackbox_t *def;
    static bool necesitaComa = false;
    uint8_t numCabeceras = LONG_ARRAY(nombresCabCampoBlackbox);
    size_t pasoDefCampo = (char*) segundaDefcampo - (char*) defCampo;
    size_t pasoNumDrivers = (char*) numDrivers - (char*) segundoNumDrivers;
    uint8_t drivers;

    // Troceamos la cabecera para no exceder el ratio de transmision. Por eso es necesario llamar la funcion varias veces

    // En la primera llamada se escribe el nombre y la coma
    if (datosTXblackbox.indiceCampo == -1) {
        if (datosTXblackbox.indiceCabecera >= numCabeceras)
            return false;

        uint32_t charsParaEscribir = strlen("C Campo x :") + strlen(nombresCabCampoBlackbox[datosTXblackbox.indiceCabecera]);

        if (!comprobarEspacioBlackbox(charsParaEscribir))
            return true;   // Se intentara otra vez

        bytesLibresCabBlackbox = printfBlackbox("C Campo %c %s:", identificador, nombresCabCampoBlackbox[datosTXblackbox.indiceCabecera]);
        bytesLibresCabBlackbox -= escribirStringBlackbox(stringEscritura);

        datosTXblackbox.indiceCampo++;
        necesitaComa = false;
    }

    for (; datosTXblackbox.indiceCampo < numCampos; datosTXblackbox.indiceCampo++) {
        def = (const defCabCampoBlackbox_t*) ((const char*)defCampo + pasoDefCampo * datosTXblackbox.indiceCampo);
        drivers = numDriversBlackbox(numDrivers[pasoNumDrivers * datosTXblackbox.indiceCampo]);

        if (drivers != 0) {
            int32_t bytesParaEscribir = 1; // Para la coma

            // La primera Cabecera es el nombre
            if (datosTXblackbox.indiceCabecera == 0)
                bytesParaEscribir += strlen(def->nombre) + strlen("[]") + 2;
            else
                bytesParaEscribir += 2;   // Las otras cabeceras son enteros de como maximo 2 digitos

            if (!comprobarEspacioBlackbox(bytesParaEscribir))
                return true;

            bytesLibresCabBlackbox -= bytesParaEscribir;

            if (necesitaComa)
            	escribirBlackbox(',');
            else
                necesitaComa = true;

            if (datosTXblackbox.indiceCabecera == 0) {
            	escribirStringBlackbox(def->nombre);

                // Comprobamos si se necesita pintar el indice entre corchetes
                if (def->indiceNombreCampo != -1)
                	printfBlackbox("[%d]", def->indiceNombreCampo);
            }
            else
            	printfBlackbox("%d", drivers);
        }
    }

    // Comprobamos si hemos terminado de escribir la linea
    if (datosTXblackbox.indiceCampo == numCampos && comprobarEspacioBlackbox(1)) {
    	bytesLibresCabBlackbox--;
    	escribirBlackbox('\n');
    	datosTXblackbox.indiceCabecera++;
    	datosTXblackbox.indiceCampo = -1;
    }

    return datosTXblackbox.indiceCabecera < numCabeceras;
}


/***************************************************************************************
**  Nombre:         bool escribirInfoSistemaBlackbox(void)
**  Descripcion:    Transmite la informacion del sistema
**  Parametros:     Ninguno
**  Retorno:        True si se ha transmitido por completo
****************************************************************************************/
bool escribirInfoSistemaBlackbox(void)
{
    if (!comprobarEspacioBlackbox(BYTES_LIBRES_CAB_POR_ITERACION_BLACKBOX))
        return false;

    char buf[TAMANIO_FECHA_HORA_FORMATEADA];

    switch (datosTXblackbox.indiceCabecera) {
        ESCRIBIR_LINEA_CAB_BLACKBOX("Firmware"         , "%s",               "URpilot");
        ESCRIBIR_LINEA_CAB_BLACKBOX("Firmware revision", "%s %s %s",         NOMBRE_FIRMWARE, VERSION_STRING, NOMBRE_PLACA);
        ESCRIBIR_LINEA_CAB_BLACKBOX("Firmware fecha"   , "%s %s",            horaCompilacionFirm(), horaCompilacionFirm());
        ESCRIBIR_LINEA_CAB_BLACKBOX("Log tiempo"       , "%s",               obtenerFechaHoraBlackbox(buf));

        default:
            return true;
    }

    datosTXblackbox.indiceCabecera++;
    return false;
}


/***************************************************************************************
**  Nombre:         char *obtenerFechaHoraBlackbox(char *buf)
**  Descripcion:    Transmite la informacion del sistema
**  Parametros:     String para la fecha y hora
**  Retorno:        True si se ha transmitido por completo
****************************************************************************************/
char *obtenerFechaHoraBlackbox(char *buf)
{
#ifdef USAR_RTC
    fechaHora_t fh;

    fechaHoraRTC(&fh);
    formatearFechaHoraLocal(buf, fh);
#else
    buf = "0000-01-01T00:00:00.000";
#endif

    return buf;
}


/***************************************************************************************
**  Nombre:         uint8_t numDriversBlackbox(numDriversBlackbox_e numDrivers)
**  Descripcion:    Comprueba el numero de elementos a escribir en la Blackbox
**  Parametros:     Enumeracion del numero de elementos dependiendo del driver
**  Retorno:        Numero de elemntos
****************************************************************************************/
uint8_t numDriversBlackbox(numDriversBlackbox_e numDrivers)
{
    switch (numDrivers) {
        case BLACKBOX_1_DRIVER:
            return 1;
            break;

        case BLACKBOX_NUM_DRIVERS_IMU:
            return numIMUsConectadas();
            break;

        case BLACKBOX_NUM_DRIVERS_BARO:
            return numBarosConectados();
            break;

        case BLACKBOX_NUM_DRIVERS_MAG:
            return numMagsConectados();
            break;

        case BLACKBOX_NUM_DRIVERS_GPS:
            return numGPSconectados();
            break;

        default:
            return 0;
            break;
    }
}


/***************************************************************************************
**  Nombre:         void escribirLogEventoBlackbox(logEvento_e evento, logEventoDatos_u *datos)
**  Descripcion:    Escribe un evento en la blackbox
**  Parametros:     Evento a escribir, datos del evento
**  Retorno:        Ninguno
****************************************************************************************/
void escribirLogEventoBlackbox(logEvento_e evento, logEventoDatos_u *datos)
{
    if (!(blackbox.estado == BLACKBOX_ESTADO_CORRIENDO || blackbox.estado == BLACKBOX_ESTADO_PAUSADO))
        return;

    escribirBlackbox('E');
    escribirBlackbox(evento);

    switch (evento) {
        case BLACKBOX_LOG_EVENTO_DESARMAR:
        	printfBlackbox("%f", datos->eventoDesarmar.razon);
            break;

        case BLACKBOX_LOG_EVENTO_MODO:
        	printfBlackbox("%f", datos->eventoModo.flags);
        	printfBlackbox("%f", datos->eventoModo.ultimosFlags);
            break;

        case BLACKBOX_LOG_EVENTO_LOG_REANUDAR:
        	printfBlackbox("%f", datos->eventoReanudarLog.logIteracion);
        	printfBlackbox("%f", datos->eventoReanudarLog.horaActual);
            break;

        case BLACKBOX_LOG_EVENTO_LOG_FIN:
        	escribirStringBlackbox("Fin del log");
        	escribirBlackbox(0);
            break;

        default:
            break;
    }
}


/***************************************************************************************
**  Nombre:         void iterarLogBlackbox(uint32_t tiempoActual)
**  Descripcion:    Realiza una nueva iteracion y llama a las funciones de escritura si es necesario
**  Parametros:     Tiempo actual en us
**  Retorno:        Ninguno
****************************************************************************************/
void iterarLogBlackbox(uint32_t tiempoActual)
{
    if (necesarioEscribirLogRapidoBlackbox())
    	escribirLogRapidoBlackbox(tiempoActual);
    else {
        if (necesarioEscribirLogLentoBlackbox())
        	escribirLogLentoBlackbox();
    }
}


/***************************************************************************************
**  Nombre:         bool necesarioEscribirLogRapidoBlackbox(void)
**  Descripcion:    Comprueba si hay que escribir el log rapido
**  Parametros:     Ninguno
**  Retorno:        True si OK
****************************************************************************************/
bool necesarioEscribirLogRapidoBlackbox(void)
{
    if (iteradorRapidoBlackbox >= blackbox.intervaloRapido) {
    	iteradorRapidoBlackbox = 0;
        return true;
    }
    else
        return false;
}

/***************************************************************************************
**  Nombre:         bool necesarioEscribirLogLentoBlackbox(void)
**  Descripcion:    Comprueba si hay que escribir el log lento
**  Parametros:     Ninguno
**  Retorno:        True si OK
****************************************************************************************/
bool necesarioEscribirLogLentoBlackbox(void)
{
    if (iteradorLentoBlackbox >= blackbox.intervaloLento) {
    	iteradorLentoBlackbox = 0;
        return true;
    }
    else
        return false;
}


/***************************************************************************************
**  Nombre:         void escribirLogRapidoBlackbox(uint32_t tiempoActual)
**  Descripcion:    Escribe los datos del log rapido
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
void escribirLogRapidoBlackbox(uint32_t tiempoActual)
{
	escribirBlackbox('R');

	printfBlackbox("%f", iteradorBlackbox);
	printfBlackbox("%f", tiempoActual);

#ifdef USAR_IMU
	uint8_t numIMUs = numIMUsConectadas();
    for (uint8_t i = 0; i < numIMUs; i++) {
    	float gIMU[3];
    	giroNumIMU(i, gIMU);
        for (uint8_t j = 0; j < 3; j++)
        	printfBlackbox("%.2f", gIMU[i]);
    }

    for (uint8_t i = 0; i < numIMUs; i++) {
        float aIMU[3];
        acelNumIMU(i, aIMU);
        for (uint8_t j = 0; j < 3; j++)
        	printfBlackbox("%.2f", aIMU[i]);
    }
#endif

#ifdef USAR_MAG
    uint8_t numMags = numMagsConectados();
    for (uint8_t i = 0; i < numMags; i++) {
        float cMag[3];
        campoNumMag(i, cMag);
        for (uint8_t j = 0; j < 3; j++)
        	printfBlackbox("%.2f", cMag[j]);
    }
#endif
#ifdef USAR_BARO
    uint8_t numBaros = numBarosConectados();
    for (uint8_t i = 0; i < numBaros; i++) {
    	printfBlackbox("%.2f", presionNumBaro(i));
    	printfBlackbox("%.2f", temperaturaNumBaro(i));
    }
#endif
#ifdef USAR_RADIO
    for (uint8_t i = 0; i < 8; i++)
    	printfBlackbox("%f", canalRadio(i));
#endif

    blackbox.logEmpezado = true;
}


/***************************************************************************************
**  Nombre:         void escribirLogLentoBlackbox(void)
**  Descripcion:    Escribe los datos del log lento
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void escribirLogLentoBlackbox(void)
{
	escribirBlackbox('L');

#ifdef USAR_GPS
	uint8_t numGPS = numGPSconectados();
    for (uint8_t i = 0; i < numGPS; i++)
    	printfBlackbox("%f", satelitesNumGPS(i));

    localizacion_t loc;
    for (uint8_t i = 0; i < numGPS; i++) {
    	localizacionNumGPS(i, &loc);
    	printfBlackbox("%f", loc.latitud);
    }

    for (uint8_t i = 0; i < numGPS; i++) {
    	localizacionNumGPS(i, &loc);
    	printfBlackbox("%f", loc.longitud);
    }

    for (uint8_t i = 0; i < numGPS; i++) {
    	localizacionNumGPS(i, &loc);
    	printfBlackbox("%f", loc.altitud);
    }

    for (uint8_t i = 0; i < numGPS; i++)
    	printfBlackbox("%.2f", vel2dNumGPS(i));

    for (uint8_t i = 0; i < numGPS; i++)
    	printfBlackbox("%.2f", velAngularNumGPS(i));
#endif

    blackbox.logEmpezado = true;
}

#endif
