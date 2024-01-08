/***************************************************************************************
**  gps.c - Funciones y variables comunes a todos los GPS
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 14/06/2019
**  Fecha de modificacion: 20/09/2020
**
**  EL proyecto URpilot NO es libre. No se puede distribuir y/o modificar este fichero
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
#include <math.h>

#include "gps.h"

#ifdef USAR_GPS
#include "gps_ublox.h"
#include "GP/gp_gps.h"
#include "Core/led_estado.h"
#include "Drivers/tiempo.h"
#include "Scheduler/scheduler.h"
#include "Comun/matematicas.h"
#include "Drivers/uart.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define BAUD_TIEMPO_MS_GPS              5000       // Tiempo que tiene que pasar para probar con otro baudrate a la hora de detectar el GPS
#define MEZCLADO_MEDIDAS_GPS            1


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    bool operativo;
	estado_t estado;
    localizacion_t localizacion;
    velocidad_t velocidad;
    float vel2d;                         // Velocidad lineal en m/s
    float velAngular;                    // Velocidad angular en centesimas de grado
} gpsGen_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static gps_t gps[NUM_MAX_GPS];
static gpsGen_t gpsGen;
static uint8_t cntGPSconectados = 0;
static const uint32_t gpsBaudrates[] = {57600U, 9600U, 115200U};
static const char gpsBloqueConfig[] = MODO_BINARIO_GPS_UBLOX;
static float pesosGPS[NUM_MAX_GPS];
static bool failsafeGPS;


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
bool iniciarDriverGPS(gps_t *dGPS);
void leerDriverGPS(gps_t *dGPS);
bool detectarGPS(gps_t *gps);
void ajustarBloqueConfigGPS(gps_t *gps, const char *mensaje, uint16_t tam);
void enviarBloqueConfigGPS(gps_t *gps);
void actualizarFailsafeGPS(void);
void calcularGPSgen(bool habMezcla);
void asignarGPSgen(void);
bool calcularPesosMezclaGPS(void);
void mezclarMedidasGPS(void);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         bool iniciarGPS(void)
**  Descripcion:    Inicia los GPS
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarGPS(void)
{
#ifdef LEER_GPS_SCHEDULER
    ajustarFrecuenciaEjecucionTarea(TAREA_LEER_GPS, PERIODO_TAREA_HZ_SCHEDULER(configGPS(0)->frecLeer));
#endif

    // Reseteamos las variables del sensor general
    memset(&gpsGen, 0, sizeof(gpsGen_t));
    gpsGen.estado.hdop = DOP_DESCONOCIDO_GPS;
    gpsGen.estado.vdop = DOP_DESCONOCIDO_GPS;

    for (uint8_t i = 0; i < NUM_MAX_GPS; i++) {

        if (configGPS(i)->tipoGPS == GPS_NINGUNO)
            continue;

        gps_t *driver = &gps[i];

        // Reseteamos las variables del sensor
        memset(driver, 0, sizeof(gps_t));

        driver->numGPS = i;
        driver->estado.necesitaReiniciar = true;
        driver->estado.status = NO_GPS;
        driver->estado.hdop = DOP_DESCONOCIDO_GPS;
        driver->estado.vdop = DOP_DESCONOCIDO_GPS;

        // Arrancamos la UART
        configIniUART_t config;
        config.baudrate = gpsBaudrates[gps->deteccion.baudrateActual];
        config.lWord = UART_LONGITUD_WORD_8;
        config.paridad = UART_NO_PARIDAD;
        config.stop = UART_BIT_STOP_1;

        if (!uartIniciada(configGPS(i)->dispUART))
            iniciarUART(configGPS(i)->dispUART, config, NULL);

        // Una vez cargada la configuracion iniciamos el sensor
        iniciarDriverGPS(driver);
    }

    return true;
}


/***************************************************************************************
**  Nombre:         bool iniciarDriverGPS(gps_t *dGPS)
**  Descripcion:    Inicia un GPS
**  Parametros:     Dispositivo a iniciar, posicion que ocupa en el array de sensores
**  Retorno:        True si ok
****************************************************************************************/
bool iniciarDriverGPS(gps_t *dGPS)
{
    iniciarGPSublox(dGPS);
    detectarGPS(dGPS);
    return true;
}


/***************************************************************************************
**  Nombre:         bool detectarGPS(gps_t *dGPS)
**  Descripcion:    Lee los datos del GPS
**  Parametros:     Puntero al sensor
**  Retorno:        Sensor detectado
****************************************************************************************/
bool detectarGPS(gps_t *dGPS)
{
    bool detectado = false;
    uint32_t tiempo = millis();

    dGPS->estado.status = NO_GPS;
    dGPS->estado.hdop = DOP_DESCONOCIDO_GPS;
    dGPS->estado.vdop = DOP_DESCONOCIDO_GPS;

    if (tiempo - dGPS->deteccion.ultimoCambioBaudMs > BAUD_TIEMPO_MS_GPS) {
        // Intenta con el siguiente baudrate
    	dGPS->deteccion.baudrateActual++;
        if (dGPS->deteccion.baudrateActual == sizeof(gpsBaudrates) / sizeof(uint32_t))
        	dGPS->deteccion.baudrateActual = 0;

        ajustarBaudRateUART(configGPS(dGPS->numGPS)->dispUART, gpsBaudrates[dGPS->deteccion.baudrateActual]);
        dGPS->deteccion.ultimoCambioBaudMs = tiempo;

        if (configGPS(dGPS->numGPS)->tipoGPS == GPS_UBLOX_NEO_6M || configGPS(dGPS->numGPS)->tipoGPS == GPS_UBLOX_NEO_7M || configGPS(gps->numGPS)->tipoGPS == GPS_UBLOX_NEO_M8)
            ajustarBloqueConfigGPS(dGPS, gpsBloqueConfig, sizeof(gpsBloqueConfig));
    }

    // Enviamos el bloque de configuracion
    enviarBloqueConfigGPS(dGPS);

    while (dGPS->bloqueConfiguracion.charRestantes == 0 && bytesRecibidosUART(configGPS(dGPS->numGPS)->dispUART) > 0) {
        uint8_t dato;
        dato = leerUART(configGPS(dGPS->numGPS)->dispUART);

        // Ejecutar un GPS uBlox a menos de 38400 dara lugar a paquetes corruptos ya que no podemos recibir los paquetes en 200 ms (5Hz).
        if ((configGPS(dGPS->numGPS)->tipoGPS == GPS_UBLOX_NEO_6M || configGPS(gps->numGPS)->tipoGPS == GPS_UBLOX_NEO_7M || configGPS(gps->numGPS)->tipoGPS == GPS_UBLOX_NEO_M8) && gpsBaudrates[gps->deteccion.baudrateActual] >= 38400 && detectarGPSublox(dGPS, dato)) {
#ifdef DEBUG
            printf("GPS detectado nº: %u\n", dGPS->numGPS + 1);
#endif
            dGPS->detectado = true;
            detectado = true;
            cntGPSconectados++;
            iniciarGPSublox(dGPS);
            dGPS->estado.necesitaReiniciar = false;
            ajustarBloqueConfigGPS(dGPS, NULL, 0);          // Detiene cualquier cadena de configuracion que este pendiente
            solcitarSiguienteConfigGPSublox(dGPS);          // Inicia el proceso de actualizacion de los rates del GPS
            break;
        }
    }

    if (detectado == true) {
    	dGPS->estado.status = NO_FIX;
    	dGPS->timing.ultimoMensajeMs = tiempo;
    	dGPS->timing.incrementoTiempoMs = TIMEOUT_MS_GPS;
    }

    return detectado;
}


/***************************************************************************************
**  Nombre:         void ajustarBloqueConfigGPS(gps_t *dGPS, const char *mensaje, uint16_t tam)
**  Descripcion:    Configura la estructura del mensaje de configuracion
**  Parametros:     Puntero al GPS, mensaje de configuracion, tamanio del mensaje
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarBloqueConfigGPS(gps_t *dGPS, const char *mensaje, uint16_t tam)
{
    dGPS->bloqueConfiguracion.mensaje = mensaje;
    dGPS->bloqueConfiguracion.charRestantes = tam;
}


/***************************************************************************************
**  Nombre:         void enviarBloqueConfigGPS(gps_t *dGPS)
**  Descripcion:    Envia el bloque de configuracion
**  Parametros:     Puntero al GPS
**  Retorno:        Ninguno
****************************************************************************************/
void enviarBloqueConfigGPS(gps_t *dGPS)
{
    // Comprueba si quedan bytes por enviar
    if (dGPS->bloqueConfiguracion.charRestantes > 0) {
        int16_t espacio;

        espacio = bytesLibresBufferTxUART(configGPS(dGPS->numGPS)->dispUART);
        if (espacio > (int16_t) dGPS->bloqueConfiguracion.charRestantes)
            espacio = dGPS->bloqueConfiguracion.charRestantes;

        while (espacio > 0) {
            escribirUART(configGPS(dGPS->numGPS)->dispUART, *dGPS->bloqueConfiguracion.mensaje);
            dGPS->bloqueConfiguracion.mensaje++;
            espacio--;
            dGPS->bloqueConfiguracion.charRestantes--;
        }
    }
}


/***************************************************************************************
**  Nombre:         void actualizarFailsafeGPS(void)
**  Descripcion:    Comprueba si no hay sensores principales operativos
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void actualizarFailsafeGPS(void)
{
    bool failsafe = true;

    for (uint8_t i = 0; i < NUM_MAX_GPS; i++) {
        gps_t *driver = &gps[i];

        if (driver->detectado && driver->estado.status > NO_FIX && !configGPS(i)->auxiliar)
        	failsafe = false;
    }

    failsafeGPS = failsafe;
}


/***************************************************************************************
**  Nombre:         void calcularGPSgen(bool habMezcla)
**  Descripcion:    Mezcla las medidas de los sensores en uno general
**  Parametros:     Habilitacion de la mezcla de varios sensores
**  Retorno:        Ninguno
****************************************************************************************/
void calcularGPSgen(bool habMezcla)
{
    if (habMezcla && calcularPesosMezclaGPS())
        mezclarMedidasGPS();
    else
    	asignarGPSgen();
}


/***************************************************************************************
**  Nombre:         void asignarGPSgen(void)
**  Descripcion:    Asigna los valores del GPS general con el sensor de mayor numero de satelites
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void asignarGPSgen(void)
{
    uint8_t satelites = 0;
    uint8_t indice = 0;
    bool encontrado = false;
    bool gpsFix = false;

	// Se busca si hay algun GPS en NO_FIX
    for (uint8_t i = 0; i < NUM_MAX_GPS; i++) {
        gps_t *driver = &gps[i];

        if (driver->detectado) {
        	indice = i;
            encontrado = true;
        }

        if (driver->detectado && driver->estado.status > NO_FIX)
            gpsFix = true;
    }

    if (gpsFix) {
    	encontrado = false;

	    // Se busca el GPS con mayor numero de satelites
        for (uint8_t i = 0; i < NUM_MAX_GPS; i++) {
            gps_t *driver = &gps[i];

            if (driver->detectado && driver->estado.status > NO_FIX && driver->estado.numSats > satelites && (!configGPS(i)->auxiliar || failsafeGPS)) {
                satelites = driver->estado.numSats;
                indice = i;
                encontrado = true;
            }
        }
    }

    if (encontrado) {
        gpsGen.operativo = true;
        gpsGen.estado = gps[indice].estado;
        gpsGen.localizacion = gps[indice].localizacion;
        gpsGen.velocidad = gps[indice].velocidad;
        gpsGen.vel2d = gps[indice].vel2d;
        gpsGen.velAngular = gps[indice].velAngular;
    }
    else {
        // Reseteamos las variables del sensor general
        memset(&gpsGen, 0, sizeof(gpsGen_t));
        gpsGen.estado.hdop = DOP_DESCONOCIDO_GPS;
        gpsGen.estado.vdop = DOP_DESCONOCIDO_GPS;
    }
}


/***************************************************************************************
**  Nombre:         bool calcularPesosMezclaGPS(void)
**  Descripcion:    Calcula los pesos de los sensores para mezclar las medidas
**  Parametros:     Ninguno
**  Retorno:        True si se puede mezclar
****************************************************************************************/
bool calcularPesosMezclaGPS(void)
{
    uint8_t fix = 0;

    // Hacen falta minimo 2 sensores para mezclar las medidas
    for (uint8_t i = 0; i < NUM_MAX_GPS; i++) {
    	gps_t *driver = &gps[i];

        if (driver->detectado && driver->estado.status > NO_FIX && (!configGPS(i)->auxiliar || failsafeGPS))
        	fix++;

        if (fix >= 2)
            break;
    }

    if (fix < 2)
        return false;

    /*
     * Se utiliza el tiempo mas antiguo distinto de cero, pero si la diferencia de tiempo es
     * excesiva, se utiliza el mas nuevo para evitar que un receptor desconectado bloquee las
     * actualizaciones
     */
    uint32_t maxMs =  0;  // La hora de llegada mas reciente del sistema distinto de cero
    uint32_t minMs = -1;  // La hora de llegada mas antigua del sistema distinto de cero
    int16_t maxPeriodoActMs = 0;

    for (uint8_t i = 0; i < NUM_MAX_GPS; i++) {
    	gps_t *driver = &gps[i];

    	if (!driver->detectado || driver->estado.status <= NO_FIX || (configGPS(i)->auxiliar && !failsafeGPS))
            continue;

        if (driver->estado.ultimaHoraGPSms > maxMs)
        	maxMs = driver->estado.ultimaHoraGPSms;

        if ((driver->estado.ultimaHoraGPSms < minMs) && (driver->estado.ultimaHoraGPSms > 0))
        	minMs = driver->estado.ultimaHoraGPSms;

        if (configGPS(i)->periodoMuestreo > maxPeriodoActMs)
        	maxPeriodoActMs = configGPS(i)->periodoMuestreo;
    }

    // Demasiado retardo para mezclar medidas
    if ((int32_t)(maxMs - minMs) > (int32_t)(2 * maxPeriodoActMs))
        return false;

    // Se calcula la suma de cuadrados de la precision teniendo en cuenta la posicion
    float sumaCuadradosPos = 0;
    for (uint8_t i = 0; i < NUM_MAX_GPS; i++) {
        gps_t *driver = &gps[i];

        if (driver->estado.status >= GPS_OK_FIX_2D && (!configGPS(i)->auxiliar || failsafeGPS)) {
            if (driver->estado.tienePrecisionHorizontal && driver->estado.precisionHorizontal > 0)
                sumaCuadradosPos += driver->estado.precisionHorizontal * driver->estado.precisionHorizontal;
            else {
                sumaCuadradosPos = 0;
                break;
            }
        }
    }

    // Se calcula la suma de cuadrados de la precision teniendo en cuenta la velocidad
    float sumaCuadradosVel = 0;
    for (uint8_t i = 0; i < NUM_MAX_GPS; i++) {
        gps_t *driver = &gps[i];

        if (driver->estado.status >= GPS_OK_FIX_3D && (!configGPS(i)->auxiliar || failsafeGPS)) {
            if (driver->estado.tienePrecisionVel && driver->estado.precisionVel > 0)
            	sumaCuadradosVel += driver->estado.precisionVel * driver->estado.precisionVel;
            else {
            	sumaCuadradosVel = 0;
                break;
            }
        }

    }

    if (sumaCuadradosPos == 0 && sumaCuadradosVel == 0)
        return false;

    float sumaPesos = 0;

    // Calculo de los pesos de posicion
    float pesosPos[NUM_MAX_GPS];
    memset(pesosPos, 0, sizeof(pesosPos));

    if (sumaCuadradosPos > 0) {
        float sumaPesosPos = 0;

        // Se calculan los pesos con la inversa de la varianza
        for (uint8_t i = 0; i < NUM_MAX_GPS; i++) {
        	gps_t *driver = &gps[i];

            if (driver->estado.status >= GPS_OK_FIX_2D && driver->estado.precisionHorizontal >= 0.001f && (!configGPS(i)->auxiliar || failsafeGPS)) {
            	pesosPos[i] = sumaCuadradosPos / (driver->estado.precisionHorizontal * driver->estado.precisionHorizontal);
            	sumaPesosPos += pesosPos[i];
            }
        }

        // Se normalizan los pesos
        if (sumaPesosPos > 0) {
            for (uint8_t i = 0; i < NUM_MAX_GPS; i++)
            	pesosPos[i] = pesosPos[i] / sumaPesosPos;

            sumaPesos += 1;
        }
    }

    // Calculo de los pesos de velocidad
    float pesosVel[NUM_MAX_GPS];
    memset(pesosVel, 0, sizeof(pesosVel));

    if (sumaCuadradosVel > 0) {
        float sumaPesosVel = 0;

        // Se calculan los pesos con la inversa de la varianza
        for (uint8_t i = 0; i< NUM_MAX_GPS; i++) {
            gps_t *driver = &gps[i];

            if (driver->estado.status >= GPS_OK_FIX_3D && driver->estado.precisionVel >= 0.001f && (!configGPS(i)->auxiliar || failsafeGPS)) {
            	pesosVel[i] = sumaCuadradosVel / (driver->estado.precisionVel * driver->estado.precisionVel);
            	sumaPesosVel += pesosVel[i];
            }
        }

        // Se normalizan los pesos
        if (sumaPesosVel > 0) {
            for (uint8_t i = 0; i < NUM_MAX_GPS; i++) {
            	pesosVel[i] = pesosVel[i] / sumaPesosVel;
            }
            sumaPesos += 1;
        }
    }

    // Calculo final de los pesos
    for (uint8_t i = 0; i < NUM_MAX_GPS; i++)
    	pesosGPS[i] = (pesosPos[i] + pesosVel[i]) / sumaPesos;

    return true;
}


/***************************************************************************************
**  Nombre:         void mezclarMedidasGPS(void)
**  Descripcion:    Mezcla las medidas de los sensores en uno general
**  Parametros:     Ninguno
**  Retorno:        Ninguno
****************************************************************************************/
void mezclarMedidasGPS(void)
{
    for (uint8_t i = 0; i < NUM_MAX_GPS; i++) {
        gps_t *driver = &gps[i];

        if (configGPS(i)->auxiliar && !failsafeGPS)
        	continue;

        gpsGen.operativo = true;

        if (driver->estado.status > gpsGen.estado.status)
            gpsGen.estado.status = driver->estado.status;

        if (driver->estado.tienePrecisionHorizontal && driver->estado.precisionHorizontal > 0 && driver->estado.precisionHorizontal < gpsGen.estado.precisionHorizontal) {
            gpsGen.estado.tienePrecisionHorizontal = true;
            gpsGen.estado.precisionHorizontal = driver->estado.precisionHorizontal;
        }

        if (driver->estado.tienePrecisionVertical && driver->estado.precisionVertical > 0 && driver->estado.precisionVertical < gpsGen.estado.precisionVertical) {
    	    gpsGen.estado.tienePrecisionVertical = true;
    	    gpsGen.estado.precisionVertical = driver->estado.precisionVertical;
        }

        if (driver->estado.tieneVelVertical)
    	    gpsGen.estado.tieneVelVertical = true;

        if (driver->estado.tienePrecisionVel && driver->estado.precisionVel > 0 && driver->estado.precisionVel < gpsGen.estado.precisionVel) {
    	    gpsGen.estado.tienePrecisionVel = true;
    	    gpsGen.estado.precisionVel = gpsGen.estado.precisionVel;
        }

        if (driver->estado.hdop > 0 && driver->estado.hdop < gpsGen.estado.hdop)
    	    gpsGen.estado.hdop = driver->estado.hdop;

        if (driver->estado.vdop > 0 && driver->estado.vdop < gpsGen.estado.vdop)
    	    gpsGen.estado.vdop = driver->estado.vdop;

        if (driver->estado.numSats > 0 && driver->estado.numSats > gpsGen.estado.numSats)
    	    gpsGen.estado.numSats = driver->estado.numSats;

        gpsGen.velocidad.norte += driver->velocidad.norte * pesosGPS[i];
        gpsGen.velocidad.este += driver->velocidad.este * pesosGPS[i];
        gpsGen.velocidad.vertical += driver->velocidad.vertical * pesosGPS[i];
    }

    float mejorPeso = 0;
    uint8_t mejorIndice = 0;
    for (uint8_t i = 0; i < NUM_MAX_GPS; i++) {
        gps_t *driver = &gps[i];

        if (pesosGPS[i] > mejorPeso) {
    	    mejorPeso = pesosGPS[i];
            mejorIndice = i;
            gpsGen.localizacion = driver->localizacion;
        }
    }

    float offsetNE[2] = {0, 0};
    float offsetAlt = 0.0f;
    for (uint8_t i = 0; i < NUM_MAX_GPS; i++) {
        gps_t *driver = &gps[i];

        if (pesosGPS[i] > 0 && i != mejorIndice) {
    	    float distNE[2];

            distanciaNE(gpsGen.localizacion, driver->localizacion, distNE);
            offsetNE[0] += distNE[0] * pesosGPS[i];
            offsetNE[1] += distNE[1] * pesosGPS[i];
            offsetAlt += (float)(driver->localizacion.altitud - gpsGen.localizacion.altitud) * pesosGPS[i];
        }
    }

    anadirOffsetLoc(offsetNE[0], offsetNE[1], offsetAlt, &gpsGen.localizacion);

    float vector[2] = {gpsGen.velocidad.norte, gpsGen.velocidad.este};
    gpsGen.vel2d = moduloVector2(vector);
    gpsGen.velAngular = envolverInt360(grados(atan2f(gpsGen.velocidad.este, gpsGen.velocidad.norte)), 1);
}


/***************************************************************************************
**  Nombre:         void leerGPS(uint32_t tiempoActual)
**  Descripcion:    Lee los datos de los GPS
**  Parametros:     Tiempo actual
**  Retorno:        Ninguno
****************************************************************************************/
void leerGPS(uint32_t tiempoActual)
{
    UNUSED(tiempoActual);

    for (uint8_t i = 0; i < NUM_MAX_GPS; i++) {
    	gps_t *driver = &gps[i];

        leerDriverGPS(driver);
    }

    actualizarFailsafeGPS();

    if (cntGPSconectados > 0)
        calcularGPSgen(MEZCLADO_MEDIDAS_GPS);
}


/***************************************************************************************
**  Nombre:         void leerDriverGPS(gps_t *dGPS)
**  Descripcion:    Lee los datos de un GPS
**  Parametros:     GPS a leer
**  Retorno:        Ninguno
****************************************************************************************/
void leerDriverGPS(gps_t *dGPS)
{
    if (dGPS->estado.necesitaReiniciar == true || dGPS->estado.status == NO_GPS) {
        // Reiniciamos el gps
        if (!detectarGPS(dGPS))
            return;
    }

    bool resultado = leerGPSublox(dGPS);
    uint32_t tiempo = millis();

    // Si no recibimos un mensaje dentro del timeout, reiniciamos el GPS
    if (!resultado) {
        if (tiempo - dGPS->timing.ultimoMensajeMs > TIMEOUT_MS_GPS) {
        	dGPS->detectado = false;
        	dGPS->estado.necesitaReiniciar = true;
            memset(&dGPS->estado, 0, sizeof(dGPS->estado));
            dGPS->estado.status = NO_GPS;
            dGPS->estado.hdop = DOP_DESCONOCIDO_GPS;
            dGPS->estado.vdop = DOP_DESCONOCIDO_GPS;
            dGPS->timing.ultimoMensajeMs = tiempo;
            dGPS->timing.incrementoTiempoMs = TIMEOUT_MS_GPS;

            // Asignamos el bloque de configuracion
            ajustarBloqueConfigGPS(dGPS, gpsBloqueConfig, sizeof(gpsBloqueConfig));
        }
    }
    else {
        // El incremento solo sera correcto después de analizar dos mensajes
    	dGPS->timing.incrementoTiempoMs = tiempo - dGPS->timing.ultimoMensajeMs;
    	dGPS->timing.ultimoMensajeMs = tiempo;
        if (dGPS->estado.status >= GPS_OK_FIX_2D)
        	dGPS->timing.ultimoFixMs = tiempo;
    }
}


/***************************************************************************************
**  Nombre:         uint8_t numGPSconectados(void)
**  Descripcion:    Devuleve el numero de GPS conectados
**  Parametros:     Ninguno
**  Retorno:        Numero de GPS conectados
****************************************************************************************/
uint8_t numGPSconectados(void)
{
    return cntGPSconectados;
}


/***************************************************************************************
**  Nombre:         bool gpsGenOperativo(void)
**  Descripcion:    Devuelve si el GPS general esta operativo
**  Parametros:     Ninguno
**  Retorno:        GPS general operativo
****************************************************************************************/
bool gpsGenOperativo(void)
{
    return gpsGen.operativo;
}


/***************************************************************************************
**  Nombre:         void localizacionGPS(localizacion_t *loc)
**  Descripcion:    Devuleve la localizacion del GPS general
**  Parametros:     Localizacion
**  Retorno:        Ninguno
****************************************************************************************/
void localizacionGPS(localizacion_t *loc)
{
    *loc = gpsGen.localizacion;
}


/***************************************************************************************
**  Nombre:         float vel2dGPS(void)
**  Descripcion:    Devuleve la velocidad 2D del GPS general
**  Parametros:     Ninguno
**  Retorno:        Velocidad 2D
****************************************************************************************/
float vel2dGPS(void)
{
    return gpsGen.vel2d;
}


/***************************************************************************************
**  Nombre:         float velAngularGPS(void)
**  Descripcion:    Devuleve la velocidad angular del GPS general
**  Parametros:     Ninguno
**  Retorno:        Velocidad angular
****************************************************************************************/
float velAngularGPS(void)
{
    return gpsGen.velAngular;
}


/***************************************************************************************
**  Nombre:         uint8_t satelitesGPS(void)
**  Descripcion:    Devuleve el numero de satelites del GPS general
**  Parametros:     Ninguno
**  Retorno:        Numero de satelites
****************************************************************************************/
uint8_t satelitesGPS(void)
{
    return gpsGen.estado.numSats;
}


/***************************************************************************************
**  Nombre:         void localizacionNumGPS(uint8_t numGPS, localizacion_t *loc)
**  Descripcion:    Devuleve la localizacion de un GPS
**  Parametros:     Numero de GPS, localizacion
**  Retorno:        Ninguno
****************************************************************************************/
void localizacionNumGPS(uint8_t numGPS, localizacion_t *loc)
{
	*loc = gps[numGPS].localizacion;
}


/***************************************************************************************
**  Nombre:         float vel2dNumGPS(uint8_t numGPS)
**  Descripcion:    Devuleve la velocidad 2D
**  Parametros:     Numero de GPS
**  Retorno:        Velocidad 2D
****************************************************************************************/
float vel2dNumGPS(uint8_t numGPS)
{
    return gps[numGPS].vel2d;
}


/***************************************************************************************
**  Nombre:         float velAngularNumGPS(uint8_t numGPS)
**  Descripcion:    Devuleve la velocidad angular
**  Parametros:     Numero de GPS
**  Retorno:        Velocidad angular
****************************************************************************************/
float velAngularNumGPS(uint8_t numGPS)
{
    return gps[numGPS].velAngular;
}


/***************************************************************************************
**  Nombre:         uint8_t satelitesNumGPS(uint8_t numGPS)
**  Descripcion:    Devuleve el numero de satelites
**  Parametros:     Numero de GPS
**  Retorno:        Numero de satelites
****************************************************************************************/
uint8_t satelitesNumGPS(uint8_t numGPS)
{
    return gps[numGPS].estado.numSats;
}


#endif
