/***************************************************************************************
**  dshot.c - Funciones relativas al protocolo DSHOT
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 27/08/2019
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
#include <math.h>

#include "dshot.h"
#ifdef USAR_MOTORES
  #ifdef USAR_DSHOT
#include "Comun/util.h"
#include "Drivers/sistema.h"


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define RETARDO_INICIAL_US_DSHOT            10000
#define RETARDO_COMANDO_US_DSHOT            1000
#define RETARDO_BEEP_US_DSHOT               100000
#define NUM_MAX_COMANDOS_DSHOT              3


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef enum {
    DSHOT_COMANDO_ESTADO_ESPERA_INACTIVO,       // Esperando que los motores esten inactivos
    DSHOT_COMANDO_ESTADO_RETARDO_INICIAL,       // Periodo de retraso inicial antes de una secuencia de comandos
    DSHOT_COMANDO_ESTADO_ACTIVO,                // Envio activo del comando (con salida repetida opcional)
    DSHOT_COMANDO_ESTADO_RETARDO_POSTERIOR,     // Periodo de retraso despues de que se haya enviado el comando
} estadoComandoDshot_e;

typedef struct {
    estadoComandoDshot_e estado;
    uint32_t retardoSiguienteCicloComando;
    uint32_t retardoDespuesComando;
    uint8_t repeticiones;
    uint8_t comando[NUM_MAX_MOTORES];
} controlComandoDshot_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/
static controlComandoDshot_t colaComandosDshot[NUM_MAX_COMANDOS_DSHOT + 1];
static uint8_t cabezaColaComandosDshot;
static uint8_t colaColaComandosDshot;
static uint32_t tiempoCicloPIDcomandoDshot = 125; // Por defecto 8KHz (125us) valor real en la inicializacion del PID


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
void ajustarCicloDshotPID(uint32_t tiempoCicloPID);
uint8_t comandoDshot(uint8_t indice);
bool colaComandosDshotLlena(void);
bool hayComandosDshotEnCola(void);
bool esUltimoComandoDshot(void);
bool comandoDshotSiendoProcesado(void);
controlComandoDshot_t* anadirComando(void);
bool todosMotoresEnReposo(uint8_t numMotores);
uint32_t ciclosComandoDshotTiempo(uint32_t retardo);
bool actualizarColaComandosDshot(void);


/***************************************************************************************
** AREA DE DEFINICION DE FUNCIONES                                                    **
****************************************************************************************/

/***************************************************************************************
**  Nombre:         void ajustarCicloDshotPID(uint32_t tiempoCicloPID)
**  Descripcion:    Ajusta el tiempo de ciclo Dshot desde el ciclo PID
**  Parametros:     Tiempo de ciclo de los PID
**  Retorno:        Ninguno
****************************************************************************************/
void ajustarCicloDshotPID(uint32_t tiempoCicloPID)
{
    tiempoCicloPIDcomandoDshot = tiempoCicloPID;
}


/***************************************************************************************
**  Nombre:         uint8_t comandoDshot(uint8_t indice)
**  Descripcion:    Obtiene el comando de PWM
**  Parametros:     Indice
**  Retorno:        Comando
****************************************************************************************/
uint8_t comandoDshot(uint8_t indice)
{
    return colaComandosDshot[colaColaComandosDshot].comando[indice];
}


/***************************************************************************************
**  Nombre:         bool colaComandosDshotLlena(void)
**  Descripcion:    Obtiene si la cola de comandos esta llena
**  Parametros:     Ninguno
**  Retorno:        True si llena
****************************************************************************************/
CODIGO_RAPIDO bool colaComandosDshotLlena(void)
{
    return (cabezaColaComandosDshot + 1) % (NUM_MAX_COMANDOS_DSHOT + 1) == colaColaComandosDshot;
}


/***************************************************************************************
**  Nombre:         bool hayComandosDshotEnCola(void)
**  Descripcion:    Obtiene si hay comandos en cola
**  Parametros:     Ninguno
**  Retorno:        True si hay
****************************************************************************************/
CODIGO_RAPIDO bool hayComandosDshotEnCola(void)
{
    return cabezaColaComandosDshot != colaColaComandosDshot;
}


/***************************************************************************************
**  Nombre:         bool esUltimoComandoDshot(void)
**  Descripcion:    Comprueba si es el ultimo comando
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool esUltimoComandoDshot(void)
{
    return ((colaColaComandosDshot + 1) % (NUM_MAX_COMANDOS_DSHOT + 1) == cabezaColaComandosDshot);
}


/***************************************************************************************
**  Nombre:         bool comandoDshotSiendoProcesado(void)
**  Descripcion:    Comprueba si hay algun comando sieno procesado
**  Parametros:     Ninguno
**  Retorno:        True si ok
****************************************************************************************/
CODIGO_RAPIDO bool comandoDshotSiendoProcesado(void)
{
    if (!hayComandosDshotEnCola())
        return false;

    controlComandoDshot_t* comando = &colaComandosDshot[colaColaComandosDshot];
    const bool comandoSiendoProcesado = comando->estado == DSHOT_COMANDO_ESTADO_RETARDO_INICIAL
                                     || comando->estado == DSHOT_COMANDO_ESTADO_ACTIVO
                                     || (comando->estado == DSHOT_COMANDO_ESTADO_RETARDO_POSTERIOR && !esUltimoComandoDshot());
    return comandoSiendoProcesado;
}


/***************************************************************************************
**  Nombre:         void escribirComandoDshot(uint8_t indice, uint8_t numMotores, uint8_t comando, bool bloqueo)
**  Descripcion:    Escribe un comando Dshot
**  Parametros:     Indice del motor, numero de motores, comando a escribir, modo bloqueo de programa o no
**  Retorno:        Ninguno
****************************************************************************************/
void escribirComandoDshot(uint8_t indice, uint8_t numMotores, uint8_t comando, bool bloqueo)
{
    if (!esProtocoloMotorDshot() || (comando > COMANDO_MAX_DSHOT) || colaComandosDshotLlena())
        return;

    uint8_t repeticiones = 1;
    uint32_t retardoDepuesComando = RETARDO_COMANDO_US_DSHOT;

    switch (comando) {
        case DSHOT_CMD_SPIN_DIRECTION_1:
        case DSHOT_CMD_SPIN_DIRECTION_2:
        case DSHOT_CMD_3D_MODE_OFF:
        case DSHOT_CMD_3D_MODE_ON:
        case DSHOT_CMD_SAVE_SETTINGS:
        case DSHOT_CMD_SPIN_DIRECTION_NORMAL:
        case DSHOT_CMD_SPIN_DIRECTION_REVERSED:
        case DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE:
        case DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY:
            repeticiones = 10;
            break;

        case DSHOT_CMD_BEACON1:
        case DSHOT_CMD_BEACON2:
        case DSHOT_CMD_BEACON3:
        case DSHOT_CMD_BEACON4:
        case DSHOT_CMD_BEACON5:
            retardoDepuesComando = RETARDO_BEEP_US_DSHOT;
            break;

        default:
            break;
    }

    if (bloqueo) {
        delayMicroseconds(DSHOT_COMANDO_ESTADO_RETARDO_INICIAL - RETARDO_COMANDO_US_DSHOT);
        for (; repeticiones; repeticiones--) {
            delayMicroseconds(RETARDO_COMANDO_US_DSHOT);

            for (uint8_t i = 0; i < numMotores; i++) {
                if ((i == indice) || (indice == TODOS_MOTORES)) {
                    motorDshot_t *const motor = motorDshot(i);
                    motor->solicitarTelemetria = true;
                    escribirPWMdshot(i, comando);
                }
            }

            actualizarPWMdshot(0);
        }
        delayMicroseconds(retardoDepuesComando);
    }
    else {
        controlComandoDshot_t *controlComando = anadirComando();
        if (controlComando) {
            controlComando->repeticiones = repeticiones;
            controlComando->retardoDespuesComando = retardoDepuesComando;
            for (uint8_t i = 0; i < numMotores; i++) {
                if (indice == i || indice == TODOS_MOTORES)
                    controlComando->comando[i] = comando;
                else
                    controlComando->comando[i] = DSHOT_CMD_MOTOR_STOP;
            }
            if (todosMotoresEnReposo(numMotores)) {
                // Se puede omitir el estado de espera inactivo de los motores
                controlComando->estado = DSHOT_COMANDO_ESTADO_RETARDO_INICIAL;
                controlComando->retardoSiguienteCicloComando = ciclosComandoDshotTiempo(RETARDO_INICIAL_US_DSHOT);
            }
            else {
                controlComando->estado = DSHOT_COMANDO_ESTADO_RETARDO_INICIAL;
                controlComando->retardoSiguienteCicloComando = 0;  // Se establecera después de que la espera inactiva se complete
            }
        }
    }
}


/***************************************************************************************
**  Nombre:         controlComandoDshot_t* anadirComando(void)
**  Descripcion:    Incrementa la cabeza del array y retorna el control del comando
**  Parametros:     Ninguno
**  Retorno:        Control de comando
****************************************************************************************/
controlComandoDshot_t* anadirComando(void)
{
    uint8_t nuevaCabeza = (cabezaColaComandosDshot + 1) % (NUM_MAX_COMANDOS_DSHOT + 1);
    if (nuevaCabeza == colaColaComandosDshot)
        return NULL;

    controlComandoDshot_t* control = &colaComandosDshot[cabezaColaComandosDshot];
    cabezaColaComandosDshot = nuevaCabeza;
    return control;
}


/***************************************************************************************
**  Nombre:         bool comandoSalidaDshotHabilitado(uint8_t numMotores)
**  Descripcion:    Esta funcion se utiliza para sincronizar el tiempo de salida del comando dshot
**                  con la temporizacion normal de salida a traves del PID. Ejemplo: un comando que
**                  necesita repetirse 10 veces a intervalos de 1ms si tenemos el PID a 8kHz terminaremos
**                  enviando el comando dshot cada 8a salida del motor
**  Parametros:     Numero de motores
**  Retorno:        True si salida habilitada
****************************************************************************************/
CODIGO_RAPIDO bool comandoSalidaDshotHabilitado(uint8_t numMotores)
{
    controlComandoDshot_t* comando = &colaComandosDshot[colaColaComandosDshot];
    switch (comando->estado) {
        case DSHOT_COMANDO_ESTADO_ESPERA_INACTIVO:
            if (todosMotoresEnReposo(numMotores)) {
                comando->estado = DSHOT_COMANDO_ESTADO_RETARDO_INICIAL;
                comando->retardoSiguienteCicloComando = ciclosComandoDshotTiempo(RETARDO_INICIAL_US_DSHOT);
            }
            break;

        case DSHOT_COMANDO_ESTADO_RETARDO_INICIAL:
            if (comando->retardoSiguienteCicloComando-- > 1)
                return false;                           // Retrasa la salida del motor hasta el inicio de la secuencia de comandos

            comando->estado = DSHOT_COMANDO_ESTADO_ACTIVO;
            comando->retardoSiguienteCicloComando = 0;  // La primera iteracion de la repeticion ocurre ahora
            FALLTHROUGH;

        case DSHOT_COMANDO_ESTADO_ACTIVO:
            if (comando->retardoSiguienteCicloComando-- > 1)
                return false;                           // Retrasa la salida del motor hasta que se repita el siguiente comando

            comando->repeticiones--;
            if (comando->repeticiones)
                comando->retardoSiguienteCicloComando = ciclosComandoDshotTiempo(RETARDO_COMANDO_US_DSHOT);
            else {
                comando->estado = DSHOT_COMANDO_ESTADO_RETARDO_POSTERIOR;
                comando->retardoSiguienteCicloComando = ciclosComandoDshotTiempo(comando->retardoDespuesComando);
                if (!esUltimoComandoDshot() && comando->retardoSiguienteCicloComando > 0) {
                    // Hay que tener en cuenta un ciclo extra de salida de motor entre comandos. De lo contrario el retardo sera RETARDO_COMANDO_DSHOT_US + 1 ciclo
                    comando->retardoSiguienteCicloComando--;
                }
            }
            break;

        case DSHOT_COMANDO_ESTADO_RETARDO_POSTERIOR:
            if (comando->retardoSiguienteCicloComando-- > 1)
                return false;  // Retrasa la salida del motor hasta el final del retardo posterior al comando

            if (actualizarColaComandosDshot())
                return false;  // Sera true si la cola de colmandos no esta vacia y nosotros deseamos esperar a que comience el siguiente comando en secuencia
    }

    return true;
}


/***************************************************************************************
**  Nombre:         bool todosMotoresEnReposo(uint8_t numMotores)
**  Descripcion:    Comprueba que todos los motores estan en reposo
**  Parametros:     Numero de motores
**  Retorno:        True si en reposo
****************************************************************************************/
bool todosMotoresEnReposo(uint8_t numMotores)
{
    bool motoresReposo = true;
    for (uint8_t i = 0; i < numMotores; i++) {
        const motorDshot_t *motor = motorDshot(i);
        if (motor->valor)
            motoresReposo = false;
    }

    return motoresReposo;
}


/***************************************************************************************
**  Nombre:         uint32_t ciclosComandoDshotTiempo(uint32_t retardo)
**  Descripcion:    Calcula los ciclos de retardo
**  Parametros:     Retardo en tiempo
**  Retorno:        Ciclos de retardo
****************************************************************************************/
CODIGO_RAPIDO uint32_t ciclosComandoDshotTiempo(uint32_t retardo)
{
    // Encuentra el numero minimo de ciclos de salida del motor para proporcionar al menos un retardo de tiempo
    uint32_t ret = retardo / tiempoCicloPIDcomandoDshot;
    if (retardo % tiempoCicloPIDcomandoDshot)
        ret++;

    return ret;
}


/***************************************************************************************
**  Nombre:         bool actualizarColaComandosDshot(void)
**  Descripcion:    Sera true si la cola de colmandos no esta vacia y nosotros deseamos esperar
**                  a que comience el siguiente comando en secuencia
**  Parametros:     Ninguno
**  Retorno:        Ciclos de retardo
****************************************************************************************/
CODIGO_RAPIDO bool actualizarColaComandosDshot(void)
{
    if (hayComandosDshotEnCola()) {
        colaColaComandosDshot = (colaColaComandosDshot + 1) % (NUM_MAX_COMANDOS_DSHOT + 1);
        if (hayComandosDshotEnCola()) {
            // Hay otro comando en cola, asi que se actualiza para que esta listo para salir
            controlComandoDshot_t* siguienteComando = &colaComandosDshot[colaColaComandosDshot];
            siguienteComando->estado = DSHOT_COMANDO_ESTADO_ACTIVO;
            siguienteComando->retardoSiguienteCicloComando = 0;
            return true;
        }
    }
    return false;
}

  #endif
#endif

