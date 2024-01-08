/***************************************************************************************
**  sd_estandar.h - Datos estandar de la tarjeta SD
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 14/09/2019
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

#ifndef __SD_ESTANDAR_H
#define __SD_ESTANDAR_H

/***************************************************************************************
** AREA DE INCLUDES                                                                   **
****************************************************************************************/
#include <stdint.h>
#include <stdbool.h>


/***************************************************************************************
** AREA DE PREPROCESADOR                                                              **
****************************************************************************************/
#define OBTENER_CAMPO_CSD_SD(csd, version, campo) \
        leerCampoBitCSDsd(csd.dato, SD_CSD_V ## version ## _ ## campo ## _OFFSET, SD_CSD_V ## version ## _ ## campo ## _LON)

// Para v1 y Standard Capacity cards
#define SD_CSD_V1_CSD_STRUCTURE_VER_OFFSET           0
#define SD_CSD_V1_CSD_STRUCTURE_VER_LON              2

#define SD_CSD_V1_TAAC_OFFSET                        8
#define SD_CSD_V1_TAAC_LON                           8

#define SD_CSD_V1_NSAC_OFFSET                        16
#define SD_CSD_V1_NSAC_LON                           8

#define SD_CSD_V1_TRAN_SPEED_OFFSET                  24
#define SD_CSD_V1_TRAN_SPEED_LON                     8

#define SD_CSD_V1_CCC_OFFSET                         32
#define SD_CSD_V1_CCC_LON                            12

#define SD_CSD_V1_READ_BLOCK_LEN_OFFSET              44
#define SD_CSD_V1_READ_BLOCK_LEN_LON                 4

#define SD_CSD_V1_READ_BLOCK_PARTIAL_ALLOWED_OFFSET  48
#define SD_CSD_V1_READ_BLOCK_PARTIAL_ALLOWED_LON     1

#define SD_CSD_V1_WRITE_BLOCK_MISALIGN_OFFSET        49
#define SD_CSD_V1_WRITE_BLOCK_MISALIGN_LON           1

#define SD_CSD_V1_READ_BLOCK_MISALIGN_OFFSET         50
#define SD_CSD_V1_READ_BLOCK_MISALIGN_LON            1

#define SD_CSD_V1_DSR_IMPLEMENTED_OFFSET             51
#define SD_CSD_V1_DSR_IMPLEMENTED_LON                1

#define SD_CSD_V1_CSIZE_OFFSET                       54
#define SD_CSD_V1_CSIZE_LON                          12

#define SD_CSD_V1_VDD_READ_CURR_MIN_OFFSET           66
#define SD_CSD_V1_VDD_READ_CURR_MIN_LON              3

#define SD_CSD_V1_VDD_READ_CURR_MAX_OFFSET           69
#define SD_CSD_V1_VDD_READ_CURR_MAX_LON              3

#define SD_CSD_V1_VDD_WRITE_CURR_MIN_OFFSET          72
#define SD_CSD_V1_VDD_WRITE_CURR_MIN_LON             3

#define SD_CSD_V1_VDD_WRITE_CURR_MAX_OFFSET          75
#define SD_CSD_V1_VDD_WRITE_CURR_MAX_LON             3

#define SD_CSD_V1_CSIZE_MULT_OFFSET                  78
#define SD_CSD_V1_CSIZE_MULT_LON                     3

#define SD_CSD_V1_ERASE_SINGLE_BLOCK_ALLOWED_OFFSET  81
#define SD_CSD_V1_ERASE_SINGLE_BLOCK_ALLOWED_LON     1

#define SD_CSD_V1_SECTOR_SIZE_OFFSET                 82
#define SD_CSD_V1_SECTOR_SIZE_LON                    7

#define SD_CSD_V1_WRITE_PROTECT_GROUP_SIZE_OFFSET    89
#define SD_CSD_V1_WRITE_PROTECT_GROUP_SIZE_LON       7

#define SD_CSD_V1_WRITE_PROTECT_GROUP_ENABLE_OFFSET  96
#define SD_CSD_V1_WRITE_PROTECT_GROUP_ENABLE_LON     1

#define SD_CSD_V1_R2W_FACTOR_OFFSET                  99
#define SD_CSD_V1_R2W_FACTOR_LON                     3

#define SD_CSD_V1_WRITE_BLOCK_LON_OFFSET             102
#define SD_CSD_V1_WRITE_BLOCK_LON_LON                4

#define SD_CSD_V1_WRITE_BLOCK_PARTIAL_ALLOWED_OFFSET 106
#define SD_CSD_V1_WRITE_BLOCK_PARTIAL_ALLOWED_LON    1

#define SD_CSD_V1_FILE_FORMAT_GROUP_OFFSET           112
#define SD_CSD_V1_FILE_FORMAT_GROUP_LON              1

#define SD_CSD_V1_COPY_OFFSET                        113
#define SD_CSD_V1_COPY_LON                           1

#define SD_CSD_V1_PERMANENT_WRITE_PROTECT_OFFSET     114
#define SD_CSD_V1_PERMANENT_WRITE_PROTECT_LON        1

#define SD_CSD_V1_TEMPORARY_WRITE_PROTECT_OFFSET     115
#define SD_CSD_V1_TEMPORARY_WRITE_PROTECT_LON        1

#define SD_CSD_V1_FILE_FORMAT_OFFSET                 116
#define SD_CSD_V1_FILE_FORMAT_LON                    2

#define SD_CSD_V1_CRC_OFFSET                         120
#define SD_CSD_V1_CRC_LON                            7

#define SD_CSD_V1_TRAILER_OFFSET                     127
#define SD_CSD_V1_TRAILER_LON                        1


// Para v2 High Capacity cards
#define SD_CSD_V2_CSD_STRUCTURE_VER_OFFSET           0
#define SD_CSD_V2_CSD_STRUCTURE_VER_LON              2

#define SD_CSD_V2_TAAC_OFFSET                        8
#define SD_CSD_V2_TAAC_LON                           8

#define SD_CSD_V2_NSAC_OFFSET                        16
#define SD_CSD_V2_NSAC_LON                           8

#define SD_CSD_V2_TRAN_SPEED_OFFSET                  24
#define SD_CSD_V2_TRAN_SPEED_LON                     8

#define SD_CSD_V2_CCC_OFFSET                         32
#define SD_CSD_V2_CCC_LON                            12

#define SD_CSD_V2_READ_BLOCK_LON_OFFSET              44
#define SD_CSD_V2_READ_BLOCK_LON_LON                 4

#define SD_CSD_V2_READ_BLOCK_PARTIAL_ALLOWED_OFFSET  48
#define SD_CSD_V2_READ_BLOCK_PARTIAL_ALLOWED_LON     1

#define SD_CSD_V2_WRITE_BLOCK_MISALIGN_OFFSET        49
#define SD_CSD_V2_WRITE_BLOCK_MISALIGN_LON           1

#define SD_CSD_V2_READ_BLOCK_MISALIGN_OFFSET         50
#define SD_CSD_V2_READ_BLOCK_MISALIGN_LON            1

#define SD_CSD_V2_DSR_IMPLEMENTED_OFFSET             51
#define SD_CSD_V2_DSR_IMPLEMENTED_LON                1

#define SD_CSD_V2_CSIZE_OFFSET                       58
#define SD_CSD_V2_CSIZE_LON                          22

#define SD_CSD_V2_ERASE_SINGLE_BLOCK_ALLOWED_OFFSET  81
#define SD_CSD_V2_ERASE_SINGLE_BLOCK_ALLOWED_LON     1

#define SD_CSD_V2_SECTOR_SIZE_OFFSET                 82
#define SD_CSD_V2_SECTOR_SIZE_LON                    7

#define SD_CSD_V2_WRITE_PROTECT_GROUP_SIZE_OFFSET    89
#define SD_CSD_V2_WRITE_PROTECT_GROUP_SIZE_LON       7

#define SD_CSD_V2_WRITE_PROTECT_GROUP_ENABLE_OFFSET  96
#define SD_CSD_V2_WRITE_PROTECT_GROUP_ENABLE_LON     1

#define SD_CSD_V2_R2W_FACTOR_OFFSET                  99
#define SD_CSD_V2_R2W_FACTOR_LON                     3

#define SD_CSD_V2_WRITE_BLOCK_LON_OFFSET             102
#define SD_CSD_V2_WRITE_BLOCK_LON_LON                4

#define SD_CSD_V2_WRITE_BLOCK_PARTIAL_ALLOWED_OFFSET 106
#define SD_CSD_V2_WRITE_BLOCK_PARTIAL_ALLOWED_LON    1

#define SD_CSD_V2_FILE_FORMAT_GROUP_OFFSET           112
#define SD_CSD_V2_FILE_FORMAT_GROUP_LON              1

#define SD_CSD_V2_COPY_OFFSET                        113
#define SD_CSD_V2_COPY_LON                           1

#define SD_CSD_V2_PERMANENT_WRITE_PROTECT_OFFSET     114
#define SD_CSD_V2_PERMANENT_WRITE_PROTECT_LON        1

#define SD_CSD_V2_TEMPORARY_WRITE_PROTECT_OFFSET     115
#define SD_CSD_V2_TEMPORARY_WRITE_PROTECT_LON        1

#define SD_CSD_V2_FILE_FORMAT_OFFSET                 116
#define SD_CSD_V2_FILE_FORMAT_LON                    2

#define SD_CSD_V2_CRC_OFFSET                         120
#define SD_CSD_V2_CRC_LON                            7

#define SD_CSD_V2_TRAILER_OFFSET                     127
#define SD_CSD_V2_TRAILER_LON                        1


#define TOKEN_INICIAR_LECTURA_BLOQUE_SD              0xFE
#define TOKEN_INICIAR_ESCRITURA_BLOQUE_SD            0xFE
#define TOKEN_INICIAR_ESCRITURA_MULTIPLES_BLOQUES_SD 0xFC
#define TOKEN_PARAR_ESCRITURA_MULTIPLES_BLOQUES_SD   0xFD

#define TAMANIO_BLOQUE_SD                            512


// El bit inactivo se establece en 1 solo cuando esta inactivo durante la fase de inicializacian
#define SD_R1_BIT_ESTADO_REPOSO                      1
#define SD_R1_BIT_ESTADO_RESET_BORRADO               2
#define SD_R1_BIT_ESTADO_COMANDO_ILEGAL              4
#define SD_R1_BIT_ESTADO_ERROR_COM_CRC               8
#define SD_R1_BIT_ESTADO_ERROR_SECUENCIA_BORRADO     16
#define SD_R1_BIT_ESTADO_ERROR_DIRECCION             32
#define SD_R1_BIT_ESTADO_ERROR_PARAMETROS            64

#define SD_CSD_STRUCTURA_VERSION_1                   0
#define SD_CSD_STRUCTURA_VERSION_2                   1

#define SD_VOLTAJE_ACEPTADO_2_7_a_3_6                0x01
#define SD_VOLTAJE_ACEPTADO_LVR                      0x02

#define SD_COMANDO_GO_IDLE_STATE                     0
#define SD_COMANDO_SEND_OP_COND                      1
#define SD_COMANDO_SEND_IF_COND                      8
#define SD_COMANDO_SEND_CSD                          9
#define SD_COMANDO_SEND_CID                          10
#define SD_COMANDO_STOP_TRANSMISSION                 12
#define SD_COMANDO_SEND_STATUS                       13
#define SD_COMANDO_SET_BLOCKLEN                      16
#define SD_COMANDO_READ_SINGLE_BLOCK                 17
#define SD_COMANDO_READ_MULTIPLE_BLOCK               18
#define SD_COMANDO_WRITE_BLOCK                       24
#define SD_COMANDO_WRITE_MULTIPLE_BLOCK              25
#define SD_COMANDO_APP_CMD                           55
#define SD_COMANDO_READ_OCR                          58

#define SD_ACOMANDO_SEND_OP_COND                     41
#define SD_ACOMANDO_SET_WR_BLK_ERASE_COUNT           23

// Estos son los peores tiempos de espera definidos para las tarjetas de alta velocidad
#define SD_TIMEOUT_LECTURA_MS                        100
#define SD_TIMEOUT_ESCRITURA_MS                      250


/***************************************************************************************
** AREA DE DEFINICION DE TIPOS                                                        **
****************************************************************************************/
typedef struct {
    uint8_t dato[16];
} registroCSDsd_t;


/***************************************************************************************
** AREA DE DECLARACION DE VARIABLES                                                   **
****************************************************************************************/


/***************************************************************************************
** AREA DE PROTOTIPOS DE FUNCION                                                      **
****************************************************************************************/
uint32_t leerCampoBitCSDsd(uint8_t *buffer, uint32_t indiceBit, uint32_t lonBit);


#endif // __SD_ESTANDAR_H
