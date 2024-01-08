/***************************************************************************************
**  stm32f767vgtx.ld - Este fichero contiene la definicion del linker script
**
**
**  Este fichero forma parte del proyecto URpilot.
**  Codigo desarrollado por el grupo de investigacion ICON de la Universidad de La Rioja
**
**  Autor: Ramon Rico
**  Fecha de creacion: 04/12/2020
**  Fecha de modificacion: 04/12/2020
**
**  El proyecto URpilot NO es libre. No se puede distribuir y/o modificar este fichero
**  bajo ningun concepto.
**
**  En caso de modificacion y/o solicitud de informacion pongase en contacto con
**  el grupo de investigacion GIS a traves de: www.unirioja.es/urpilot
**
****************************************************************************************/

/* Definicion de la memoria */
MEMORY
{
    ITCM_RAM (rx)                  : ORIGIN = 0x00000000, LENGTH = 16K 

    ITCM_FLASH_STARTUP (rx)        : ORIGIN = 0x00200000, LENGTH = 32K 
    ITCM_FLASH_CONFIG (r)          : ORIGIN = 0x00208000, LENGTH = 32K 
    ITCM_FLASH_PROGRAM (rx)        : ORIGIN = 0x00210000, LENGTH = 960K 
    
    AXIM_FLASH_STARTUP (rx)        : ORIGIN = 0x08000000, LENGTH = 32K
    AXIM_FLASH_CONFIG (r)          : ORIGIN = 0x08008000, LENGTH = 32K
    AXIM_FLASH_PROGRAM (rx)        : ORIGIN = 0x08010000, LENGTH = 960K

    DTCM_RAM (rwx)                 : ORIGIN = 0x20000000, LENGTH = 128K
    SRAM1 (rwx)                    : ORIGIN = 0x20020000, LENGTH = 368K 
    SRAM2 (rwx)                    : ORIGIN = 0x2007C000, LENGTH = 16K 
}

REGION_ALIAS("CODIGO_RAPIDO", ITCM_RAM)

REGION_ALIAS("FLASH_ARRANQUE", AXIM_FLASH_STARTUP)
REGION_ALIAS("FLASH_CONFIG", AXIM_FLASH_CONFIG)
REGION_ALIAS("FLASH_PROGRAMA", AXIM_FLASH_PROGRAM)

REGION_ALIAS("RAM_STACK", DTCM_RAM)
REGION_ALIAS("RAM_RAPIDA", DTCM_RAM)
REGION_ALIAS("RAM", DTCM_RAM)
REGION_ALIAS("RAM2", SRAM2)

INCLUDE "stm32f7xx.ld"
