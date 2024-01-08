/***************************************************************************************
**  stm32f7xx.ld - Este fichero contiene la definicion del linker script de la familia 7
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

/* Definimos el stack */
_estack = ORIGIN(RAM_STACK) + LENGTH(RAM_STACK);

/* Tamanio del stack y heap*/
_Min_Heap_Size = 0x2000 ;                          
_Min_Stack_Size = 0x800 ;                            

/* Direcciones de la zona de configuracion de parametros */
inicioRegionConfig = ORIGIN(FLASH_CONFIG);
finRegionConfig = ORIGIN(FLASH_CONFIG) + LENGTH(FLASH_CONFIG);

/* Funcion inicial del programa */
ENTRY(Reset_Handler)

/* Definicion de las secciones */
SECTIONS
{
    /* Para los Cortex-M el principio del codigo de inicio se almacena en la seccion .isr_vector */
    .isr_vector :
    {
        . = ALIGN(512);
        PROVIDE(inicioTablaVectoresISR = .); 
        KEEP(*(.isr_vector))                /* Vector de interrupciones */
        . = ALIGN(4);
    } >FLASH_ARRANQUE

    /* El codigo principal va en la seccion .text section */
    .text :
    {
        . = ALIGN(4);
        *(.text)           /* Codigo */
        *(.text*)          /* Codigo */
        *(.rodata)         /* Constantes, strings, etc... */
        *(.rodata*)        /* Constantes, strings, etc... */
        *(.glue_7)         /* ARM a codigo thumb */
        *(.glue_7t)        /* Thumb a codigo ARM */
        *(.eh_frame)

        KEEP (*(.init))
        KEEP (*(.fini))

        . = ALIGN(4);
        _etext = .;        
    } >FLASH_PROGRAMA

    .preinit_array     :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
        . = ALIGN(4);
    } >FLASH_PROGRAMA
  
    .init_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
        . = ALIGN(4);
    } >FLASH_PROGRAMA
  
    .fini_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
        . = ALIGN(4);
    } >FLASH_PROGRAMA
  
    /* Las funciones criticas van en la ITCM RAM */
    _sicodigoRapido = LOADADDR(.codigoRapido); 
    .codigoRapido :
    {
        . = ALIGN(4);
        _scodigoRapido = .; 
        *(.codigoRapido)
        *(.codigoRapido*)
        . = ALIGN(4);
        _ecodigoRapido = .; 
    } >CODIGO_RAPIDO AT> FLASH_PROGRAMA

    .ARM.extab   : 
    { 
        *(.ARM.extab* .gnu.linkonce.armextab.*) 
    } >FLASH_PROGRAMA

    .ARM : 
    {
        __exidx_start = .;
        *(.ARM.exidx*) __exidx_end = .;
    } >FLASH_PROGRAMA

    .registroGP :
    {
        PROVIDE_HIDDEN (_sregistroGP = .);
        KEEP (*(.registroGP))
        KEEP (*(SORT(.registroGP.*)))
        PROVIDE_HIDDEN (_eregistroGP = .);
    } >FLASH_PROGRAMA

    .resetGP :
    {
        PROVIDE_HIDDEN (_sresetGP = .);
        KEEP (*(.resetGP))
        PROVIDE_HIDDEN (_eresetGP = .);
    } >FLASH_PROGRAMA

    /* Usado para iniciar la seccion .data */
    _sidata = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4);
        _sdata = .;      
        *(.data)           
        *(.data*)        

        . = ALIGN(4);
        _edata = .;      
    } >RAM AT> FLASH_PROGRAMA
    
    /* Seccion de datos RAM estandar sin inicializar */
    . = ALIGN(4);
    .bss (NOLOAD) : 
    {
        _sbss = .;       
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)

        . = ALIGN(4);
        _ebss = .;       
        __bss_end__ = _ebss;       
    } >RAM
    
    _sisram2_data = LOADADDR(.sram2_data);
    .sram2_data :
    {
        . = ALIGN(4);
        _ssram2_data = .;      
        *(.sram2_data)           
        *(.sram2_data*)        

        . = ALIGN(4);
        _esram2_data = .;      
    } >RAM2 AT> FLASH_PROGRAMA
    
    . = ALIGN(4);
    .sram2_bss (NOLOAD) : 
    {
        _ssram2_bss = .;         
        __sram2_bss__ = _ssram2_bss;
        *(.sram2_bss)
        *(.sram2_bss*)
        *(COMMON)

        . = ALIGN(4);
        _esram2_bss = .;       
        __sram2_bss_end__ = _esram2_bss;       
    } >RAM2
    
    _siramRapida_data = LOADADDR(.ramRapida_data);
    .ramRapida_data :
    {
        . = ALIGN(4);
        _sramRapida_data = .;       
        *(.ramRapida_data)          
        *(.ramRapida_data*)         

        . = ALIGN(4);
        _eramRapida_data = .;   
    } >RAM_RAPIDA AT >FLASH_PROGRAMA
      
    . = ALIGN(4);
    .ramRapida_bss (NOLOAD) :
    {
        _sramRapida_bss = .;  

        *(.ramRapida_bss)
        *(SORT_BY_ALIGNMENT(.ramRapida_bss*))

        . = ALIGN(4);
        _eramRapida_bss = .;  
    } >RAM_RAPIDA
      
    /*
     * Usado para validacion
     * Chequea si hay suficiente RAM para el stack. Deberia generar un error si no hay espacio
     */
    _heap_stack_end = ORIGIN(RAM_STACK) + LENGTH(RAM_STACK) - 8; /* 8 bytes para permitir el alineamiento */
    _heap_stack_begin = _heap_stack_end - _Min_Stack_Size  - _Min_Heap_Size;
    . = _heap_stack_begin;
    ._user_heap_stack :
    {
        . = ALIGN(4);
        PROVIDE ( end = . );
        PROVIDE ( _end = . );
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(4);
    } >RAM_STACK = 0xa5                     /* 0xa5 para comprobar en tiempo real el stack */
    
    /* Borra la informacion de las librerias del compilador */
    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }

    .ARM.attributes 0 : { *(.ARM.attributes) }
}