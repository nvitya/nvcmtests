MEMORY
{
  TCRAM (xrw) :  ORIGIN = 0x00000000, LENGTH = 64K
}

REGION_ALIAS("TEXT",  TCRAM);
REGION_ALIAS("RAM",  TCRAM);
REGION_ALIAS("RAM2", TCRAM);
REGION_ALIAS("CODERAM", TCRAM);
REGION_ALIAS("ITCRAM", TCRAM);
REGION_ALIAS("DTCRAM", TCRAM);

INCLUDE nvcm_default_noflash.ld

/* Debugger Setup for this linker script:

- GDB Initialization Commands:
    set {int}0x40023844 = 0x4000  (enables SYSCTRL)
    set {int}0x40013800 = 3       (remaps the SRAM to 0x0000 0000)                  
- set RAM Application
- disable "Pre-run/Restart reset"
- Use Generic Cortex-M4 target with J-Link (without Flash support)

*/