
MEMORY
{
  FLASH (rx)   :  ORIGIN = 0x08000000, LENGTH = 128K
  TCRAM (xrw)  :  ORIGIN = 0x10000000, LENGTH = 8K 
  RAM2 (xrw)    :  ORIGIN = 0x20000000, LENGTH = 32K
}

REGION_ALIAS("CODERAM", RAM2);
REGION_ALIAS("DTCRAM", TCRAM);
REGION_ALIAS("RAM", TCRAM);
REGION_ALIAS("ITCRAM", TCRAM);

INCLUDE nvcm_default_coderam.ld
