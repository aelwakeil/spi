
/* STM32F103RGT6 ===> ROM 1MB, RAM 80k */
MEMORY
{
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 1M
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 80K
}

/* Include the common ld script. */
INCLUDE libopencm3_stm32f1.ld