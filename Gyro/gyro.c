#include "stm32f4xx.h"

int main(void)
{
/*At this stage the microcontroller clock setting is already configured, 
  this is done through SystemInit() function which is called from startup
  files (startup_stm32f40xx.s/startup_stm32f427x.s) before to branch to 
  application main. 
  To reconfigure the default setting of SystemInit() function, refer to
  system_stm32f4xx.c file
*/  
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);

	while (1) {

	}
}