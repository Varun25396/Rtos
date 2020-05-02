
/* Includes ------------------------------------------------------------------*/
#include "main.h"

//lecture - equal priority tasks, using different functions

/* FreeRTOS.org includes. */

//lecture - FreeRTOS.h includes many other includes of FreeRTOS
//lecture - refer to FreeRTOS.h for more comments
//lecture - task.h is needed for using task creation APIs and other objects

#include "FreeRTOS.h"
#include "task.h"

/* Used as a loop counter to create a very crude delay. */
#define mainDELAY_LOOP_COUNT            ( 0xfffff )

#define HEART_BEAT_PERIOD				( 2000 )


/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
//lecture - task prototypes
static void SystemClock_Config(void);
static void Error_Handler(void);




/* Private functions ---------------------------------------------------------*/
static void vTask_LED_ON( void *pvParameters );
static void vTask_LED_OFF( void *pvParameters );
 /*-----------------------------------------------------------*/

int xLastWakeTime1 = 0;

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
 /* This sample code shows how to use STM32F4xx GPIO HAL API to toggle PD12, PD13,
    PD14, and PD14 IOs (connected to LED4, LED3, LED5 and LED6 on STM32F401C-DISCO board (MB1115B))
    in an infinite loop.
    To proceed, 3 steps are required: */

  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
 
  /* Configure LED3, LED4, LED5 and LED6 */
  BSP_LED_Init(LED3);
//  BSP_LED_Init(LED4);
//  BSP_LED_Init(LED5);
//  BSP_LED_Init(LED6);

  /* Configure the system clock to 168 MHz */
  SystemClock_Config();
  //lecture - refer to task.h and tasks.c for more comments on xTaskCreate()
  //          and xTaskGenericCreate()
  //
  //lecture - in this implementation, the stack allocated will be used as task stack, which
  //          is independent of the main stack - this will be clear, if you look at the
  //          internals of task creation API and context switching mechanism
  //
  /* Create one of the two tasks. */
  xTaskCreate(    vTask_LED_ON,         /* Pointer to the function that implements the task. */
                                  "Task LED ON",       /* Text name for the task.  This is to facilitate debugging only. */
                                  240,            /* Stack depth in words. */
                                  NULL,           /* We are not using the task parameter. */
								  (tskIDLE_PRIORITY + 2UL),                      /* This task will run at priority 2. */
                                  NULL );         /* We are not using the task handle. */

  /* Create the other task in exactly the same way. */
  xTaskCreate( vTask_LED_OFF, "Task LED OFF", 240, NULL, (tskIDLE_PRIORITY + 2UL), NULL );

  /* Start the scheduler so our tasks start executing. */
//
  //
  //lecture - vTaskStartScheduler() initializes hw timer, creates idle task and loads the most
  //          eligible task on the processor - control is passed to the kernel from here !!!!
  //
  //
  //lecture - if vTaskStartScheduler() is successful,
  //
  //lecture - refer to task.h and tasks.c for more comments on vTaskStartScheduler()
  //
  //
  //
  vTaskStartScheduler();


  //lecture - we will reach here, if there is error - otherwise, we should never reach here

  /* If all is well we will never reach here as the scheduler will now be
  running.  If we do reach here then it is likely that there was insufficient
  heap available for the idle task to be created. */
  for( ;; );
  return 0;

}

/*-----------------------------------------------------------*/
#if 0
void vTask1( void *pvParameters )
{
const char *pcTaskName = "Task 1 is running\n";
volatile unsigned long ul;

        /* As per most tasks, this task is implemented in an infinite loop. */
        for( ;; )
        {
        	BSP_LED_On(LED3);
        	vTaskDelay( 50 / portTICK_RATE_MS );

        	BSP_LED_Off(LED3);
        	vTaskDelay( 150 / portTICK_RATE_MS );

        	BSP_LED_On(LED3);
        	vTaskDelay( 50 / portTICK_RATE_MS );

        	BSP_LED_Off(LED3);
        	vTaskDelay( 750 / portTICK_RATE_MS );

        }
}
#endif

static void vTask_LED_ON( void *pvParameters )
{
	portTickType xLocalLastWakeTime;

   xLastWakeTime1 = xTaskGetTickCount();
   xLocalLastWakeTime = xLastWakeTime1;

   vTaskDelayUntil( &xLocalLastWakeTime, ( 50 / portTICK_RATE_MS ) );
    /* As per most tasks, this task is implemented in an infinite loop. */
    for( ;; )
    {
    	BSP_LED_On(LED3);
    	vTaskDelay( 200 / portTICK_RATE_MS );
    	BSP_LED_On(LED3);
    	vTaskDelayUntil( &xLocalLastWakeTime, ( HEART_BEAT_PERIOD / portTICK_RATE_MS ) );
    }

}

static void vTask_LED_OFF( void *pvParameters )
{
	portTickType xLocalLastWakeTime;

   xLocalLastWakeTime = xLastWakeTime1;

   vTaskDelayUntil( &xLocalLastWakeTime, ( 100 / portTICK_RATE_MS ) );

    /* As per most tasks, this task is implemented in an infinite loop. */
    for( ;; )
    {
    	//vTaskDelay( 50 / portTICK_RATE_MS );
    	BSP_LED_Off(LED3);
    	vTaskDelay( 200 / portTICK_RATE_MS );
    	BSP_LED_Off(LED3);
    	vTaskDelayUntil( &xLocalLastWakeTime, ( HEART_BEAT_PERIOD / portTICK_RATE_MS ) );
    }

}


/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
//
//lecture - we may use hooks for
//          printing diagnostics, statistics and time-stamps ???
//
//lecture - typically invoked, when there is a heap allocation failure !!!
void vApplicationMallocFailedHook( void )
{
        /* This function will only be called if an API call to create a task, queue
        or semaphore fails because there is too little heap RAM remaining. */
        for( ;; );
}
/*-----------------------------------------------------------*/


//lecture - can be used, if we need to check stack overflow
//          there are different techniques to check stack overflow
//          use appropriate technique as per need
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
        /* This function will only be called if a task overflows its stack.  Note
        that stack overflow checking does slow down the context switch
        implementation. */
        for( ;; );
}
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
//lecture - we may use the idle hook to print the diagnostics and
//          system information - this will not interfere with
//          the real-time tasks


void vApplicationIdleHook( void )
{
        /* This example does not use the idle hook to perform any processing. */
}
/*-----------------------------------------------------------*/

//lecture - if need to process some code, during every system tick
//          processing ???

void vApplicationTickHook( void )
{
        /* This example does not use the tick hook to perform any processing. */
}




/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED5 on */
  BSP_LED_On(LED5);
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
