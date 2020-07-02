/**
  ******************************************************************************
  * @file    UART/UART_HyperTerminal_DMA/Src/stm32l1xx_it.c
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_it.h"
#include "stm32l1xx_hal.h"

// extern PCD_HandleTypeDef hpcd_USB_FS;
// void USB_HP_IRQHandler(void)
// {
//   /* USER CODE BEGIN USB_HP_IRQn 0 */

//   /* USER CODE END USB_HP_IRQn 0 */
//   HAL_PCD_IRQHandler(&hpcd_USB_FS);
//   /* USER CODE BEGIN USB_HP_IRQn 1 */

//   /* USER CODE END USB_HP_IRQn 1 */
// }

// /**
//   * @brief This function handles USB low priority interrupt.
//   */
// void USB_LP_IRQHandler(void)
// {
//   /* USER CODE BEGIN USB_LP_IRQn 0 */

//   /* USER CODE END USB_LP_IRQn 0 */
//   HAL_PCD_IRQHandler(&hpcd_USB_FS);
//   /* USER CODE BEGIN USB_LP_IRQn 1 */

//   /* USER CODE END USB_LP_IRQn 1 */
// }
/** @addtogroup STM32L1xx_HAL_LL_MIX_Examples
  * @{
  */

/** @addtogroup UART_HyperTerminal_IT
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
/*
void HardFault_Handler(void)
{
  while (1)
  {
  }
}
*/

/*!
 * \brief  This function handles Hard Fault exception.
 * \param  None
 * \retval None
 */


#if defined( HARD_FAULT_HANDLER_ENABLED )
void HardFault_Handler_C( unsigned int *args )
{
    volatile unsigned int stacked_r0;
    volatile unsigned int stacked_r1;
    volatile unsigned int stacked_r2;
    volatile unsigned int stacked_r3;
    volatile unsigned int stacked_r12;
    volatile unsigned int stacked_lr;
    volatile unsigned int stacked_pc;
    volatile unsigned int stacked_psr;

    stacked_r0 = ( ( unsigned long) args[0] );
    stacked_r1 = ( ( unsigned long) args[1] );
    stacked_r2 = ( ( unsigned long) args[2] );
    stacked_r3 = ( ( unsigned long) args[3] );

    stacked_r12 = ( ( unsigned long) args[4] );
    stacked_lr = ( ( unsigned long) args[5] );
    stacked_pc = ( ( unsigned long) args[6] );
    stacked_psr = ( ( unsigned long) args[7] );

    ( void )stacked_r0;
    ( void )stacked_r1;
    ( void )stacked_r2;
    ( void )stacked_r3;

    ( void )stacked_r12;
    ( void )stacked_lr ;
    ( void )stacked_pc ;
    ( void )stacked_psr;

    while( 1 );
}

#if defined(__CC_ARM)
__asm void HardFault_Handler(void)
{
    TST LR, #4
    ITE EQ
    MRSEQ r0, MSP
    MRSNE r0, PSP
    B __cpp(HardFault_Handler_C)
}
#elif defined(__ICCARM__)
void HardFault_Handler(void)
{
    __asm("TST LR, #4");
    __asm("ITE EQ");
    __asm("MRSEQ r0, MSP");
    __asm("MRSNE r0, PSP");
    __asm("B HardFault_Handler_C");
}
#elif defined(__GNUC__)
void HardFault_Handler(void)
{
    __asm volatile( "TST LR, #4" );
    __asm volatile( "ITE EQ" );
    __asm volatile( "MRSEQ R0, MSP" );
    __asm volatile( "MRSNE R0, PSP" );
    __asm volatile( "B HardFault_Handler_C" );
}
#else
    #warning Not supported compiler type
#endif

#endif


/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
   /* Go to infinite loop when Usage Fault exception occurs */
    while ( 1 )
    {
    }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
/*
void SysTick_Handler(void)
{
  //HAL_IncTick();
}
*/
/******************************************************************************/
/*                 STM32L1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s), for the        */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l1xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles UART interrupt request.
  * @param  None
  * @retval None
  */
void USARTx_IRQHandler(void)
{

}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
