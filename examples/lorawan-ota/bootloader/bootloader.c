#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "stm32l1xx.h"
#include "board.h"
#include "ota.h"

#define FIFO_TX_SIZE      128
uint8_t UARTTxBuffer[FIFO_TX_SIZE];

#define FIFO_RX_SIZE      128
uint8_t UARTRxBuffer[FIFO_RX_SIZE];
Uart_t Uart1;

static uint8_t IrqNestLevel = 0;

void BoardDisableIrq( void )
{
  __disable_irq( );
  IrqNestLevel++;
}

void BoardEnableIrq( void )
{
  IrqNestLevel--;
  if( IrqNestLevel == 0 )
  {
    __enable_irq( );
  }
}

void SysTick_Handler( void )
{
  HAL_IncTick( );
  HAL_SYSTICK_IRQHandler( );
}

void SystemClockConfig( void )
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __HAL_RCC_PWR_CLK_ENABLE( );

  __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;

  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2; //16*4/2=32

  RCC_OscInitStruct.HSICalibrationValue=(uint32_t)((RCC->ICSCR & RCC_ICSCR_HSITRIM) >> POSITION_VAL(RCC_ICSCR_HSITRIM));

  if( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
  {
    assert_param( FAIL );
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 ) != HAL_OK )
  {
    assert_param( FAIL );
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit ) != HAL_OK )
  {
    assert_param( FAIL );
  }

  HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq( ) / 1000 );
  
  HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );
  
  // HAL_NVIC_GetPriorityGrouping
  HAL_NVIC_SetPriorityGrouping( NVIC_PRIORITYGROUP_4 );
  
  // SysTick_IRQn interrupt configuration
  HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}

void  DebugPrintf (char  *p_fmt, ...)
{
    // if(1)
    // {
    //     char    str[200u];
    //     uint8_t  len;

    //     va_list     vArgs;


    //     va_start(vArgs, p_fmt);

    //     vsprintf((char       *)str,
    //              (char const *)p_fmt,
    //                            vArgs);

    //     va_end(vArgs);

    //     len = strlen(str);

    //     UartPutBuffer(&Uart1, (uint8_t *)str, len);
    // }

}

typedef void (*pFunction)(void); /*!< Function pointer definition */

void
jump_to_image(uint32_t destination_address)
{
    uint32_t i;
    uint32_t  JumpAddress = *(__IO uint32_t*)(destination_address + OTA_METADATA_SPACE + 4);
    pFunction Jump        = (pFunction)JumpAddress;

    DebugPrintf("Jump = %x \r\n", Jump);
    DebugPrintf("_estack = %x \r\n", *(__IO uint32_t*)(destination_address + OTA_METADATA_SPACE));

    for(i = 0; i < 1000000; i++);

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;
    HAL_RCC_DeInit();
    HAL_DeInit();

    __set_MSP(*(__IO uint32_t*)(destination_address + OTA_METADATA_SPACE));
    Jump();
}

int
main(void)
{
  uint32_t i;
  //SCB->VTOR = 0x8004000;
  HAL_Init( );
  SystemClockConfig( );
  FLASH_Init();

  // FifoInit( &Uart1.FifoRx, UARTRxBuffer, FIFO_RX_SIZE );
  // FifoInit( &Uart1.FifoTx, UARTTxBuffer, FIFO_TX_SIZE );
  // Uart1.IrqNotify = NULL;
  // UartInit( &Uart1, UART_1, UART_TX, UART_RX );
  // UartConfig( &Uart1, RX_TX, 115200, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );
  // UartPutBuffer(&Uart1, "-------ota-bootloader-------\r\n", strlen("-------ota-bootloader-------\r\n"));

  //  (1) Get the metadata of whatever firmware is currently installed
  OTAMetadata_t current_firmware;
  get_current_metadata( &current_firmware );

  //  (2) Verify the current firmware! (Recompute the CRC over the internal flash image)
  verify_current_firmware( &current_firmware );

  //  (3) Are there any newer firmware images in ext-flash?
  uint8_t newest_ota_slot = find_newest_ota_image();
  OTAMetadata_t newest_firmware;
  while( get_ota_slot_metadata( newest_ota_slot, &newest_firmware ) );

  //  (4) Is the current image valid?
  if ( validate_ota_metadata( &current_firmware ) ) {
    if ( ( newest_ota_slot > 0 ) && (newest_firmware.version > current_firmware.version) ) {
      //  If there's a newer firmware image than the current firmware, install
      //  the newer version!
      update_firmware( newest_ota_slot );
      while(1);
      NVIC_SystemReset(); // reboot
    } else {
      //  If our image is valid, and there's nothing newer, then boot the firmware.
      //while(1);
      jump_to_image( (CURRENT_FIRMWARE<<12) );
    }
  } else {
    //  If our image is not valid, install the newest valid image we have.
    //  Note: This can be the Golden Image, when newest_ota_slot = 0.
    update_firmware( newest_ota_slot );
    while(1);
    NVIC_SystemReset(); // reboot
  }
  //  main() *should* never return - we should have rebooted or branched
  //  to other code by now.
  while(1);
  return 0;
}
