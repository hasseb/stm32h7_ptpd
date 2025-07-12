#include <limits.h>
#include "ethptp.h"
#include "ptpd/src/ptpd_constants.h"
#include "stm32h7xx_hal_eth.h"

// WARNING: This modules requires the STM32 Ethernet peripheral be initialized
// from within another module to function. Normally this is done by the module
// that initialize the STM32 Ethernet perhipheral for network communcation.

// Conversion from hardware to PTP format.
static uint32_t subsecond_to_nanosecond(uint32_t subsecond_value)
{
  uint64_t val = subsecond_value * 1000000000ll;
  val >>= 31;
  return val;
}

// Conversion from PTP to hardware format.
static uint32_t nanosecond_to_subsecond(uint32_t subsecond_value)
{
  uint64_t val = subsecond_value * 0x80000000ll;
  val /= 1000000000;
  return val;
}

/***
  **
  ** The functions below were ported from the old STM32 Standard Peripheral Library.
  **
  **/

/**
  * @brief  Updated the PTP block for fine correction with the Time Stamp Addend register value.
  * @param  None
  * @retval None
  */
void ETH_EnablePTPTimeStampAddend(void)
{
  /* Enable the PTP block update with the Time Stamp Addend register value */
  ETH->MACTSCR |= ETH_MACTSCR_TSADDREG;
}

/**
  * @brief  Enable the PTP Time Stamp interrupt trigger
  * @param  None
  * @retval None
  */
void ETH_EnablePTPTimeStampInterruptTrigger(void)
{
  /* Enable the PTP target time interrupt */
  ETH->MACTSCR |= ETH_MACIER_TSIE;
}

/**
  * @brief  Updated the PTP system time with the Time Stamp Update register value.
  * @param  None
  * @retval None
  */
void ETH_EnablePTPTimeStampUpdate(void)
{
  /* Enable the PTP system time update with the Time Stamp Update register value */
  ETH->MACTSCR |= ETH_MACTSCR_TSUPDT;
}

/**
  * @brief  Initialize the PTP Time Stamp
  * @param  None
  * @retval None
  */
void ETH_InitializePTPTimeStamp(void)
{
  /* Initialize the PTP Time Stamp */
  ETH->MACTSCR |= ETH_MACTSCR_TSINIT;
}

/**
  * @brief  Selects the PTP Update method
  * @param  UpdateMethod: the PTP Update method
  *   This parameter can be one of the following values:
  *     @arg ETH_PTP_FineUpdate   : Fine Update method
  *     @arg ETH_PTP_CoarseUpdate : Coarse Update method
  * @retval None
  */
void ETH_PTPUpdateMethodConfig(uint32_t UpdateMethod)
{
  /* Check the parameters */
  assert_param(IS_ETH_PTP_UPDATE(UpdateMethod));

  if (UpdateMethod != ETH_PTP_CoarseUpdate)
  {
    /* Enable the PTP Fine Update method */
    ETH->MACTSCR |= ETH_MACTSCR_TSCFUPDT;
  }
  else
  {
    /* Disable the PTP Coarse Update method */
    ETH->MACTSCR &= (~(uint32_t)ETH_MACTSCR_TSCFUPDT);
  }
}

/**
  * @brief  Enables or disables the PTP time stamp for transmit and receive frames.
  * @param  NewState: new state of the PTP time stamp for transmit and receive frames
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void ETH_PTPTimeStampCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the PTP time stamp for transmit and receive frames */
    ETH->MACTSCR |= ETH_MACTSCR_TSENA | ETH_MACTSCR_TSIPV4ENA | ETH_MACTSCR_TSIPV6ENA | ETH_MACTSCR_TSENALL;
  }
  else
  {
    /* Disable the PTP time stamp for transmit and receive frames */
    ETH->MACTSCR &= (~(uint32_t)ETH_MACTSCR_TSENA);
  }
}

/**
  * @brief  Checks whether the specified ETHERNET PTP flag is set or not.
  * @param  ETH_PTP_FLAG: specifies the flag to check.
  *   This parameter can be one of the following values:
  *     @arg ETH_PTP_FLAG_TSARU : Addend Register Update
  *     @arg ETH_PTP_FLAG_TSITE : Time Stamp Interrupt Trigger Enable
  *     @arg ETH_PTP_FLAG_TSSTU : Time Stamp Update
  *     @arg ETH_PTP_FLAG_TSSTI  : Time Stamp Initialize
  * @retval The new state of ETHERNET PTP Flag (SET or RESET).
  */
FlagStatus ETH_GetPTPFlagStatus(uint32_t ETH_PTP_FLAG)
{
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_ETH_PTP_GET_FLAG(ETH_PTP_FLAG));

  if ((ETH->MACTSCR & ETH_PTP_FLAG) != (uint32_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @brief  Sets the system time Sub-Second Increment value.
  * @param  SubSecondValue: specifies the PTP Sub-Second Increment Register value.
  * @retval None
  */
void ETH_SetPTPSubSecondIncrement(uint32_t SubSecondValue)
{
  /* Check the parameters */
  assert_param(IS_ETH_PTP_SUBSECOND_INCREMENT(SubSecondValue));

  /* Set the PTP Sub-Second Increment Register */
  ETH->MACSSIR = (SubSecondValue<<16);
}

/**
  * @brief  Sets the Time Stamp update sign and values.
  * @param  Sign: specifies the PTP Time update value sign.
  *   This parameter can be one of the following values:
  *     @arg ETH_PTP_PositiveTime : positive time value.
  *     @arg ETH_PTP_NegativeTime : negative time value.
  * @param  SecondValue: specifies the PTP Time update second value.
  * @param  SubSecondValue: specifies the PTP Time update sub-second value.
  *   This parameter is a 31 bit value, bit32 correspond to the sign.
  * @retval None
  */
void ETH_SetPTPTimeStampUpdate(uint32_t Sign, uint32_t SecondValue, uint32_t SubSecondValue)
{
  /* Check the parameters */
  assert_param(IS_ETH_PTP_TIME_SIGN(Sign));
  assert_param(IS_ETH_PTP_TIME_STAMP_UPDATE_SUBSECOND(SubSecondValue));

  /* Set the PTP Time Update High Register */
  ETH->MACSTSUR = SecondValue;

  /* Set the PTP Time Update Low Register with sign */
  ETH->MACSTNUR = Sign | SubSecondValue;
}

/**
  * @brief  Sets the Time Stamp Addend value.
  * @param  Value: specifies the PTP Time Stamp Addend Register value.
  * @retval None
  */
void ETH_SetPTPTimeStampAddend(uint32_t Value)
{
  /* Set the PTP Time Stamp Addend Register */
  ETH->MACTSAR = Value;
}

/**
  * @brief  Sets the Target Time registers values.
  * @param  HighValue: specifies the PTP Target Time High Register value.
  * @param  LowValue: specifies the PTP Target Time Low Register value.
  * @retval None
  */
void ETH_SetPTPTargetTime(uint32_t HighValue, uint32_t LowValue)
{
  /* Set the PTP Target Time High Register */
  ETH->MACSTSR = HighValue;
  /* Set the PTP Target Time Low Register */
  ETH->MACSTNR = LowValue;
}

/**
  * @brief  Get the specified ETHERNET PTP register value.
  * @param  ETH_PTPReg: specifies the ETHERNET PTP register.
  *   This parameter can be one of the following values:
  *     @arg ETH_PTPTSCR  : Time Stamp Control Register
  *     @arg ETH_PTPSSIR  : Sub-Second Increment Register
  *     @arg ETH_PTPTSHR  : Time Stamp High Register
  *     @arg ETH_PTPTSLR  : Time Stamp Low Register
  *     @arg ETH_PTPTSHUR : Time Stamp High Update Register
  *     @arg ETH_PTPTSLUR : Time Stamp Low Update Register
  *     @arg ETH_PTPTSAR  : Time Stamp Addend Register
  *     @arg ETH_PTPTTHR  : Target Time High Register
  *     @arg ETH_PTPTTLR  : Target Time Low Register
  * @retval The value of ETHERNET PTP Register value.
  */
uint32_t ETH_GetPTPRegister(uint32_t ETH_PTPReg)
{
  /* Check the parameters */
  assert_param(IS_ETH_PTP_REGISTER(ETH_PTPReg));

  /* Return the selected register value */
  return (*(__IO uint32_t *)(ETH_MAC_BASE + ETH_PTPReg));
}

/**
  * @brief  Sets the frequency of the PPS output.
  * @param  Freq: specifies the frequency of the PPS output in Hz as 2^Freq.
  * @retval None
  */
static void ETH_PTPSetPPSFreq(uint8_t Freq)
{
  ETH->MACPPSCR = Freq;
}

/* Examples of subsecond increment and addend values using HCLK = 240 MHz

 Addend * Increment = 2^63 / HCLK

 ptp_tick = Increment * 10^9 / 2^31

 +-----------+-----------+------------+
 | ptp tick  | Increment | Addend     |
 +-----------+-----------+------------+
 |   20 ns   |    43     | 0x35455A81 |
 +-----------+-----------+------------+
*/

#define ADJ_FREQ_BASE_ADDEND      0x35455A81
#define ADJ_FREQ_BASE_INCREMENT   43

// Update method is ETH_PTP_FineUpdate or ETH_PTP_CoarseUpdate.
void ethptp_start(uint32_t update_method)
{
  // Mask the time stamp trigger interrupt by setting bit 9 in the MACIMR register.
  ETH->MACIER &= ~(ETH_MACIER_TSIE);

  // Program Time stamp register bit 0 to enable time stamping.
  ETH_PTPTimeStampCmd(ENABLE);

  // Program the Subsecond increment register based on the PTP clock frequency.
  ETH_SetPTPSubSecondIncrement(ADJ_FREQ_BASE_INCREMENT); // to achieve 20 ns accuracy, the value is ~ 43

  if (update_method == ETH_PTP_FineUpdate)
  {
    // If you are using the Fine correction method, program the Time stamp addend register
    // and set Time stamp control register bit 5 (addend register update).
    ETH_SetPTPTimeStampAddend(ADJ_FREQ_BASE_ADDEND);
    ETH_EnablePTPTimeStampAddend();

    // Poll the Time stamp control register until bit 5 is cleared.
    while(ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSARU) == SET);
  }

  // To select the Fine correction method (if required),
  // program Time stamp control register  bit 1.
  ETH_PTPUpdateMethodConfig(update_method);

  // Program the Time stamp high update and Time stamp low update registers
  // with the appropriate time value.
  ETH_SetPTPTimeStampUpdate(ETH_PTP_PositiveTime, 0, 0);

  // Set Time stamp control register bit 2 (Time stamp init).
  ETH_InitializePTPTimeStamp();

  // Set PPS frequency to 128 Hz
	ETH_PTPSetPPSFreq(7);
}

// Get the PTP time.
void ethptp_get_time(ptptime_t *timestamp)
{
  int32_t hi_reg = 0;
  int32_t lo_reg = 0;
  int32_t hi_reg_after = 0;

  // The problem is we are reading two 32-bit registers that form
  // a 64-bit value, but it's possible the high 32-bits of the value
  // rolls over before we read the low 32-bits of the value. To avoid
  // this situation we read the high 32-bits twice and determine which
  // high 32-bits the low 32-bit are associated with.
  __disable_irq();
  hi_reg = ETH_GetPTPRegister(ETH_PTPTSHR);
  lo_reg = ETH_GetPTPRegister(ETH_PTPTSLR);
  hi_reg_after = ETH_GetPTPRegister(ETH_PTPTSHR);
  __enable_irq();

  // Did a roll over occur while reading?
  if (hi_reg != hi_reg_after)
  {
    // We now know a roll over occurred. If the rollover occured before
    // the reading of the low 32-bits we move the substitute the second
    // 32-bit high value for the first 32-bit high value.
    if (lo_reg < (INT_MAX / 2)) hi_reg = hi_reg_after;
  }

  // Now convert the raw registers values into timestamp values.
  timestamp->tv_nsec = subsecond_to_nanosecond(lo_reg);
  timestamp->tv_sec = hi_reg;
}

// Set the PTP time.
void ethptp_set_time(ptptime_t *timestamp)
{
  uint32_t sign;
  uint32_t second_value;
  uint32_t nanosecond_value;
  uint32_t subsecond_value;

  // Determine sign and correct second and nanosecond values.
  if (timestamp->tv_sec < 0 || (timestamp->tv_sec == 0 && timestamp->tv_nsec < 0))
  {
    sign = ETH_PTP_NegativeTime;
    second_value = -timestamp->tv_sec;
    nanosecond_value = -timestamp->tv_nsec;
  }
  else
  {
    sign = ETH_PTP_PositiveTime;
    second_value = timestamp->tv_sec;
    nanosecond_value = timestamp->tv_nsec;
  }

  // Convert nanosecond to subseconds.
  subsecond_value = nanosecond_to_subsecond(nanosecond_value);

  // Write the offset (positive or negative) in the Time stamp update
  // high and low registers.
  ETH_SetPTPTimeStampUpdate(sign, second_value, subsecond_value);

  // Set Time stamp control register bit 2 (Time stamp init).
  ETH_InitializePTPTimeStamp();

  // The Time stamp counter starts operation as soon as it is initialized
  // with the value written in the Time stamp update register.
  while (ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSSTI) == SET);
}

// Adjust the PTP system clock rate by the specified value in parts-per-billion.
void ethptp_adj_freq(int32_t adj_ppb)
{
  // Adjust the fixed base frequency by parts-per-billion.
  // addend = base + ((base * adj_ppb) / 1000000000);
	uint32_t addend = ADJ_FREQ_BASE_ADDEND + (int32_t) ((((int64_t) ADJ_FREQ_BASE_ADDEND) * adj_ppb) / 1000000000);

  /* 32bit estimation
  	ADJ_LIMIT = ((1l<<63)/275/ADJ_FREQ_BASE_ADDEND) = 11258181 = 11 258 ppm*/
  	if( adj_ppb > ADJ_FREQ_MAX) adj_ppb = ADJ_FREQ_MAX;
  	if( adj_ppb < -ADJ_FREQ_MAX) adj_ppb = -ADJ_FREQ_MAX;

  	//uint32_t addend = ((((275LL * adj_ppb)>>8) * (ADJ_FREQ_BASE_ADDEND>>24))>>6) + ADJ_FREQ_BASE_ADDEND;

  // Set the time stamp addend register with new rate value and set ETH_TPTSCR.
  ETH_SetPTPTimeStampAddend(addend);
  ETH_EnablePTPTimeStampAddend();
}

