/** @brief Disable and turn off the ADC.
  * @param  ADC         Pointer to the ADC peripheral
  * @return             None.
  */
__INLINE static void ADC_DeInit(ADC_Type *ADC)
{
    ADC_SetStartMode(ADC_Start_None);
    SYSCON_DisablePowerLines(SYSCON_PowerMode_Run, SYSCON_PowerLine_ADC);
    SYSCON_DisableAHBClockLines(SYSCON_AHBClockLine_ADC);
}


/** @brief Initialize the ADC.
  * @param  ADC    Pointer to the ADC peripheral
  * @param  Init   Settings to initialize the ADC with
  * @return        None.
  */
__INLINE static void ADC_Init(ADC_Type *ADC, ADC_Init_Type *Init)
{
    lpclib_assert(ADC_IS_INPUT_MASK(Init->ChannelMask));
    lpclib_assert(Init->ClockDivisor <= 255);
    lpclib_assert(ADC_IS_BURST_MODE(Init->BurstMode));
    lpclib_assert(ADC_IS_BURST_RESOLUTION(Init->Resolution));

    SYSCON_EnablePowerLines(SYSCON_PowerMode_Run, SYSCON_PowerLine_ADC);
    SYSCON_EnableAHBClockLines(SYSCON_AHBClockLine_ADC);

    ADC->CR = (Init->ChannelMask << ADC_SEL_Shift)
              | (Init->ClockDivisor << ADC_CLKDIV_Shift)
              | (Init->BurstMode ? ADC_BURST : 0)
              | (Init->BurstResolution << ADC_CLKS_Shift);
}


/** @defgroup ADC_Initialization_Structure ADC Initialization Structure
  * @{
  */

/**
  * @brief ADC Initialization Settings Structure
  */
typedef struct {
    uint16_t ChannelMask;              /*!< Channels (1 bit per channel) to configure in ADC     */
    uint8_t  ClockDivisor;             /*!< ADC Clock Divisor                                    */
    uint8_t  BurstMode;                /*!< Whether to set Burst Mode                            */
    uint8_t  BurstResolution;          /*!< Resolution of Burst Readings                         */
} ADC_Init_Type;








/** @defgroup SSP_Initialization SSP Initialization
  * @{
  */

/*! @brief Synchronous Serial Peripheral Simple Initialization Structure */
typedef struct {
    uint8_t Mode;
    uint8_t WordLength;
    uint8_t FrameFormat;
    uint8_t ClockPolarity;
    uint8_t ClockPhase;
    uint8_t ClockPrescaler;
    uint8_t ClockRate;
} __attribute__((packed)) SSP_Init_Type;

/** @} */

#if 0
/** @brief Initialize SSP for Communications
  * @param  SSP         The SSP device to initialize
  * @param  Init        The set of configuration parameters to set on SSP
  * @return None.
  */
void SSP_Init(SSP_Type *SSP, SSP_InitType *Init)
{
    /* Validate Parameters */
    lpclib_assert(SSP_IS_WORD_LENGTH(Init->WordLength));
    lpclib_assert(SSP_IS_FRAME_FORMAT(Init->FrameFormat));
    lpclib_assert(SSP_IS_CLOCK_POLARITY(Init->ClockPolarity));
    lpclib_assert(SSP_IS_CLOCK_PHASE(Init->ClockPhase));
    lpclib_assert(SSP_IS_MODE(Init->Mode));
    lpclib_assert(SSP_IS_CLOCK_RATE(Init->ClockRate));
    lpclib_assert(Init->ClockPrescaler <= 254);
    lpclib_assert((Init->ClockPrescaler & 0x01) == 0);

    if (SSP == SSP0) {
        SYSCON_EnableAHBClockLines(SYSCON_AHBClockLine_SSP0);
        SYSCON_SetSSP0ClockDivider(Init->AHBClockDivisor);
        SYSCON_AssertPeripheralResets(SYSCON_PeripheralReset_SSP0);
        SYSCON_DeassertPeripheralResets(SYSCON_PeripheralReset_SSP0);
    } else {
        SYSCON_EnableAHBClockLines(SYSCON_AHBClockLine_SSP1);
        SYSCON_SetSSP1ClockDivider(Init->AHBClockDivisor);
        SYSCON_AssertPeripheralResets(SYSCON_ResetPeriph_SSP1);
        SYSCON_DeassertPeripheralResets(SYSCON_ResetPeriph_SSP1);
    }

    /* Set Clock Prescaler */
    SSP->CPSR = (Init->ClockPrescaler);

    /* Set line control parameters */
    SSP->CR0 = ((Init->WordLength - 1) | Init->FrameFormat
               | Init->ClockPolarity | Init->ClockPhase
               | (Init->ClockRate << 8));

    /* Set Master/Slave Mode and Enable the SSP */
    SSP->CR1 = Init->Mode;
    SSP->CR1 |= SSP_SSE;
}
#endif

/** @brief De-Initialize SSP (reset the SSP, which will clear config bits)
  * @param  SSP         The SSP device to de-initialize
  * @return None.
  */
__INLINE static void SSP_DeInit(SSP_Type *SSP)
{
    if (SSP == SSP0) {
        SYSCON_AssertPeripheralResets(SYSCON_PeripheralReset_SSP0);
    } else if (SSP == SSP1) {
        SYSCON_AssertPeripheralResets(SYSCON_PeripheralReset_SSP1);
    }
}


/** @brief Initialize the I2C controller
  * @param  Init        Settings with which to initialize the I2C controller
  * @return             None.
  */
__INLINE static void I2C_Init(I2C_Init_Type *Init)
{
}


