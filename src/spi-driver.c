/*
 * myhal_spi_driver.c
 *
 *  Created on: Jul 25, 2024
 *      Author: Christopher Edwards
 */

#include <spi-driver.h>

/**
 * Initialize GPIOA and SPI1 Clocks
 */
void __SPI_Clock_Init(void)
{
	RCC->AHB1ENR |= (1 << 0); // enable GPIOA
	MYHAL_RCC_SPI1_CLK_ENABLE(); // enable SPI1
//	RCC->APB2ENR |= (1 << 12); // enable SPI1
}

/**
 * Initialize GPIO Pins for SPI1 PA5-PA7. PA4 initialized in __SPI_Conf_SSM
 */
void __SPI_GPIO_Init(void)
{
	/**
	 * PA5 - SPI2 Clock
	 * PA6 - SPI2 MISO
	 * PA7 - SPI2 MOSI
	 */
	GPIOA->MODER |= (0b10 << 5 * 2) | (0b10 << 6 * 2) | (0b10 << 7 * 2); // AF Mode
	GPIOA->OTYPER &= ~((1 << 5) | (1 << 6) | (1 << 7)); // Push Pull Output
	GPIOA->PUPDR &= ~((0b11 << 5 * 2) | (0b11 << 6 * 2) | (0b11 << 7 * 2)); // No Pull
	GPIOA->OSPEEDR |= (0b11 << 5 * 2) | (0b11 << 6 * 2) | (0b11 << 7 * 2); // High Speed
	GPIOA->AFR[0] |= (0b0101 << 5 * 4) | (0b0101 << 6 * 4) | (0b0101 << 7 * 4); // Alt Func 5
}

/**
 * Enable SPI1 NVIC Interrupt
 */
static void __NVIC_SPI1_Enable(void)
{
	NVIC->ISER[1] |= (1 << 3);
}

/**
 * Configure master vs slave mode
 *
 * @param master_mode - 1 for master, 0 for slave
 */
static void __SPI_Conf_Mode(SPI_TypeDef *SPIx, uint32_t master_mode)
{
	if (master_mode)
	{
		SPIx->CR1 |= SPI_REG_CR1_MSTR;
	}
	else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_MSTR;
	}
}

/**
 * Configure direction of SPI
 *
 * @param dir - Either Full (0) or Half Duplex (1)
 */
static void __SPI_Conf_Dir(SPI_TypeDef *SPIx, uint32_t dir)
{
	if (dir)
	{
		SPIx->CR1 |= SPI_REG_CR1_BIDIMODE;
	}
	else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_BIDIMODE;
	}
}

/**
 * Configure Endianness of data
 *
 * @param lsbfirst - MSB first (0) or LSB first (1)
 */
static void __SPI_Conf_Endian(SPI_TypeDef *SPIx, uint8_t lsbfirst)
{
	if (lsbfirst)
	{
		SPIx->CR1 |= (1 << 7); // LSB first
	}
	else
	{
		SPIx->CR1 &= ~(1 << 7); // MSB First
	}
}

/**
 * Configure SPI data size
 *
 * @param datasize_16 -  16 bit (1) or 8 bit (0)
 */
static void __SPI_Conf_DataSize(SPI_TypeDef *SPIx, uint32_t datasize_16)
{
	if (datasize_16)
	{
	SPIx->CR1 |= SPI_REG_CR1_DFF;
	}
	else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_DFF;
	}
}

/**
 * Configures Baud Rate
 *
 * @param baud - 3 bit baud rate
 */
static void __SPI_Conf_BaudRate(SPI_TypeDef *SPIx, uint8_t baud)
{
	baud = baud & (0b00000111); // ensure 3 bits
	SPIx->CR1 |= (baud << 3);
}

/**
 * Configure Polarity and Phase
 */
static void __SPI_Conf_Phase_Polarity(SPI_TypeDef *SPIx, uint32_t phase, uint32_t polarity)
{
	if (phase)
	{
		SPIx->CR1 |= SPI_REG_CR1_CPHA;
	}
	else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_CPHA;
	}
	if (polarity)
	{
		SPIx->CR1 |= SPI_REG_CR1_CPOL;
	}
	else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_CPOL;
	}
}

/**
 * Configure MCU for SSM or HSM. Configures NSS GPIO Pin
 *
 * @param ssm_enable - 1 for SW Slave Management, 0 for HW Slave Management
 */
static void __SPI_Conf_SSM(SPI_TypeDef *SPIx, uint8_t ssm_enable)
{
	if (ssm_enable)
	{
		SPIx->CR1 |= SPI_REG_CR1_SSM;

		// PA4 GPIO Output Mode
		GPIOA->MODER |= (0b01 << 4 * 2); // AF Mode
		GPIOA->OTYPER &= ~(1 << 4); // Push Pull Output
		GPIOA->PUPDR &= ~(0b11 << 4 * 2); // No Pull
		GPIOA->OSPEEDR &= ~(0b11 << 4 * 2); // High Speed
	}
	else
	{
		SPIx->CR1 &= ~(SPI_REG_CR1_SSM);
		SPIx->CR2 |= SPI_REG_CR2_SSOE; // Enable NSS pin

		// PA4 NSS Mode
		GPIOA->MODER |= (0b10 << 4 * 2); // AF Mode
		GPIOA->OTYPER &= ~(1 << 4); // Push Pull Output
		GPIOA->PUPDR &= ~(0b11 << 4 * 2); // No Pull
		GPIOA->OSPEEDR |= (0b11 << 4 * 2); // High Speed
		GPIOA->AFR[0] |= (0b0101 << 4 * 4); // Alt Func 5
	}
	SPI1->CR2 |= (1 << 2); // Set SSOE
}

/**
 * Enable SPI device
 */
static void __SPI_Enable(SPI_TypeDef *SPIx)
{
	SPIx->CR1 |= SPI_REG_CR1_SPE;
}

/**
 * Disable SPI device
 */
static void __SPI_Disable(SPI_TypeDef *SPIx)
{
	SPIx->CR1 &= ~SPI_REG_CR1_SPE;
}

/*
 * Enables TXE Interrupt
 */
static void __SPI_INT_TXE_Enable(SPI_TypeDef *SPIx)
{
	SPIx->CR2 |= SPI_REG_CR2_TXEIE_ENABLE;
}

/*
 * Enables RXNE Interrupt
 */
static void __SPI_INT_RXNE_Enable(SPI_TypeDef *SPIx)
{
	SPIx->CR2 |= SPI_REG_CR2_RXNEIE_ENABLE;
}

/**
 * Disables TXE Interrupt
 */
static void __SPI_INT_TXE_Disable(SPI_TypeDef *SPIx)
{
	SPIx->CR2 &= ~SPI_REG_CR2_TXEIE_ENABLE;
}

/**
 * Disables RXNE Interrupt
 */
static void __SPI_INT_RXNE_Disable(SPI_TypeDef *SPIx)
{
	SPIx->CR2 &= ~SPI_REG_CR2_RXNEIE_ENABLE;
}

/**
 * Disables TXE interrupt. Puts SPI in ready state.
 * Gets called once TX Buffer is empty
 */
static void __SPI_INT_TXE_Close(spi_handler_t *spi_handler)
{
	while (spi_handler->State == SPI_STATE_BUSY);

	__SPI_INT_TXE_Disable(spi_handler->Instance);
	spi_handler->State = SPI_STATE_READY;
}

/**
 * Close RXNE interrupt. Puts SPI in ready state.
 * Called when no more RX Data to be ready
 */
static void __SPI_INT_RXNE_Close(spi_handler_t *spi_handler)
{
	// Wait for SPI to finish transmission
	while (spi_handler->State == SPI_STATE_BUSY);

	__SPI_INT_RXNE_Disable(spi_handler->Instance);
	spi_handler->State = SPI_STATE_READY;
}


/**
 * Initialize SPI device
 *
 * @param	spi_handler - base address of SPI peripheral
 */
void SPI_Init(spi_handler_t *spi_handler)
{
	__SPI_GPIO_Init();
	__SPI_Clock_Init();

	__SPI_Conf_Mode(spi_handler->Instance, spi_handler->Init.Mode);
	__SPI_Conf_Dir(spi_handler->Instance, spi_handler->Init.Direction);
	__SPI_Conf_Endian(spi_handler->Instance, spi_handler->Init.FirstBit);
	__SPI_Conf_DataSize(spi_handler->Instance, spi_handler->Init.DataSize);
	__SPI_Conf_BaudRate(spi_handler->Instance, spi_handler->Init.Prescaler);
	__SPI_Conf_Phase_Polarity(spi_handler->Instance, spi_handler->Init.CLKPhase, spi_handler->Init.CLKPolarity);
	__SPI_Conf_SSM(spi_handler->Instance, spi_handler->Init.NSS);
}

/**
 * Transmit data across MOSI Line. Discards received data
 *
 * @param tx_data_buf - pointer to buffer containing TX data
 * @param size - number of bytes of data to be sent
 */
void SPI_Transmit(spi_handler_t *spi_handler, BYTE *tx_data_buf, UINT size)
{
	if ( (spi_handler->Instance->SR & (SPI_REG_CR1_SPE)) == RESET)
	{
//		__SPI_Enable(spi_handler->Instance);
		SPI1->CR1 = 876U;
	}


	spi_handler->pTxBuf = tx_data_buf;
	spi_handler->TxCount = size;

	while (spi_handler->TxCount > 0)
	{
		// Wait for TX data to be sent
		while ( (spi_handler->Instance->SR & SPI_REG_SR_TXE_FLAG) == RESET);

		// Send data
		spi_handler->Instance->DR = *spi_handler->pTxBuf;
		spi_handler->pTxBuf++;
		spi_handler->TxCount--;

		// Wait for transaction to finish
		while( (spi_handler->Instance->SR & SPI_REG_SR_RXNE_FLAG) == RESET
				&& (spi_handler->Instance->SR & SPI_REG_SR_BUSY_FLAG) != RESET);

		// Read out and discard RX data to avoid Overrun Error
		spi_handler->Instance->DR;
	}
}

/**
 * Receives data across MISO Line. Transmits dummy data
 *
 * @param rx_data_buf - pointer to buffer containing received RX data
 * @param size - number of bytes of data to be received
 */
void SPI_Receive(spi_handler_t *spi_handler, BYTE *rx_data_buf, UINT size)
{
	if ( (spi_handler->Instance->SR & (SPI_REG_CR1_SPE)) == RESET)
	{
		__SPI_Enable(spi_handler->Instance);
	}
	spi_handler->pTxBuf = (uint8_t *)NULL;
	spi_handler->TxCount = 0;

	spi_handler->pRxBuf = rx_data_buf;
	spi_handler->RxCount = size;

	while (spi_handler->RxCount > 0)
	{
		// Wait TX data to be sent
		while ( (spi_handler->Instance->SR & SPI_REG_SR_TXE_FLAG) == RESET);

		// Send dummy data
		spi_handler->Instance->DR = *spi_handler->pTxBuf;

		// Wait for RX data to be received
		while( (spi_handler->Instance->SR & SPI_REG_SR_RXNE_FLAG) == RESET);

		// Receive data
		*spi_handler->pRxBuf = *(uint8_t*)&spi_handler->Instance->DR;
		spi_handler->pRxBuf++;
		spi_handler->RxCount--;

		// Wait for transaction to finish
		while ( (spi_handler->Instance->SR & SPI_REG_SR_BUSY_FLAG) != RESET);
	}
}

/**
 * Transmit data across MOSI Line and receive data across MISO Line
 *
 * @param tx_data_buf - pointer to buffer containing TX data
 * @param rx_data_buf - pointer to buffer containing received RX data
 * @param size - number of bytes of data to be sent and received
 */
void SPI_TransmitReceive(spi_handler_t *spi_handler, BYTE *tx_data_buf, BYTE *rx_data_buf, UINT size)
{
	if ( (spi_handler->Instance->SR & (SPI_REG_CR1_SPE)) == RESET)
	{
		__SPI_Enable(spi_handler->Instance);
	}

	spi_handler->pTxBuf = tx_data_buf;
	spi_handler->TxCount = size;

	spi_handler->pRxBuf = rx_data_buf;
	spi_handler->RxCount = size;

	while (spi_handler->TxCount > 0)
	{
		// Wait TX data to be sent
		while ( (spi_handler->Instance->SR & SPI_REG_SR_TXE_FLAG) == RESET);

		// Send data
		spi_handler->Instance->DR = *spi_handler->pTxBuf;
		spi_handler->pTxBuf++;
		spi_handler->TxCount--;

		// Wait for RX data to be received
		while( (spi_handler->Instance->SR & SPI_REG_SR_RXNE_FLAG) == RESET);

		// Receive data
		*spi_handler->pRxBuf = *(uint8_t*)&spi_handler->Instance->DR;
		spi_handler->pRxBuf++;
		spi_handler->RxCount--;

		// Wait for transaction to finish
		while ( (spi_handler->Instance->SR & SPI_REG_SR_BUSY_FLAG) != RESET);
	}
}

/**
 * Transmit data across MOSI Line in Interrupt Mode. Discards received data
 *
 * @param tx_data_buf - pointer to buffer containing TX data
 * @param size - number of bytes of data to be sent
 */
void SPI_Transmit_IT(spi_handler_t *spi_handler, uint8_t *tx_data_buf, uint32_t len)
{
	spi_handler->pTxBuf = tx_data_buf;
	spi_handler->TxCount = len; // decrements as data transmitted

	spi_handler->State = SPI_STATE_BUSY_TX;


	// NVIC SPI1 Interrupt enable
	if ( !(NVIC->ISER[1] & (1 << 3)) )
	{
		__NVIC_SPI1_Enable();
	}

	if ((spi_handler->Instance->CR1 & SPI_REG_CR1_SPE) == RESET)
	{
		__SPI_Enable(spi_handler->Instance);
	}

	/* This function gets called to TX the tx_data_buf right? So before we've TX'd anything, this function gets called
	 * to kick off the process. Obviously the TX buf is going to be empty before we load anything into it. So once we
	 * set the TXEIE bit, the TXE interrupt will be generated and it'll start sending out the tx_data_buf. */
	__SPI_INT_TXE_Enable(spi_handler->Instance); // enables TX empty interrupt when TX done
}


/**
 *  Receives data across MISO Line in Interrupt mode. Transmits dummy data
 *
 * @param rx_data_buf - pointer to buffer containing received RX data
 * @param size - number of bytes of data to be received
 *
 */
void SPI_Receive_IT(spi_handler_t *spi_handler, uint8_t *rx_data_buf, uint32_t size)
{
	spi_handler->pTxBuf = (uint8_t *)NULL;
	spi_handler->TxCount = 0;

	spi_handler->pRxBuf = rx_data_buf;
	spi_handler->RxCount = size;

	// NVIC SPI1 Interrupt enable
	if ( !(NVIC->ISER[1] & (1 << 3)) )
	{
		__NVIC_SPI1_Enable();
	}

	spi_handler->State = SPI_STATE_BUSY_TX;

	if ((spi_handler->Instance->CR1 & SPI_REG_CR1_SPE) == RESET)
	{
		__SPI_Enable(spi_handler->Instance);
	}

	// Empty the DR, resets RXNE so false RXNE not generated immediately
	spi_handler->Instance->DR;

	__SPI_INT_RXNE_Enable(spi_handler->Instance);

}

/**
 * Transmit data across MOSI Line and receive data across MISO Line in Interrupt Mode
 *
 * @param tx_data_buf - pointer to buffer containing TX data
 * @param rx_data_buf - pointer to buffer containing received RX data
 * @param size - number of bytes of data to be sent and received
 */
void SPI_TransmitReceive_IT(spi_handler_t *spi_handler, uint8_t *tx_data_buf, uint32_t size_tx, uint8_t *rx_data_buf, uint32_t size_rx)
{

	spi_handler->pTxBuf = tx_data_buf;
	spi_handler->TxCount = size_tx;

	spi_handler->pRxBuf = rx_data_buf;
	spi_handler->RxCount = size_rx;

	spi_handler->State = SPI_STATE_BUSY_RX;

	// NVIC SPI1 enable
	if ( !(NVIC->ISER[1] & (1 << 3)) )
	{
		__NVIC_SPI1_Enable();
	}

	// Empty the DR, resets RXNE
	spi_handler->Instance->DR;

	if ( !(spi_handler->Instance->CR1 & SPI_REG_CR1_SPE) )
	{
		__SPI_Enable(spi_handler->Instance);
	}

	// Enable TXE and RXNE Interrupts
	__SPI_INT_RXNE_Enable(spi_handler->Instance);
	__SPI_INT_TXE_Enable(spi_handler->Instance);
}

/**
 * Determines which SPI interrupt (TXE and/or RXNE) to handle
 */
void SPI_INT_IRQ_Handler(spi_handler_t *spi_handler)
{
	uint32_t status_reg = spi_handler->Instance->SR;
	uint32_t cr2_reg = spi_handler->Instance->CR2;

	// Check RXNE Interrupt event
	if ( (status_reg & SPI_REG_SR_RXNE_FLAG) && (cr2_reg & SPI_REG_CR2_RXNEIE_ENABLE) )
	{
		SPI_INT_RXNE_Handler(spi_handler);
		return;
	}

	// Check TXE Interrupt event
	if ( (status_reg & SPI_REG_SR_TXE_FLAG) && (cr2_reg & SPI_REG_CR2_TXEIE_ENABLE) )
	{
		SPI_INT_TXE_Handler(spi_handler);
		return;
	}
}

/**
 * Handles TXE interrupt, only gets called when TXEIE = 1, TXE = 1
 * Calls from within master or slave TX, so TX buffer address already in pTxBuf
 */
void SPI_INT_TXE_Handler(spi_handler_t *spi_handler)
{
	// TX in 8-bit mode
	if (spi_handler->Init.DataSize == SPI_8BIT_DF_ENABLE)
	{
		spi_handler->Instance->DR = (*spi_handler->pTxBuf++);
		spi_handler->TxCount--;
	}

	// TX in 16-bit mode
	else
	{
		spi_handler->Instance->DR = *( (uint16_t*)spi_handler->pTxBuf ); // sends the first 2 bytes
		spi_handler->pTxBuf+=2;
		spi_handler->TxCount-=2;
	}

	// No more data to TX
	if (spi_handler->TxCount == 0)
	{
		__SPI_INT_TXE_Close(spi_handler);
	}
}


/**
 * Handles RXNE interrupt, only gets called when RXNEIE=1 and RXNE=1
 */
void SPI_INT_RXNE_Handler(spi_handler_t *spi_handler)
{
	// RX 8-bit
	if (spi_handler->Init.DataSize == SPI_8BIT_DF_ENABLE)
	{
		*spi_handler->pRxBuf = *(uint8_t *)&spi_handler->Instance->DR;
		spi_handler->pRxBuf++;
		spi_handler->RxCount--;
	}

	// RX 16-bit
	else
	{
		*spi_handler->pRxBuf = *(uint16_t *)&spi_handler->Instance->DR;
		spi_handler->pRxBuf+=2;
		spi_handler->RxCount-=2;
	}

	// No more data to RX
	if (spi_handler->RxCount == 0)
	{
		__SPI_INT_RXNE_Close(spi_handler);
	}
}

