/*
 * spi-driver.h
 *
 *  Created on: Jul 25, 2024
 *      Author: Christopher Edwards
 */

#ifndef INC_SPI_DRIVER_H_
#define INC_SPI_DRIVER_H_

#include "stm32f4xx_hal.h"
#include "integer.h"

/* SPI_CR1 Bit definitions */
#define SPI_REG_CR1_BIDIMODE		( (uint32_t) 1 << 15)
#define SPI_ENABLE_2_LINE_UNI_DIR	0
#define SPI_ENABLE_1_LINE_BIDI		1

#define SPI_REG_CR1_DFF				( (uint32_t) 1 << 11)
#define SPI_8BIT_DF_ENABLE			0
#define SPI_16BIT_DF_ENABLE			1

#define SPI_REG_CR1_SSM				( (uint32_t) 1 << 9)
#define SPI_SSM_ENABLE				1
#define SPI_SSM_DISABLE				0

#define SPI_REG_CR1_SSI				( (uint32_t) 1 << 8)

#define SPI_CR1_LSB_FIRST			( (uint32_t) 1 << 7)
#define SPI_TX_MSB_FIRST			0
#define SPI_TX_LSB_FIRST			1

#define SPI_REG_CR1_SPE				( (uint32_t ) 1 << 6)

#define SPI_REG_CR1_BR_PCLK_DIV_2	( (uint32_t) 0 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_4	( (uint32_t) 1 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_8	( (uint32_t) 2 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_16	( (uint32_t) 3 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_32	( (uint32_t) 4 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_64	( (uint32_t) 5 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_128	( (uint32_t) 6 << 3)
#define SPI_REG_CR1_BR_PCLK_DIV_256	( (uint32_t) 7 << 3)

#define SPI_REG_CR1_MSTR			( (uint32_t) 1 << 2)
#define SPI_MASTER_MODE_SEL			1
#define SPI_SLAVE_MODE_SEL			0

#define SPI_REG_CR1_CPOL			( (uint32_t) 1 << 1)

#define SPI_CPOL_LOW				0
#define SPI_CPOL_HIGH				1

#define SPI_REG_CR1_CPHA			( (uint32_t) 1 << 0)
#define SPI_FIRST_CLOCK_TRANS		0
#define SPI_SECOND_CLOCK_TRANS		1

/* SPI_CR2 Bit definitions */
#define SPI_REG_CR2_TXEIE_ENABLE	( (uint32_t) 1 << 7)
#define SPI_REG_CR2_RXNEIE_ENABLE	( (uint32_t) 1 << 6)
#define SPI_REG_CR2_ERRIE_ENABLE	( (uint32_t) 1 << 5)

#define SPI_REG_CR2_FRAME_FORMAT	( (uint32_t) 1 << 4)
#define SPI_MOTOROLA_MODE			0
#define SPI_TI_MODE					1

#define SPI_REG_CR2_SSOE			( (uint32_t) 1 << 2)

/* SPI_SR Bit definitions */
#define SPI_REG_SR_FRE_FLAG			( (uint32_t) 1 << 8)
#define SPI_REG_SR_BUSY_FLAG		( (uint32_t) 1 << 7)
#define SPI_REG_SR_TXE_FLAG			( (uint32_t) 1 << 1)
#define SPI_REG_SR_RXNE_FLAG		( (uint32_t) 1 << 0)

#define SPI_1 						SPI1
#define SPI_2 						SPI2
#define SPI_3 						SPI3

#define SPI_IS_BUSY 				1
#define SPI_IS_NOT_BUSY				0

#define MYHAL_RCC_SPI1_CLK_ENABLE()	(RCC->APB2ENR |= (1 << 12))
#define MYHAL_RCC_SPI2_CLK_ENABLE()	(RCC->APB1ENR |= (1 << 14))
#define MYHAL_RCC_SPI3_CLK_ENABLE()	(RCC->APB1ENR |= (1 << 15))
#define MYHAL_RCC_SPI4_CLK_ENABLE() (RCC->APB2ENR |= (1 << 13))


/* SPI State Struct */
typedef enum
{
	SPI_STATE_RESET		= 0x00,	// not initialized or disabled
	SPI_STATE_READY		= 0x01,	// initialized and ready (ONLY state where it's good)
	SPI_STATE_BUSY		= 0x02, // busy
	SPI_STATE_BUSY_TX		= 0x12,	// TX busy
	SPI_STATE_BUSY_RX		= 0x22, // RX busy
	SPI_STATE_BUSY_TX_RX	= 0x32, // both busy
	SPI_STATE_ERROR		= 0x03	// error
} myhal_spi_state_t;

/* SPI Configuration Struct */
typedef struct
{
	uint32_t Mode;			// Master or Slave mode
	uint32_t Direction;		// bidi or 2 line uni di
	uint32_t DataSize;		// 8 bit or 16 bit data transmission
	uint32_t CLKPolarity;
	uint32_t CLKPhase;
	uint32_t NSS;			// which slave management (HW or SW)
	uint32_t Prescaler;
	uint32_t FirstBit;		// data transfer start from MSB or LSB
} spi_init_t;

/* SPI handler struct */
typedef struct
{
	SPI_TypeDef		*Instance;		// SPI peripheral base address -  the actual peripheral
	spi_init_t		Init;			// SPI configuration params
	uint8_t			*pTxBuf;	// Pointer to TX buffer; a global, in-system buffer
	uint16_t		TxCount;	// TX transfer count
	uint8_t			*pRxBuf;	// pointer to RX buffer
	uint16_t		RxCount;	// RX transfer counter
	myhal_spi_state_t	State;		// State of peripheral, must be ready
} spi_handler_t;

/**
 * Initialize SPI device
 *
 * @param spi_handler - Handler for SPI1.
 */
void SPI_Init(spi_handler_t *spi_handler);

/**
 * Transmit data across MOSI Line. Discards received data
 *
 * @param tx_data_buf - pointer to buffer containing TX data
 * @param size - number of bytes of data to be sent
 */
void SPI_Transmit(spi_handler_t *spi_handler, BYTE *tx_data_buf, UINT size);

/**
 * Receives data across MISO Line. Transmits dummy data
 *
 * @param rx_data_buf - pointer to buffer containing received RX data
 * @param size - number of bytes of data to be received
 */
void SPI_Receive(spi_handler_t *spi_handler, BYTE *rx_data_buf, UINT size);

/**
 * Transmit data across MOSI Line and receive data across MISO Line
 *
 * @param tx_data_buf - pointer to buffer containing TX data
 * @param rx_data_buf - pointer to buffer containing received RX data
 * @param size - number of bytes of data to be sent and received
 */
void SPI_TransmitReceive(spi_handler_t *spi_handler, BYTE *tx_data_buf, BYTE *rx_data_buf, UINT size);

/**
 * Transmit data across MOSI Line in Interrupt Mode. Discards received data
 *
 * @param tx_data_buf - pointer to buffer containing TX data
 * @param size - number of bytes of data to be sent
 */
void SPI_Transmit_IT(spi_handler_t *spi_handler, uint8_t *tx_data_buf, uint32_t len);

/**
 *  Receives data across MISO Line in Interrupt mode. Transmits dummy data
 *
 * @param rx_data_buf - pointer to buffer containing received RX data
 * @param size - number of bytes of data to be received
 *
 */
void SPI_Receive_IT(spi_handler_t *spi_handler, uint8_t *rx_data_buf, uint32_t size);

/**
 * Transmit data across MOSI Line and receive data across MISO Line in Interrupt Mode
 *
 * @param tx_data_buf - pointer to buffer containing TX data
 * @param rx_data_buf - pointer to buffer containing received RX data
 * @param size - number of bytes of data to be sent and received
 */
void SPI_TransmitReceive_IT(spi_handler_t *spi_handler, uint8_t *tx_data_buf, uint32_t size_tx, uint8_t *rx_data_buf, uint32_t size_rx);

/**
 * Determines which SPI interrupt (TXE and/or RXNE) to handle
 */
void SPI_INT_IRQ_Handler(spi_handler_t *spi_handler);

/**
 * Handles TXE interrupt, only gets called when TXEIE = 1, TXE = 1
 * Calls from within master or slave TX, so TX buffer address already in pTxBuf
 */
void SPI_INT_TXE_Handler(spi_handler_t *spi_handler);

/**
 * Handles RXNE interrupt, only gets called when RXNEIE=1 and RXNE=1
 */
void SPI_INT_RXNE_Handler(spi_handler_t *spi_handler);

#endif /* INC_SPI_DRIVER_H_ */
