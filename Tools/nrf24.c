#include "nrf24.h"
#include <stdio.h>

static uint8_t nRF24_ReadReg(uint8_t reg)
{
	uint8_t value;

	nRF24_CSN_L();
	nRF24_LL_RW(reg & nRF24_MASK_REG_MAP);
	value = nRF24_LL_RW(nRF24_CMD_NOP);
	nRF24_CSN_H();

	return value;
}

static void nRF24_WriteReg(uint8_t reg, uint8_t value)
{
	nRF24_CSN_L();
	if (reg < nRF24_CMD_W_REGISTER)
	{
		/* This is a register access */
		nRF24_LL_RW(nRF24_CMD_W_REGISTER | (reg & nRF24_MASK_REG_MAP));
		nRF24_LL_RW(value);
	}
	else
	{
		/* This is a single byte command or future command/register */
		nRF24_LL_RW(reg);
		if ((reg != nRF24_CMD_FLUSH_TX) && (reg != nRF24_CMD_FLUSH_RX) && \
			(reg != nRF24_CMD_REUSE_TX_PL) && (reg != nRF24_CMD_NOP))
		{
			/* Send register value */
			nRF24_LL_RW(value);
		}
	}
	nRF24_CSN_H();
}

static void nRF24_ReadMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count)
{
	nRF24_CSN_L();
	nRF24_LL_RW(reg);
	while (count--)
	{
		*pBuf++ = nRF24_LL_RW(nRF24_CMD_NOP);
	}
	nRF24_CSN_H();
}

static void nRF24_WriteMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count)
{
	nRF24_CSN_L();
	nRF24_LL_RW(reg);
	while (count--)
	{
		nRF24_LL_RW(*pBuf++);
	}
	nRF24_CSN_H();
}

void nRF24_Init(void)
{
	/* Write to registers their initial values */
	nRF24_WriteReg(nRF24_REG_CONFIG, 0x08);
	nRF24_WriteReg(nRF24_REG_EN_AA, 0x3F);
	nRF24_WriteReg(nRF24_REG_EN_RXADDR, 0x03);
	nRF24_WriteReg(nRF24_REG_SETUP_AW, 0x03);
	nRF24_WriteReg(nRF24_REG_SETUP_RETR, 0x03);
	nRF24_WriteReg(nRF24_REG_RF_CH, 0x02);
	nRF24_WriteReg(nRF24_REG_RF_SETUP, 0x0E);
	nRF24_WriteReg(nRF24_REG_STATUS, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P0, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P1, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P2, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P3, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P4, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P5, 0x00);
	nRF24_WriteReg(nRF24_REG_DYNPD, 0x00);
	nRF24_WriteReg(nRF24_REG_FEATURE, 0x00);

	/* Clear the FIFO's */
	nRF24_FlushRX();
	nRF24_FlushTX();

	/* Clear any pending interrupt flags */
	nRF24_ClearIRQFlags();

	/* Deassert CSN pin (chip release) */
	nRF24_CSN_H();
}

uint8_t nRF24_Check(void)
{
	uint8_t rxbuf[5];
	uint8_t i;
	uint8_t *ptr = (uint8_t *)nRF24_TEST_ADDR;

	/* Write test TX address and read TX_ADDR register */
	nRF24_WriteMBReg(nRF24_CMD_W_REGISTER | nRF24_REG_TX_ADDR, ptr, 5);
	nRF24_ReadMBReg(nRF24_CMD_R_REGISTER | nRF24_REG_TX_ADDR, rxbuf, 5);
	
	/* Compare buffers, return error on first mismatch */
	for (i = 0; i < 5; i++)
	{
		if (rxbuf[i] != *ptr++) return 0;
	}

	return 1;
}

void nRF24_SetPowerMode(uint8_t mode)
{
	uint8_t reg;

	reg = nRF24_ReadReg(nRF24_REG_CONFIG);
	if (mode == nRF24_PWR_UP)
	{
		/* Set the PWR_UP bit of CONFIG register to wake the transceiver */
		/* It goes into Stanby-I mode with consumption about 26uA */
		reg |= nRF24_CONFIG_PWR_UP;
	} else {
		/* Clear the PWR_UP bit of CONFIG register to put the transceiver */
		/* into power down mode with consumption about 900nA */
		reg &= ~nRF24_CONFIG_PWR_UP;
	}
	nRF24_WriteReg(nRF24_REG_CONFIG, reg);
}

void nRF24_SetOperationalMode(uint8_t mode)
{
	uint8_t reg;

	/* Configure PRIM_RX bit of the CONFIG register */
	reg  = nRF24_ReadReg(nRF24_REG_CONFIG);
	reg &= ~nRF24_CONFIG_PRIM_RX;
	reg |= (mode & nRF24_CONFIG_PRIM_RX);
	nRF24_WriteReg(nRF24_REG_CONFIG, reg);
}

void nRF24_SetCRCScheme(uint8_t scheme)
{
	uint8_t reg;

	/* Configure EN_CRC[3] and CRCO[2] bits of the CONFIG register */
	reg  = nRF24_ReadReg(nRF24_REG_CONFIG);
	reg &= ~nRF24_MASK_CRC;
	reg |= (scheme & nRF24_MASK_CRC);
	nRF24_WriteReg(nRF24_REG_CONFIG, reg);
}

void nRF24_SetRFChannel(uint8_t channel)
{
	nRF24_WriteReg(nRF24_REG_RF_CH, channel);
}

void nRF24_SetAutoRetr(uint8_t ard, uint8_t arc)
{
	/* Set auto retransmit settings (SETUP_RETR register) */
	nRF24_WriteReg(nRF24_REG_SETUP_RETR, (uint8_t)((ard << 4) | (arc & nRF24_MASK_RETR_ARC)));
}

void nRF24_SetAddrWidth(uint8_t addr_width)
{
	nRF24_WriteReg(nRF24_REG_SETUP_AW, addr_width - 2);
}

void nRF24_SetAddr(uint8_t pipe, const uint8_t *addr)
{
	uint8_t addr_width;

	/* RX_ADDR_Px register */
	switch (pipe)
	{
		case nRF24_PIPETX:
		case nRF24_PIPE0:
		case nRF24_PIPE1:
			/* Get address width */
			addr_width = nRF24_ReadReg(nRF24_REG_SETUP_AW) + 1;
			/* Write address in reverse order (LSByte first) */
			addr += addr_width;
			nRF24_CSN_L();
			nRF24_LL_RW(nRF24_CMD_W_REGISTER | nRF24_ADDR_REGS[pipe]);
			do {
				nRF24_LL_RW(*addr--);
			} while (addr_width--);
			nRF24_CSN_H();
			break;
		case nRF24_PIPE2:
		case nRF24_PIPE3:
		case nRF24_PIPE4:
		case nRF24_PIPE5:
			/* Write address LSBbyte (only first byte from the addr buffer) */
			nRF24_WriteReg(nRF24_ADDR_REGS[pipe], *addr);
			break;
		default:
			/* Incorrect pipe number -> do nothing */
			break;
	}
}

void nRF24_SetTXPower(uint8_t tx_pwr)
{
	uint8_t reg;

	/* Configure RF_PWR[2:1] bits of the RF_SETUP register */
	reg  = nRF24_ReadReg(nRF24_REG_RF_SETUP);
	reg &= ~nRF24_MASK_RF_PWR;
	reg |= tx_pwr;
	nRF24_WriteReg(nRF24_REG_RF_SETUP, reg);
}

void nRF24_SetDataRate(uint8_t data_rate)
{
	uint8_t reg;

	/* Configure RF_DR_LOW[5] and RF_DR_HIGH[3] bits of the RF_SETUP register */
	reg  = nRF24_ReadReg(nRF24_REG_RF_SETUP);
	reg &= ~nRF24_MASK_DATARATE;
	reg |= data_rate;
	nRF24_WriteReg(nRF24_REG_RF_SETUP, reg);
}

void nRF24_SetRXPipe(uint8_t pipe, uint8_t aa_state, uint8_t payload_len)
{
	uint8_t reg;

	/* Enable the specified pipe (EN_RXADDR register) */
	reg = (nRF24_ReadReg(nRF24_REG_EN_RXADDR) | (1 << pipe)) & nRF24_MASK_EN_RX;
	nRF24_WriteReg(nRF24_REG_EN_RXADDR, reg);

	/* Set RX payload length (RX_PW_Px register) */
	nRF24_WriteReg(nRF24_RX_PW_PIPE[pipe], payload_len & nRF24_MASK_RX_PW);

	/* Set auto acknowledgment for a specified pipe (EN_AA register) */
	reg = nRF24_ReadReg(nRF24_REG_EN_AA);
	if (aa_state == nRF24_AA_ON)
	{
		reg |=  (1 << pipe);
	} else {
		reg &= ~(1 << pipe);
	}
	nRF24_WriteReg(nRF24_REG_EN_AA, reg);
}

void nRF24_ClosePipe(uint8_t pipe)
{
	uint8_t reg;

	reg  = nRF24_ReadReg(nRF24_REG_EN_RXADDR);
	reg &= ~(1 << pipe);
	reg &= nRF24_MASK_EN_RX;
	nRF24_WriteReg(nRF24_REG_EN_RXADDR, reg);
}

void nRF24_EnableAA(uint8_t pipe)
{
	uint8_t reg;

	/* Set bit in EN_AA register */
	reg  = nRF24_ReadReg(nRF24_REG_EN_AA);
	reg |= (1 << pipe);
	nRF24_WriteReg(nRF24_REG_EN_AA, reg);
}

void nRF24_DisableAA(uint8_t pipe)
{
	uint8_t reg;

	if (pipe > 5)
	{
		/* Disable Auto-ACK for ALL pipes */
		nRF24_WriteReg(nRF24_REG_EN_AA, 0x00);
	} else {
		/* Clear bit in the EN_AA register */
		reg  = nRF24_ReadReg(nRF24_REG_EN_AA);
		reg &= ~(1 << pipe);
		nRF24_WriteReg(nRF24_REG_EN_AA, reg);
	}
}

uint8_t nRF24_GetStatus(void)
{
	return nRF24_ReadReg(nRF24_REG_STATUS);
}

uint8_t nRF24_GetIRQFlags(void)
{
	return (nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_STATUS_IRQ);
}

uint8_t nRF24_GetStatus_RXFIFO(void)
{
	return (nRF24_ReadReg(nRF24_REG_FIFO_STATUS) & nRF24_MASK_RXFIFO);
}

uint8_t nRF24_GetStatus_TXFIFO(void)
{
	return ((nRF24_ReadReg(nRF24_REG_FIFO_STATUS) & nRF24_MASK_TXFIFO) >> 4);
}

uint8_t nRF24_GetRXSource(void)
{
	return ((nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO) >> 1);
}

uint8_t nRF24_GetRetransmitCounters(void)
{
	return (nRF24_ReadReg(nRF24_REG_OBSERVE_TX));
}

void nRF24_ResetPLOS(void)
{
	uint8_t reg;

	/* The PLOS counter is reset after write to RF_CH register */
	reg = nRF24_ReadReg(nRF24_REG_RF_CH);
	nRF24_WriteReg(nRF24_REG_RF_CH, reg);
}

void nRF24_FlushTX(void)
{
	nRF24_WriteReg(nRF24_CMD_FLUSH_TX, nRF24_CMD_NOP);
}

void nRF24_FlushRX(void)
{
	nRF24_WriteReg(nRF24_CMD_FLUSH_RX, nRF24_CMD_NOP);
}

void nRF24_ClearIRQFlags(void)
{
	uint8_t reg;

	/* Clear RX_DR, TX_DS and MAX_RT bits of the STATUS register */
	reg  = nRF24_ReadReg(nRF24_REG_STATUS);
	reg |= nRF24_MASK_STATUS_IRQ;
	nRF24_WriteReg(nRF24_REG_STATUS, reg);
}

void nRF24_WritePayload(uint8_t *pBuf, uint8_t length)
{
	nRF24_WriteMBReg(nRF24_CMD_W_TX_PAYLOAD, pBuf, length);
}

nRF24_RXResult nRF24_ReadPayload(uint8_t *pBuf, uint8_t *length)
{
	uint8_t pipe;

	/* Extract a payload pipe number from the STATUS register */
	pipe = (nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO) >> 1;

	/* RX FIFO empty? */
	if (pipe < 6)
	{
		/* Get payload length */
		*length = nRF24_ReadReg(nRF24_RX_PW_PIPE[pipe]);

		/* Read a payload from the RX FIFO */
		if (*length)
		{
			nRF24_ReadMBReg(nRF24_CMD_R_RX_PAYLOAD, pBuf, *length);
		}

		return ((nRF24_RXResult)pipe);
	}

	/* The RX FIFO is empty */
	*length = 0;

	return nRF24_RX_EMPTY;
}
