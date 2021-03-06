#include <cstdint>
#include "board.hpp"
#include "chip.h"
#include "FreeRTOS.h"
#include "board.hpp"
#include "config.hpp"

#ifdef IS_DEBUG_BOARD
	extern const uint32_t OscRateIn = 12000000;
#else
	extern const uint32_t OscRateIn = 24000000;
#endif
extern const uint32_t RTCOscRateIn = 32768;

namespace board {
	void setup_clocking(void)
	{
		/* Disconnect the Main PLL if it is connected already */
		if (Chip_Clock_IsMainPLLConnected()) {
			Chip_Clock_DisablePLL(SYSCTL_MAIN_PLL, SYSCTL_PLL_CONNECT);
		}

		/* Disable the PLL if it is enabled */
		if (Chip_Clock_IsMainPLLEnabled()) {
			Chip_Clock_DisablePLL(SYSCTL_MAIN_PLL, SYSCTL_PLL_ENABLE);
		}

		/* Enable the crystal */
		if (!Chip_Clock_IsCrystalEnabled())
			Chip_Clock_EnableCrystal();
		while(!Chip_Clock_IsCrystalEnabled()) {}

		/* Set PLL0 Source to Crystal Oscillator */
		Chip_Clock_SetCPUClockDiv(0);
		Chip_Clock_SetMainPLLSource(SYSCTL_PLLCLKSRC_MAINOSC);

#ifdef IS_DEBUG_BOARD
		/* FCCO = ((15+1) * 2 * 12MHz) / (0+1) = 384MHz */
		Chip_Clock_SetupPLL(SYSCTL_MAIN_PLL, 15, 0);
#else
		/* FCCO = ((7+1) * 2 * 24MHz) / (0+1) = 384MHz */
		Chip_Clock_SetupPLL(SYSCTL_MAIN_PLL, 7, 0);
#endif

		Chip_Clock_EnablePLL(SYSCTL_MAIN_PLL, SYSCTL_PLL_ENABLE);

		/* 384MHz / (3+1) = 96MHz */
		Chip_Clock_SetCPUClockDiv(3);
		while (!Chip_Clock_IsMainPLLLocked()) {} /* Wait for the PLL to Lock */

		Chip_Clock_EnablePLL(SYSCTL_MAIN_PLL, SYSCTL_PLL_CONNECT);
	}

	void ssp_init(LPC_SSP_T *ssp) {
		switch((uint32_t)ssp) {
		case LPC_SSP0_BASE:
			break;
		case LPC_SSP1_BASE:
			Chip_IOCON_PinMux(LPC_IOCON, SSP1_CS_PORT, SSP1_CS_PIN, IOCON_MODE_INACT, IOCON_FUNC2);
			Chip_IOCON_PinMux(LPC_IOCON, SSP1_SCK_PORT, SSP1_SCK_PIN, IOCON_MODE_INACT, IOCON_FUNC2);
			Chip_IOCON_PinMux(LPC_IOCON, SSP1_MISO_PORT, SSP1_MISO_PIN, IOCON_MODE_INACT, IOCON_FUNC2);
			Chip_IOCON_PinMux(LPC_IOCON, SSP1_MOSI_PORT, SSP1_MOSI_PIN, IOCON_MODE_INACT, IOCON_FUNC2);
			break;
		default:
			configASSERT(0);
		}
	}

	void uart_init(LPC_USART_T *uart) {
		switch((uint32_t)uart) {
#ifdef IS_DEBUG_BOARD
		case LPC_UART0_BASE:
			Chip_IOCON_PinMux(LPC_IOCON, UART0_TX_PORT, UART0_TX_PIN, IOCON_MODE_INACT, IOCON_FUNC1);
			Chip_IOCON_PinMux(LPC_IOCON, UART0_RX_PORT, UART0_RX_PIN, IOCON_MODE_INACT, IOCON_FUNC1);
			break;
		case LPC_UART1_BASE:
			Chip_IOCON_PinMux(LPC_IOCON, UART1_TX_PORT, UART1_TX_PIN, IOCON_MODE_INACT, IOCON_FUNC1);
			Chip_IOCON_PinMux(LPC_IOCON, UART1_RX_PORT, UART1_RX_PIN, IOCON_MODE_INACT, IOCON_FUNC1);
			break;
		case LPC_UART2_BASE:
			Chip_IOCON_PinMux(LPC_IOCON, UART2_TX_PORT, UART2_TX_PIN, IOCON_MODE_INACT, IOCON_FUNC1);
			Chip_IOCON_PinMux(LPC_IOCON, UART2_RX_PORT, UART2_RX_PIN, IOCON_MODE_INACT, IOCON_FUNC1);
			break;
		case LPC_UART3_BASE:
			Chip_IOCON_PinMux(LPC_IOCON, UART3_TX_PORT, UART3_TX_PIN, IOCON_MODE_INACT, IOCON_FUNC2);
			Chip_IOCON_PinMux(LPC_IOCON, UART3_RX_PORT, UART3_RX_PIN, IOCON_MODE_INACT, IOCON_FUNC2);
			break;
#else
		case LPC_UART0_BASE:
			Chip_IOCON_PinMux(LPC_IOCON, UART0_TX_PORT, UART0_TX_PIN, IOCON_MODE_INACT, IOCON_FUNC1);
			Chip_IOCON_PinMux(LPC_IOCON, UART0_RX_PORT, UART0_RX_PIN, IOCON_MODE_INACT, IOCON_FUNC1);
			break;
		case LPC_UART1_BASE:
			Chip_IOCON_PinMux(LPC_IOCON, UART1_TX_PORT, UART1_TX_PIN, IOCON_MODE_INACT, IOCON_FUNC2);
			Chip_IOCON_PinMux(LPC_IOCON, UART1_RX_PORT, UART1_RX_PIN, IOCON_MODE_INACT, IOCON_FUNC2);
			break;
		case LPC_UART2_BASE:
			Chip_IOCON_PinMux(LPC_IOCON, UART2_TX_PORT, UART2_TX_PIN, IOCON_MODE_INACT, IOCON_FUNC2);
			Chip_IOCON_PinMux(LPC_IOCON, UART2_RX_PORT, UART2_RX_PIN, IOCON_MODE_INACT, IOCON_FUNC2);
			break;
		case LPC_UART3_BASE:
			Chip_IOCON_PinMux(LPC_IOCON, UART3_TX_PORT, UART3_TX_PIN, IOCON_MODE_INACT, IOCON_FUNC2);
			Chip_IOCON_PinMux(LPC_IOCON, UART3_RX_PORT, UART3_RX_PIN, IOCON_MODE_INACT, IOCON_FUNC2);
			break;
#endif
		default:
			configASSERT(0);
		}
	}

	void mb1240_init() {
		Chip_IOCON_PinMux(LPC_IOCON, ULTRASONIC_TIMER_CAP_PORT, ULTRASONIC_TIMER_CAP_PIN, IOCON_MODE_INACT, IOCON_FUNC3);
	}
}
