#include "board.hpp"
#include <cstdint>
#include "chip.h"
#include "FreeRTOS.h"

extern const uint32_t OscRateIn = 12000000;
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

		/* FCCO = ((15+1) * 2 * 12MHz) / (0+1) = 384MHz */
		Chip_Clock_SetupPLL(SYSCTL_MAIN_PLL, 15, 0);

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
			Chip_IOCON_PinMux(LPC_IOCON, 0, 6, IOCON_MODE_INACT, IOCON_FUNC2);
			Chip_IOCON_PinMux(LPC_IOCON, 0, 7, IOCON_MODE_INACT, IOCON_FUNC2);
			Chip_IOCON_PinMux(LPC_IOCON, 0, 8, IOCON_MODE_INACT, IOCON_FUNC2);
			Chip_IOCON_PinMux(LPC_IOCON, 0, 9, IOCON_MODE_INACT, IOCON_FUNC2);
			break;
		default:
			configASSERT(0);
		}
	}

	void uart_init(LPC_USART_T *uart) {
		switch((uint32_t)uart) {
		case LPC_UART0_BASE:
			break;
		case LPC_UART1_BASE:
			Chip_IOCON_PinMux(LPC_IOCON, 0, 15, IOCON_MODE_INACT, IOCON_FUNC1);
			Chip_IOCON_PinMux(LPC_IOCON, 0, 16, IOCON_MODE_INACT, IOCON_FUNC1);
			break;
		case LPC_UART2_BASE:
			break;
		case LPC_UART3_BASE:
			Chip_IOCON_PinMux(LPC_IOCON, 0, 0, IOCON_MODE_INACT, IOCON_FUNC2);
			Chip_IOCON_PinMux(LPC_IOCON, 0, 1, IOCON_MODE_INACT, IOCON_FUNC2);
			break;
		default:
			configASSERT(0);
		}
	}

	void cc1120_init() {
		Chip_GPIO_WriteDirBit(LPC_GPIO, 2, 13, true);

		Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, CC1120_GPIO3_INTPORT, 1 << CC1120_GPIO3_PIN);
		Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, CC1120_GPIO2_INTPORT, 1 << CC1120_GPIO2_PIN);
		Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, CC1120_GPIO0_INTPORT, 1 << CC1120_GPIO0_PIN);

		Chip_GPIO_SetPinDIRInput(LPC_GPIO, CC1120_GPIO3_INTPORT, CC1120_GPIO3_PIN);
		Chip_GPIO_SetPinDIRInput(LPC_GPIO, CC1120_GPIO2_INTPORT, CC1120_GPIO2_PIN);
		Chip_GPIO_SetPinDIRInput(LPC_GPIO, CC1120_GPIO0_INTPORT, CC1120_GPIO0_PIN);

		Chip_GPIOINT_SetIntRising(LPC_GPIOINT, CC1120_GPIO3_INTPORT, 1 << CC1120_GPIO3_PIN);
		Chip_GPIOINT_SetIntRising(LPC_GPIOINT, CC1120_GPIO2_INTPORT, CC1120_GPIO2_PIN);
		Chip_GPIOINT_SetIntRising(LPC_GPIOINT, CC1120_GPIO0_INTPORT, CC1120_GPIO0_PIN);
	}

	void cc1120_gpio0_set_rising_edge_int(bool enable) {
		if (enable) {
			LPC_GPIOINT->IO0.ENR |= (1 << CC1120_GPIO0_PIN);
		}
		else {
			LPC_GPIOINT->IO0.ENR &= ~(1 << CC1120_GPIO0_PIN);
		}
	}

	void cc1120_gpio0_set_falling_edge_int(bool enable) {
		if (enable) {
			LPC_GPIOINT->IO0.ENF |= (1 << CC1120_GPIO0_PIN);
		}
		else {
			LPC_GPIOINT->IO0.ENF &= ~(1 << CC1120_GPIO0_PIN);
		}
	}

	void cc1120_gpio2_set_rising_edge_int(bool enable) {
		if (enable) {
			LPC_GPIOINT->IO2.ENR |= (1 << CC1120_GPIO2_PIN);
		}
		else {
			LPC_GPIOINT->IO2.ENR &= ~(1 << CC1120_GPIO2_PIN);
		}
	}

	void cc1120_gpio2_set_falling_edge_int(bool enable) {
		if (enable) {
			LPC_GPIOINT->IO2.ENF |= (1 << CC1120_GPIO2_PIN);
		}
		else {
			LPC_GPIOINT->IO2.ENF &= ~(1 << CC1120_GPIO2_PIN);
		}
	}

	bool cc1120_is_gpio3_int(void) {
		if (Chip_GPIOINT_GetStatusRising(LPC_GPIOINT, CC1120_GPIO3_INTPORT) & (1 << CC1120_GPIO3_PIN)) {
			Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, CC1120_GPIO3_INTPORT, 1 << CC1120_GPIO3_PIN);
			return true;
		}
		return false;
	}

	bool cc1120_is_gpio2_int(void) {
		if (Chip_GPIOINT_GetStatusRising(LPC_GPIOINT, CC1120_GPIO2_INTPORT) & (1 << CC1120_GPIO2_PIN)) {
			Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, CC1120_GPIO2_INTPORT, 1 << CC1120_GPIO2_PIN);
			return true;
		}
		return false;
	}

	bool cc1120_is_gpio0_int(void) {
		if (Chip_GPIOINT_GetStatusRising(LPC_GPIOINT, CC1120_GPIO0_INTPORT) & (1 << CC1120_GPIO0_PIN)) {
			Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, CC1120_GPIO0_INTPORT, 1 << CC1120_GPIO0_PIN);
			return true;
		}
		return false;
	}

}
