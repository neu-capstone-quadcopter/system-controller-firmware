#include <cstdint>
#include "chip.h"

extern const uint32_t OscRateIn = 12000000;
extern const uint32_t RTCOscRateIn = 32768;

void Board_SSP_Init(LPC_SSP_T *ssp) {
	switch((uint32_t)ssp) {
	case LPC_SSP0_BASE:
		break;
	case LPC_SSP1_BASE:
		Chip_IOCON_PinMux(LPC_IOCON, 0, 6, IOCON_MODE_INACT, IOCON_FUNC2);
		Chip_IOCON_PinMux(LPC_IOCON, 0, 7, IOCON_MODE_INACT, IOCON_FUNC2);
		Chip_IOCON_PinMux(LPC_IOCON, 0, 8, IOCON_MODE_INACT, IOCON_FUNC2);
		Chip_IOCON_PinMux(LPC_IOCON, 0, 9, IOCON_MODE_INACT, IOCON_FUNC2);
		break;
	}
}
