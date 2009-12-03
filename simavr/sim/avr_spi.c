/*
	avr_spi.c

	Copyright 2008, 2009 Michel Pollet <buserror@gmail.com>

 	This file is part of simavr.

	simavr is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	simavr is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with simavr.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include "avr_spi.h"

static uint8_t avr_spi_read(struct avr_t * avr, uint8_t addr, void * param)
{
	avr_spi_t * p = (avr_spi_t *)param;
	uint8_t v = p->input_data_register;
	p->input_data_register = 0;
//	printf("** PIN%c = %02x\n", p->name, v);
	return v;
}

static void avr_spi_write(struct avr_t * avr, uint8_t addr, uint8_t v, void * param)
{
	avr_spi_t * p = (avr_spi_t *)param;

	if (addr == p->r_spdr) {
	//	printf("UDR%c(%02x) = %02x\n", p->name, addr, v);
		avr_core_watch_write(avr, addr, v);

		if (avr_regbit_get(avr, p->spe)) {
			// in master mode, any byte is sent as it comes..
			if (avr_regbit_get(avr, p->mstr)) {
				avr_raise_irq(p->io.irq + SPI_IRQ_OUTPUT, v);
			}
		}
	}
}

static void avr_spi_irq_input(struct avr_irq_t * irq, uint32_t value, void * param)
{
	avr_spi_t * p = (avr_spi_t *)param;
	avr_t * avr = p->io.avr;

	// check to see fi receiver is enabled
	if (!avr_regbit_get(avr, p->spe))
		return;

	// double buffer the input.. ?
	p->input_data_register = value;
	avr_raise_interrupt(avr, &p->spi);

	// if in slave mode, 
	// 'output' the byte only when we received one...
	if (!avr_regbit_get(avr, p->mstr)) {
		avr_raise_irq(p->io.irq + SPI_IRQ_OUTPUT, avr->data[p->r_spdr]);
	}
}

void avr_spi_reset(struct avr_io_t *io)
{
	avr_spi_t * p = (avr_spi_t *)io;
	avr_irq_register_notify(p->io.irq + SPI_IRQ_INPUT, avr_spi_irq_input, p);
}

static	avr_io_t	_io = {
	.kind = "spi",
	.reset = avr_spi_reset,
};

void avr_spi_init(avr_t * avr, avr_spi_t * p)
{
	p->io = _io;
	avr_register_io(avr, &p->io);

	printf("%s SPI%c init\n", __FUNCTION__, p->name);

	// allocate this module's IRQ
	p->io.irq_count = SPI_IRQ_COUNT;
	p->io.irq = avr_alloc_irq(0, p->io.irq_count);
	p->io.irq_ioctl_get = AVR_IOCTL_SPI_GETIRQ(p->name);

	avr_register_io_write(avr, p->r_spdr, avr_spi_write, p);
	avr_register_io_read(avr, p->r_spdr, avr_spi_read, p);
}
