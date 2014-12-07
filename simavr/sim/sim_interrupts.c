/*
	sim_interrupts.c

	Copyright 2008-2012 Michel Pollet <buserror@gmail.com>

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


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include "sim_interrupts.h"
#include "sim_avr.h"
#include "sim_core.h"

// modulo a cursor value on the pending interrupt fifo
#define INT_FIFO_SIZE (sizeof(table->pending) / sizeof(avr_int_vector_t *))
#define INT_FIFO_MOD(_v) ((_v) &  (INT_FIFO_SIZE - 1))

#define ASSERT(value) if(value) printf(# value);

void
avr_interrupt_init(
		avr_t * avr )
{
	avr_int_table_p table = &avr->interrupts;
	memset(table, 0, sizeof(*table));
}

void
avr_interrupt_reset(
		avr_t * avr )
{
	printf("%s\n", __func__);
	avr_int_table_p table = &avr->interrupts;
	table->pending = 0;
	avr->interrupt_state = 0;
}

void
avr_register_vector(
		avr_t *avr,
		avr_int_vector_t * vector)
{
	if (!vector->vector)
		return;

	avr_int_table_p table = &avr->interrupts;

	vector->irq.irq = vector->vector;

	ASSERT(table->vector[vector->vector]);
	table->vector[vector->vector] = vector;

	if (vector->trace)
		printf("%s register vector %d (enabled %04x:%d)\n", __FUNCTION__, vector->vector, vector->enable.reg, vector->enable.bit);

	if (!vector->enable.reg)
		AVR_LOG(avr, LOG_WARNING, "INT: avr_register_vector: No 'enable' bit on vector %d !\n", vector->vector);
}

int
avr_has_pending_interrupts(
		avr_t * avr)
{
	avr_int_table_p table = &avr->interrupts;

	return 0 != table->pending;
}

int
avr_is_interrupt_pending(
		avr_t * avr,
		avr_int_vector_t * vector)
{
	avr_int_table_p table = &avr->interrupts;

	return table->pending & (1 << vector->vector);
}

int
avr_is_interrupt_enabled(
		avr_t * avr,
		avr_int_vector_t * vector)
{
	return avr_regbit_get(avr, vector->enable);
}

int
avr_raise_interrupt(
		avr_t * avr,
		avr_int_vector_t * vector)
{
	if (!vector || !vector->vector)
		return 0;
	if (vector->trace)
		printf("%s raising %d (enabled %d)\n", __FUNCTION__, vector->vector, avr_regbit_get(avr, vector->enable));
	if (avr_is_interrupt_pending(avr, vector)) {
		if (vector->trace)
			printf("%s trying to double raise %d (enabled %d)\n", __FUNCTION__, vector->vector, avr_regbit_get(avr, vector->enable));
		return 0;
	}
	// always mark the 'raised' flag to one, even if the interrupt is disabled
	// this allow "polling" for the "raised" flag, like for non-interrupt
	// driven UART and so so. These flags are often "write one to clear"
	if (vector->raised.reg)
		avr_regbit_set(avr, vector->raised);

	avr_raise_irq(&vector->irq, 1);

	// If the interrupt is enabled, attempt to wake the core
	if (avr_regbit_get(avr, vector->enable)) {
		avr_int_table_p table = &avr->interrupts;

		table->pending |= 1 << vector->vector;

		if (avr->sreg[S_I] && avr->interrupt_state == 0)
			avr->interrupt_state = 1;
		if (avr->state == cpu_Sleeping) {
			if (vector->trace)
				printf("Waking CPU due to interrupt\n");
			avr->state = cpu_Running;	// in case we were sleeping
		}
	}
	// return 'raised' even if it was already pending
	return 1;
}

void
avr_clear_interrupt(
		avr_t * avr,
		avr_int_vector_t * vector)
{
	if (!vector)
		return;
	if (vector->trace)
		printf("%s cleared %d\n", __FUNCTION__, vector->vector);
	avr->interrupts.pending &= ~(1 << vector->vector);
	avr_raise_irq(&vector->irq, 0);
	if (vector->raised.reg && !vector->raise_sticky)
		avr_regbit_clear(avr, vector->raised);
}

int
avr_clear_interrupt_if(
		avr_t * avr,
		avr_int_vector_t * vector,
		uint8_t old)
{
	if (avr_regbit_get(avr, vector->raised)) {
		avr_clear_interrupt(avr, vector);
		return 1;
	}
	avr_regbit_setto(avr, vector->raised, old);
	return 0;
}

avr_irq_t *
avr_get_interrupt_irq(
		avr_t * avr,
		uint8_t v)
{
	avr_int_table_p table = &avr->interrupts;
	for (int i = 0; i < table->vector_count; i++)
		if (table->vector[i]->vector == v)
			return &table->vector[i]->irq;
	return NULL;
}

static inline uint8_t avr_ffsll(uint64_t llval)
{
	uint64_t vec = 1;
	int count = 64;
	
	while (count--) {
		if (llval & vec)
			return (64 - count);
		vec <<= 1;
	}
	
	return 0;
}

/*
 * check whether interrupts are pending. If so, check if the interrupt "latency" is reached,
 * and if so triggers the handlers and jump to the vector.
 */
void
avr_service_interrupts(
		avr_t * avr)
{
	if (!avr->sreg[S_I])
		return;

	if (avr->interrupt_state) {
		if (avr->interrupt_state < 0) {
			avr->interrupt_state++;
			if (avr->interrupt_state == 0)
				avr->interrupt_state = avr_has_pending_interrupts(avr);
			return;
		}
	} else
		return;

	avr_int_table_p table = &avr->interrupts;

	int ivec = avr_ffsll(table->pending >> 1);

	avr_int_vector_t * vector = table->service[ivec];

	// if that single interrupt is masked, ignore it and continue
	// could also have been disabled, or cleared
	if (!ivec || !avr_regbit_get(avr, vector->enable)) {
		avr->interrupt_state = avr_has_pending_interrupts(avr);
	} else {
		if (vector && vector->trace)
			printf("%s calling %d\n", __FUNCTION__, (int)vector->vector);
		_avr_push_addr(avr, avr->pc);
		avr_sreg_set(avr, S_I, 0);
		avr->pc = vector->vector * avr->vector_size;

		avr_clear_interrupt(avr, vector);
	}
}

