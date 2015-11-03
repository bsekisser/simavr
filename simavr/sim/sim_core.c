/*
	sim_core.c

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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "sim_avr.h"
#include "sim_core.h"
#include "sim_gdb.h"
#include "avr_flash.h"
#include "avr_watchdog.h"

// SREG bit names
const char * _sreg_bit_name = "cznvshti";

/*
 * Handle "touching" registers, marking them changed.
 * This is used only for debugging purposes to be able to
 * print the effects of each instructions on registers
 */
#if CONFIG_SIMAVR_TRACE

#define T(w) w

#define REG_TOUCH(a, r) (a)->trace_data->touched[(r) >> 5] |= (1 << ((r) & 0x1f))
#define REG_ISTOUCHED(a, r) ((a)->trace_data->touched[(r) >> 5] & (1 << ((r) & 0x1f)))

/*
 * This allows a "special case" to skip instruction tracing when in these
 * symbols since printf() is useful to have, but generates a lot of cycles.
 */
int dont_trace(const char * name)
{
	return (
		!strcmp(name, "uart_putchar") ||
		!strcmp(name, "fputc") ||
		!strcmp(name, "printf") ||
		!strcmp(name, "vfprintf") ||
		!strcmp(name, "__ultoa_invert") ||
		!strcmp(name, "__prologue_saves__") ||
		!strcmp(name, "__epilogue_restores__"));
}

int donttrace = 0;

#define STATE(_f, args...) { \
	if (avr->trace) {\
		if (avr->trace_data->codeline && avr->trace_data->codeline[avr->pc>>1]) {\
			const char * symn = avr->trace_data->codeline[avr->pc>>1]->symbol; \
			int dont = 0 && dont_trace(symn);\
			if (dont!=donttrace) { \
				donttrace = dont;\
				DUMP_REG();\
			}\
			if (donttrace==0)\
				printf("%04x: %-25s " _f, avr->pc, symn, ## args);\
		} else \
			printf("%s: %04x: " _f, __FUNCTION__, avr->pc, ## args);\
		}\
	}
#define SREG() if (avr->trace && donttrace == 0) {\
	printf("%04x: \t\t\t\t\t\t\t\t\tSREG = ", avr->pc); \
	for (int _sbi = 0; _sbi < 8; _sbi++)\
		printf("%c", avr->sreg[_sbi] ? toupper(_sreg_bit_name[_sbi]) : '.');\
	printf("\n");\
}

void crash(avr_t* avr)
{
	DUMP_REG();
	printf("*** CYCLE %" PRI_avr_cycle_count "PC %04x\n", avr->cycle, avr->pc);

	for (int i = OLD_PC_SIZE-1; i > 0; i--) {
		int pci = (avr->trace_data->old_pci + i) & 0xf;
		printf(FONT_RED "*** %04x: %-25s RESET -%d; sp %04x\n" FONT_DEFAULT,
				avr->trace_data->old[pci].pc, avr->trace_data->codeline ? avr->trace_data->codeline[avr->trace_data->old[pci].pc>>1]->symbol : "unknown", OLD_PC_SIZE-i, avr->trace_data->old[pci].sp);
	}

	printf("Stack Ptr %04x/%04x = %d \n", _avr_sp_get(avr), avr->ramend, avr->ramend - _avr_sp_get(avr));
	DUMP_STACK();

	avr_sadly_crashed(avr, 0);
}
#else
	#if 1
		#define T(w)
		#define REG_TOUCH(a, r)
		#define STATE(_f, args...)
		#define SREG()
	#else
		#define T(w) w
		#define REG_TOUCH(a, r)
		#define STATE(_f, args...) \
			do { \
				printf("%04x: " _f, avr->pc, ## args); \
			} while (0);
	
		#define SREG() \
			do { \
				printf("%04x: \t\t\t\t\t\t\t\t\tSREG = ", avr->pc); \
				for (int _sbi = 0; _sbi < 8; _sbi++)\
					printf("%c", avr->sreg[_sbi] ? toupper(_sreg_bit_name[_sbi]) : '.');\
				printf("\n");\
			} while (0);
	#endif

void crash(avr_t* avr)
{
	avr_sadly_crashed(avr, 0);
}
#endif

static inline uint16_t
_avr_data_read16le(
	avr_t * avr,
	uint16_t addr)
{
	uint16_t retval = avr->data[addr] | (avr->data[addr + 1] << 8);
	return retval;
}

static inline uint32_t
_avr_extend_flash_read32le(
	avr_t * avr,
	avr_flashaddr_t addr)
{
	uint32_t retval = avr->extend_flash[addr >> 1];
	return retval;
}

static inline void
_avr_extend_flash_write32le(
	avr_t * avr,
	avr_flashaddr_t addr,  uint32_t value)
{
	avr->extend_flash[addr >> 1] = value;
}

static inline uint16_t
_avr_flash_read16le(
	avr_t * avr,
	avr_flashaddr_t addr)
{
	uint16_t retval = avr->flash[addr] | (avr->flash[addr + 1] << 8);
	return retval;
}

void avr_core_watch_write(avr_t *avr, uint16_t addr, uint8_t v)
{
	if (addr > avr->ramend) {
		AVR_LOG(avr, LOG_ERROR, "CORE: *** Invalid write address PC=%04x SP=%04x O=%04x Address %04x=%02x out of ram\n",
				avr->pc, _avr_sp_get(avr), _avr_flash_read16le(avr, avr->pc), addr, v);
		crash(avr);
	}
	if (addr < 32) {
		AVR_LOG(avr, LOG_ERROR, "CORE: *** Invalid write address PC=%04x SP=%04x O=%04x Address %04x=%02x low registers\n",
				avr->pc, _avr_sp_get(avr), _avr_flash_read16le(avr, avr->pc), addr, v);
		crash(avr);
	}
#if AVR_STACK_WATCH
	/*
	 * this checks that the current "function" is not doctoring the stack frame that is located
	 * higher on the stack than it should be. It's a sign of code that has overrun it's stack
	 * frame and is munching on it's own return address.
	 */
	if (avr->trace_data->stack_frame_index > 1 && addr > avr->trace_data->stack_frame[avr->trace_data->stack_frame_index-2].sp) {
		printf( FONT_RED "%04x : munching stack SP %04x, A=%04x <= %02x\n" FONT_DEFAULT, avr->pc, _avr_sp_get(avr), addr, v);
	}
#endif

	if (avr->gdb) {
		avr_gdb_handle_watchpoints(avr, addr, AVR_GDB_WATCH_WRITE);
	}

	avr->data[addr] = v;
}

uint8_t avr_core_watch_read(avr_t *avr, uint16_t addr)
{
	if (addr > avr->ramend) {
		AVR_LOG(avr, LOG_ERROR, FONT_RED "CORE: *** Invalid read address PC=%04x SP=%04x O=%04x Address %04x out of ram (%04x)\n" FONT_DEFAULT,
				avr->pc, _avr_sp_get(avr), _avr_flash_read16le(avr, avr->pc), addr, avr->ramend);
		crash(avr);
	}

	if (avr->gdb) {
		avr_gdb_handle_watchpoints(avr, addr, AVR_GDB_WATCH_READ);
	}

	return avr->data[addr];
}

/*
 * Set a register (r < 256)
 * if it's an IO register (> 31) also (try to) call any callback that was
 * registered to track changes to that register.
 */
static inline void _avr_set_r(avr_t * avr, uint16_t r, uint8_t v)
{
	REG_TOUCH(avr, r);

	if (r == R_SREG) {
		avr->data[R_SREG] = v;
		// unsplit the SREG
		SET_SREG_FROM(avr, v);
		SREG();
	}
	if (r > 31) {
		avr_io_addr_t io = AVR_DATA_TO_IO(r);
		if (avr->io[io].w.c)
			avr->io[io].w.c(avr, r, v, avr->io[io].w.param);
		else
			avr->data[r] = v;
		if (avr->io[io].irq) {
			avr_raise_irq(avr->io[io].irq + AVR_IOMEM_IRQ_ALL, v);
			for (int i = 0; i < 8; i++)
				avr_raise_irq(avr->io[io].irq + i, (v >> i) & 1);				
		}
	} else
		avr->data[r] = v;
}

static inline void
_avr_set_r16le(
	avr_t * avr,
	uint16_t r,
	uint16_t v)
{
	_avr_set_r(avr, r, v);
	_avr_set_r(avr, r + 1, v >> 8);
}

static inline void
_avr_set_r16le_hl(
	avr_t * avr,
	uint16_t r,
	uint16_t v)
{
	_avr_set_r(avr, r + 1, v >> 8);
	_avr_set_r(avr, r , v);
}

/*
 * Stack pointer access
 */
inline uint16_t _avr_sp_get(avr_t * avr)
{
	return _avr_data_read16le(avr, R_SPL);
}

inline void _avr_sp_set(avr_t * avr, uint16_t sp)
{
	_avr_set_r16le(avr, R_SPL, sp);
}

/*
 * Set any address to a value; split between registers and SRAM
 */
static inline void _avr_set_ram(avr_t * avr, uint16_t addr, uint8_t v)
{
	if (addr < MAX_IOs + 31)
		_avr_set_r(avr, addr, v);
	else
		avr_core_watch_write(avr, addr, v);
}

/*
 * Get a value from SRAM.
 */
static inline uint8_t _avr_get_ram(avr_t * avr, uint16_t addr)
{
	if (addr == R_SREG) {
		/*
		 * SREG is special it's reconstructed when read
		 * while the core itself uses the "shortcut" array
		 */
		READ_SREG_INTO(avr, avr->data[R_SREG]);
		
	} else if (addr > 31 && addr < 31 + MAX_IOs) {
		avr_io_addr_t io = AVR_DATA_TO_IO(addr);
		
		if (avr->io[io].r.c)
			avr->data[addr] = avr->io[io].r.c(avr, addr, avr->io[io].r.param);
		
		if (avr->io[io].irq) {
			uint8_t v = avr->data[addr];
			avr_raise_irq(avr->io[io].irq + AVR_IOMEM_IRQ_ALL, v);
			for (int i = 0; i < 8; i++)
				avr_raise_irq(avr->io[io].irq + i, (v >> i) & 1);				
		}
	}
	return avr_core_watch_read(avr, addr);
}

/*
 * Stack push accessors.
 */
static inline void _avr_push8(avr_t * avr, uint16_t v)
{
	uint16_t sp = _avr_sp_get(avr);
	_avr_set_ram(avr, sp, v);
	_avr_sp_set(avr, sp-1);
}

static inline uint8_t _avr_pop8(avr_t * avr)
{
	uint16_t sp = _avr_sp_get(avr) + 1;
	uint8_t res = _avr_get_ram(avr, sp);
	_avr_sp_set(avr, sp);
	return res;
}

int _avr_push_addr(avr_t * avr, avr_flashaddr_t addr)
{
	uint16_t sp = _avr_sp_get(avr);
	addr >>= 1;
	for (int i = 0; i < avr->address_size; i++, addr >>= 8, sp--) {
		_avr_set_ram(avr, sp, addr);	
	}
	_avr_sp_set(avr, sp);
	return avr->address_size;
}

avr_flashaddr_t _avr_pop_addr(avr_t * avr)
{
	uint16_t sp = _avr_sp_get(avr) + 1;
	avr_flashaddr_t res = 0;
	for (int i = 0; i < avr->address_size; i++, sp++) {
		res = (res << 8) | _avr_get_ram(avr, sp);
	}
	res <<= 1;
	_avr_sp_set(avr, sp -1);
	return res;
}

/*
 * "Pretty" register names
 */
const char * reg_names[255] = {
		[R_XH] = "XH", [R_XL] = "XL",
		[R_YH] = "YH", [R_YL] = "YL",
		[R_ZH] = "ZH", [R_ZL] = "ZL",
		[R_SPH] = "SPH", [R_SPL] = "SPL",
		[R_SREG] = "SREG",
};


const char * avr_regname(uint8_t reg)
{
	if (!reg_names[reg]) {
		char tt[16];
		if (reg < 32)
			sprintf(tt, "r%d", reg);
		else
			sprintf(tt, "io:%02x", reg);
		reg_names[reg] = strdup(tt);
	}
	return reg_names[reg];
}

/*
 * Called when an invalid opcode is decoded
 */
static void _avr_invalid_opcode(avr_t * avr)
{
#if CONFIG_SIMAVR_TRACE
	printf( FONT_RED "*** %04x: %-25s Invalid Opcode SP=%04x O=%04x \n" FONT_DEFAULT,
			avr->pc, avr->trace_data->codeline[avr->pc>>1]->symbol, _avr_sp_get(avr), _avr_flash_read16le(avr, avr->pc));
#else
	AVR_LOG(avr, LOG_ERROR, FONT_RED "CORE: *** %04x: Invalid Opcode SP=%04x O=%04x \n" FONT_DEFAULT,
			avr->pc, _avr_sp_get(avr), _avr_flash_read16le(avr, avr->pc));
#endif
}

#if CONFIG_SIMAVR_TRACE
/*
 * Dump changed registers when tracing
 */
void avr_dump_state(avr_t * avr)
{
	if (!avr->trace || donttrace)
		return;

	int doit = 0;

	for (int r = 0; r < 3 && !doit; r++)
		if (avr->trace_data->touched[r])
			doit = 1;
	if (!doit)
		return;
	printf("                                       ->> ");
	const int r16[] = { R_SPL, R_XL, R_YL, R_ZL };
	for (int i = 0; i < 4; i++)
		if (REG_ISTOUCHED(avr, r16[i]) || REG_ISTOUCHED(avr, r16[i]+1)) {
			REG_TOUCH(avr, r16[i]);
			REG_TOUCH(avr, r16[i]+1);
		}

	for (int i = 0; i < 3*32; i++)
		if (REG_ISTOUCHED(avr, i)) {
			printf("%s=%02x ", avr_regname(i), avr->data[i]);
		}
	printf("\n");
}
#endif

static inline uint32_t
_make_opcode_h8_0r8_1r8_2r8(
	uint8_t _h, 
	uint8_t _r0, 
	uint8_t _r1, 
	uint8_t _r2)
{
	uint32_t opcode_out = (_h & 0xff);
	opcode_out <<= 8;
	opcode_out |= (_r2 & 0xff);
	opcode_out <<= 8;
	opcode_out |= (_r1 & 0xff);
	opcode_out <<= 8;
	opcode_out |= (_r0 & 0xff);
	return opcode_out;
}

#define get_d5(o) \
		const uint8_t d = (o >> 4) & 0x1f;

#define get_vd5(o) \
		get_d5(o) \
		const uint8_t vd = avr->data[d];

#define get_r5(o) \
		const uint8_t r = ((o >> 5) & 0x10) | (o & 0xf);

#define get_d5_a6(o) \
		get_d5(o); \
		const uint8_t A = ((((o >> 9) & 3) << 4) | ((o) & 0xf)) + 32;

#define get_vd5_s3(o) \
		get_vd5(o); \
		const uint8_t s = o & 7;

#define get_vd5_s3_mask(o) \
		get_vd5_s3(o); \
		const uint8_t mask = 1 << s;

#define get_vd5_vr5(o) \
		get_r5(o); \
		get_d5(o); \
		const uint8_t vd = avr->data[d], vr = avr->data[r];

#define get_d5_vr5(o) \
		get_d5(o); \
		get_r5(o); \
		const uint8_t vr = avr->data[r];
		
#define get_h4_k8(o) \
		const uint8_t h = 16 + ((o >> 4) & 0xf); \
		const uint8_t k = ((o & 0x0f00) >> 4) | (o & 0xf);

#define get_vh4_k8(o) \
		get_h4_k8(o) \
		const uint8_t vh = avr->data[h];

#define get_d5_q6(o) \
		get_d5(o) \
		const uint8_t q = ((o & 0x2000) >> 8) | ((o & 0x0c00) >> 7) | (o & 0x7);

#define get_io5(o) \
		const uint8_t io = ((o >> 3) & 0x1f) + 32;

#define get_io5_b3(o) \
		get_io5(o); \
		const uint8_t b = o & 0x7;

#define get_io5_b3mask(o) \
		get_io5(o); \
		const uint8_t mask = 1 << (o & 0x7);

//	const int16_t o = ((int16_t)(op << 4)) >> 3; // CLANG BUG!
#define get_o12(op) \
		const int16_t o = ((int16_t)((op << 4) & 0xffff)) >> 3;

#define get_sreg_bit(o) \
		const uint8_t b = (o >> 4) & 7;

#define get_R(_xop, _num, _name) \
		const uint8_t _name = (_xop >> (_num << 3)) & 0xff;

#define get_RvR(_xop, _num, _name) \
		get_R(_xop, _num, _name); \
		const uint8_t v ## _name = avr->data[_name];

#define get_RvR16le(_xop, _num, _name) \
		get_R(_xop, _num, _name); \
		const uint16_t v ## _name = _avr_data_read16le(avr, _name);

typedef uint32_t (*avr_inst_opcode_xlat_pfn)(
	avr_t * avr, 
	uint32_t opcode, 
	uint32_t * extend_opcode, 
	uint8_t handler,
	avr_flashaddr_t new_pc);

#define INST_OPCODE_XLAT_PFN_CALL(_pfn) \
	_pfn(avr, opcode, &extend_opcode, handler, *new_pc);

#define INST_OPCODE_XLAT_DECL(_xlat) \
	static uint32_t \
	_avr_inst_opcode_xlat_ ## _xlat( \
		avr_t * avr, \
		uint32_t opcode, \
		uint32_t * extend_opcode, \
		uint8_t handler, \
		avr_flashaddr_t new_pc)

INST_OPCODE_XLAT_DECL(ALL)
{
	return *extend_opcode = opcode;
}

INST_OPCODE_XLAT_DECL(ABS22)
{
	return *extend_opcode = opcode;
}

INST_OPCODE_XLAT_DECL(A5B3)
{
	return *extend_opcode = opcode;
}

INST_OPCODE_XLAT_DECL(D5)
{
	return *extend_opcode = opcode;
}

INST_OPCODE_XLAT_DECL(D5A6)
{
	return *extend_opcode = opcode;
}

INST_OPCODE_XLAT_DECL(D5B3)
{
	return *extend_opcode = opcode;
}

INST_OPCODE_XLAT_DECL(D3R3)
{
	return *extend_opcode = opcode;
}

INST_OPCODE_XLAT_DECL(D4R4)
{
	return *extend_opcode = opcode;
}

INST_OPCODE_XLAT_DECL(D5R5)
{
	return *extend_opcode = opcode;
}

INST_OPCODE_XLAT_DECL(D5rXYZ)
{
	return *extend_opcode = opcode;
}

INST_OPCODE_XLAT_DECL(D5rYZ_Q6)
{
	return *extend_opcode = opcode;
}

INST_OPCODE_XLAT_DECL(H4K8)
{
	return *extend_opcode = opcode;
}

INST_OPCODE_XLAT_DECL(O7S3)
{
	return *extend_opcode = opcode;
}

INST_OPCODE_XLAT_DECL(O12)
{
	return *extend_opcode = opcode;
}

#define get_vp2_k6(_xop) \
		get_RvR16le(_xop, 0, p); \
		get_R(_xop, 1, k);

INST_OPCODE_XLAT_DECL(P2K6)
{
	const uint8_t p = 24 + ((opcode >> 3) & 0x6);
	const uint8_t k = ((opcode & 0x00c0) >> 2) | (opcode & 0xf);
	return *extend_opcode = _make_opcode_h8_0r8_1r8_2r8(handler, p, k, 0);
}

INST_OPCODE_XLAT_DECL(SREG)
{
	return *extend_opcode = opcode;
}

/*
 * Add a "jump" address to the jump trace buffer
 */
#if CONFIG_SIMAVR_TRACE
#define TRACE_JUMP()\
	avr->trace_data->old[avr->trace_data->old_pci].pc = avr->pc;\
	avr->trace_data->old[avr->trace_data->old_pci].sp = _avr_sp_get(avr);\
	avr->trace_data->old_pci = (avr->trace_data->old_pci + 1) & (OLD_PC_SIZE-1);\

#if AVR_STACK_WATCH
#define STACK_FRAME_PUSH()\
	avr->trace_data->stack_frame[avr->trace_data->stack_frame_index].pc = avr->pc;\
	avr->trace_data->stack_frame[avr->trace_data->stack_frame_index].sp = _avr_sp_get(avr);\
	avr->trace_data->stack_frame_index++; 
#define STACK_FRAME_POP()\
	if (avr->trace_data->stack_frame_index > 0) \
		avr->trace_data->stack_frame_index--;
#else
#define STACK_FRAME_PUSH()
#define STACK_FRAME_POP()
#endif
#else /* CONFIG_SIMAVR_TRACE */

#define TRACE_JUMP()
#define STACK_FRAME_PUSH()
#define STACK_FRAME_POP()

#endif

/****************************************************************************\
 *
 * Helper functions for calculating the status register bit values.
 * See the Atmel data sheet for the instruction set for more info.
 *
\****************************************************************************/

static  void
_avr_flags_zns (struct avr_t * avr, uint8_t res)
{
	avr->sreg[S_Z] = res == 0;
	avr->sreg[S_N] = (res >> 7) & 1;
	avr->sreg[S_S] = avr->sreg[S_N] ^ avr->sreg[S_V];
}

static  void
_avr_flags_zns16 (struct avr_t * avr, uint16_t res)
{
	avr->sreg[S_Z] = res == 0;
	avr->sreg[S_N] = (res >> 15) & 1;
	avr->sreg[S_S] = avr->sreg[S_N] ^ avr->sreg[S_V];
}

static  void
_avr_flags_add_zns (struct avr_t * avr, uint8_t res, uint8_t rd, uint8_t rr)
{
	/* carry & half carry */
	uint8_t add_carry = (rd & rr) | (rr & ~res) | (~res & rd);
	avr->sreg[S_H] = (add_carry >> 3) & 1;
	avr->sreg[S_C] = (add_carry >> 7) & 1;

	/* overflow */
	avr->sreg[S_V] = (((rd & rr & ~res) | (~rd & ~rr & res)) >> 7) & 1;

	/* zns */
	_avr_flags_zns(avr, res);
}


static  void
_avr_flags_sub_zns (struct avr_t * avr, uint8_t res, uint8_t rd, uint8_t rr)
{
	/* carry & half carry */
	uint8_t sub_carry = (~rd & rr) | (rr & res) | (res & ~rd);
	avr->sreg[S_H] = (sub_carry >> 3) & 1;
	avr->sreg[S_C] = (sub_carry >> 7) & 1;

	/* overflow */
	avr->sreg[S_V] = (((rd & ~rr & ~res) | (~rd & rr & res)) >> 7) & 1;

	/* zns */
	_avr_flags_zns(avr, res);
}

static  void
_avr_flags_Rzns (struct avr_t * avr, uint8_t res)
{
	if (res)
		avr->sreg[S_Z] = 0;
	avr->sreg[S_N] = (res >> 7) & 1;
	avr->sreg[S_S] = avr->sreg[S_N] ^ avr->sreg[S_V];
}

static  void
_avr_flags_sub_Rzns (struct avr_t * avr, uint8_t res, uint8_t rd, uint8_t rr)
{
	/* carry & half carry */
	uint8_t sub_carry = (~rd & rr) | (rr & res) | (res & ~rd);
	avr->sreg[S_H] = (sub_carry >> 3) & 1;
	avr->sreg[S_C] = (sub_carry >> 7) & 1;

	/* overflow */
	avr->sreg[S_V] = (((rd & ~rr & ~res) | (~rd & rr & res)) >> 7) & 1;

	_avr_flags_Rzns(avr, res);
}

static  void
_avr_flags_zcvs (struct avr_t * avr, uint8_t res, uint8_t vr)
{
	avr->sreg[S_Z] = res == 0;
	avr->sreg[S_C] = vr & 1;
	avr->sreg[S_V] = avr->sreg[S_N] ^ avr->sreg[S_C];
	avr->sreg[S_S] = avr->sreg[S_N] ^ avr->sreg[S_V];
}

static  void
_avr_flags_zcnvs (struct avr_t * avr, uint8_t res, uint8_t vr)
{
	avr->sreg[S_Z] = res == 0;
	avr->sreg[S_C] = vr & 1;
	avr->sreg[S_N] = res >> 7;
	avr->sreg[S_V] = avr->sreg[S_N] ^ avr->sreg[S_C];
	avr->sreg[S_S] = avr->sreg[S_N] ^ avr->sreg[S_V];
}

static  void
_avr_flags_znv0s (struct avr_t * avr, uint8_t res)
{
	avr->sreg[S_V] = 0;
	_avr_flags_zns(avr, res);
}

static inline int _avr_is_instruction_32_bits(avr_t * avr, avr_flashaddr_t pc)
{
	uint16_t o = _avr_flash_read16le(avr, pc) & 0xfc0f;
	return	o == 0x9200 || // STS ! Store Direct to Data Space
			o == 0x9000 || // LDS Load Direct from Data Space
			o == 0x940c || // JMP Long Jump
			o == 0x940d || // JMP Long Jump
			o == 0x940e ||  // CALL Long Call to sub
			o == 0x940f; // CALL Long Call to sub
}

typedef void (*avr_inst_pfn)(
	avr_t * avr,
	uint32_t opcode,
	uint16_t * cycle,
	avr_flashaddr_t * new_pc);

#define INST_PFN_CALL(_pfn) \
	_pfn(avr, opcode, &cycle, &new_pc);

#define INST_PFN_SUB_CALL(_pfn) \
	_pfn(avr, opcode, cycle, new_pc);

#define INST_SUB_CALL(_opname, _args...) \
	_avr_inst_ ## _opname(avr, opcode, cycle, new_pc, ## _args)

#define INST_CALL(_opname, _args...) \
	_avr_inst_ ## _opname(avr, opcode, &cycle, &new_pc, ## _args)

#define INST_DECL(_opname, _args...) \
	static void \
		_avr_inst_ ## _opname( \
			avr_t * avr, \
			uint32_t opcode, \
			uint16_t * cycle, \
			avr_flashaddr_t * new_pc, \
			## _args)

#define INLINE_INST inline

#define INLINE_INST_DECL(_opname, _args...) \
	INLINE_INST INST_DECL(_opname, ## _args)

#define INST_SUB_CALL_DECL(_opname, _subcall_opname, _args...) \
	INST_DECL(_opname) \
	{ \
		INST_SUB_CALL(_subcall_opname, ## _args); \
	}

#define INST_OPCODE(_opname) \
	_avr_inst_opcode_ ## _opname

#define INST_AS_OPCODE_SUB_CALL_DECL(_opname, _subcall_opname, _args...) \
	INST_SUB_CALL_DECL(_opname, _subcall_opname, INST_OPCODE(_opname), ## _args)

#define INST_MASK_ABS22		0xfe0e
#define INST_MASK_ALL		0xffff
#define INST_MASK_A5B3		0xff00
#define INST_MASK_D3R3		0xff88
#define INST_MASK_D4R4		0xff00
#define INST_MASK_D5		0xfe0f
#define INST_MASK_D5A6		0xf800
#define INST_MASK_D5B3		0xfe08
#define INST_MASK_D5rXYZ	0xfe03
#define INST_MASK_D5rYZ_Q6	0xd200
#define INST_MASK_D5R5		0xfc00
#define INST_MASK_H4K8		0xf000
#define INST_MASK_O7S3		0xfc00
#define INST_MASK_O12		0xf000
#define INST_MASK_P2K6		0xff00
#define INST_MASK_SREG		0xff8f

#define INST_ESAC_TABLE /* primary list of avr instruction translation handlers */\
	INST_ESAC(0x0000, ALL, nop) /* NOP */\
	INST_ESAC(0x0100, D4R4, movw) /* MOVW -- 0x0100 -- Copy Register Word -- 0000 0001 dddd rrrr */\
	INST_ESAC(0x0200, D4R4, muls) /* MULS -- 0x0200 -- Multiply Signed -- 0000 0010 dddd rrrr */\
	INST_ESAC(0x0300, D3R3, mulsu) /* MULSU -- 0x0300 -- Multiply Signed Unsigned -- 0000 0011 0ddd 0rrr */\
	INST_ESAC(0x0308, D3R3, fmul) /* FMUL -- 0x0308 -- Fractional Multiply Unsigned -- 0000 0011 0ddd 1rrr */\
	INST_ESAC(0x0380, D3R3, fmuls) /* FMULS -- 0x0380 -- Multiply Signed -- 0000 0011 1ddd 0rrr */\
	INST_ESAC(0x0388, D3R3, fmulsu) /* FMULSU -- 0x0388 -- Multiply Signed Unsigned -- 0000 0011 1ddd 1rrr */\
	INST_ESAC(0x0400, D5R5, cpc) /* CPC -- 0x0400 -- Compare with carry -- 0000 01rd dddd rrrr */\
	INST_ESAC(0x0800, D5R5, sbc) /* SBC -- 0x0800 -- Subtract with carry -- 0000 10rd dddd rrrr */\
	INST_ESAC(0x0c00, D5R5, add) /* ADD -- 0x0c00 -- Add without carry -- 0000 11rd dddd rrrr */\
	INST_ESAC(0x1000, D5R5, cpse) /* CPSE -- 0x1000 -- Compare, skip if equal -- 0001 00rd dddd rrrr */\
	INST_ESAC(0x1400, D5R5, cp) /* CP -- 0x1400 -- Compare -- 0001 01rd dddd rrrr */\
	INST_ESAC(0x1800, D5R5, sub) /* SUB -- 0x1800-- Subtract without carry -- 0001 10rd dddd rrrr */\
	INST_ESAC(0x1c00, D5R5, addc) /* ADD -- 0x1c00-- Add with carry -- 0001 11rd dddd rrrr */\
	INST_ESAC(0x2000, D5R5, and) /* AND -- 0x2000 -- Logical AND -- 0010 00rd dddd rrrr */\
	INST_ESAC(0x2400, D5R5, eor) /* EOR -- 0x2400 -- Logical Exclusive OR -- 0010 01rd dddd rrrr */\
	INST_ESAC(0x2800, D5R5, or) /* OR -- 0x2800 -- Logical OR -- 0010 10rd dddd rrrr */\
	INST_ESAC(0x2c00, D5R5, mov) /* MOV -- 0x2c00 -- 0010 11rd dddd rrrr */\
	INST_ESAC(0x3000, H4K8, cpi) /* CPI -- 0x3000 -- Compare Immediate -- 0011 kkkk hhhh kkkk */\
	INST_ESAC(0x4000, H4K8, sbci) /* SBCI -- 0x4000-- Subtract Immediate With Carry -- 0100 kkkk hhhh kkkk */\
	INST_ESAC(0x5000, H4K8, subi) /* SUBI -- 0x5000 -- Subtract Immediate -- 0101 kkkk hhhh kkkk */\
	INST_ESAC(0x6000, H4K8, ori) /* ORI aka SBR -- 0x6000 -- Logical OR with Immediate -- 0110 kkkk hhhh kkkk */\
	INST_ESAC(0x7000, H4K8, andi) /* ANDI	-- 0x7000 -- Logical AND with Immediate -- 0111 kkkk hhhh kkkk */\
	INST_ESAC(0x8000, D5rYZ_Q6, ldd) /* LD (LDD) -- Load Indirect -- 10q0 qqsd dddd yqqq */\
	INST_ESAC(0x8200, D5rYZ_Q6, std) /* ST (STD) -- Store Indirect -- 10q0 qqsd dddd yqqq */\
	INST_ESAC(0x9000, D5, lds) /* LDS -- 0x9000 -- Load Direct from Data Space, 32 bits -- 1001 0000 0000 0000 */\
	INST_ESAC(0x9004, D5, lpm_z) /* LPM -- Load Program Memory -- 1001 000d dddd 01oo */\
	INST_ESAC(0x9005, D5, lpm_z_post_inc) /* LPM -- Load Program Memory -- 1001 000d dddd 01oo */\
	INST_ESAC(0x9001, D5rXYZ, ld_post_inc) /* LD -- 0x9001 -- Load Indirect from Data using XYZ++ -- 1001 00sd dddd iioo */\
	INST_ESAC(0x9002, D5rXYZ, ld_pre_dec) /* LD -- 0x9002 -- Load Indirect from Data using --XYZ -- 1001 00sd dddd iioo */\
	INST_ESAC(0x9006, D5, elpm_z) /* ELPM -- Load Program Memory -- 1001 000d dddd 01oo */\
	INST_ESAC(0x9007, D5, elpm_z_post_inc) /* ELPM -- Load Program Memory -- 1001 000d dddd 01oo */\
	INST_ESAC(0x900c, D5, ld_no_op) /* LD -- Load Indirect from Data using X -- 1001 000d dddd 11oo */\
	INST_ESAC(0x900f, D5, pop) /* POP -- 0x900f -- 1001 000d dddd 1111 */\
	INST_ESAC(0x9200, D5, sts) /* STS -- Store Direct to Data Space, 32 bits -- 1001 0010 0000 0000 */\
	INST_ESAC(0x9201, D5rXYZ, st_post_inc) /* ST -- Store Indirect Data Space XYZ++ -- 1001 001d dddd iioo */\
	INST_ESAC(0x9202, D5rXYZ, st_pre_dec) /* ST -- Store Indirect Data Space --XYZ -- 1001 001d dddd iioo */\
	INST_ESAC(0x920c, D5, st_no_op) /* ST -- Store Indirect Data Space X -- 1001 001d dddd 11oo */\
	INST_ESAC(0x920f, D5, push) /* PUSH -- 0x920f -- 1001 001d dddd 1111 */\
	INST_ESAC(0x9400, D5, com) /* COM -- 0x9400 -- One’s Complement -- 1001 010d dddd 0000 */\
	INST_ESAC(0x9401, D5, neg) /* NEG -- 0x9401 -- Two’s Complement -- 1001 010d dddd 0001 */\
	INST_ESAC(0x9402, D5, swap) /* SWAP -- 0x9402 -- Swap Nibbles -- 1001 010d dddd 0010 */\
	INST_ESAC(0x9403, D5, inc) /* INC -- 0x9403 -- Increment -- 1001 010d dddd 0011 */\
	INST_ESAC(0x9405, D5, asr) /* ASR -- 0x9405 -- Arithmetic Shift Right -- 1001 010d dddd 0101 */\
	INST_ESAC(0x9406, D5, lsr) /* LSR -- 0x9406 -- Logical Shift Right -- 1001 010d dddd 0110 */\
	INST_ESAC(0x9407, D5, ror) /* ROR -- 0x9407 -- Rotate Right -- 1001 010d dddd 0111 */\
	INST_ESAC(0x9408, SREG, set_sreg) /* SET -- 0x9408 -- Set SREG Bit -- 1001 0100 Bbbb 1000 */\
	INST_ESAC(0x9409, ALL, ijmp) /* IJMP --0x9409 -- Indirect jump -- 1001 010c 000e 1001 */\
	INST_ESAC(0x940a, D5, dec) /* DEC -- 0x940a -- Decrement -- 1001 010d dddd 1010 */\
	INST_ESAC(0x940c, ABS22, ljmp) /* LJMP -- 0x940c -- Long Call to sub, 32 bits -- 1001 010a aaaa 11ca */\
	INST_ESAC(0x940e, ABS22, lcall) /* LCALL -- 0x940e -- Long Call to sub, 32 bits -- 1001 010a aaaa 11ca */\
	INST_ESAC(0x9419, ALL, eijmp) /* EIJMP -- 0x9419 -- Indirect jump -- 1001 010c 000e 1001 */\
	INST_ESAC(0x9488, SREG, clr_sreg) /* CLR -- 0x9408 -- Set SREG Bit -- 1001 0100 Bbbb 1000 */\
	INST_ESAC(0x9508, ALL, ret) /* RET -- 0x9508 -- Return -- 1001 0101 0000 1000 */\
	INST_ESAC(0x9509, ALL, icall) /* ICALL -- 0x9509 -- Indirect Call to Subroutine -- 1001 010c 000e 1001 */\
	INST_ESAC(0x9518, ALL, reti) /* RETI -- 0x9518 -- Return from Interrupt -- 1001 0101 0001 1000 */\
	INST_ESAC(0x9519, ALL, eicall) /* EICALL -- 0x9519 -- Indirect Call to Subroutine -- 1001 010c 000e 1001 */\
	INST_ESAC(0x9588, ALL, sleep) /* SLEEP -- 0x9588 -- 1001 0101 1000 1000 */\
	INST_ESAC(0x9598, ALL, break) /* BREAK -- 0x9598 -- 1001 0101 1001 1000 */\
	INST_ESAC(0x95a8, ALL, wdr) /* WDR -- 0x95a8 -- Watchdog Reset -- 1001 0101 1010 1000 */\
	INST_ESAC(0x95c8, ALL, lpm_r0_z) /* LPM -- 0x95c8 -- Load Program Memory R0 <- (Z) -- 1001 0101 1100 1000 */\
	INST_ESAC(0x95e8, ALL, spm) /* SPM -- 0x95e8 -- Store Program Memory -- 1001 0101 1110 1000 */\
	INST_ESAC(0x9600, P2K6, adiw) /* ADIW -- 0x9600 -- Add Immediate to Word -- 1001 0110 KKpp KKKK */\
	INST_ESAC(0x9700, P2K6, sbiw) /* SBIW -- 0x9700 -- Subtract Immediate from Word -- 1001 0111 KKpp KKKK */\
	INST_ESAC(0x9800, A5B3, cbi) /* CBI -- 0x9800 -- Clear Bit in I/O Register -- 1001 1000 AAAA Abbb */\
	INST_ESAC(0x9900, A5B3, sbic) /* SBIC -- 0x9900 -- Skip if Bit in I/O Register is Cleared -- 1001 1001 AAAA Abbb */\
	INST_ESAC(0x9a00, A5B3, sbi) /* SBI -- 0x9a00 -- Set Bit in I/O Register -- 1001 1010 AAAA Abbb */\
	INST_ESAC(0x9b00, A5B3, sbis) /* SBIS -- 0x9b00 -- Skip if Bit in I/O Register is Set -- 1001 1011 AAAA Abbb */\
	INST_ESAC(0x9c00, D5R5, mul) /* MUL -- 0x9c00 -- Multiply Unsigned -- 1001 11rd dddd rrrr */\
	INST_ESAC(0xb000, D5A6, in) /* IN Rd,A -- 0xb000 -- 1011 0AAd dddd AAAA */\
	INST_ESAC(0xb800, D5A6, out) /* OUT A,Rr -- 0xb800 -- 1011 1AAd dddd AAAA */\
	INST_ESAC(0xc000, O12, rjmp) /* RJMP -- 0xc000 -- 1100 kkkk kkkk kkkk */\
	INST_ESAC(0xd000, O12, rcall) /* RCALL -- 0xd000 -- 1101 kkkk kkkk kkkk */\
	INST_ESAC(0xe000, H4K8, ldi) /* LDI Rd, K aka SER (LDI r, 0xff) -- 0xe000 -- 1110 kkkk dddd kkkk */\
	INST_ESAC(0xf000, O7S3, brxs) /* BRXS -- 0xf000 -- Branch if bit in SREG is set -- 1111 0Boo oooo osss */\
	INST_ESAC(0xf400, O7S3, brxc) /* BRXC -- 0xf400 -- Branch if bit in SREG is clear -- 1111 0Boo oooo osss */\
	INST_ESAC(0xf800, D5B3, bld) /* BST -- 0xf800 -- Bit Store from T into a Bit in Register -- 1111 10sd dddd 0bbb */\
	INST_ESAC(0xfa00, D5B3, bst) /* BLD -- 0xfa00 -- Bit Store from Bit in Register to T -- 1111 10sd dddd 0bbb */\
	INST_ESAC(0xfc00, D5B3, sbrc) /* SBRC -- 0xfc00 -- Skip if Bit in Register is Clear -- 1111 11sd dddd 0bbb */\
	INST_ESAC(0xfe00, D5B3, sbrs) /* SBRS -- 0xfe00 -- Skip if Bit in Register is Set -- 1111 11sd dddd 0bbb */\

#undef INST_ESAC
#define INST_ESAC(_opcode, _opmask, _opname, _args...) \
	_avr_inst_ ## _opcode ## _ ##  _opname,

enum { // this table provides instruction case indices
	INST_ESAC_NONE = 0, // starting with zero...  bad...  special case.
	INST_ESAC_TABLE // standard avr instructions
	INST_ESAC_TABLE_COUNT
};

#undef INST_ESAC
#define INST_ESAC(_opcode, _opmask, _opname, _args...) \
	INST_OPCODE(_opname) = _opcode,

enum { // this table provides opcode cross references
	INST_ESAC_TABLE
};

enum {
	INST_OP_NONE = 0,
	INST_OP_ADD,
	INST_OP_AND,
	INST_OP_EOR,
	INST_OP_OR,
	INST_OP_SUB,
};

enum {
	INST_FLAG_BIT_CARRY = 0,
	INST_FLAG_BIT_SAVE_RESULT,
};
	
#define INST_FLAG_NONE 0
#define INST_FLAG(_flag) (1 << INST_FLAG_BIT_ ## _flag)

/*
 * begin common code handlers
 *
 * all handlers in this section should probably be inlined
 * incomming flags should be constant variables
 *
 * depends heavily on dce (dead code elimination)
 */
INLINE_INST_DECL(adiw_sbiw, const uint16_t as_opcode)
{
	const int add = (as_opcode & 0x0100) == 0;

	get_vp2_k6(opcode);
	uint16_t res = vp;
	if (add)
		res += k;
	else
		res -= k;
	STATE("%s %s:%s[%04x], 0x%02x\n", add ? "adiw" : "sbiw", avr_regname(p), avr_regname(p + 1), vp, k);
	_avr_set_r16le_hl(avr, p, res);
	if (add) {
		avr->sreg[S_V] = ((~vp & res) >> 15) & 1;
		avr->sreg[S_C] = ((~res & vp) >> 15) & 1;
	} else {
		avr->sreg[S_V] = ((vp & ~res) >> 15) & 1;
		avr->sreg[S_C] = ((res & ~vp) >> 15) & 1;
	}
	_avr_flags_zns16(avr, res);
	SREG();
	(*cycle)++;
}


INLINE_INST_DECL(alu_common_helper, const uint8_t operation, const uint8_t flags, uint8_t r0, uint8_t vr0, uint8_t r1, uint8_t vr1)
{
	const int carry = 0 != (flags & INST_FLAG(CARRY));
	const int save_result = 0 != (flags & INST_FLAG(SAVE_RESULT));

	uint8_t res = vr0;
	
	switch (operation) {
		case	INST_OP_ADD:
			res += vr1 + (carry ? avr->sreg[S_C] : 0);
			break;
		case	INST_OP_AND:
			res &= vr1;
			break;
		case	INST_OP_EOR:
			res ^= vr1;
			break;
		case	INST_OP_NONE: {
				printf("%s: opcode = %04x, operation = %02x, flags = %04x\n", 
					__FUNCTION__, opcode, operation, flags);
		} break;
		case	INST_OP_OR:
			res |= vr1;
			break;
		case	INST_OP_SUB:
			res -= vr1 + (carry ? avr->sreg[S_C] : 0);
			break;
	}

	int trace = 0;
	T(trace = 1;)
	if (trace) {
		switch (operation) {
			case	INST_OP_ADD: {
				if (r0 == r1) {
					STATE("%s %s[%02x] = %02x\n", carry ? "rol" : "lsl" , avr_regname(r0), vr0, res);
				} else {
					STATE("%s %s[%02x], %s[%02x] = %02x\n", carry ? "addc" : "add",
						avr_regname(r0), vr0, avr_regname(r1), vr1, res);
				}
			} break;
			case INST_OP_AND: {
				if (r0 == r1) {
					STATE("tst %s[%02x]\n", avr_regname(r0), vr0);
				} else {
					if (r1 != 0xff) {
						STATE("and %s[%02x], %s[%02x] = %02x\n", avr_regname(r0), vr0, avr_regname(r1), vr1, res);
					} else {
						STATE("andi %s[%02x], %02x = %02x\n", avr_regname(r0), vr0, vr1, res);
					}
				}
			} break;
			case INST_OP_EOR: {
				if (r0 == r1) {
					STATE("clr %s[%02x]\n", avr_regname(r0), vr0);
				} else {
					STATE("eor %s[%02x], %s[%02x] = %02x\n", avr_regname(r0), vr0, avr_regname(r0), vr1, res);
				}
			} break;
			case INST_OP_NONE: {
			} break;
			case INST_OP_OR: {
				if (r1 != 0xff) {
					STATE("or %s[%02x], %s[%02x] = %02x\n", avr_regname(r0), vr0, avr_regname(r1), vr1, res);
				} else {
					STATE("ori %s[%02x], %02x = %02x\n", avr_regname(r0), vr0, vr1, res);
				}
			} break;
			case INST_OP_SUB: {
				if (r1 != 0xff) {
					const char __attribute__((unused)) * opname[2][2]=
						{ { "cp", "cpc" }, { "sub", "sbc" } };
					STATE("%s %s[%02x], %s[%02x] = %02x\n",
						opname[save_result][carry], 
						avr_regname(r0), vr0, avr_regname(r1), vr1, res);
				} else {
					const char __attribute__((unused)) * opname[2][2]=
						{ { "cpi", "" }, { "subi", "sbci" } };
					STATE("%s %s[%02x], %02x = %02x\n",
						opname[save_result][carry], 
						avr_regname(r0), vr0, vr1, res);
				}
			} break;
		}
	}

	if (save_result)
		_avr_set_r(avr, r0, res);

	switch (operation) {
		case INST_OP_ADD:
			_avr_flags_add_zns(avr, res, vr0, vr1);
			break;
		case INST_OP_AND:
		case INST_OP_EOR:
		case INST_OP_OR:
			_avr_flags_znv0s(avr, res);
			break;
		case INST_OP_NONE:
			break;
		case INST_OP_SUB:
			if (carry)
				_avr_flags_sub_Rzns(avr, res, vr0, vr1);
			else
				_avr_flags_sub_zns(avr, res, vr0, vr1);
			break;
	}

	if (operation) {
		SREG();
	}
}

INLINE_INST_DECL(bld_bst, const uint16_t as_opcode)
{
	const int load = (as_opcode & 0x0200) == 0;

	get_vd5_s3_mask(opcode);

	if (load) {
		uint8_t v = (vd & ~mask) | (avr->sreg[S_T] ? mask : 0);
		STATE("bld %s[%02x], 0x%02x = %02x\n", avr_regname(d), vd, mask, v);
		_avr_set_r(avr, d, v);
	} else {
		STATE("bst %s[%02x], 0x%02x\n", avr_regname(d), vd, mask);
		avr->sreg[S_T] = (vd >> s) & 1;
	}
	
	SREG();
}

INLINE_INST_DECL(brxc_brxs, const uint16_t as_opcode)
{
	const int set = (as_opcode & 0x0400) == 0;		// this bit means BRXC otherwise BRXS

	int16_t o = ((int16_t)(opcode << 6)) >> 9; // offset
	uint8_t s = opcode & 7;
	int branch = (avr->sreg[s] && set) || (!avr->sreg[s] && !set);
	const char *names[2][8] = {
			{ "brcc", "brne", "brpl", "brvc", NULL, "brhc", "brtc", "brid"},
			{ "brcs", "breq", "brmi", "brvs", NULL, "brhs", "brts", "brie"},
	};
	if (names[set][s]) {
		STATE("%s .%d [%04x]\t; Will%s branch\n", names[set][s], o, *new_pc + (o << 1), branch ? "":" not");
	} else {
		STATE("%s%c .%d [%04x]\t; Will%s branch\n", set ? "brbs" : "brbc", _sreg_bit_name[s], o, *new_pc + (o << 1), branch ? "":" not");
	}
	if (branch) {
		(*cycle)++; // 2 cycles if taken, 1 otherwise
		*new_pc = *new_pc + (o << 1);
	}
}

INLINE_INST_DECL(call_jmp_ei, const uint16_t as_opcode)
{
	const int e = as_opcode & 0x10;
	const int c = as_opcode & 0x100;

	if (e && !avr->eind)
		_avr_invalid_opcode(avr);
	uint32_t z = _avr_data_read16le(avr, R_ZL);
	if (e)
		z |= avr->data[avr->eind] << 16;
	STATE("%si%s Z[%04x]\n", e ? "e" : "", c ? "call" : "jmp", z << 1);
	if (c)
		*cycle += _avr_push_addr(avr, *new_pc) - 1;
	*new_pc = z << 1;
	(*cycle)++;
	TRACE_JUMP();
}

INLINE_INST_DECL(call_jmp_long, const uint16_t as_opcode)
{
	const int call = as_opcode & 2;

	avr_flashaddr_t a = ((opcode & 0x01f0) >> 3) | (opcode & 1);
	uint16_t x = _avr_flash_read16le(avr, *new_pc);
	a = (a << 16) | x;
	STATE("%s 0x%06x\n", call ? "call" : "jmp", a);
	if (call)
		*cycle += 1 + _avr_push_addr(avr, *new_pc + 2);
	else
		*cycle += 2;
	*new_pc = a << 1;
	TRACE_JUMP();
	STACK_FRAME_PUSH();
}

INLINE_INST_DECL(call_jmp_r, const uint16_t as_opcode)
{
	const int call = as_opcode & 0x1000;

	get_o12(opcode);
	STATE("r%s .%d [%04x]\n", call ? "call" : "jmp", o >> 1, *new_pc + o);
	if(call)
		*cycle += _avr_push_addr(avr, *new_pc);
	else
		(*cycle)++;
	*new_pc = *new_pc + o;
	if (!call || (o != 0)) {
		TRACE_JUMP();
	}
	if(call) {
		STACK_FRAME_PUSH();
	}
}

INLINE_INST_DECL(cbi_sbi, const uint16_t as_opcode)
{
	const int set = as_opcode & 0x0200;

	get_io5_b3mask(opcode);
	uint8_t res = _avr_get_ram(avr, io);
	if(set)
		res |= mask;
	else
		res &= ~mask;
	STATE("%s %s[%04x], 0x%02x = %02x\n", set ? "sbi" : "cbi", avr_regname(io), avr->data[io], mask, res);
	_avr_set_ram(avr, io, res);
	(*cycle)++;
}

INLINE_INST_DECL(elpm_lpm, const uint16_t as_opcode)
{
	const int lpm_r0_z = as_opcode == INST_OPCODE(lpm_r0_z); /* ? LPM 0, Z */
	const int elpm = (!lpm_r0_z) ? (as_opcode & 2) : 0;
	
	if (elpm && !avr->rampz)
		_avr_invalid_opcode(avr);

	uint8_t rzd = 0;
	int op = 0;
	
	if (!lpm_r0_z) {
		get_d5(opcode);

		rzd = d;
		op = opcode & 1;
	}
	
	uint32_t z = _avr_data_read16le(avr, R_ZL);
	
	if (elpm) {
		uint8_t rampzv = avr->data[avr->rampz];
		STATE("elpm %s, (Z[%02x:%04x]%s)\n", avr_regname(rzd), rampzv, z & 0xffff, op ? "+" : "");
		z |= rampzv << 16;
	} else {
		STATE("lpm %s, (Z[%04x]%s)\n", avr_regname(rzd), z, op ? "+" : "");
	}

	_avr_set_r(avr, rzd, avr->flash[z]);
	if (op) {
		z++;
		if (elpm)
			_avr_set_r(avr, avr->rampz, z >> 16);
			
		_avr_set_r16le_hl(avr, R_ZL, z);
	}
	*cycle += 2; // 3 cycles
}

INLINE_INST_DECL(in_out, const uint16_t as_opcode)
{
	const int out = as_opcode & 0x800;

	get_d5_a6(opcode);
	STATE("%s %s, %s[%02x]\n", out ? "out" : "in" , avr_regname(A), avr_regname(d), avr->data[d]);
	if (out)
		_avr_set_ram(avr, A, avr->data[d]);
	else
		_avr_set_r(avr, d, _avr_get_ram(avr, A));
}

/*
 * Load store instructions
 *
 * 1001 00sr rrrr iioo
 * s = 0 = load, 1 = store
 * ii = 16 bits register index, 11 = X, 10 = Y, 00 = Z
 * oo = 1) post increment, 2) pre-decrement
 *
 * Addendum notes:
 *	ii == 00,  oo == 00 lds/sts
 *		s == 0 --> lds, s == 1 --> sts
 *	ii == 01 (e)lpm
 *	ii == 11, oo == 11 pop/push
 *		s == 0 --> pop, s == 1 --> push
 */
INLINE_INST_DECL(ld_st, const uint16_t as_opcode)
{
	const int load = !(as_opcode & 0x0200);
	const int op = as_opcode & 3;

	uint8_t vd = 0;
	get_d5(opcode);
	if (!load)
		vd = avr->data[d];

	uint8_t  rXYZ = ((uint8_t []){R_ZL, 0x00, R_YL, R_XL})[(opcode & 0x000c) >> 2];
	uint16_t vXYZ = _avr_data_read16le(avr, rXYZ);

	if (load) {
		STATE("ld %s, %s%c[%04x]%s\n", 
			avr_regname(d), op == 2 ? "--" : "", 
			*avr_regname(rXYZ), vXYZ, op == 1 ? "++" : "");
	} else {
		STATE("st %s%c[%04x]%s, %s[%02x] \n", 
			op == 2 ? "--" : "", *avr_regname(rXYZ), vXYZ, 
			op == 1 ? "++" : "", avr_regname(d), vd);
	}

	(*cycle)++; // 2 cycles (1 for tinyavr, except with inc/dec 2)
	if (op == 2) vXYZ--;
	if (load)
		vd = _avr_get_ram(avr, vXYZ);
	else
		_avr_set_ram(avr, vXYZ, vd);
	if (op == 1) vXYZ++;
	if (op)
		_avr_set_r16le_hl(avr, rXYZ, vXYZ);
	if (load)
		_avr_set_r(avr, d, vd);
}

/*
 * Load (LDD/STD) store instructions
 *
 * 10q0 qqsd dddd yqqq
 * s = 0 = load, 1 = store
 * y = 16 bits register index, 1 = Y, 0 = X
 * q = 6 bit displacement
 */
INLINE_INST_DECL(ldd_std, const uint16_t as_opcode)
{
	const int load = !(as_opcode & 0x0200);

	get_d5_q6(opcode);
	uint8_t  rYZ = (opcode & 0x0008) ? R_YL : R_ZL;
	uint16_t vYZ = _avr_data_read16le(avr, rYZ);
	if (load) {
		uint8_t vvr = _avr_get_ram(avr, vYZ + q);
		STATE("ld %s, (%c+%d[%04x])=[%02x]\n", 
			avr_regname(d), *avr_regname(rYZ), 
			q, vYZ + q, vvr);
		_avr_set_r(avr, d, vvr);
	} else {
		uint8_t vd = avr->data[d];
		STATE("st (%c+%d[%04x]), %s[%02x]\n", 
			*avr_regname(rYZ), q, vYZ + q, 
			avr_regname(d), vd);
		_avr_set_ram(avr, vYZ + q, vd);
	}
	(*cycle)++; // 2 cycles, 3 for tinyavr
}

INLINE_INST_DECL(lds_sts, const uint16_t as_opcode)
{
	const int load = !(as_opcode & 0x0200);

	get_vd5(opcode);
	uint16_t x = _avr_flash_read16le(avr, *new_pc);
	*new_pc += 2;

	if (load) {
		STATE("lds %s[%02x], 0x%04x\n", avr_regname(d), vd, x);
		_avr_set_r(avr, d, _avr_get_ram(avr, x));
	} else {
		STATE("sts 0x%04x, %s[%02x]\n", x, avr_regname(d), vd);
	}

	(*cycle)++; // 2 cycles

	if (!load)
		_avr_set_ram(avr, x, vd);
}

INLINE_INST_DECL(mul_complex, const uint16_t as_opcode)
{
	int8_t r = 16 + (opcode & 0x7);
	int8_t d = 16 + ((opcode >> 4) & 0x7);
	int16_t res = 0;
	uint8_t c = 0;
	T(const char * name = "";)
	switch (as_opcode & 0x88) {
		case 0x00: 	// MULSU -- Multiply Signed Unsigned -- 0000 0011 0ddd 0rrr
			res = ((uint8_t)avr->data[r]) * ((int8_t)avr->data[d]);
			c = (res >> 15) & 1;
			T(name = "mulsu";)
			break;
		case 0x08: 	// FMUL -- Fractional Multiply Unsigned -- 0000 0011 0ddd 1rrr
			res = ((uint8_t)avr->data[r]) * ((uint8_t)avr->data[d]);
			c = (res >> 15) & 1;
			res <<= 1;
			T(name = "fmul";)
			break;
		case 0x80: 	// FMULS -- Multiply Signed -- 0000 0011 1ddd 0rrr
			res = ((int8_t)avr->data[r]) * ((int8_t)avr->data[d]);
			c = (res >> 15) & 1;
			res <<= 1;
			T(name = "fmuls";)
			break;
		case 0x88: 	// FMULSU -- Multiply Signed Unsigned -- 0000 0011 1ddd 1rrr
			res = ((uint8_t)avr->data[r]) * ((int8_t)avr->data[d]);
			c = (res >> 15) & 1;
			res <<= 1;
			T(name = "fmulsu";)
			break;
	}
	(*cycle)++;
	STATE("%s %s[%d], %s[%02x] = %d\n", name, avr_regname(d), ((int8_t)avr->data[d]), avr_regname(r), ((int8_t)avr->data[r]), res);
	_avr_set_r16le(avr, 0, res);
	avr->sreg[S_C] = c;
	avr->sreg[S_Z] = res == 0;
	SREG();
}

INLINE_INST_DECL(skip_if, uint16_t res)
{
	if (res) {
		if (_avr_is_instruction_32_bits(avr, *new_pc)) {
			*new_pc += 4; *cycle += 2;
		} else {
			*new_pc += 2; (*cycle)++;
		}
	}
}

INLINE_INST_DECL(skip_io_r_logic, const uint16_t as_opcode, uint8_t rio, uint8_t vrio, uint8_t mask, char *opname_array[2])
{
	const int set = (as_opcode & 0x0200) != 0;

	int branch = ((vrio & mask) && set) || (!(vrio & mask) && !set);
	STATE("%s %s[%02x], 0x%02x\t; Will%s branch\n", opname_array[set], avr_regname(rio), vrio, mask, branch ? "":" not");
	INST_SUB_CALL(skip_if, branch);
}

INLINE_INST_DECL(sbic_sbis, const uint16_t as_opcode)
{
	get_io5_b3mask(opcode);
	uint8_t vio = _avr_get_ram(avr, io);
	char *opname_array[2] = { "sbic", "sbis" };
	INST_SUB_CALL(skip_io_r_logic, as_opcode, io, vio, mask, opname_array);
}

INLINE_INST_DECL(sbrc_sbrs, const uint16_t as_opcode)
{
	get_vd5_s3_mask(opcode);
	char *opname_array[2] = { "sbrc", "sbrs" };
	INST_SUB_CALL(skip_io_r_logic, as_opcode, d, vd, mask, opname_array);
}

INLINE_INST_DECL(sreg_cl_se, const uint16_t as_opcode)
{
	const int clr = as_opcode & 0x0080;

	get_sreg_bit(opcode);
	STATE("%s%c\n", clr ? "cl" : "se", _sreg_bit_name[b]);
	avr_sreg_set(avr, b, clr ? 0 : 1);
	SREG();
}

INLINE_INST_DECL(d5r5_common_helper, const uint8_t operation, const uint8_t flags)
{
	get_vd5_vr5(opcode);
	INST_SUB_CALL(alu_common_helper, operation, flags, d, vd, r, vr);
}

INLINE_INST_DECL(h4k8_common_helper, const uint8_t operation, const uint8_t flags)
{
	get_vh4_k8(opcode);
	INST_SUB_CALL(alu_common_helper, operation, flags, h, vh, -1, k);
}

/*
 * end common code, begin specialized instruction handlers
 *
 * inlining here may not be as important.
 */

INST_SUB_CALL_DECL(add, d5r5_common_helper, INST_OP_ADD, INST_FLAG(SAVE_RESULT))
INST_SUB_CALL_DECL(addc, d5r5_common_helper, INST_OP_ADD, INST_FLAG(CARRY) | INST_FLAG(SAVE_RESULT))

INST_AS_OPCODE_SUB_CALL_DECL(adiw, adiw_sbiw)

INST_SUB_CALL_DECL(and, d5r5_common_helper, INST_OP_AND, INST_FLAG(SAVE_RESULT))
INST_SUB_CALL_DECL(andi, h4k8_common_helper, INST_OP_AND, INST_FLAG(SAVE_RESULT))

INST_DECL(asr)
{
	get_vd5(opcode);
	uint8_t res = ((int8_t)vd) >> 1; /* (vd >> 1) | (vd & 0x80); */
	STATE("asr %s[%02x]\n", avr_regname(d), vd);
	_avr_set_r(avr, d, res);
	_avr_flags_zcnvs(avr, res, vd);
	SREG();
}

INST_AS_OPCODE_SUB_CALL_DECL(bld, bld_bst)

INST_AS_OPCODE_SUB_CALL_DECL(brxc, brxc_brxs)
INST_AS_OPCODE_SUB_CALL_DECL(brxs, brxc_brxs)

INST_AS_OPCODE_SUB_CALL_DECL(bst, bld_bst)

INST_DECL(break)
{
	STATE("break\n");
	if (avr->gdb) {
		// if gdb is on, we break here as in here
		// and we do so until gdb restores the instruction
		// that was here before
		avr->state = cpu_StepDone;
		*new_pc = avr->pc;
		*cycle = 0;
	}
}

INST_AS_OPCODE_SUB_CALL_DECL(cbi, cbi_sbi)

INST_AS_OPCODE_SUB_CALL_DECL(clr_sreg, sreg_cl_se)

INST_DECL(com)
{
	get_vd5(opcode);
	uint8_t res = 0xff - vd;
	STATE("com %s[%02x] = %02x\n", avr_regname(d), vd, res);
	_avr_set_r(avr, d, res);
	_avr_flags_znv0s(avr, res);
	avr->sreg[S_C] = 1;
	SREG();
}

INST_SUB_CALL_DECL(cp, d5r5_common_helper, INST_OP_SUB, INST_OP_NONE)
INST_SUB_CALL_DECL(cpc, d5r5_common_helper, INST_OP_SUB, INST_FLAG(CARRY))
INST_SUB_CALL_DECL(cpi, h4k8_common_helper, INST_OP_SUB, INST_OP_NONE)

INST_DECL(cpse)
{
	get_vd5_vr5(opcode);
	uint16_t res = vd == vr;
	STATE("cpse %s[%02x], %s[%02x]\t; Will%s skip\n", avr_regname(d), vd, avr_regname(r), vr, res ? "":" not");
	INST_SUB_CALL(skip_if, res);
}

INST_DECL(dec)
{
	get_vd5(opcode);
	uint8_t res = vd - 1;
	STATE("dec %s[%02x] = %02x\n", avr_regname(d), vd, res);
	_avr_set_r(avr, d, res);
	avr->sreg[S_V] = res == 0x7f;
	_avr_flags_zns(avr, res);
	SREG();
}

INST_AS_OPCODE_SUB_CALL_DECL(eicall, call_jmp_ei)
INST_AS_OPCODE_SUB_CALL_DECL(eijmp, call_jmp_ei)

INST_AS_OPCODE_SUB_CALL_DECL(elpm_z, elpm_lpm)
INST_AS_OPCODE_SUB_CALL_DECL(elpm_z_post_inc, elpm_lpm)

INST_SUB_CALL_DECL(eor, d5r5_common_helper, INST_OP_EOR, INST_FLAG(SAVE_RESULT))

INST_AS_OPCODE_SUB_CALL_DECL(fmul, mul_complex)
INST_AS_OPCODE_SUB_CALL_DECL(fmuls, mul_complex)
INST_AS_OPCODE_SUB_CALL_DECL(fmulsu, mul_complex)

INST_AS_OPCODE_SUB_CALL_DECL(icall, call_jmp_ei)
INST_AS_OPCODE_SUB_CALL_DECL(ijmp, call_jmp_ei)

INST_AS_OPCODE_SUB_CALL_DECL(in, in_out)

INST_DECL(inc)
{
	get_vd5(opcode);
	uint8_t res = vd + 1;
	STATE("inc %s[%02x] = %02x\n", avr_regname(d), vd, res);
	_avr_set_r(avr, d, res);
	avr->sreg[S_V] = res == 0x80;
	_avr_flags_zns(avr, res);
	SREG();
}

INST_AS_OPCODE_SUB_CALL_DECL(lcall, call_jmp_long)

INST_AS_OPCODE_SUB_CALL_DECL(ld_no_op, ld_st)
INST_AS_OPCODE_SUB_CALL_DECL(ld_post_inc, ld_st)
INST_AS_OPCODE_SUB_CALL_DECL(ld_pre_dec, ld_st)

INLINE_INST_DECL(ldi)
{
	get_h4_k8(opcode);
	STATE("ldi %s, 0x%02x\n", avr_regname(h), k);
	_avr_set_r(avr, h, k);
}

INST_AS_OPCODE_SUB_CALL_DECL(ldd, ldd_std)

INST_AS_OPCODE_SUB_CALL_DECL(lds, lds_sts)

INST_AS_OPCODE_SUB_CALL_DECL(ljmp, call_jmp_long)

INST_AS_OPCODE_SUB_CALL_DECL(lpm_r0_z, elpm_lpm)
INST_AS_OPCODE_SUB_CALL_DECL(lpm_z, elpm_lpm)
INST_AS_OPCODE_SUB_CALL_DECL(lpm_z_post_inc, elpm_lpm)

INST_DECL(lsr)
{
	get_vd5(opcode);
	uint8_t res = vd >> 1;
	STATE("lsr %s[%02x]\n", avr_regname(d), vd);
	_avr_set_r(avr, d, res);
	avr->sreg[S_N] = 0;
	_avr_flags_zcvs(avr, res, vd);
	SREG();
}

INST_DECL(mov)
{
	get_d5_vr5(opcode);
	uint8_t res = vr;
	STATE("mov %s, %s[%02x] = %02x\n", avr_regname(d), avr_regname(r), vr, res);
	_avr_set_r(avr, d, res);
}

INST_DECL(movw)
{
	uint8_t d = (opcode >> 3) & 0x1e; /* ((opcode >> 4) & 0x0f) << 1; */
	uint8_t r = (opcode & 0xf) << 1;
	uint16_t vr = _avr_data_read16le(avr, r);
	STATE("movw %s:%s, %s:%s[%04x]\n", avr_regname(d), avr_regname(d+1), avr_regname(r), avr_regname(r+1), vr);
	_avr_set_r16le(avr, d, vr);
}

INST_DECL(mul)
{
	get_vd5_vr5(opcode);
	uint16_t res = vd * vr;
	STATE("mul %s[%02x], %s[%02x] = %04x\n", avr_regname(d), vd, avr_regname(r), vr, res);
	(*cycle)++;
	_avr_set_r16le(avr, 0, res);
	avr->sreg[S_Z] = res == 0;
	avr->sreg[S_C] = (res >> 15) & 1;
	SREG();
}

INST_DECL(muls)
{
	int8_t r = 16 + (opcode & 0xf);
	int8_t d = 16 + ((opcode >> 4) & 0xf);
	int16_t res = ((int8_t)avr->data[r]) * ((int8_t)avr->data[d]);
	STATE("muls %s[%d], %s[%02x] = %d\n", avr_regname(d), ((int8_t)avr->data[d]), avr_regname(r), ((int8_t)avr->data[r]), res);
	_avr_set_r16le(avr, 0, res);
	avr->sreg[S_C] = (res >> 15) & 1;
	avr->sreg[S_Z] = res == 0;
	(*cycle)++;
	SREG();
}

INST_AS_OPCODE_SUB_CALL_DECL(mulsu, mul_complex)

INST_DECL(neg)
{
	get_vd5(opcode);
	uint8_t res = 0x00 - vd;
	STATE("neg %s[%02x] = %02x\n", avr_regname(d), vd, res);
	_avr_set_r(avr, d, res);
	avr->sreg[S_H] = ((res | vd) >> 3) & 1; /* ((res >> 3) | (vd >> 3)) & 1; */
	avr->sreg[S_V] = res == 0x80;
	avr->sreg[S_C] = res != 0;
	_avr_flags_zns(avr, res);
	SREG();
}

INST_DECL(nop)
{
	STATE("nop\n");
}

INST_SUB_CALL_DECL(or, d5r5_common_helper, INST_OP_OR, INST_FLAG(SAVE_RESULT))
INST_SUB_CALL_DECL(ori, h4k8_common_helper, INST_OP_OR, INST_FLAG(SAVE_RESULT))

INST_DECL(pop)
{
	get_d5(opcode);
	uint8_t vsp = _avr_pop8(avr);
	_avr_set_r(avr, d, vsp);
	T(uint16_t sp = _avr_sp_get(avr);)
	STATE("pop %s (@%04x)[%02x]\n", avr_regname(d), sp, vsp);
	(*cycle)++;
}

INST_DECL(push)
{
	get_vd5(opcode);
	_avr_push8(avr, vd);
	T(uint16_t sp = _avr_sp_get(avr);)
	STATE("push %s[%02x] (@%04x)\n", avr_regname(d), vd, sp);
	(*cycle)++;
}

INST_AS_OPCODE_SUB_CALL_DECL(out, in_out)

INST_AS_OPCODE_SUB_CALL_DECL(rcall, call_jmp_r)

INST_DECL(ret)
{
	*new_pc = _avr_pop_addr(avr);
	*cycle += 1 + avr->address_size;
	STATE("ret%s\n", opcode & 0x10 ? "i" : "");
	TRACE_JUMP();
	STACK_FRAME_POP();
}

INST_DECL(reti)
{
	avr_sreg_set(avr, S_I, 1);
	avr_interrupt_reti(avr);
	INST_SUB_CALL(ret);
}

INST_AS_OPCODE_SUB_CALL_DECL(rjmp, call_jmp_r)

INST_DECL(ror)
{
	get_vd5(opcode);
	uint8_t res = (avr->sreg[S_C] ? 0x80 : 0) | vd >> 1;
	STATE("ror %s[%02x]\n", avr_regname(d), vd);
	_avr_set_r(avr, d, res);
	_avr_flags_zcnvs(avr, res, vd);
	SREG();
}

INST_SUB_CALL_DECL(sbc, d5r5_common_helper, INST_OP_SUB, INST_FLAG(CARRY) | INST_FLAG(SAVE_RESULT))
INST_SUB_CALL_DECL(sbci, h4k8_common_helper, INST_OP_SUB, INST_FLAG(CARRY) | INST_FLAG(SAVE_RESULT))

INST_AS_OPCODE_SUB_CALL_DECL(sbi, cbi_sbi)

INST_AS_OPCODE_SUB_CALL_DECL(sbic, sbic_sbis)
INST_AS_OPCODE_SUB_CALL_DECL(sbis, sbic_sbis)

INST_AS_OPCODE_SUB_CALL_DECL(sbiw, adiw_sbiw)

INST_AS_OPCODE_SUB_CALL_DECL(sbrc, sbrc_sbrs)
INST_AS_OPCODE_SUB_CALL_DECL(sbrs, sbrc_sbrs)

INST_AS_OPCODE_SUB_CALL_DECL(std, ldd_std)

INST_AS_OPCODE_SUB_CALL_DECL(sts, lds_sts)

INST_SUB_CALL_DECL(sub, d5r5_common_helper, INST_OP_SUB, INST_FLAG(SAVE_RESULT))
INST_SUB_CALL_DECL(subi, h4k8_common_helper, INST_OP_SUB, INST_FLAG(SAVE_RESULT))

INST_DECL(sleep)
{
	STATE("sleep\n");
	/* Don't sleep if there are interrupts about to be serviced.
	 * Without this check, it was possible to incorrectly enter a state
	 * in which the cpu was sleeping and interrupts were disabled. For more
	 * details, see the commit message. */
	if (!avr_has_pending_interrupts(avr) || !avr->sreg[S_I])
		avr->state = cpu_Sleeping;
}

INST_DECL(spm)
{
	STATE("spm\n");
	avr_ioctl(avr, AVR_IOCTL_FLASH_SPM, 0);
}

INST_AS_OPCODE_SUB_CALL_DECL(set_sreg, sreg_cl_se)

INST_AS_OPCODE_SUB_CALL_DECL(st_no_op, ld_st)
INST_AS_OPCODE_SUB_CALL_DECL(st_post_inc, ld_st)
INST_AS_OPCODE_SUB_CALL_DECL(st_pre_dec, ld_st)

INST_DECL(swap)
{
	get_vd5(opcode);
	uint8_t res = (vd >> 4) | (vd << 4) ;
	STATE("swap %s[%02x] = %02x\n", avr_regname(d), vd, res);
	_avr_set_r(avr, d, res);
}

INST_DECL(wdr)
{
	STATE("wdr\n");
	avr_ioctl(avr, AVR_IOCTL_WATCHDOG_RESET, 0);
}

typedef struct avr_inst_decode_elem_t {
	uint16_t opcode;
	uint16_t mask;
	avr_inst_pfn inst_pfn;
	avr_inst_opcode_xlat_pfn xlat_pfn;
	char *opname;
}avr_inst_decode_elem_t, *avr_inst_decode_elem_p;

#undef XLAT_INST_ESAC
#define XLAT_INST_ESAC(_opcode, _opmask, _xlat, _opname, _args...) \
	{ _opcode, INST_MASK_ ## _opmask, _avr_inst_ ## _opname, _avr_inst_opcode_xlat_ ## _xlat, #_opname },

#undef INST_ESAC
#define INST_ESAC(_opcode, _opmask, _opname, _args...) \
	XLAT_INST_ESAC(_opcode, _opmask, _opmask, _opname, _args...)

INST_DECL(decode_one);
static avr_inst_decode_elem_t _avr_inst_opcode_table[] =  {
	{ -1, -1, _avr_inst_decode_one, 0, "" }, // yes virginia, sometimes you can have a free lunch.
	INST_ESAC_TABLE
};

static void
_avr_inst_collision_detected(
	avr_t * avr,
	uint32_t opcode,
	avr_inst_decode_elem_p table_elem,
	uint32_t extend_opcode)
{
	avr_inst_decode_elem_p extend_table_elem = &_avr_inst_opcode_table[extend_opcode >> 24];
	printf("%s:@ opcode %04x, mask %04x (%s); " 
		"extend_opcode %08x (opcode %04x, mask %04x (%s))\n", 
		__FUNCTION__, table_elem->opcode, table_elem->mask, table_elem->opname, 
		extend_opcode, extend_table_elem->opcode, extend_table_elem->mask, extend_table_elem->opname);
}

#define IF_OP(_opcode, _opmask) \
	if ((_opcode) == (opcode & (_opmask)))

#undef INST_ESAC
#undef XLAT_INST_ESAC

/*
 * Main opcode decoder
 * 
 * The original decoder was written by following the datasheet in no particular order.
 * It has since been modified and rewritten.
 * 
 * + It doesn't check whether the core it's emulating is supposed to have the 
 *	fancy instructions, like multiply and such.
 * + Folding when implimented and if used, cycle accounting is to remain accurate but
 *	instructions will complete before interrupts or timers are handled.
 * 
 */
 
INST_DECL(decode_one)
{
	int invalid_opcode = 1;
	uint32_t extend_opcode = 0;

	opcode = _avr_flash_read16le(avr, avr->pc);

	avr_inst_decode_elem_p table_elem = &_avr_inst_opcode_table[1];
	for(uint8_t handler = 1; handler < INST_ESAC_TABLE_COUNT; handler++) {
		if (0 == extend_opcode) {
			IF_OP(table_elem->opcode, table_elem->mask) {
				invalid_opcode = 0;
				opcode |= (handler << 24);
				opcode = INST_OPCODE_XLAT_PFN_CALL(table_elem->xlat_pfn);
				INST_PFN_SUB_CALL(table_elem->inst_pfn);
			}
		} else
			if (0) _avr_inst_collision_detected(avr, opcode, table_elem, extend_opcode);
		table_elem++;
	}

	if (invalid_opcode)
		_avr_invalid_opcode(avr);

	if (extend_opcode)
		_avr_extend_flash_write32le(avr, avr->pc, extend_opcode);
	else
		printf("%s: %06x, %04x, %08x >> not translated.\n", 
			__FUNCTION__, avr->pc, opcode, extend_opcode);
}

avr_flashaddr_t avr_run_one(avr_t * avr)
{
run_one_again:
#if CONFIG_SIMAVR_TRACE
	/*
	 * this traces spurious reset or bad jumps
	 */
	if ((avr->pc == 0 && avr->cycle > 0) || avr->pc >= avr->codeend || _avr_sp_get(avr) > avr->ramend) {
		avr->trace = 1;
		STATE("RESET\n");
		crash(avr);
	}
	avr->trace_data->touched[0] = avr->trace_data->touched[1] = avr->trace_data->touched[2] = 0;
#endif

	/* Ensure we don't crash simavr due to a bad instruction reading past
	 * the end of the flash.
	 */
	if (unlikely(avr->pc >= avr->flashend)) {
		STATE("CRASH\n");
		crash(avr);
		return 0;
	}

	uint32_t		opcode = _avr_extend_flash_read32le(avr, avr->pc);
	avr_flashaddr_t		new_pc = avr->pc + 2;	// future "default" pc
	uint16_t		cycle = 1;

	INST_PFN_CALL(_avr_inst_opcode_table[opcode >> 24].inst_pfn);

	avr->cycle += cycle;

	if ((avr->state == cpu_Running) && 
		(avr->run_cycle_count > cycle) && 
		(avr->interrupt_state == 0))
	{
		avr->run_cycle_count -= cycle;
		avr->pc = new_pc;
		goto run_one_again;
	}
	
	return new_pc;
}


