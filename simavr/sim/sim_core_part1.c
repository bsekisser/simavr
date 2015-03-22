typedef struct _inst_op_t {
	uint16_t	opcode;
	uint16_t	mask;
	uint32_t	r[3];
}_inst_op_t, *_inst_op_p;

typedef _inst_op_list_p *_inst_op_t[];

static _inst_op_list_p _decode_inst(avr_t * avr, _inst_op_list_p op_list)
{
	return(&op_list[1]);
}

#define _adc(x) r0 = r1 + r2 + _avr_sreg(avr, S_C)
#define _add(x) r0 = r1 + r2
#define _add_flags _add; _avr_flags_add_zns(avr, r0, r1, r2);
#define _sbc(x) r0 = r1 - r2 - _avr_sreg(avr, S_C)
#define _sbc_flags _sbc; _avr_flags_sub_Rzns(avr, r0, r1, r2);
#define _sub(x) r0 = r1 - r2

#define _if_op(_pc_opcode, _inst_opcode, _mask, _z) \
	if (_inst_opcode == (_pc_inst[_pc_opcode].opcode & _mask)) { \
		_z \
	}

#define _if_op_op_rh_rl(

#define BEGIN_COMBINING_OP_DEFN(_combining_inst_name) \
	static void _combining_inst ## _combining_inst_name(avt_t * avr, _inst_p _pc_inst) \
	{ \
	
#define END_COMBINING_OP_DEFN \
	}

#define _use_opcode(_name, r0, r1, r2)

#define _inst_d5_mask		0xFE0F
#define _inst_d5a6_mask		0xF800

#define _inst_lsr_opcode	0x9406
#define _inst_pop_opcode	0x900F
#define _inst_out_opcode	0xB800
#define _inst_ror_opcode	0x9407

INST(ADD, 0XFC00, 0x0400, _d5r5, _store, _add_flags)
INST(CPC, 0XFC00, 0x0400, _d5r5, _no_store, _sbc_flags)
INST_COMPLEX(ADC, _d5r5, d5, r5, d5 == r5, d5_rol, d5, 0, 0)
INST_COMBINING(ADD, 
