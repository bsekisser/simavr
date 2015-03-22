#define _OP(_op) _op_list[_op]
#define _OP_R(_op, _op_r) _OP(_op)->r[_op_r]

#define _test_op(_test_idx, _test_op, _test_mask) \
	(_inst ## _test_op == (_OP(_test_idx)->opcode & _test_mask))

#define _if_op1_and_op2(_idx1, _op1, _op1_mask, _idx2, _op2, _op2_mask, _z) \
	if (_test_op(_idx1, _op1, _op1_mask) && _test_op(_idx2, _op2, _op2_mask)) { \
		_z \
	}
	
#define _if_op1_and_op2_and_16r1_and_16r2(_idx1, _op1, _op1_mask, _idx2, _op2, _op2_mask, _z) \
	if (_test_op(_idx1, _op1, _op1_mask) && _test_op(_idx2, _op2, _op2_mask)) { \
		_z \
	}

#define _if_op(_pc_opcode, _inst_opcode, _mask, _z) \
	if _test_op(_0, _if_op, _if_op_mask) { \
		_z \
	}


/* combining instruction magic */

BEGIN_COMBINING_OP_DEFN(_d5r5, _adc)
	_if_op1r1_equals_op1r2(_adc, _USE_OPCODE(_rol, _OP1R1, 0, 0))
END_COMBINING_OP_DEFN		

BEGIN_COMBINING_OP_DEFN(_d5r5, _add_adc)
	_if_op_r1_equals_r2
END_COMBINING_OP_DEFN		


BEGIN_COMBINING_OP_DEFN(_d5r5, _add)
	_if_op1_and_op2_and_16r1_and_16r2(_d5r5, _add, _d5r5, _adc), CALL_COMBINING_DECODE_INST(_add_adc)) {
		if(_OP_R(0, 1) == _OP_R(0, 2))
			_USE_OPCODE_OP_R1(d5_lsl_rol);
		else
			_USE_OPCODE_OP_R1_R2(d5r5_add_adc)
	}
END_COMBINING_OP_DEFN


BEGIN_COMBINING_OP_DEFN(_lds)
	_if_op1_and_op2(0, _lds, 1, _lds, CALL_COMBINING_DECODE_INST(_lds_lds))
	_if_op1_and_op2(0, _lds, 1, _tst, CALL_COMBINING_DECODE_INST(_lds_tst)))
END_COMBINING_OP_DEFN		

BEGIN_COMBINING_OP_DEFN(_lsr_ror)
END_COMBINING_OP_DEFN

BEGIN_COMBINING_OP_DEFN(_lsr)
	_if_op1_and_op2(0, _lsr, 1, _ror, CALL_COMBINING_DECODE_INST(_lsr_ror))
END_COMBINING_OP_DEFN

BEGIN_COMBINING_OP_DEFN(_pop)
	_if_op1r1h_and_op2r1l(_d5, _pop, _d5, _pop, _use_opcode(_pop_pop16be, _OP2R1, 0, 0))
	_if_op1r1l_and_op2r1h(_d5, _pop, _d5, _pop, _use_opcode(_pop_pop16le, _OP1R1, 0, 0))
	_if_op1r1_equals_op2r1(_d5, _pop, _d5a6, _out, _use_decode(_d5a6_pop_out, _pc_inst[1].r1, _pc_inst[2].r2, 0))
END_COMBINING_OP_DEFN

