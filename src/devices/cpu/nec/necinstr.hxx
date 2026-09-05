// license:BSD-3-Clause
// copyright-holders:Bryan McPhail
#define OP(num,func_name) void nec_common_device::func_name()

uint8_t nec_common_device::start_rep()
{
	uint8_t next = fetchop();

	switch(next) { /* Segments */
		case 0x26:  m_seg_prefix=true; m_prefix_base=Sreg(DS1)<<4;    next = fetchop();  CLK(2); break;
		case 0x2e:  m_seg_prefix=true; m_prefix_base=Sreg(PS)<<4;     next = fetchop();  CLK(2); break;
		case 0x36:  m_seg_prefix=true; m_prefix_base=Sreg(SS)<<4;     next = fetchop();  CLK(2); break;
		case 0x3e:  m_seg_prefix=true; m_prefix_base=Sreg(DS0)<<4;    next = fetchop();  CLK(2); break;
	}

	return next;
}

void nec_common_device::cont_rep()
{
	m_ip = m_rep_ip;
	m_seg_prefix = bool(BIT(m_rep_params, 2));

	const uint8_t opcode = m_rep_params & 3;
	const uint8_t next = m_rep_params >> 8 & 0xff;
	m_rep_params = 0;

	// continue REP loop where it left off
	switch(opcode) {
		case 0: do_repnc(next); break;
		case 1: do_repc(next); break;
		case 2: do_repne(next); break;
		case 3: do_repe(next); break;
	}
}

void nec_common_device::do_repnc(uint8_t next)
{
	uint16_t c = Wreg(CW);
	switch(next) {
		case 0x6c:  CLK(2); if (c) do { i_insb();  c--; } while (c>0 && !CF && m_icount>0); Wreg(CW)=c; break;
		case 0x6d:  CLK(2); if (c) do { i_insw();  c--; } while (c>0 && !CF && m_icount>0); Wreg(CW)=c; break;
		case 0x6e:  CLK(2); if (c) do { i_outsb(); c--; } while (c>0 && !CF && m_icount>0); Wreg(CW)=c; break;
		case 0x6f:  CLK(2); if (c) do { i_outsw(); c--; } while (c>0 && !CF && m_icount>0); Wreg(CW)=c; break;
		case 0xa4:  CLK(2); if (c) do { i_movsb(); c--; } while (c>0 && !CF && m_icount>0); Wreg(CW)=c; break;
		case 0xa5:  CLK(2); if (c) do { i_movsw(); c--; } while (c>0 && !CF && m_icount>0); Wreg(CW)=c; break;
		case 0xa6:  CLK(2); if (c) do { i_cmpsb(); c--; } while (c>0 && !CF && m_icount>0); Wreg(CW)=c; break;
		case 0xa7:  CLK(2); if (c) do { i_cmpsw(); c--; } while (c>0 && !CF && m_icount>0); Wreg(CW)=c; break;
		case 0xaa:  CLK(2); if (c) do { i_stosb(); c--; } while (c>0 && !CF && m_icount>0); Wreg(CW)=c; break;
		case 0xab:  CLK(2); if (c) do { i_stosw(); c--; } while (c>0 && !CF && m_icount>0); Wreg(CW)=c; break;
		case 0xac:  CLK(2); if (c) do { i_lodsb(); c--; } while (c>0 && !CF && m_icount>0); Wreg(CW)=c; break;
		case 0xad:  CLK(2); if (c) do { i_lodsw(); c--; } while (c>0 && !CF && m_icount>0); Wreg(CW)=c; break;
		case 0xae:  CLK(2); if (c) do { i_scasb(); c--; } while (c>0 && !CF && m_icount>0); Wreg(CW)=c; break;
		case 0xaf:  CLK(2); if (c) do { i_scasw(); c--; } while (c>0 && !CF && m_icount>0); Wreg(CW)=c; break;
		default:    logerror("%06x: REPNC invalid\n",PC()); c = 0; (this->*s_nec_instruction[next])(); break;
	}
	if (c && !CF) {
		m_rep_ip = m_ip;
		m_ip = m_prev_ip;
		m_rep_params = next << 8 | 0 | (m_seg_prefix ? 4 : 0);
	}
	m_seg_prefix=false;
}

void nec_common_device::do_repc(uint8_t next)
{
	uint16_t c = Wreg(CW);
	switch(next) {
		case 0x6c:  CLK(2); if (c) do { i_insb();  c--; } while (c>0 && CF && m_icount>0); Wreg(CW)=c; break;
		case 0x6d:  CLK(2); if (c) do { i_insw();  c--; } while (c>0 && CF && m_icount>0); Wreg(CW)=c; break;
		case 0x6e:  CLK(2); if (c) do { i_outsb(); c--; } while (c>0 && CF && m_icount>0); Wreg(CW)=c; break;
		case 0x6f:  CLK(2); if (c) do { i_outsw(); c--; } while (c>0 && CF && m_icount>0); Wreg(CW)=c; break;
		case 0xa4:  CLK(2); if (c) do { i_movsb(); c--; } while (c>0 && CF && m_icount>0); Wreg(CW)=c; break;
		case 0xa5:  CLK(2); if (c) do { i_movsw(); c--; } while (c>0 && CF && m_icount>0); Wreg(CW)=c; break;
		case 0xa6:  CLK(2); if (c) do { i_cmpsb(); c--; } while (c>0 && CF && m_icount>0); Wreg(CW)=c; break;
		case 0xa7:  CLK(2); if (c) do { i_cmpsw(); c--; } while (c>0 && CF && m_icount>0); Wreg(CW)=c; break;
		case 0xaa:  CLK(2); if (c) do { i_stosb(); c--; } while (c>0 && CF && m_icount>0); Wreg(CW)=c; break;
		case 0xab:  CLK(2); if (c) do { i_stosw(); c--; } while (c>0 && CF && m_icount>0); Wreg(CW)=c; break;
		case 0xac:  CLK(2); if (c) do { i_lodsb(); c--; } while (c>0 && CF && m_icount>0); Wreg(CW)=c; break;
		case 0xad:  CLK(2); if (c) do { i_lodsw(); c--; } while (c>0 && CF && m_icount>0); Wreg(CW)=c; break;
		case 0xae:  CLK(2); if (c) do { i_scasb(); c--; } while (c>0 && CF && m_icount>0); Wreg(CW)=c; break;
		case 0xaf:  CLK(2); if (c) do { i_scasw(); c--; } while (c>0 && CF && m_icount>0); Wreg(CW)=c; break;
		default:    logerror("%06x: REPC invalid\n",PC()); c = 0; (this->*s_nec_instruction[next])(); break;
	}
	if (c && CF) {
		m_rep_ip = m_ip;
		m_ip = m_prev_ip;
		m_rep_params = next << 8 | 1 | (m_seg_prefix ? 4 : 0);
	}
	m_seg_prefix=false;
}

void nec_common_device::do_repne(uint8_t next)
{
	uint16_t c = Wreg(CW);
	switch(next) {
		case 0x6c:  CLK(2); if (c) do { i_insb();  c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0x6d:  CLK(2); if (c) do { i_insw();  c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0x6e:  CLK(2); if (c) do { i_outsb(); c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0x6f:  CLK(2); if (c) do { i_outsw(); c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0xa4:  CLK(2); if (c) do { i_movsb(); c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0xa5:  CLK(2); if (c) do { i_movsw(); c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0xa6:  CLK(2); if (c) do { i_cmpsb(); c--; } while (c>0 && !ZF && m_icount>0);    Wreg(CW)=c; break;
		case 0xa7:  CLK(2); if (c) do { i_cmpsw(); c--; } while (c>0 && !ZF && m_icount>0);    Wreg(CW)=c; break;
		case 0xaa:  CLK(2); if (c) do { i_stosb(); c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0xab:  CLK(2); if (c) do { i_stosw(); c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0xac:  CLK(2); if (c) do { i_lodsb(); c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0xad:  CLK(2); if (c) do { i_lodsw(); c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0xae:  CLK(2); if (c) do { i_scasb(); c--; } while (c>0 && !ZF && m_icount>0);    Wreg(CW)=c; break;
		case 0xaf:  CLK(2); if (c) do { i_scasw(); c--; } while (c>0 && !ZF && m_icount>0);    Wreg(CW)=c; break;
		default:    logerror("%06x: REPNE invalid\n",PC()); c = 0; (this->*s_nec_instruction[next])(); break;
	}
	if (c && !(ZF && ((next & 0x86) == 0x86))) {
		m_rep_ip = m_ip;
		m_ip = m_prev_ip;
		m_rep_params = next << 8 | 2 | (m_seg_prefix ? 4 : 0);
	}
	m_seg_prefix=false;
}

void nec_common_device::do_repe(uint8_t next)
{
	uint16_t c = Wreg(CW);
	switch(next) {
		case 0x6c:  CLK(2); if (c) do { i_insb();  c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0x6d:  CLK(2); if (c) do { i_insw();  c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0x6e:  CLK(2); if (c) do { i_outsb(); c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0x6f:  CLK(2); if (c) do { i_outsw(); c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0xa4:  CLK(2); if (c) do { i_movsb(); c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0xa5:  CLK(2); if (c) do { i_movsw(); c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0xa6:  CLK(2); if (c) do { i_cmpsb(); c--; } while (c>0 && ZF && m_icount>0);    Wreg(CW)=c; break;
		case 0xa7:  CLK(2); if (c) do { i_cmpsw(); c--; } while (c>0 && ZF && m_icount>0);    Wreg(CW)=c; break;
		case 0xaa:  CLK(2); if (c) do { i_stosb(); c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0xab:  CLK(2); if (c) do { i_stosw(); c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0xac:  CLK(2); if (c) do { i_lodsb(); c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0xad:  CLK(2); if (c) do { i_lodsw(); c--; } while (c>0 && m_icount>0); Wreg(CW)=c; break;
		case 0xae:  CLK(2); if (c) do { i_scasb(); c--; } while (c>0 && ZF && m_icount>0);    Wreg(CW)=c; break;
		case 0xaf:  CLK(2); if (c) do { i_scasw(); c--; } while (c>0 && ZF && m_icount>0);    Wreg(CW)=c; break;
		default:    logerror("%06x: REPE invalid\n",PC()); c = 0; (this->*s_nec_instruction[next])(); break;
	}
	if (c && !(!ZF && ((next & 0x86) == 0x86))) {
		m_rep_ip = m_ip;
		m_ip = m_prev_ip;
		m_rep_params = next << 8 | 3 | (m_seg_prefix ? 4 : 0);
	}
	m_seg_prefix=false;
}

OP( 0x00, i_add_br8  ) { DEF_br8;   ADDB;   PutbackRMByte(ModRM,dst);   CLKM(2,2,2,16,16,7);        }
OP( 0x01, i_add_wr16 ) { DEF_wr16;  ADDW;   PutbackRMWord(ModRM,dst);   CLKR(24,24,11,24,16,7,2,m_EA);}
OP( 0x02, i_add_r8b  ) { DEF_r8b;   ADDB;   RegByte(ModRM)=dst;         CLKM(2,2,2,11,11,6);        }
OP( 0x03, i_add_r16w ) { DEF_r16w;  ADDW;   RegWord(ModRM)=dst;         CLKR(15,15,8,15,11,6,2,m_EA); }
OP( 0x04, i_add_ald8 ) { DEF_ald8;  ADDB;   Breg(AL)=dst;           CLKS(4,4,2);                }
OP( 0x05, i_add_axd16) { DEF_axd16; ADDW;   Wreg(AW)=dst;           CLKS(4,4,2);                }
OP( 0x06, i_push_es  ) { PUSH(Sreg(DS1));   CLKS(12,8,3);   }
OP( 0x07, i_pop_es   ) { POP(Sreg(DS1));    CLKS(12,8,5);   }

OP( 0x08, i_or_br8   ) { DEF_br8;   ORB;    PutbackRMByte(ModRM,dst);   CLKM(2,2,2,16,16,7);        }
OP( 0x09, i_or_wr16  ) { DEF_wr16;  ORW;    PutbackRMWord(ModRM,dst);   CLKR(24,24,11,24,16,7,2,m_EA);}
OP( 0x0a, i_or_r8b   ) { DEF_r8b;   ORB;    RegByte(ModRM)=dst;         CLKM(2,2,2,11,11,6);        }
OP( 0x0b, i_or_r16w  ) { DEF_r16w;  ORW;    RegWord(ModRM)=dst;         CLKR(15,15,8,15,11,6,2,m_EA); }
OP( 0x0c, i_or_ald8  ) { DEF_ald8;  ORB;    Breg(AL)=dst;           CLKS(4,4,2);                }
OP( 0x0d, i_or_axd16 ) { DEF_axd16; ORW;    Wreg(AW)=dst;           CLKS(4,4,2);                }
OP( 0x0e, i_push_cs  ) { PUSH(Sreg(PS));    CLKS(12,8,3);   }
OP( 0x0f, i_pre_nec  ) { uint32_t ModRM, tmp, tmp2;
	switch (fetch()) {
		case 0x10 : BITOP_BYTE; CLKS(3,3,4); tmp2 = Breg(CL) & 0x7; m_ZeroVal = (tmp & (1<<tmp2)) ? 1 : 0; m_CarryVal=m_OverVal=0; break; /* Test */
		case 0x11 : BITOP_WORD; CLKS(3,3,4); tmp2 = Breg(CL) & 0xf; m_ZeroVal = (tmp & (1<<tmp2)) ? 1 : 0; m_CarryVal=m_OverVal=0; break; /* Test */
		case 0x12 : BITOP_BYTE; CLKS(5,5,4); tmp2 = Breg(CL) & 0x7; tmp &= ~(1<<tmp2);  PutbackRMByte(ModRM,tmp);   break; /* Clr */
		case 0x13 : BITOP_WORD; CLKS(5,5,4); tmp2 = Breg(CL) & 0xf; tmp &= ~(1<<tmp2);  PutbackRMWord(ModRM,tmp);   break; /* Clr */
		case 0x14 : BITOP_BYTE; CLKS(4,4,4); tmp2 = Breg(CL) & 0x7; tmp |= (1<<tmp2);   PutbackRMByte(ModRM,tmp);   break; /* Set */
		case 0x15 : BITOP_WORD; CLKS(4,4,4); tmp2 = Breg(CL) & 0xf; tmp |= (1<<tmp2);   PutbackRMWord(ModRM,tmp);   break; /* Set */
		case 0x16 : BITOP_BYTE; CLKS(4,4,4); tmp2 = Breg(CL) & 0x7; BIT_NOT;            PutbackRMByte(ModRM,tmp);   break; /* Not */
		case 0x17 : BITOP_WORD; CLKS(4,4,4); tmp2 = Breg(CL) & 0xf; BIT_NOT;            PutbackRMWord(ModRM,tmp);   break; /* Not */

		case 0x18 : BITOP_BYTE; CLKS(4,4,4); tmp2 = (fetch()) & 0x7;    m_ZeroVal = (tmp & (1<<tmp2)) ? 1 : 0; m_CarryVal=m_OverVal=0; break; /* Test */
		case 0x19 : BITOP_WORD; CLKS(4,4,4); tmp2 = (fetch()) & 0xf;    m_ZeroVal = (tmp & (1<<tmp2)) ? 1 : 0; m_CarryVal=m_OverVal=0; break; /* Test */
		case 0x1a : BITOP_BYTE; CLKS(6,6,4); tmp2 = (fetch()) & 0x7;    tmp &= ~(1<<tmp2);      PutbackRMByte(ModRM,tmp);   break; /* Clr */
		case 0x1b : BITOP_WORD; CLKS(6,6,4); tmp2 = (fetch()) & 0xf;    tmp &= ~(1<<tmp2);      PutbackRMWord(ModRM,tmp);   break; /* Clr */
		case 0x1c : BITOP_BYTE; CLKS(5,5,4); tmp2 = (fetch()) & 0x7;    tmp |= (1<<tmp2);       PutbackRMByte(ModRM,tmp);   break; /* Set */
		case 0x1d : BITOP_WORD; CLKS(5,5,4); tmp2 = (fetch()) & 0xf;    tmp |= (1<<tmp2);       PutbackRMWord(ModRM,tmp);   break; /* Set */
		case 0x1e : BITOP_BYTE; CLKS(5,5,4); tmp2 = (fetch()) & 0x7;    BIT_NOT;                PutbackRMByte(ModRM,tmp);   break; /* Not */
		case 0x1f : BITOP_WORD; CLKS(5,5,4); tmp2 = (fetch()) & 0xf;    BIT_NOT;                PutbackRMWord(ModRM,tmp);   break; /* Not */

		case 0x20 : ADD4S; CLKS(7,7,2); break;
		case 0x22 : SUB4S; CLKS(7,7,2); break;
		case 0x26 : CMP4S; CLKS(7,7,2); break;
		case 0x28 : ModRM = fetch(); tmp = GetRMByte(ModRM); tmp <<= 4; tmp |= Breg(AL) & 0xf; Breg(AL) = (Breg(AL) & 0xf0) | ((tmp>>8)&0xf); tmp &= 0xff; PutbackRMByte(ModRM,tmp); CLKM(13,13,9,28,28,15); break;
		case 0x2a : ModRM = fetch(); tmp = GetRMByte(ModRM); tmp2 = (Breg(AL) & 0xf)<<4; Breg(AL) = (Breg(AL) & 0xf0) | (tmp&0xf); tmp = tmp2 | (tmp>>4);   PutbackRMByte(ModRM,tmp); CLKM(17,17,13,32,32,19); break;
		case 0x31 : // INS reg1,reg2
			ModRM = fetch();
			tmp = GetRMByte(0xc0 | (ModRM & 7)) & 0x0f;
			tmp2 = (GetRMByte(0xc0 | ((ModRM >> 3) & 7)) & 0x0f) + 1;
			PutMemW(DS1, Wreg(IY), (GetMemW(DS1, Wreg(IY)) & ~(((1 << tmp2) - 1) << tmp)) | ((Wreg(AW) & ((1 << tmp2) - 1)) << tmp));
			if (tmp + tmp2 > 15) {
				Wreg(IY) += 2;
				PutMemW(DS1, Wreg(IY), (GetMemW(DS1, Wreg(IY)) & ~((1 << (tmp2 - (16 - tmp))) - 1)) | ((Wreg(AW) >> (16 - tmp)) & ((1 << (tmp2 - (16 - tmp))) - 1)));
			}
			PutRMByte(ModRM, (tmp + tmp2) & 0x0f);
			// When and how many extra cycles are taken is not documented:
			// V20: 35-113 cycles
			// V30: 35-133 cycles, odd addresses
			// V30: 31-117 cycles, even addresses
			// V33: 39-77 cycles, odd addresses
			// V33: 37-69 cycles, even addresses
			if (Wreg(IY) & 1) {
				CLKS(113, 113, 77);
			} else {
				CLKS(113, 117, 69);
			}
			break;
		case 0x33 : // EXT reg1,reg2
			ModRM = fetch();
			tmp = GetRMByte(0xc0 | (ModRM & 7)) & 0x0f;
			tmp2 = (GetRMByte(0xc0 | ((ModRM >> 3) & 7)) & 0x0f) + 1;
			Wreg(AW) = GetMemW(DS0, Wreg(IX)) >> tmp;
			if (tmp + tmp2 > 15) {
				Wreg(IX) += 2;
				Wreg(AW) |= GetMemW(DS0, Wreg(IX)) << (16 - tmp);
			}
			Wreg(AW) &= ((1 << tmp2) - 1);
			PutRMByte(ModRM, (tmp + tmp2) & 0x0f);
			// When and how many extra cycles are taken is not documented:
			// V20: 34-59 cycles
			// V30: 34-59 cycles, odd addresses
			// V30: 26-55 cycles, even addresses
			// V33: 33-63 cycles, odd addresses
			// V33: 29-61 cycles, even addresses
			if (Wreg(IX) & 1) {
				CLKS(59, 59, 63);
			} else {
				CLKS(59, 55, 62);
			}
			break;
		case 0x39 : // INS reg,imm4
			ModRM = fetch();
			tmp = GetRMByte(ModRM) & 0x0f;
			tmp2 = (fetch() & 0x0f) + 1;
			PutMemW(DS1, Wreg(IY), (GetMemW(DS1, Wreg(IY)) & ~(((1 << tmp2) - 1) << tmp)) | ((Wreg(AW) & ((1 << tmp2) - 1)) << tmp));
			if (tmp + tmp2 > 15) {
				Wreg(IY) += 2;
				PutMemW(DS1, Wreg(IY), (GetMemW(DS1, Wreg(IY)) & ~((1 << (tmp2 - (16 - tmp))) - 1)) | ((Wreg(AW) >> (16 - tmp)) & ((1 << (tmp2 - (16 - tmp))) - 1)));
			}
			PutRMByte(ModRM, (tmp + tmp2) & 0x0f);
			// When and how many extra cycles are taken is not documented:
			// V20: 75-103 cycles
			// V30: 75-103 cycles, odd addresses
			// V30: 67-87 cycles, even addresses
			// V33: 39-77 cycles, odd addresses
			// V33: 37-69 cycles, even addresses
			if (Wreg(IY) & 1) {
				CLKS(103, 103, 77);
			} else {
				CLKS(103, 87, 69);
			}
			break;
		case 0x3b : // EXT reg,imm4
			ModRM = fetch();
			tmp = GetRMByte(ModRM) & 0x0f;
			tmp2 = (fetch() & 0x0f) + 1;
			Wreg(AW) = GetMemW(DS0, Wreg(IX)) >> tmp;
			if (tmp + tmp2 > 15) {
				Wreg(IX) += 2;
				Wreg(AW) |= GetMemW(DS0, Wreg(IX)) << (16 - tmp);
			}
			Wreg(AW) &= ((1 << tmp2) - 1);
			PutRMByte(ModRM, (tmp + tmp2) & 0x0f);
			// When and how many extra cycles are taken is not documented:
			// V20: 25-52 cycles
			// V30: 25-52 cycles, odd addresses
			// V30: 21-44 cycles, even addresses
			// V33: 33-63 cycles, odd addresses
			// V33: 29-61 cycles, even addresses
			if (Wreg(IX) & 1) {
				CLKS(52, 52, 63);
			} else {
				CLKS(52, 44, 62);
			}
			break;
		case 0xe0 : BRKXA(true); CLK(12); break;
		case 0xf0 : BRKXA(false); CLK(12); break;
		case 0xff : BRKEM; CLK(50); break;
		default:    logerror("%06x: Unknown V20 instruction\n",PC()); break;
	}
}

OP( 0x10, i_adc_br8  ) { DEF_br8;   ADDCB;   PutbackRMByte(ModRM,dst);   CLKM(2,2,2,16,16,7);        }
OP( 0x11, i_adc_wr16 ) { DEF_wr16;  ADDCW;   PutbackRMWord(ModRM,dst);   CLKR(24,24,11,24,16,7,2,m_EA);}
OP( 0x12, i_adc_r8b  ) { DEF_r8b;   ADDCB;   RegByte(ModRM)=dst;         CLKM(2,2,2,11,11,6);        }
OP( 0x13, i_adc_r16w ) { DEF_r16w;  ADDCW;   RegWord(ModRM)=dst;         CLKR(15,15,8,15,11,6,2,m_EA); }
OP( 0x14, i_adc_ald8 ) { DEF_ald8;  ADDCB;   Breg(AL)=dst;           CLKS(4,4,2);                }
OP( 0x15, i_adc_axd16) { DEF_axd16; ADDCW;   Wreg(AW)=dst;           CLKS(4,4,2);                }
OP( 0x16, i_push_ss  ) { PUSH(Sreg(SS));        CLKS(12,8,3);   }
OP( 0x17, i_pop_ss   ) { POP(Sreg(SS));     CLKS(12,8,5);   m_no_interrupt=1; }

OP( 0x18, i_sbb_br8  ) { DEF_br8;   SUBCB;   PutbackRMByte(ModRM,dst);   CLKM(2,2,2,16,16,7);        }
OP( 0x19, i_sbb_wr16 ) { DEF_wr16;  SUBCW;   PutbackRMWord(ModRM,dst);   CLKR(24,24,11,24,16,7,2,m_EA);}
OP( 0x1a, i_sbb_r8b  ) { DEF_r8b;   SUBCB;   RegByte(ModRM)=dst;         CLKM(2,2,2,11,11,6);        }
OP( 0x1b, i_sbb_r16w ) { DEF_r16w;  SUBCW;   RegWord(ModRM)=dst;         CLKR(15,15,8,15,11,6,2,m_EA); }
OP( 0x1c, i_sbb_ald8 ) { DEF_ald8;  SUBCB;   Breg(AL)=dst;           CLKS(4,4,2);                }
OP( 0x1d, i_sbb_axd16) { DEF_axd16; SUBCW;   Wreg(AW)=dst;           CLKS(4,4,2);    }
OP( 0x1e, i_push_ds  ) { PUSH(Sreg(DS0));       CLKS(12,8,3);   }
OP( 0x1f, i_pop_ds   ) { POP(Sreg(DS0));        CLKS(12,8,5);   }

OP( 0x20, i_and_br8  ) { DEF_br8;   ANDB;   PutbackRMByte(ModRM,dst);   CLKM(2,2,2,16,16,7);        }
OP( 0x21, i_and_wr16 ) { DEF_wr16;  ANDW;   PutbackRMWord(ModRM,dst);   CLKR(24,24,11,24,16,7,2,m_EA);}
OP( 0x22, i_and_r8b  ) { DEF_r8b;   ANDB;   RegByte(ModRM)=dst;         CLKM(2,2,2,11,11,6);        }
OP( 0x23, i_and_r16w ) { DEF_r16w;  ANDW;   RegWord(ModRM)=dst;         CLKR(15,15,8,15,11,6,2,m_EA); }
OP( 0x24, i_and_ald8 ) { DEF_ald8;  ANDB;   Breg(AL)=dst;           CLKS(4,4,2);                }
OP( 0x25, i_and_axd16) { DEF_axd16; ANDW;   Wreg(AW)=dst;           CLKS(4,4,2);    }
OP( 0x26, i_es       ) { m_seg_prefix=true;    m_prefix_base=Sreg(DS1)<<4;    CLK(2);     (this->*s_nec_instruction[fetchop()])(); m_seg_prefix=false; }
OP( 0x27, i_daa      ) { ADJ4(6,0x60);                                  CLKS(3,3,2);    }

OP( 0x28, i_sub_br8  ) { DEF_br8;   SUBB;   PutbackRMByte(ModRM,dst);   CLKM(2,2,2,16,16,7);        }
OP( 0x29, i_sub_wr16 ) { DEF_wr16;  SUBW;   PutbackRMWord(ModRM,dst);   CLKR(24,24,11,24,16,7,2,m_EA);}
OP( 0x2a, i_sub_r8b  ) { DEF_r8b;   SUBB;   RegByte(ModRM)=dst;         CLKM(2,2,2,11,11,6);        }
OP( 0x2b, i_sub_r16w ) { DEF_r16w;  SUBW;   RegWord(ModRM)=dst;         CLKR(15,15,8,15,11,6,2,m_EA); }
OP( 0x2c, i_sub_ald8 ) { DEF_ald8;  SUBB;   Breg(AL)=dst;           CLKS(4,4,2);                }
OP( 0x2d, i_sub_axd16) { DEF_axd16; SUBW;   Wreg(AW)=dst;           CLKS(4,4,2);    }
OP( 0x2e, i_cs       ) { m_seg_prefix=true;    m_prefix_base=Sreg(PS)<<4; CLK(2);     (this->*s_nec_instruction[fetchop()])(); m_seg_prefix=false; }
OP( 0x2f, i_das      ) { ADJ4(-6,-0x60);                                CLKS(3,3,2);    }

OP( 0x30, i_xor_br8  ) { DEF_br8;   XORB;   PutbackRMByte(ModRM,dst);   CLKM(2,2,2,16,16,7);        }
OP( 0x31, i_xor_wr16 ) { DEF_wr16;  XORW;   PutbackRMWord(ModRM,dst);   CLKR(24,24,11,24,16,7,2,m_EA);}
OP( 0x32, i_xor_r8b  ) { DEF_r8b;   XORB;   RegByte(ModRM)=dst;         CLKM(2,2,2,11,11,6);        }
OP( 0x33, i_xor_r16w ) { DEF_r16w;  XORW;   RegWord(ModRM)=dst;         CLKR(15,15,8,15,11,6,2,m_EA); }
OP( 0x34, i_xor_ald8 ) { DEF_ald8;  XORB;   Breg(AL)=dst;           CLKS(4,4,2);                }
OP( 0x35, i_xor_axd16) { DEF_axd16; XORW;   Wreg(AW)=dst;           CLKS(4,4,2);    }
OP( 0x36, i_ss       ) { m_seg_prefix=true;    m_prefix_base=Sreg(SS)<<4; CLK(2);     (this->*s_nec_instruction[fetchop()])(); m_seg_prefix=false; }
OP( 0x37, i_aaa      ) { ADJB(6, (Breg(AL) > 0xf9) ? 2 : 1);        CLKS(7,7,4);    }

OP( 0x38, i_cmp_br8  ) { DEF_br8;   SUBB;                   CLKM(2,2,2,11,11,6); }
OP( 0x39, i_cmp_wr16 ) { DEF_wr16;  SUBW;                   CLKR(15,15,8,15,11,6,2,m_EA);}
OP( 0x3a, i_cmp_r8b  ) { DEF_r8b;   SUBB;                   CLKM(2,2,2,11,11,6); }
OP( 0x3b, i_cmp_r16w ) { DEF_r16w;  SUBW;                   CLKR(15,15,8,15,11,6,2,m_EA); }
OP( 0x3c, i_cmp_ald8 ) { DEF_ald8;  SUBB;                   CLKS(4,4,2); }
OP( 0x3d, i_cmp_axd16) { DEF_axd16; SUBW;                   CLKS(4,4,2);    }
OP( 0x3e, i_ds       ) { m_seg_prefix=true;    m_prefix_base=Sreg(DS0)<<4;    CLK(2);     (this->*s_nec_instruction[fetchop()])(); m_seg_prefix=false; }
OP( 0x3f, i_aas      ) { ADJB(-6, (Breg(AL) < 6) ? -2 : -1);        CLKS(7,7,4);    }

OP( 0x40, i_inc_ax  ) { IncWordReg(AW);                     CLK(2); }
OP( 0x41, i_inc_cx  ) { IncWordReg(CW);                     CLK(2); }
OP( 0x42, i_inc_dx  ) { IncWordReg(DW);                     CLK(2); }
OP( 0x43, i_inc_bx  ) { IncWordReg(BW);                     CLK(2); }
OP( 0x44, i_inc_sp  ) { IncWordReg(SP);                     CLK(2); }
OP( 0x45, i_inc_bp  ) { IncWordReg(BP);                     CLK(2); }
OP( 0x46, i_inc_si  ) { IncWordReg(IX);                     CLK(2); }
OP( 0x47, i_inc_di  ) { IncWordReg(IY);                     CLK(2); }

OP( 0x48, i_dec_ax  ) { DecWordReg(AW);                     CLK(2); }
OP( 0x49, i_dec_cx  ) { DecWordReg(CW);                     CLK(2); }
OP( 0x4a, i_dec_dx  ) { DecWordReg(DW);                     CLK(2); }
OP( 0x4b, i_dec_bx  ) { DecWordReg(BW);                     CLK(2); }
OP( 0x4c, i_dec_sp  ) { DecWordReg(SP);                     CLK(2); }
OP( 0x4d, i_dec_bp  ) { DecWordReg(BP);                     CLK(2); }
OP( 0x4e, i_dec_si  ) { DecWordReg(IX);                     CLK(2); }
OP( 0x4f, i_dec_di  ) { DecWordReg(IY);                     CLK(2); }

OP( 0x50, i_push_ax ) { PUSH(Wreg(AW));                 CLKS(12,8,3); }
OP( 0x51, i_push_cx ) { PUSH(Wreg(CW));                 CLKS(12,8,3); }
OP( 0x52, i_push_dx ) { PUSH(Wreg(DW));                 CLKS(12,8,3); }
OP( 0x53, i_push_bx ) { PUSH(Wreg(BW));                 CLKS(12,8,3); }
OP( 0x54, i_push_sp ) { PUSH(Wreg(SP));                 CLKS(12,8,3); }
OP( 0x55, i_push_bp ) { PUSH(Wreg(BP));                 CLKS(12,8,3); }
OP( 0x56, i_push_si ) { PUSH(Wreg(IX));                 CLKS(12,8,3); }
OP( 0x57, i_push_di ) { PUSH(Wreg(IY));                 CLKS(12,8,3); }

OP( 0x58, i_pop_ax  ) { POP(Wreg(AW));                  CLKS(12,8,5); }
OP( 0x59, i_pop_cx  ) { POP(Wreg(CW));                  CLKS(12,8,5); }
OP( 0x5a, i_pop_dx  ) { POP(Wreg(DW));                  CLKS(12,8,5); }
OP( 0x5b, i_pop_bx  ) { POP(Wreg(BW));                  CLKS(12,8,5); }
OP( 0x5c, i_pop_sp  ) { POP(Wreg(SP));                  CLKS(12,8,5); }
OP( 0x5d, i_pop_bp  ) { POP(Wreg(BP));                  CLKS(12,8,5); }
OP( 0x5e, i_pop_si  ) { POP(Wreg(IX));                  CLKS(12,8,5); }
OP( 0x5f, i_pop_di  ) { POP(Wreg(IY));                  CLKS(12,8,5); }

OP( 0x60, i_pusha  ) {
	unsigned tmp=Wreg(SP);
	PUSH(Wreg(AW));
	PUSH(Wreg(CW));
	PUSH(Wreg(DW));
	PUSH(Wreg(BW));
	PUSH(tmp);
	PUSH(Wreg(BP));
	PUSH(Wreg(IX));
	PUSH(Wreg(IY));
	CLKS(67,35,20);
}
static unsigned nec_popa_tmp;
OP( 0x61, i_popa  ) {
	POP(Wreg(IY));
	POP(Wreg(IX));
	POP(Wreg(BP));
	POP(nec_popa_tmp);
	POP(Wreg(BW));
	POP(Wreg(DW));
	POP(Wreg(CW));
	POP(Wreg(AW));
	CLKS(75,43,22);
}
OP( 0x62, i_chkind  ) {
	uint32_t low,high,tmp;
	GetModRM;
	low = GetRMWord(ModRM);
	high= GetnextRMWord;
	tmp= RegWord(ModRM);
	if (tmp<low || tmp>high) {
		nec_interrupt(NEC_CHKIND_VECTOR, BRK);
	}
	CLK(20);
	logerror("%06x: bound %04x high %04x low %04x tmp\n",PC(),high,low,tmp);
}
OP( 0x64, i_repnc    ) { do_repnc(start_rep()); }
OP( 0x65, i_repc     ) { do_repc(start_rep()); }
OP( 0x68, i_push_d16 ) { uint32_t tmp; tmp = fetchword(); PUSH(tmp); CLKW(12,12,5,12,8,5,Wreg(SP));  }
OP( 0x69, i_imul_d16 ) { uint32_t tmp; DEF_r16w; tmp = fetchword(); dst = (int32_t)((int16_t)src)*(int32_t)((int16_t)tmp); m_CarryVal = m_OverVal = (((int32_t)dst) >> 15 != 0) && (((int32_t)dst) >> 15 != -1); RegWord(ModRM)=(WORD)dst; CLKM(42,42,12,52,48,16); }
OP( 0x6a, i_push_d8  ) { uint32_t tmp = (WORD)((int16_t)((int8_t)fetch())); PUSH(tmp); CLKW(11,11,5,11,7,3,Wreg(SP));  }
OP( 0x6b, i_imul_d8  ) { uint32_t src2; DEF_r16w; src2= (WORD)((int16_t)((int8_t)fetch())); dst = (int32_t)((int16_t)src)*(int32_t)((int16_t)src2); m_CarryVal = m_OverVal = (((int32_t)dst) >> 15 != 0) && (((int32_t)dst) >> 15 != -1); RegWord(ModRM)=(WORD)dst; CLKM(34,34,12,44,40,16); }
OP( 0x6c, i_insb     ) { PutMemB(DS1,Wreg(IY),read_port_byte(Wreg(DW))); Wreg(IY)+= -2 * m_DF + 1; CLK(8); }
OP( 0x6d, i_insw     ) { PutMemW(DS1,Wreg(IY),read_port_word(Wreg(DW))); Wreg(IY)+= -4 * m_DF + 2; CLKS(18,10,8); }
OP( 0x6e, i_outsb    ) { write_port_byte(Wreg(DW),GetMemB(DS0,Wreg(IX))); Wreg(IX)+= -2 * m_DF + 1; CLK(8); }
OP( 0x6f, i_outsw    ) { write_port_word(Wreg(DW),GetMemW(DS0,Wreg(IX))); Wreg(IX)+= -4 * m_DF + 2; CLKS(18,10,8); }

OP( 0x70, i_jo      ) { JMP( OF);               CLKS(4,4,3); }
OP( 0x71, i_jno     ) { JMP(!OF);               CLKS(4,4,3); }
OP( 0x72, i_jc      ) { JMP( CF);               CLKS(4,4,3); }
OP( 0x73, i_jnc     ) { JMP(!CF);               CLKS(4,4,3); }
OP( 0x74, i_jz      ) { JMP( ZF);               CLKS(4,4,3); }
OP( 0x75, i_jnz     ) { JMP(!ZF);               CLKS(4,4,3); }
OP( 0x76, i_jce     ) { JMP(CF || ZF);          CLKS(4,4,3); }
OP( 0x77, i_jnce    ) { JMP(!(CF || ZF));       CLKS(4,4,3); }
OP( 0x78, i_js      ) { JMP( SF);               CLKS(4,4,3); }
OP( 0x79, i_jns     ) { JMP(!SF);               CLKS(4,4,3); }
OP( 0x7a, i_jp      ) { JMP( PF);               CLKS(4,4,3); }
OP( 0x7b, i_jnp     ) { JMP(!PF);               CLKS(4,4,3); }
OP( 0x7c, i_jl      ) { JMP((SF!=OF)&&(!ZF));   CLKS(4,4,3); }
OP( 0x7d, i_jnl     ) { JMP((ZF)||(SF==OF));    CLKS(4,4,3); }
OP( 0x7e, i_jle     ) { JMP((ZF)||(SF!=OF));    CLKS(4,4,3); }
OP( 0x7f, i_jnle    ) { JMP((SF==OF)&&(!ZF));   CLKS(4,4,3); }

OP( 0x80, i_80pre   ) { uint32_t dst, src; GetModRM; dst = GetRMByte(ModRM); src = fetch();
	if (ModRM >=0xc0 ) CLKS(4,4,2); else if ((ModRM & 0x38)==0x38) CLKS(13,13,6); else CLKS(18,18,7);
	switch (ModRM & 0x38) {
		case 0x00: ADDB;            PutbackRMByte(ModRM,dst);   break;
		case 0x08: ORB;             PutbackRMByte(ModRM,dst);   break;
		case 0x10: ADDCB;           PutbackRMByte(ModRM,dst);   break;
		case 0x18: SUBCB;           PutbackRMByte(ModRM,dst);   break;
		case 0x20: ANDB;            PutbackRMByte(ModRM,dst);   break;
		case 0x28: SUBB;            PutbackRMByte(ModRM,dst);   break;
		case 0x30: XORB;            PutbackRMByte(ModRM,dst);   break;
		case 0x38: SUBB;            break;  /* CMP */
	}
}

OP( 0x81, i_81pre   ) { uint32_t dst, src; GetModRM; dst = GetRMWord(ModRM); src = fetch(); src+= (fetch() << 8);
	if (ModRM >=0xc0 ) CLKS(4,4,2); else if ((ModRM & 0x38)==0x38) CLKW(17,17,8,17,13,6,m_EA); else CLKW(26,26,11,26,18,7,m_EA);
	switch (ModRM & 0x38) {
		case 0x00: ADDW;            PutbackRMWord(ModRM,dst);   break;
		case 0x08: ORW;             PutbackRMWord(ModRM,dst);   break;
		case 0x10: ADDCW;           PutbackRMWord(ModRM,dst);   break;
		case 0x18: SUBCW;           PutbackRMWord(ModRM,dst);   break;
		case 0x20: ANDW;            PutbackRMWord(ModRM,dst);   break;
		case 0x28: SUBW;            PutbackRMWord(ModRM,dst);   break;
		case 0x30: XORW;            PutbackRMWord(ModRM,dst);   break;
		case 0x38: SUBW;            break;  /* CMP */
	}
}

OP( 0x82, i_82pre   ) { uint32_t dst, src; GetModRM; dst = GetRMByte(ModRM); src = (BYTE)((int8_t)fetch());
	if (ModRM >=0xc0 ) CLKS(4,4,2); else if ((ModRM & 0x38)==0x38) CLKS(13,13,6); else CLKS(18,18,7);
	switch (ModRM & 0x38) {
		case 0x00: ADDB;            PutbackRMByte(ModRM,dst);   break;
		case 0x08: ORB;             PutbackRMByte(ModRM,dst);   break;
		case 0x10: ADDCB;           PutbackRMByte(ModRM,dst);   break;
		case 0x18: SUBCB;           PutbackRMByte(ModRM,dst);   break;
		case 0x20: ANDB;            PutbackRMByte(ModRM,dst);   break;
		case 0x28: SUBB;            PutbackRMByte(ModRM,dst);   break;
		case 0x30: XORB;            PutbackRMByte(ModRM,dst);   break;
		case 0x38: SUBB;            break;  /* CMP */
	}
}

OP( 0x83, i_83pre   ) { uint32_t dst, src; GetModRM; dst = GetRMWord(ModRM); src = (WORD)((int16_t)((int8_t)fetch()));
	if (ModRM >=0xc0 ) CLKS(4,4,2); else if ((ModRM & 0x38)==0x38) CLKW(17,17,8,17,13,6,m_EA); else CLKW(26,26,11,26,18,7,m_EA);
	switch (ModRM & 0x38) {
		case 0x00: ADDW;            PutbackRMWord(ModRM,dst);   break;
		case 0x08: ORW;             PutbackRMWord(ModRM,dst);   break;
		case 0x10: ADDCW;           PutbackRMWord(ModRM,dst);   break;
		case 0x18: SUBCW;           PutbackRMWord(ModRM,dst);   break;
		case 0x20: ANDW;            PutbackRMWord(ModRM,dst);   break;
		case 0x28: SUBW;            PutbackRMWord(ModRM,dst);   break;
		case 0x30: XORW;            PutbackRMWord(ModRM,dst);   break;
		case 0x38: SUBW;            break;  /* CMP */
	}
}

OP( 0x84, i_test_br8  ) { DEF_br8;  ANDB;   CLKM(2,2,2,10,10,6);        }
OP( 0x85, i_test_wr16 ) { DEF_wr16; ANDW;   CLKR(14,14,8,14,10,6,2,m_EA); }
// Byte XCH mem,reg's V30 mem-branch value (18) didn't match either the
// manual's flat 16 (byte-operand XCH mem forms have no V20/V30 split,
// same as everywhere else byte memory forms were checked this
// investigation) or its own word-form sibling at 0x87 just below, which
// already correctly uses 16 for V30's even-address case. Small, isolated
// 2-cycle overcount fixed to match.
OP( 0x86, i_xchg_br8  ) { DEF_br8;  RegByte(ModRM)=dst; PutbackRMByte(ModRM,src); CLKM(3,3,3,16,16,8); }
OP( 0x87, i_xchg_wr16 ) { DEF_wr16; RegWord(ModRM)=dst; PutbackRMWord(ModRM,src); CLKR(24,24,12,24,16,8,3,m_EA); }

OP( 0x88, i_mov_br8   ) { uint8_t  src; GetModRM; src = RegByte(ModRM);   PutRMByte(ModRM,src);   CLKM(2,2,2,9,9,3);          }
OP( 0x89, i_mov_wr16  ) { uint16_t src; GetModRM; src = RegWord(ModRM);   PutRMWord(ModRM,src);   CLKR(13,13,5,13,9,3,2,m_EA);  }
OP( 0x8a, i_mov_r8b   ) { uint8_t  src; GetModRM; src = GetRMByte(ModRM); RegByte(ModRM)=src;     CLKM(2,2,2,11,11,5);        }
OP( 0x8b, i_mov_r16w  ) { uint16_t src; GetModRM; src = GetRMWord(ModRM); RegWord(ModRM)=src;     CLKR(15,15,7,15,11,5,2,m_EA);     }
OP( 0x8c, i_mov_wsreg ) { GetModRM;
	switch (ModRM & 0x38) {
		case 0x00: PutRMWord(ModRM,Sreg(DS1)); CLKR(14,14,5,14,10,3,2,m_EA); break;
		case 0x08: PutRMWord(ModRM,Sreg(PS)); CLKR(14,14,5,14,10,3,2,m_EA); break;
		case 0x10: PutRMWord(ModRM,Sreg(SS)); CLKR(14,14,5,14,10,3,2,m_EA); break;
		case 0x18: PutRMWord(ModRM,Sreg(DS0)); CLKR(14,14,5,14,10,3,2,m_EA); break;
		default:   logerror("%06x: MOV Sreg - Invalid register\n",PC());
	}
}
OP( 0x8d, i_lea       ) { uint16_t ModRM = fetch(); if (ModRM >= 0xc0) logerror("LDEA invalid mode %Xh\n", ModRM); else { (void)(this->*s_GetEA[ModRM])(); RegWord(ModRM)=m_EO; }  CLKS(4,4,2); }
OP( 0x8e, i_mov_sregw ) { uint16_t src; GetModRM; src = GetRMWord(ModRM); CLKR(15,15,7,15,11,5,2,m_EA);
	switch (ModRM & 0x38) {
		case 0x00: Sreg(DS1) = src; break; /* mov es,ew */
		case 0x08: Sreg(PS) = src; break; /* mov cs,ew */
		case 0x10: Sreg(SS) = src; break; /* mov ss,ew */
		case 0x18: Sreg(DS0) = src; break; /* mov ds,ew */
		default:   logerror("%06x: MOV Sreg - Invalid register\n",PC());
	}
	m_no_interrupt=1;
}
OP( 0x8f, i_popw ) { uint16_t tmp; GetModRM; POP(tmp); PutRMWord(ModRM,tmp); CLK(21); }
OP( 0x90, i_nop  ) { CLK(3); /* { if (m_MF == 0) printf("90 -> %06x: \n",PC()); }  */ }
OP( 0x91, i_xchg_axcx ) { XchgAWReg(CW); CLK(3); }
OP( 0x92, i_xchg_axdx ) { XchgAWReg(DW); CLK(3); }
OP( 0x93, i_xchg_axbx ) { XchgAWReg(BW); CLK(3); }
OP( 0x94, i_xchg_axsp ) { XchgAWReg(SP); CLK(3); }
OP( 0x95, i_xchg_axbp ) { XchgAWReg(BP); CLK(3); }
OP( 0x96, i_xchg_axsi ) { XchgAWReg(IX); CLK(3); }
OP( 0x97, i_xchg_axdi ) { XchgAWReg(IY); CLK(3); }

OP( 0x98, i_cbw       ) { Breg(AH) = (Breg(AL) & 0x80) ? 0xff : 0;      CLK(2); }
// Manual gives CWD as data-dependent, "4 or 5"; this used the flat lower
// bound instead of the upper/worst-case convention used everywhere else
// in this file for data-dependent ranges (see MUL/DIV a few hundred
// lines down). Tiny (1-cycle) magnitude, fixed anyway for consistency.
OP( 0x99, i_cwd       ) { Wreg(DW) = (Breg(AH) & 0x80) ? 0xffff : 0;    CLK(5); }
OP( 0x9a, i_call_far  ) { uint32_t tmp, tmp2; tmp = fetchword(); tmp2 = fetchword(); PUSH(Sreg(PS)); PUSH(m_ip); m_ip = (WORD)tmp; Sreg(PS) = (WORD)tmp2; CHANGE_PC; CLKW(29,29,13,29,21,9,Wreg(SP)); }
OP( 0x9b, i_wait      ) { if (!m_poll_state) m_ip--; CLK(5); }
OP( 0x9c, i_pushf     ) { uint16_t tmp = CompressFlags(); PUSH( tmp ); CLKS(12,8,3); }
OP( 0x9d, i_popf      ) { uint32_t tmp; POP(tmp); ExpandFlags(tmp); CLKS(12,8,5); if (m_TF) nec_trap(); }
OP( 0x9e, i_sahf      ) { uint32_t tmp = (CompressFlags() & 0xff00) | (Breg(AH) & 0xd5); ExpandFlags(tmp); CLKS(3,3,2); }
OP( 0x9f, i_lahf      ) { Breg(AH) = CompressFlags() & 0xff; CLKS(3,3,2); }

OP( 0xa0, i_mov_aldisp ) { uint32_t addr; addr = fetchword(); Breg(AL) = GetMemB(DS0, addr); CLKS(10,10,5); }
OP( 0xa1, i_mov_axdisp ) { uint32_t addr; addr = fetchword(); Wreg(AW) = GetMemW(DS0, addr); CLKW(14,14,7,14,10,5,addr); }
OP( 0xa2, i_mov_dispal ) { uint32_t addr; addr = fetchword(); PutMemB(DS0, addr, Breg(AL));  CLKS(9,9,3); }
OP( 0xa3, i_mov_dispax ) { uint32_t addr; addr = fetchword(); PutMemW(DS0, addr, Wreg(AW));  CLKW(13,13,5,13,9,3,addr); }
// MOVS/CMPS/SCAS/LODS/STOS per-execution costs below are cross-referenced
// against NEC's own "uPD70108/uPD70116 (V20/V30) User's Manual" (Oct 1986),
// Section 12 - the per-instruction "REPEAT" clock figures (the per-iteration
// rate used inside a REP loop, which is what these flat CLK values are
// meant to approximate, since this emulator charges the same constant on
// every call whether the opcode runs standalone or is repeated by REP).
// V33 (uPD70136) postdates this manual and isn't covered by it, so its
// values are left exactly as they were - only V20/V30 are corrected here.
OP( 0xa4, i_movsb      ) { uint32_t tmp = GetMemB(DS0,Wreg(IX)); PutMemB(DS1,Wreg(IY), tmp); Wreg(IY) += -2 * m_DF + 1; Wreg(IX) += -2 * m_DF + 1; CLKS(8,8,6); }
// MOVSW: manual's REP MOVBKW cost is 11+16/rep unless BOTH pointers are
// even (11+8/rep) - V20 (uPD70108, 8-bit bus) never gets the discount, so
// it's flat 16. This macro can only test one address (IY, the
// destination, matching the convention already used below for STOS/SCAS
// which also key off IY); the previous constants (8/4) were exactly half
// of the correct per-iteration rate (16/8) in both branches - a clean 2x
// undercount, not just a rounding difference.
OP( 0xa5, i_movsw      ) { uint32_t tmp = GetMemW(DS0,Wreg(IX)); PutMemW(DS1,Wreg(IY), tmp); Wreg(IY) += -4 * m_DF + 2; Wreg(IX) += -4 * m_DF + 2; CLKW(16,16,10,16,8,10,Wreg(IY)); }
OP( 0xa6, i_cmpsb      ) { uint32_t src = GetMemB(DS1, Wreg(IY)); uint32_t dst = GetMemB(DS0, Wreg(IX)); SUBB; Wreg(IY) += -2 * m_DF + 1; Wreg(IX) += -2 * m_DF + 1; CLKS(14,14,14); }
// CMPSW previously used a flat 14 (the V30 best-case "both addresses
// even" rate) for every chip type and every alignment. The manual gives
// V20=22 always (no alignment benefit, 8-bit bus), and V30 22/18/14
// depending on which of IX/IY are odd; this only tests one address (IX,
// the source, matching LODS's convention below) so it can't capture the
// "only one of the two is odd" (18) case, but it does fix the far more
// common "V20, or V30 with a misaligned pointer" undercount (14 instead
// of 22).
OP( 0xa7, i_cmpsw      ) { uint32_t src = GetMemW(DS1, Wreg(IY)); uint32_t dst = GetMemW(DS0, Wreg(IX)); SUBW; Wreg(IY) += -4 * m_DF + 2; Wreg(IX) += -4 * m_DF + 2; CLKW(22,22,14,22,14,14,Wreg(IX)); }

OP( 0xa8, i_test_ald8  ) { DEF_ald8;  ANDB; CLKS(4,4,2); }
OP( 0xa9, i_test_axd16 ) { DEF_axd16; ANDW; CLKS(4,4,2); }
OP( 0xaa, i_stosb      ) { PutMemB(DS1,Wreg(IY),Breg(AL));  Wreg(IY) += -2 * m_DF + 1; CLKS(4,4,3);  }
OP( 0xab, i_stosw      ) { PutMemW(DS1,Wreg(IY),Wreg(AW));  Wreg(IY) += -4 * m_DF + 2; CLKW(8,8,5,8,4,3,Wreg(IY)); }
// LODSB/LODSW and SCASB/SCASW previously reused STOS's constants
// verbatim (identical (4,4,3)/(8,8,5,8,4,3) tuples on all three
// opcodes) even though the manual gives each of the three instructions
// its own, different REP-rate: STOS=4/8/4 (byte/odd-word/even-word),
// LODS=9/13/9, SCAS=10/14/10. STOS's numbers happen to already be
// correct (verified against the manual, left unchanged above); LODS and
// SCAS were undercounted by the amount STOS is cheaper than each of
// them.
OP( 0xac, i_lodsb      ) { Breg(AL) = GetMemB(DS0,Wreg(IX)); Wreg(IX) += -2 * m_DF + 1; CLKS(9,9,3);  }
OP( 0xad, i_lodsw      ) { Wreg(AW) = GetMemW(DS0,Wreg(IX)); Wreg(IX) += -4 * m_DF + 2; CLKW(13,13,5,13,9,3,Wreg(IX)); }
OP( 0xae, i_scasb      ) { uint32_t src = GetMemB(DS1, Wreg(IY)); uint32_t dst = Breg(AL); SUBB; Wreg(IY) += -2 * m_DF + 1; CLKS(10,10,3);  }
OP( 0xaf, i_scasw      ) { uint32_t src = GetMemW(DS1, Wreg(IY)); uint32_t dst = Wreg(AW); SUBW; Wreg(IY) += -4 * m_DF + 2; CLKW(14,14,5,14,10,3,Wreg(IY)); }

OP( 0xb0, i_mov_ald8  ) { Breg(AL) = fetch();   CLKS(4,4,2); }
OP( 0xb1, i_mov_cld8  ) { Breg(CL) = fetch(); CLKS(4,4,2); }
OP( 0xb2, i_mov_dld8  ) { Breg(DL) = fetch(); CLKS(4,4,2); }
OP( 0xb3, i_mov_bld8  ) { Breg(BL) = fetch(); CLKS(4,4,2); }
OP( 0xb4, i_mov_ahd8  ) { Breg(AH) = fetch(); CLKS(4,4,2); }
OP( 0xb5, i_mov_chd8  ) { Breg(CH) = fetch(); CLKS(4,4,2); }
OP( 0xb6, i_mov_dhd8  ) { Breg(DH) = fetch(); CLKS(4,4,2); }
OP( 0xb7, i_mov_bhd8  ) { Breg(BH) = fetch();   CLKS(4,4,2); }

OP( 0xb8, i_mov_axd16 ) { Breg(AL) = fetch();    Breg(AH) = fetch();    CLKS(4,4,2); }
OP( 0xb9, i_mov_cxd16 ) { Breg(CL) = fetch();    Breg(CH) = fetch();    CLKS(4,4,2); }
OP( 0xba, i_mov_dxd16 ) { Breg(DL) = fetch();    Breg(DH) = fetch();    CLKS(4,4,2); }
OP( 0xbb, i_mov_bxd16 ) { Breg(BL) = fetch();    Breg(BH) = fetch();    CLKS(4,4,2); }
OP( 0xbc, i_mov_spd16 ) { Wreg(SP) = fetchword();   CLKS(4,4,2); }
OP( 0xbd, i_mov_bpd16 ) { Wreg(BP) = fetchword();   CLKS(4,4,2); }
OP( 0xbe, i_mov_sid16 ) { Wreg(IX) = fetchword();   CLKS(4,4,2); }
OP( 0xbf, i_mov_did16 ) { Wreg(IY) = fetchword();   CLKS(4,4,2); }

OP( 0xc0, i_rotshft_bd8 ) {
	uint32_t src, dst; uint8_t c;
	GetModRM; src = (unsigned)GetRMByte(ModRM); dst=src;
	c=fetch();
	CLKM(7,7,2,19,19,6);
	if (c) switch (ModRM & 0x38) {
		case 0x00: do { ROL_BYTE;  c--; CLK(1); } while (c>0); PutbackRMByte(ModRM,(BYTE)dst); break;
		case 0x08: do { ROR_BYTE;  c--; CLK(1); } while (c>0); PutbackRMByte(ModRM,(BYTE)dst); break;
		case 0x10: do { ROLC_BYTE; c--; CLK(1); } while (c>0); PutbackRMByte(ModRM,(BYTE)dst); break;
		case 0x18: do { RORC_BYTE; c--; CLK(1); } while (c>0); PutbackRMByte(ModRM,(BYTE)dst); break;
		case 0x20: SHL_BYTE(c); break;
		case 0x28: SHR_BYTE(c); break;
		case 0x30: logerror("%06x: Undefined opcode 0xc0 0x30 (SHLA)\n",PC()); break;
		case 0x38: SHRA_BYTE(c); break;
	}
}

OP( 0xc1, i_rotshft_wd8 ) {
	uint32_t src, dst;  uint8_t c;
	GetModRM; src = (unsigned)GetRMWord(ModRM); dst=src;
	c=fetch();
	CLKM(7,7,2,27,19,6);
	if (c) switch (ModRM & 0x38) {
		case 0x00: do { ROL_WORD;  c--; CLK(1); } while (c>0); PutbackRMWord(ModRM,(WORD)dst); break;
		case 0x08: do { ROR_WORD;  c--; CLK(1); } while (c>0); PutbackRMWord(ModRM,(WORD)dst); break;
		case 0x10: do { ROLC_WORD; c--; CLK(1); } while (c>0); PutbackRMWord(ModRM,(WORD)dst); break;
		case 0x18: do { RORC_WORD; c--; CLK(1); } while (c>0); PutbackRMWord(ModRM,(WORD)dst); break;
		case 0x20: SHL_WORD(c); break;
		case 0x28: SHR_WORD(c); break;
		case 0x30: logerror("%06x: Undefined opcode 0xc1 0x30 (SHLA)\n",PC()); break;
		case 0x38: SHRA_WORD(c); break;
	}
}

// RET/RETF (all 4 forms, this one and the 3 more at 0xc3/0xca/0xcb)
// show the exact same pattern just found and fixed on IRET: NEC's
// manual (RET near/far, with/without a pop-value operand) gives V20
// flat and correctly matches what was already here in every case, but
// V30 splits by SP address parity (odd/even) and these were all flat
// at the odd-address (more expensive) value instead of splitting - the
// common case (word-aligned SP) is the cheaper even-address number.
// Manual: RET near, pop-value: V20=24, V30 odd=24/even=20.
OP( 0xc2, i_ret_d16  ) { uint32_t count = fetch(); count += fetch() << 8; POP(m_ip); Wreg(SP)+=count; CHANGE_PC; CLKW(24,24,10,24,20,10,Wreg(SP)); }
// Manual: RET near, no operand: V20=19, V30 odd=19/even=15.
OP( 0xc3, i_ret      ) { POP(m_ip); CHANGE_PC; CLKW(19,19,10,19,15,10,Wreg(SP)); }
OP( 0xc4, i_les_dw   ) { GetModRM; WORD tmp = GetRMWord(ModRM); RegWord(ModRM)=tmp; Sreg(DS1) = GetnextRMWord; CLKW(26,26,14,26,18,10,m_EA); }
OP( 0xc5, i_lds_dw   ) { GetModRM; WORD tmp = GetRMWord(ModRM); RegWord(ModRM)=tmp; Sreg(DS0) = GetnextRMWord; CLKW(26,26,14,26,18,10,m_EA); }
OP( 0xc6, i_mov_bd8  ) { GetModRM; PutImmRMByte(ModRM); CLK((ModRM >=0xc0) ? 4 : 11); }
OP( 0xc7, i_mov_wd16 ) { GetModRM; PutImmRMWord(ModRM); CLK((ModRM >=0xc0) ? 4 : 15); }

OP( 0xc8, i_enter ) {
	uint32_t nb = fetch();
	uint32_t i,level;

	CLK(23);
	nb += fetch() << 8;
	level = fetch();
	PUSH(Wreg(BP));
	Wreg(BP)=Wreg(SP);
	Wreg(SP) -= nb;
	for (i=1;i<level;i++) {
		PUSH(GetMemW(SS,Wreg(BP)-i*2));
		CLK(16);
	}
	if (level) PUSH(Wreg(BP));
}
OP( 0xc9, i_leave ) {
	Wreg(SP)=Wreg(BP);
	POP(Wreg(BP));
	CLK(8);
}
// Same RET/RETF fix as 0xc2/0xc3 above, far forms. Manual: RET far,
// pop-value: V20=32, V30 odd=32/even=24.
OP( 0xca, i_retf_d16  ) { uint32_t count = fetch(); count += fetch() << 8; POP(m_ip); POP(Sreg(PS)); Wreg(SP)+=count; CHANGE_PC; CLKW(32,32,16,32,24,16,Wreg(SP)); }
// Manual: RET far, no operand: V20=29, V30 odd=29/even=21.
OP( 0xcb, i_retf      ) { POP(m_ip); POP(Sreg(PS)); CHANGE_PC; CLKW(29,29,16,29,21,16,Wreg(SP)); }
OP( 0xcc, i_int3      ) { nec_interrupt(3, BRK); CLKS(50,50,24); }
OP( 0xcd, i_int       ) { nec_interrupt(fetch(), BRK); CLKS(50,50,24); }
OP( 0xce, i_into      ) { if (OF) { nec_interrupt(NEC_BRKV_VECTOR, BRK); CLKS(52,52,26); } else CLK(3); }
// Revisited (again): three rounds ago this was checked only against
// i86.cpp's Intel 8086 reference (32) as a fallback proxy, found to be
// an OVER-count relative to that, and set aside. This round, checked
// against the actual NEC manual (RETI, its own name for IRET): V20=39
// flat - matches what's already here - but V30 splits 39 (odd SP) / 27
// (even SP), which this flat CLKS(39,39,...) doesn't model. That part
// is genuine and well-evidenced (same manual, same technique as the
// RET/RETF fix just above, which IS safe and stays applied).
//
// BUT: implementing the V30 split here (tried as CLKW(39,39,19,39,27,
// 19,Wreg(SP)), i.e. -12 cycles on the common even-SP case) caused a
// confirmed, reproducible regression under the exact WLTCDIAG
// reproduction sequence - INTERRUPT CONTROL, which passes on every
// prior round's baseline, started failing instead of DMA CONTROL.
// Bisected by reverting IRET alone while keeping RET/RETF applied:
// behaviour returned to normal (PROGRAMMABLE TIMER passes, DMA CONTROL
// fails as always) with IRET reverted, confirming IRET specifically -
// not RET/RETF - is responsible. Left at the original flat 39 per the
// "revert and report rather than ship a net trade-off" rule. This
// strongly suggests INTERRUPT CONTROL's current pass is itself
// accidentally propped up by this same overcount (i.e. there is likely
// a *different*, still-undiagnosed bug elsewhere - possibly in the
// interrupt controller/PIC chain in wltc.cpp, or another instruction on
// the interrupt-return path - that this correct IRET timing exposes),
// mirroring the DMA CONTROL situation this whole investigation started
// from. Worth its own dedicated round rather than reverting-and-losing
// this finding entirely.
OP( 0xcf, i_iret      ) { POP(m_ip); POP(Sreg(PS)); i_popf(); CHANGE_PC; CLKS(39,39,19); }

// Register-direct branch of ROL/ROR/ROLC(RCL)/RORC(RCR)/SHL/SHR/SHRA(SAR)
// reg,1 was 6 for V20/V30 (V33 already correctly 2). Checked every one of
// the 7 sub-operations individually against their own dedicated page in
// NEC's uPD70108/uPD70116 manual, Section 12 (not just SHL, in case some
// carry-involving form like ROLC/RORC genuinely differed) - all seven give
// an unconditional "Clocks: 2" for the register form on both V20 and V30,
// with no W=0/W=1 split (register-direct shift cost doesn't depend on
// operand size) and no sub-opcode exception. MAME's independent i86.cpp
// 8086 reference agrees (ROT_REG_1=2). Checked whether 6 might be a
// deliberate, documented V-series quirk before touching this: git blame/
// log -p --follow and a content pickaxe search across 100k+ commits of
// local history (back to at least 2011) show this exact constant has
// never been modified - no commit, message, or nearby code comment
// explains or justifies it. (An earlier version of this comment cited
// "MUL/DIV" as having such odd/even-address notes elsewhere in the file;
// that was a misattribution on read-back - those detailed notes actually
// sit on the INS/EXT bit-field instructions above, not MUL/DIV. MUL/DIV
// turned out to have their own, different bug - see the fix a few
// screens down.) nec.cpp's own file header states plainly this core is
// "99% accurate...there are still some complex situations where cycle
// counts are wrong" with an open "Todo: double check cycle timing is
// 100%" from the original 2000 rewrite - and every other register-direct
// ModRM opcode already audited in this file (ADD, MOV, INC, TEST reg,reg)
// is correctly 2, so this was the outlier, not the established pattern.
//
// Memory-operand branch, byte form: manual gives a flat 16 with no
// V20/V30 distinction for the byte-operand memory form (checked all 7
// sub-ops individually, not just SHL - ROL/ROR/ROLC/RORC/SHR/SHRA mem,1
// all read "When W=0: 16" uniformly), which is exactly what this already
// models. No change needed here.
//
// SHL/SHR/SHRA double-charge: CLKM above already deposits the COMPLETE
// flat cost for this fixed shift-by-1 form (same as ROL/ROR/ROLC/RORC,
// which take nothing further). But SHL_BYTE/SHR_BYTE/SHRA_BYTE are the
// SAME macros 0xd2 (shift-by-CL) uses, where they're called ONCE with
// c=CL and their own embedded CLK(c) is the intended per-bit cost on top
// of 0xd2's separate, smaller base charge. Reusing them here with c=1
// makes them silently add one more CLK(1) after CLKM's already-total
// value - found by comparing this to the 0xd2/0xd3 callers, not by the
// real-hardware SHL AX,1 gap (that gap is much larger and unexplained by
// this alone - see V30BENCH results). CLK(-1) below undoes just that
// extra add, leaving ROL/ROR/ROLC/RORC (which never had it) untouched.
OP( 0xd0, i_rotshft_b ) {
	uint32_t src, dst; GetModRM; src = (uint32_t)GetRMByte(ModRM); dst=src;
	CLKM(2,2,2,16,16,7);
	switch (ModRM & 0x38) {
		case 0x00: ROL_BYTE;  PutbackRMByte(ModRM,(BYTE)dst); m_OverVal = (src^dst)&0x80; break;
		case 0x08: ROR_BYTE;  PutbackRMByte(ModRM,(BYTE)dst); m_OverVal = (src^dst)&0x80; break;
		case 0x10: ROLC_BYTE; PutbackRMByte(ModRM,(BYTE)dst); m_OverVal = (src^dst)&0x80; break;
		case 0x18: RORC_BYTE; PutbackRMByte(ModRM,(BYTE)dst); m_OverVal = (src^dst)&0x80; break;
		case 0x20: SHL_BYTE(1); CLK(-1); m_OverVal = (src^dst)&0x80; break;
		case 0x28: SHR_BYTE(1); CLK(-1); m_OverVal = (src^dst)&0x80; break;
		case 0x30: logerror("%06x: Undefined opcode 0xd0 0x30 (SHLA)\n",PC()); break;
		case 0x38: SHRA_BYTE(1); CLK(-1); m_OverVal = 0; break;
	}
}

// Register-direct branch: same fix as 0xd0 above (word form; manual
// gives no byte/word distinction for the register-direct single-bit
// form, so it's the same flat 2 for all three chip types).
//
// Memory-operand branch: manual gives V20=24 flat and V30=24 (odd
// address) / 16 (even address) for all 7 sub-ops' mem,1 word form
// (checked individually, same as the byte form above) - this was
// CLKM's flat, unsplit 16 for V30, i.e. always the even-address best
// case, the same "silently even-only" gap already found and fixed via
// CLKR in INC/DEC mem and PUSH mem a couple of rounds ago. Since the
// register-direct branch is now a uniform 2 across V20/V30/V33 (per the
// fix above), CLKR's single flat "vall" register-form slot can still
// exactly represent it.
// Same SHL/SHR/SHRA double-charge as 0xd0 above (see the comment there):
// CLKR already deposits the complete flat/odd/even cost for this
// fixed-shift-by-1 word form, and SHL_WORD/SHR_WORD/SHRA_WORD's own
// embedded CLK(1) (there for 0xd3's shift-by-CL reuse) adds one more on
// top. CLK(-1) undoes just that.
OP( 0xd1, i_rotshft_w ) {
	uint32_t src, dst; GetModRM; src = (uint32_t)GetRMWord(ModRM); dst=src;
	CLKR(24,24,7,24,16,7,2,m_EA);
	switch (ModRM & 0x38) {
		case 0x00: ROL_WORD;  PutbackRMWord(ModRM,(WORD)dst); m_OverVal = (src^dst)&0x8000; break;
		case 0x08: ROR_WORD;  PutbackRMWord(ModRM,(WORD)dst); m_OverVal = (src^dst)&0x8000; break;
		case 0x10: ROLC_WORD; PutbackRMWord(ModRM,(WORD)dst); m_OverVal = (src^dst)&0x8000; break;
		case 0x18: RORC_WORD; PutbackRMWord(ModRM,(WORD)dst); m_OverVal = (src^dst)&0x8000; break;
		case 0x20: SHL_WORD(1); CLK(-1); m_OverVal = (src^dst)&0x8000;  break;
		case 0x28: SHR_WORD(1); CLK(-1); m_OverVal = (src^dst)&0x8000;  break;
		case 0x30: logerror("%06x: Undefined opcode 0xd1 0x30 (SHLA)\n",PC()); break;
		case 0x38: SHRA_WORD(1); CLK(-1); m_OverVal = 0; break;
	}
}

OP( 0xd2, i_rotshft_bcl ) {
	uint32_t src, dst; uint8_t c; GetModRM; src = (uint32_t)GetRMByte(ModRM); dst=src;
	c=Breg(CL);
	CLKM(7,7,2,19,19,6);
	if (c) switch (ModRM & 0x38) {
		case 0x00: do { ROL_BYTE;  c--; CLK(1); } while (c>0); PutbackRMByte(ModRM,(BYTE)dst); break;
		case 0x08: do { ROR_BYTE;  c--; CLK(1); } while (c>0); PutbackRMByte(ModRM,(BYTE)dst); break;
		case 0x10: do { ROLC_BYTE; c--; CLK(1); } while (c>0); PutbackRMByte(ModRM,(BYTE)dst); break;
		case 0x18: do { RORC_BYTE; c--; CLK(1); } while (c>0); PutbackRMByte(ModRM,(BYTE)dst); break;
		case 0x20: SHL_BYTE(c); break;
		case 0x28: SHR_BYTE(c); break;
		case 0x30: logerror("%06x: Undefined opcode 0xd2 0x30 (SHLA)\n",PC()); break;
		case 0x38: SHRA_BYTE(c); break;
	}
}

OP( 0xd3, i_rotshft_wcl ) {
	uint32_t src, dst; uint8_t c; GetModRM; src = (uint32_t)GetRMWord(ModRM); dst=src;
	c=Breg(CL);
	CLKM(7,7,2,27,19,6);
	if (c) switch (ModRM & 0x38) {
		case 0x00: do { ROL_WORD;  c--; CLK(1); } while (c>0); PutbackRMWord(ModRM,(WORD)dst); break;
		case 0x08: do { ROR_WORD;  c--; CLK(1); } while (c>0); PutbackRMWord(ModRM,(WORD)dst); break;
		case 0x10: do { ROLC_WORD; c--; CLK(1); } while (c>0); PutbackRMWord(ModRM,(WORD)dst); break;
		case 0x18: do { RORC_WORD; c--; CLK(1); } while (c>0); PutbackRMWord(ModRM,(WORD)dst); break;
		case 0x20: SHL_WORD(c); break;
		case 0x28: SHR_WORD(c); break;
		case 0x30: logerror("%06x: Undefined opcode 0xd3 0x30 (SHLA)\n",PC()); break;
		case 0x38: SHRA_WORD(c); break;
	}
}

OP( 0xd4, i_aam    ) { fetch(); Breg(AH) = Breg(AL) / 10; Breg(AL) %= 10; SetSZPF_Word(Wreg(AW)); CLKS(15,15,12); }
OP( 0xd5, i_aad    ) { fetch(); Breg(AL) = Breg(AH) * 10 + Breg(AL); Breg(AH) = 0; SetSZPF_Byte(Breg(AL)); CLKS(7,7,8); }
OP( 0xd6, i_setalc ) { Breg(AL) = (CF)?0xff:0x00; CLK(3); logerror("%06x: Undefined opcode (SETALC)\n",PC()); }
OP( 0xd7, i_trans  ) { uint32_t dest = (Wreg(BW)+Breg(AL))&0xffff; Breg(AL) = GetMemB(DS0, dest); CLKS(9,9,5); }
OP( 0xd8, i_fpo    ) { GetModRM; GetRMByte(ModRM); CLK(2);  logerror("%06x: Unimplemented floating point control %04x\n",PC(),ModRM); }

OP( 0xe0, i_loopne ) { int8_t disp = (int8_t)fetch(); Wreg(CW)--; if (!ZF && Wreg(CW)) { m_ip = (WORD)(m_ip+disp); /*CHANGE_PC;*/ CLKS(14,14,6); } else CLKS(5,5,3); }
OP( 0xe1, i_loope  ) { int8_t disp = (int8_t)fetch(); Wreg(CW)--; if ( ZF && Wreg(CW)) { m_ip = (WORD)(m_ip+disp); /*CHANGE_PC;*/ CLKS(14,14,6); } else CLKS(5,5,3); }
OP( 0xe2, i_loop   ) { int8_t disp = (int8_t)fetch(); Wreg(CW)--; if (Wreg(CW)) { m_ip = (WORD)(m_ip+disp); /*CHANGE_PC;*/ CLKS(13,13,6); } else CLKS(5,5,3); }
OP( 0xe3, i_jcxz   ) { int8_t disp = (int8_t)fetch(); if (Wreg(CW) == 0) { m_ip = (WORD)(m_ip+disp); /*CHANGE_PC;*/ CLKS(13,13,6); } else CLKS(5,5,3); }
OP( 0xe4, i_inal   ) { uint8_t port = fetch(); Breg(AL) = read_port_byte(port); CLKS(9,9,5);  }
OP( 0xe5, i_inax   ) { uint8_t port = fetch(); Wreg(AW) = read_port_word(port); CLKW(13,13,7,13,9,5,port); }
OP( 0xe6, i_outal  ) { uint8_t port = fetch(); write_port_byte(port, Breg(AL)); CLKS(8,8,3);  }
OP( 0xe7, i_outax  ) { uint8_t port = fetch(); write_port_word(port, Wreg(AW)); CLKW(12,12,5,12,8,3,port);    }

OP( 0xe8, i_call_d16 ) { uint32_t tmp; tmp = fetchword(); PUSH(m_ip); m_ip = (WORD)(m_ip+(int16_t)tmp); CHANGE_PC; CLK(24); }
OP( 0xe9, i_jmp_d16  ) { uint32_t tmp; tmp = fetchword(); m_ip = (WORD)(m_ip+(int16_t)tmp); CHANGE_PC; CLK(15); }
OP( 0xea, i_jmp_far  ) { uint32_t tmp,tmp1; tmp = fetchword(); tmp1 = fetchword(); Sreg(PS) = (WORD)tmp1;     m_ip = (WORD)tmp; CHANGE_PC; CLK(27);  }
OP( 0xeb, i_jmp_d8   ) { int tmp = (int)((int8_t)fetch()); CLK(12); m_ip = (WORD)(m_ip+tmp); }
OP( 0xec, i_inaldx   ) { Breg(AL) = read_port_byte(Wreg(DW)); CLKS(8,8,5);}
OP( 0xed, i_inaxdx   ) { Wreg(AW) = read_port_word(Wreg(DW)); CLKW(12,12,7,12,8,5,Wreg(DW)); }
OP( 0xee, i_outdxal  ) { write_port_byte(Wreg(DW), Breg(AL)); CLKS(8,8,3);  }
OP( 0xef, i_outdxax  ) { write_port_word(Wreg(DW), Wreg(AW)); CLKW(12,12,5,12,8,3,Wreg(DW)); }

OP( 0xf0, i_lock     ) { LOGMASKED(LOG_BUSLOCK, "%06x: Warning - BUSLOCK\n",PC()); m_no_interrupt=1; CLK(2); }
OP( 0xf2, i_repne    ) { do_repne(start_rep()); }
OP( 0xf3, i_repe     ) { do_repe(start_rep()); }
OP( 0xf4, i_hlt ) { m_halted=1; m_icount=0; }
OP( 0xf5, i_cmc ) { m_CarryVal = !CF; CLK(2); }
// MULU/MUL/DIVU/DIV byte forms (this opcode) had been charging exactly
// the same V20/V30 constants as the WORD forms at 0xf7 below - only the
// V33 slot differed between the two, everything else was copy-pasted.
// NEC's manual (Section 12) confirms real V20/V30 byte-operand
// multiply/divide is genuinely cheaper than the word form (fewer
// bits to shift-and-add/subtract), and gives no V20-vs-V30 distinction
// at all for byte operands (register OR memory) - unlike several word
// forms in this same group, which do differ by chip/address parity.
// MULU/MUL/DIV are explicitly data-dependent on real hardware ("21 or
// 22", "33 to 39", etc.) but this file has no precedent anywhere for
// modelling that - every other data-dependent range in this exact
// group (the WORD forms just below) is already approximated with a
// single flat value at the top/worst-case end of its documented range,
// so the byte-form fix below follows that same established convention
// rather than introducing a new modelling approach:
//   MULU reg8/mem8: 21-22 / 27-28  -> 22 / 28
//   MUL   reg8/mem8: 33-39 / 39-45  -> 39 / 45
//   DIVU  reg8/mem8: 19 (fixed) / 25 (fixed) -> 19 / 25 (not a range,
//     unlike the signed forms - real DIVU has no early-out/negation step)
//   DIV   reg8/mem8: 29-34 / 35-40  -> 34 / 40
// (mem forms also lose the V20 vs V30 split of 57/53, 53/49 etc. that
// had been inherited from the word form - the manual has no such split
// for byte-operand memory forms.)
OP( 0xf6, i_f6pre ) { uint32_t tmp; uint32_t uresult,uresult2; int32_t result,result2;
	GetModRM; tmp = GetRMByte(ModRM);
	switch (ModRM & 0x38) {
		case 0x00: tmp &= fetch(); m_CarryVal = m_OverVal = 0; SetSZPF_Byte(tmp); CLK((ModRM >= 0xc0) ? 4 : 11); break; /* TEST */
		case 0x08: logerror("%06x: Undefined opcode 0xf6 0x08\n",PC()); break;
		case 0x10: PutbackRMByte(ModRM,~tmp); CLK((ModRM >= 0xc0) ? 2 : 16); break; /* NOT */
		case 0x18: m_CarryVal=(tmp!=0); tmp=(~tmp)+1; SetSZPF_Byte(tmp); PutbackRMByte(ModRM,tmp&0xff); CLK((ModRM >= 0xc0) ? 2 : 16); break; /* NEG */
		case 0x20: uresult = Breg(AL)*tmp; Wreg(AW)=(WORD)uresult; m_CarryVal=m_OverVal=(Breg(AH)!=0); CLKM(22,22,8,28,28,12); break; /* MULU */
		case 0x28: result = (int16_t)((int8_t)Breg(AL))*(int16_t)((int8_t)tmp); Wreg(AW)=(WORD)result; m_CarryVal=m_OverVal=(Breg(AH)!=0); CLKM(39,39,8,45,45,12); break; /* MUL */
		case 0x30:
			if (tmp)
				DIVUB
			else
			{
				m_CarryVal = m_OverVal = 0;
				nec_interrupt(NEC_DIVIDE_VECTOR, BRK);
			}

			CLKM(19,19,11,25,25,15);
			break;
		case 0x38:
			if (tmp)
				DIVB
			else
			{
				m_CarryVal = m_OverVal = 0;
				nec_interrupt(NEC_DIVIDE_VECTOR, BRK);
			}

			CLKM(34,34,17,40,40,20);
			break;
	}
}

OP( 0xf7, i_f7pre   ) { uint32_t tmp,tmp2; uint32_t uresult,uresult2; int32_t result,result2;
	GetModRM; tmp = GetRMWord(ModRM);
	switch (ModRM & 0x38) {
		case 0x00: tmp2 = fetchword(); tmp &= tmp2; m_CarryVal = m_OverVal = 0; SetSZPF_Word(tmp); CLK((ModRM >= 0xc0) ? 4 : 11); break; /* TEST */
		case 0x08: logerror("%06x: Undefined opcode 0xf7 0x08\n",PC()); break;
		case 0x10: PutbackRMWord(ModRM,~tmp); CLK((ModRM >= 0xc0) ? 2 : 16); break; /* NOT */
		case 0x18: m_CarryVal=(tmp!=0); tmp=(~tmp)+1; SetSZPF_Word(tmp); PutbackRMWord(ModRM,tmp&0xffff); CLK((ModRM >= 0xc0) ? 2 : 16); break; /* NEG */
		// MULU mem16's V30 slot (36) had been used unchanged for both
		// V20 and V30, unlike its three siblings just below (MUL,
		// DIVU, DIV) which all correctly differ V20 from V30 for the
		// word memory form. Manual: V20=39-40 flat, V30=39-40 (odd) /
		// 35-36 (even) - so V20 wants the word-form's own worst case
		// (40), not the value borrowed from V30's even-address case.
		case 0x20: uresult = Wreg(AW)*tmp; Wreg(AW)=uresult&0xffff; Wreg(DW)=((uint32_t)uresult)>>16; m_CarryVal=m_OverVal=(Wreg(DW)!=0); CLKM(30,30,12,40,36,16); break; /* MULU */
		case 0x28: result = (int32_t)((int16_t)Wreg(AW))*(int32_t)((int16_t)tmp); Wreg(AW)=result&0xffff; Wreg(DW)=result>>16; m_CarryVal=m_OverVal=(Wreg(DW)!=0); CLKM(47,47,12,57,53,16); break; /* MUL */
		case 0x30:
			if (tmp)
				DIVUW
			else
			{
				m_CarryVal = m_OverVal = 0;
				nec_interrupt(NEC_DIVIDE_VECTOR, BRK);
			}

			CLKM(25,25,19,35,31,23);
			break;
		case 0x38:
			if (tmp)
				DIVW
			else
			{
				m_CarryVal = m_OverVal = 0;
				nec_interrupt(NEC_DIVIDE_VECTOR, BRK);
			}
			CLKM(43,43,24,53,49,28);
			break;
	}
}

OP( 0xf8, i_clc   ) { m_CarryVal = 0;  CLK(2); }
OP( 0xf9, i_stc   ) { m_CarryVal = 1;  CLK(2); }
OP( 0xfa, i_di    ) { SetIF(0);         CLK(2); }
OP( 0xfb, i_ei    ) { SetIF(1);         CLK(2); }
OP( 0xfc, i_cld   ) { SetDF(0);         CLK(2); }
OP( 0xfd, i_std   ) { SetDF(1);         CLK(2); }
OP( 0xfe, i_fepre ) { uint32_t tmp, tmp1; GetModRM; tmp=GetRMByte(ModRM);
	switch(ModRM & 0x38) {
		case 0x00: tmp1 = tmp+1; m_OverVal = (tmp==0x7f); SetAF(tmp1,tmp,1); SetSZPF_Byte(tmp1); PutbackRMByte(ModRM,(BYTE)tmp1); CLKM(2,2,2,16,16,7); break; /* INC */
		case 0x08: tmp1 = tmp-1; m_OverVal = (tmp==0x80); SetAF(tmp1,tmp,1); SetSZPF_Byte(tmp1); PutbackRMByte(ModRM,(BYTE)tmp1); CLKM(2,2,2,16,16,7); break; /* DEC */
		default:   logerror("%06x: FE Pre with unimplemented mod\n",PC());
	}
}
OP( 0xff, i_ffpre ) { uint32_t tmp, tmp1; GetModRM; tmp=GetRMWord(ModRM);
	switch(ModRM & 0x38) {
		// INC/DEC mem, word form: the manual gives V20=24 flat and
		// V30=24 (odd address) / 16 (even address), which is exactly the
		// pattern the neighbouring ADD/SUB/etc memory-operand opcodes
		// (e.g. 0x01 a few hundred lines up) already model correctly via
		// CLKR - INC/DEC just used the flat, unsplit CLKM(...,16,7)
		// instead, silently applying V30's best-case (even-address) 16
		// regardless of actual alignment (undercounting the odd-address
		// case by 8). Switched to CLKR to match the established pattern;
		// V33 has no manual data so its value (7) is unchanged.
		case 0x00: tmp1 = tmp+1; m_OverVal = (tmp==0x7fff); SetAF(tmp1,tmp,1); SetSZPF_Word(tmp1); PutbackRMWord(ModRM,(WORD)tmp1); CLKR(24,24,7,24,16,7,2,m_EA); break; /* INC */
		case 0x08: tmp1 = tmp-1; m_OverVal = (tmp==0x8000); SetAF(tmp1,tmp,1); SetSZPF_Word(tmp1); PutbackRMWord(ModRM,(WORD)tmp1); CLKR(24,24,7,24,16,7,2,m_EA); break; /* DEC */
		case 0x10: PUSH(m_ip); m_ip = (WORD)tmp; CHANGE_PC; CLK((ModRM >= 0xc0) ? 16 : 20); break; /* CALL */
		// CALL FAR / JMP / JMP FAR indirect-via-memory: a prior fix this
		// session moved all three off flat, chip-independent
		// placeholders onto MAME's i86.cpp 8086 reference (CALL_M32=37,
		// JMP_M16=18, JMP_M32=24), on the stated grounds that no
		// verified V30 number was on hand for these forms. All three
		// numbers WERE on hand - just not looked at: NEC's own
		// uPD70108/uPD70116 manual gives each its own entry, under its
		// own "BR"/"CALL" mnemonics rather than Intel's "JMP":
		//   CALL memptr32 (p.12-144): 47 V20, 47 V30-odd, 31 V30-even
		//   BR memptr16   (p.12-155): 24 V20, 24 V30-odd, 20 V30-even
		//   BR memptr32   (p.12-156): 35 V20, 35 V30-odd, 27 V30-even
		// - each with a genuine odd/even split the flat CLKM used here
		// could never express, on top of being the wrong magnitude to
		// begin with. Found while chasing why MISURA.COM's LOOP B (built
		// almost entirely around the CALL FAR form) measured
		// substantially slower in emulation than on two real WLTC
		// units - switched all three to CLKR to carry the real splits.
		// V33 has no manual data for any of them, so its values (37 /
		// 18 / 24, unchanged) are still just carried over from the same
		// 8086 reference as before, not verified.
		// The register-direct forms (BR regptr16 for JMP=case 0x20: a
		// flat 11, matching what CLKM already had) aren't affected by
		// any of this - CALL FAR/JMP FAR have no legitimate
		// register-direct encoding at all (there's nowhere to hold a
		// full CS:IP pair in one register), so their register-branch
		// constants (16/15) are unreachable in practice and left as is.
		case 0x18: tmp1 = Sreg(PS); Sreg(PS) = GetnextRMWord; PUSH(tmp1); PUSH(m_ip); m_ip = tmp; CHANGE_PC; CLKR(47,47,37,47,31,37,16,m_EA); break; /* CALL FAR */
		case 0x20: m_ip = tmp; CHANGE_PC; CLKR(24,24,18,24,20,18,11,m_EA); break; /* JMP */
		case 0x28: m_ip = tmp; Sreg(PS) = GetnextRMWord; CHANGE_PC; CLKR(35,35,24,35,27,24,15,m_EA); break; /* JMP FAR */
		// PUSH mem: was a flat CLK(4) for every case, vastly below the
		// manual's memory-operand figures (V20=26, V30=26 odd/18 even
		// address). The manual also shows the *register*-operand form
		// of PUSH (reg16/sreg) costs 12/8 depending on stack-pointer
		// parity rather than being a flat, chip-independent value at
		// all - CLKR's register branch can't express that per-parity
		// split, so 18 (V30's even-address rate, and the closest
		// approximation to the far more common word-aligned-stack case)
		// is used for it here; that's still a large, well-evidenced
		// improvement over the previous flat 4 in every case. V33 has
		// no manual data so its value (4) is unchanged.
		case 0x30: PUSH(tmp); CLKR(26,26,4,18,18,4,18,m_EA); break;
		default:   logerror("%06x: FF Pre with unimplemented mod\n",PC());
	}
}

void nec_common_device::i_invalid()
{
	CLK(10);
	logerror("%06x: Invalid Opcode\n",PC());
}
