// license:BSD-3-Clause
// copyright-holders:Fausto Pracek
/*

    Wang LapTop Computer (WLTC)

    Portable V30-based computer released by Wang Laboratories in 1987
    (name plate: Model No. WLTC, FCC ID B4Y8P7WLTC, made in Japan).
    It runs Wang software natively and can emulate an IBM PC-XT via
    the "Translator" (XLAT.SYS) shipped with its MS-DOS 3.20.

    Main board:
    - NEC D70116C-8 (V30, 8 MHz)
    - NEC D71054G (8254 timer clone)
    - NEC D71059G (8259 interrupt controller clone)
    - Zilog Z8530APS SCC (serial)
    - NCR 53C80 SCSI controller, JVC JD-3812MOBO 10 MB Winchester
    - Toshiba T7779 (probable LCD controller)
    - TMS70C40FS (probable keyboard microcontroller)
    - Wang custom gate arrays: WL3026, WL3028, WL3030, WL3031, WL3032
    - 512K RAM, expandable to 1M (OPT RAM PCB)
    - LCD 80x25 text, 320x200 / 640x200 graphics
    - built-in thermal 24-dot printer, optional modem

    The BIOS occupies 0xe0000-0xfffff. The reset vector executes
    mov al,0x10 / int 0x88, the same convention as the Wang PC.

    Reference: Wang Laptop Computer Product Maintenance Manual,
    742-1747-1 (bitsavers).

    TODO:
    - everything: this skeleton only maps CPU, RAM and BIOS ROM,
      to map the I/O space from the unmapped access log

*/

#include "emu.h"
#include "cpu/nec/nec.h"
#include "machine/timer.h"
#include "screen.h"


namespace {

class wltc_state : public driver_device
{
public:
	wltc_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_shadow(*this, "shadow"),
		m_lowram(*this, "lowram"),
		m_fram(*this, "fram")
	{ }

	void wltc(machine_config &config);

protected:
	virtual void machine_reset() override ATTR_COLD;

private:
	required_device<v30_device> m_maincpu;
	required_shared_ptr<uint16_t> m_shadow;
	required_shared_ptr<uint16_t> m_lowram;
	required_shared_ptr<uint16_t> m_fram;
	bool m_boot_mirror = false;
	std::vector<uint8_t> m_fseg_logged;
	uint8_t m_ivt_seed_rom[0x40];

	void mem_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;

	// temporary reconnaissance handlers: log every I/O access with the PC
	uint16_t io_r(offs_t offset, uint16_t mem_mask);
	void io_w(offs_t offset, uint16_t data, uint16_t mem_mask);

	// F segment: reads come from the EPROMs (verified on real hardware),
	// but the video subsystem accepts writes there (VRAM around 0xf2000,
	// parameter registers at 0xf13xx/0xf18xx/0xf20xx/0xf22xx). Capture
	// every write into a side buffer, mirrored as plain RAM at 0xa0000
	// for inspection, and log the first write to each word address.
	uint16_t fseg_r(offs_t offset, uint16_t mem_mask);
	void fseg_w(offs_t offset, uint16_t data, uint16_t mem_mask);

	// LCD: render the captured VRAM window (0xf2000+) as a 320x200
	// 1bpp bitmap, the Industry Standard graphics geometry from the
	// maintenance manual, until the real controller is understood
	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	// Tick source: the real-hardware IVT dump shows the hardware
	// interrupts on vectors 0x80-0x87 (the D71059 at IBM-style 0x20/0x21
	// is programmed with vector base 0x80; measured IMR 0xbc = IRQ0
	// timer, IRQ1 keyboard, IRQ6 floppy enabled). Fire IRQ0 = INT 80h
	// at 60 Hz until the PIT/PIC pair is properly emulated.
	TIMER_DEVICE_CALLBACK_MEMBER(tick) { m_maincpu->set_input_line(0, HOLD_LINE); }
	IRQ_CALLBACK_MEMBER(irq_ack) { return 0x80; }
};


uint32_t wltc_state::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	// 80x25 text from the CGA-style buffer at 0xb8000, rendered with
	// the 8x16 font found in the EPROMs (char 0 glyph at MYE800+0x75d1),
	// LCD yellow-green like the real display
	const rgb_t fg(0x30, 0x38, 0x20), bg(0xc8, 0xd4, 0x40);
	auto const *text = reinterpret_cast<const uint8_t *>(m_lowram.target());  // unused fallback
	auto const *tram = reinterpret_cast<const uint8_t *>(memshare("textram")->ptr());
	auto const *font = memregion("bios")->base() + 0x8000 + 0x75d1;
	(void)text;
	for (int y = cliprect.top(); y <= cliprect.bottom(); y++)
	{
		int const row = y >> 4, line = y & 15;
		uint32_t *dst = &bitmap.pix(y, cliprect.left());
		for (int x = cliprect.left(); x <= cliprect.right(); x++)
		{
			int const col = x >> 3;
			uint8_t const chr = tram[(row * 80 + col) * 2];
			uint8_t const attr = tram[(row * 80 + col) * 2 + 1];
			uint8_t const bits = font[chr * 16 + line];
			bool on = BIT(bits, 7 - (x & 7));
			if (attr & 0x70)  // crude reverse video
				on = !on;
			*dst++ = on ? fg : bg;
		}
	}
	return 0;
}

uint16_t wltc_state::fseg_r(offs_t offset, uint16_t mem_mask)
{
	return reinterpret_cast<const uint16_t *>(memregion("bios")->base() + 0x10000)[offset];
}

void wltc_state::fseg_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	COMBINE_DATA(&m_fram[offset]);
	if (!m_fseg_logged[offset])
	{
		m_fseg_logged[offset] = 1;
		logerror("%06x: fseg_w %05x = %04x mask %04x\n",
				m_maincpu->pc(), 0xf0000 + (offset << 1), data, mem_mask);
	}
}

uint16_t wltc_state::io_r(offs_t offset, uint16_t mem_mask)
{
	if (!machine().side_effects_disabled())
		logerror("%06x: io_r %04x mask %04x\n", m_maincpu->pc(), offset << 1, mem_mask);

	// Reads of the boot overlay switch to the underlying RAM at the
	// port 0x200 read inside the relocation walk: all ROM-sourced
	// pipeline reads are done by then.
	if ((offset << 1) == 0x200 && m_boot_mirror && !machine().side_effects_disabled())
	{
		logerror("boot overlay reads -> RAM (0200 read)\n");
		m_maincpu->space(AS_PROGRAM).install_ram(0x00400, 0x0f7ff,
				reinterpret_cast<uint8_t *>(m_lowram.target()) + 0x400);
		m_maincpu->space(AS_PROGRAM).install_ram(0x10000, 0x103ff,
				reinterpret_cast<uint8_t *>(m_lowram.target()) + 0x10000);
		m_boot_mirror = false;
	}

	// idle values measured on the real machine (LCD model) with DEBUG:
	switch (offset << 1)
	{
	case 0x2a08: return 0x0044; // handshake status, bit7 = busy, measured idle
	case 0x2b02: return 0x00fe; // measured 0xfc idle; bit1 (ready to accept) forced high
	case 0x2e1e: return 0x00f4; // mode/config register, measured on real hardware:
	                            // 0xf4 in Wang mode, 0xfc in Industry Standard mode
	                            // (bit3 = IS mode); bit7=1 (display type, LCD) selects
	                            // the 0xde/0xbb constant set at POST
	}
	// reads of 0x2a00 must NOT return the 0xdb probe signature: the
	// optional device is absent on the reference machine too
	return 0xffff;
}

void wltc_state::io_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	logerror("%06x: io_w %04x = %04x mask %04x\n", m_maincpu->pc(), offset << 1, data, mem_mask);

	// Writing 1 to port 0x2b1e triggers a CPU reset and advances the
	// boot phase: the gate array swaps the INT 88h vector from the
	// phase-A hardware init entry (E000:0019, the relocating start with
	// the boot mirror) to the runtime AL-function dispatcher
	// (E000:0643, the value observed on a running machine). Only the
	// CPU resets: shadow RAM and the rest of the IVT survive.
	if ((offset << 1) == 0x2b1e && (data & 1))
	{
		logerror("cpu reset via 2b1e, INT88 vector -> phase B init\n");
		// vector 0x88 lives in the ROM-backed window: patch the backing
		// buffer directly (offset 0x04 + (0x88-0x80)*4)
		const int off = 0x04 + 8 * 4;
		m_ivt_seed_rom[off] = 0x19; m_ivt_seed_rom[off + 1] = 0x00;
		m_ivt_seed_rom[off + 2] = 0x00; m_ivt_seed_rom[off + 3] = 0xe0;
		m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero);
	}
}


void wltc_state::machine_reset()
{
	// The BIOS runs the E segment from RAM shadowed over the EPROMs: the
	// cold start patches its own dispatch stubs at 0xe0004+ and keeps
	// data and stacks in segment E35F. Preload the shadow from the
	// EPROMs; the F segment stays ROM for reads.
	memcpy(m_shadow, memregion("bios")->base(), 0x10000);
	m_fseg_logged.assign(0x8000, 0);

	// Boot overlay: reads come from the EPROM mirrored at 0x400, writes
	// go through to the RAM underneath (the ES=0 clear that zeroes the
	// BDA/IVT area is intentional - it initializes the low data area,
	// and its zeros must land). The mirror spans 0x400-0xf7ff plus the
	// spill at 0x10000-0x103ff (the vector-install path ends with
	// retf 1000:000D into mirrored code at ROM 0xfc0d); 0xf800-0xffff
	// stays RAM for the manufactured interrupt frames at 0000:FFEx.
	// Reads switch to the underlying RAM at the port 0x200 read inside
	// the relocation walk (see io_r).
	m_maincpu->space(AS_PROGRAM).install_rom(0x00400, 0x0f7ff, memregion("bios")->base());
	m_maincpu->space(AS_PROGRAM).install_writeonly(0x00400, 0x0f7ff,
			reinterpret_cast<uint8_t *>(m_lowram.target()) + 0x400);
	m_maincpu->space(AS_PROGRAM).install_rom(0x10000, 0x103ff, memregion("bios")->base() + 0xfc00);
	m_maincpu->space(AS_PROGRAM).install_writeonly(0x10000, 0x103ff,
			reinterpret_cast<uint8_t *>(m_lowram.target()) + 0x10000);
	m_boot_mirror = true;

	// The reset vector executes mov al,0x10 / int 0x88, so the gate
	// array must seed the interrupt vector table at power-on. Use the
	// vectors read from a running machine with DEBUG (D 0:200 L 40):
	// they are position-independent ROM entry points (8B/8D even match
	// the original ROM dispatch stubs), so the same block plausibly
	// gets seeded at reset. INT 88h = E000:0643, whose AL dispatcher
	// handles the cold start function AL=0x10.
	// INT 88h starts at the E000:0643 AL dispatcher (the constant the
	// live machine shows): phase A = function 0x10 programs the gate
	// array and hits port 0x2b1e, which resets the CPU with the vector
	// swapped to the phase-B hardware init entry E000:0019.
	// Every unspecialized vector points at the plain iret at E000:0384
	// (measured on real hardware: vector 0x63 - the target of the BRKN
	// trap hidden at F30E2 - holds exactly that), so stray traps bounce
	// instead of falling into zeroed memory.
	for (int i = 0; i < 256; i++)
		m_maincpu->space(AS_PROGRAM).write_dword(i * 4, 0xe0000384);
	static const uint32_t ivt_seed[16] = {
		0xe00000f1, 0xe0000148, 0xe0000177, 0xe00001d0,  // 80-83: IRQ0-3
		0xe000020d, 0xe0000263, 0xe00002b8, 0xe00002e3,  // 84-87: IRQ4-7
		0xe0000643, 0xe00000eb, 0xe0000384, 0xe25f0000,  // 88-8B
		0xe0000384, 0xe33201e4, 0xe0000384, 0xe0000384   // 8C-8F
	};
	for (int i = 0; i < 16; i++)
		m_maincpu->space(AS_PROGRAM).write_dword((0x80 + i) * 4, ivt_seed[i]);

	// The gate-array-provided vectors are ROM-backed: the intentional
	// ES=0 clear that zeroes the low data area at boot must not destroy
	// them (vector 0x63 still reads its seed on a running machine).
	// Model the 0x63 slot and the 0x80-0x8F block as small ROM windows.
	auto put32 = [this](int off, uint32_t v)
	{
		m_ivt_seed_rom[off] = v & 0xff;
		m_ivt_seed_rom[off + 1] = (v >> 8) & 0xff;
		m_ivt_seed_rom[off + 2] = (v >> 16) & 0xff;
		m_ivt_seed_rom[off + 3] = (v >> 24) & 0xff;
	};
	put32(0x00, 0xe0000384);                                 // vector 0x63
	for (int i = 0; i < 16; i++)
		put32(0x04 + i * 4, ivt_seed[i]);
	m_maincpu->space(AS_PROGRAM).install_rom(0x18c, 0x18f, &m_ivt_seed_rom[0x00]);
	m_maincpu->space(AS_PROGRAM).install_rom(0x200, 0x23f, &m_ivt_seed_rom[0x04]);
}


void wltc_state::mem_map(address_map &map)
{
	map(0x00000, 0x7ffff).ram().share("lowram");
	// CGA-style text buffer: the character output service runs with
	// DS=B800 and 80-column rows, attribute 0x07 - the standard IBM
	// text segment, kept by the BIOS as the source for the LCD refresh
	map(0xb8000, 0xbffff).ram().share("textram");
	// debug window: mirror of everything the BIOS writes into the F
	// segment (VRAM and video registers), readable from Lua for dumps
	map(0xa0000, 0xaffff).ram().share("fram");
	// E segment = shadow RAM (the BIOS patches its stubs and keeps its
	// data segment E35F there); F segment: reads from ROM (the real
	// machine preserves the EPROM content at runtime, verified live),
	// writes captured by the video-window handler.
	map(0xe0000, 0xeffff).ram().share("shadow");
	map(0xf0000, 0xfffff).rw(FUNC(wltc_state::fseg_r), FUNC(wltc_state::fseg_w));
}

void wltc_state::io_map(address_map &map)
{
	map(0x0000, 0xffff).rw(FUNC(wltc_state::io_r), FUNC(wltc_state::io_w));
}


static INPUT_PORTS_START( wltc )
INPUT_PORTS_END


void wltc_state::wltc(machine_config &config)
{
	V30(config, m_maincpu, 8'000'000); // NEC D70116C-8
	m_maincpu->set_addrmap(AS_PROGRAM, &wltc_state::mem_map);
	m_maincpu->set_addrmap(AS_IO, &wltc_state::io_map);
	m_maincpu->set_irq_acknowledge_callback(FUNC(wltc_state::irq_ack));

	TIMER(config, "tick").configure_periodic(FUNC(wltc_state::tick), attotime::from_hz(60));

	screen_device &screen(SCREEN(config, "screen", SCREEN_TYPE_LCD));
	screen.set_refresh_hz(60);
	screen.set_size(640, 400);
	screen.set_visarea(0, 639, 0, 399);
	screen.set_screen_update(FUNC(wltc_state::screen_update));
}


ROM_START( wltc )
	ROM_REGION16_LE( 0x20000, "bios", ROMREGION_ERASEFF )
	// BIOS 4.02.03, four 27C256 EPROMs, file names are the load addresses
	ROM_SYSTEM_BIOS( 0, "v40203", "BIOS 4.02.03" )
	ROMX_LOAD( "mye000.bin", 0x00000, 0x8000, CRC(5d715606) SHA1(08a0e1e4ebd7f979781c62d0b19c785966dd9d94), ROM_BIOS(0) )
	ROMX_LOAD( "mye800.bin", 0x08000, 0x8000, CRC(e25f4f8d) SHA1(f0bcaeb2120177ca023b5b8076852a1dfe4d5635), ROM_BIOS(0) )
	ROMX_LOAD( "myf000.bin", 0x10000, 0x8000, CRC(b0d23b9d) SHA1(c068b6c897b2ffea222ff7a38d3f39ac6fba54a4), ROM_BIOS(0) )
	ROMX_LOAD( "myf800.bin", 0x18000, 0x8000, CRC(0d458043) SHA1(5019b085fa245dd3461d8d8811d40064875b605b), ROM_BIOS(0) )
	// earlier revision (1985/1986 copyright), 64K at 0xf0000, even/odd EPROM pair
	ROM_SYSTEM_BIOS( 1, "v1986", "1986 BIOS" )
	ROMX_LOAD( "mainboard_a.bin", 0x10000, 0x8000, CRC(2ac9a03c) SHA1(29b5a0d5343f770628ed0089a2b8a87d518bb251), ROM_BIOS(1) | ROM_SKIP(1) )
	ROMX_LOAD( "mainboard_b.bin", 0x10001, 0x8000, CRC(f38aec77) SHA1(926b947a7411a2bfa6394b6b9dfd5118bcb27228), ROM_BIOS(1) | ROM_SKIP(1) )
ROM_END

} // anonymous namespace


//    YEAR  NAME  PARENT  COMPAT  MACHINE  INPUT  CLASS       INIT        COMPANY              FULLNAME               FLAGS
COMP( 1987, wltc, 0,      0,      wltc,    wltc,  wltc_state, empty_init, "Wang Laboratories", "Wang LapTop Computer", MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
