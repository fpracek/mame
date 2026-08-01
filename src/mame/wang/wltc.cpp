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
#include "machine/am9517a.h"
#include "machine/ins8250.h"
#include "machine/pic8259.h"
#include "machine/pit8253.h"
#include "machine/ncr5380.h"
#include "machine/z80scc.h"
#include "bus/nscsi/devices.h"
#include "machine/timer.h"
#include "screen.h"


namespace {

class wltc_state : public driver_device
{
public:
	wltc_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_pit(*this, "pit"),
		m_dmac(*this, "dmac"),
		m_pic(*this, "pic"),
		m_uart(*this, "uart"),
		m_scc(*this, "scc"),
		m_scsi(*this, "scsi5380"),
		m_shadow(*this, "shadow"),
		m_lowram(*this, "lowram"),
		m_fram(*this, "fram")
	{ }

	void wltc(machine_config &config);

protected:
	virtual void machine_reset() override ATTR_COLD;

private:
	required_device<v30_device> m_maincpu;
	required_device<pit8254_device> m_pit;
	required_device<am9517a_device> m_dmac;
	required_device<pic8259_device> m_pic;
	required_device<ins8250_device> m_uart;
	required_device<scc8530_device> m_scc;
	required_device<ncr5380_device> m_scsi;
	required_shared_ptr<uint16_t> m_shadow;
	required_shared_ptr<uint16_t> m_lowram;
	required_shared_ptr<uint16_t> m_fram;
	bool m_boot_mirror = false;
	std::vector<uint8_t> m_fseg_logged;
	uint8_t m_ivt_seed_rom[0x240];

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

	// Interrupt sources: the real-hardware IVT dump shows the hardware
	// interrupts on vectors 0x80-0x87 (the D71059 at IBM-style 0x20/0x21
	// is programmed with vector base 0x80; measured IMR 0xbc = IRQ0
	// timer, IRQ1 keyboard, IRQ6 floppy enabled). Until the PIT/PIC
	// pair is properly emulated: periodic tick on IRQ0, and a keyboard
	// microcontroller reply interrupt on IRQ1 shortly after each
	// command byte written to port 0x2c1e.
	uint8_t m_irq_vector = 0x80;
	uint8_t m_kb_reply = 0;
	uint8_t m_index_sel = 0xff;
	uint8_t m_rtc[0x40];
	uint16_t m_unmapped_value = 0xffff;
	uint8_t m_vram_bank[2] = { 0, 0 };
	// video memory behind the 8K window at 0xf2000: sixteen banks, of
	// which the first two hold the 640x200 frame (640/8 * 200 = 16000
	// bytes, just under two banks)
	std::unique_ptr<uint8_t[]> m_vram;
	// Boot-time tick as NMI: the whole boot runs with IF clear (no sti
	// executed until the E0084 path), yet the hlt/inc-cw delay loops
	// must advance - only NMI wakes a halted V30 with interrupts off,
	// and the seeded default vector 2 (plain iret) is exactly enough.
	bool m_legacy_bios = false;
	TIMER_DEVICE_CALLBACK_MEMBER(tick)
	{
		// the 1986 BIOS sets up its own vectors and timer: no scaffolding
		if (!m_legacy_bios)
			m_maincpu->pulse_input_line(INPUT_LINE_NMI, attotime::zero);
	}
	TIMER_CALLBACK_MEMBER(kb_reply_cb)
	{
		// The event ISR at E000:9BD2 dispatches on the event code in AL
		// (and si,0xff / cmp 0xf / call cs:[si*2+0x2a32]); event 6 is
		// the keyboard handler at E98AB. Deliver the code in AL with
		// the interrupt, the way the gate array does on real hardware,
		// and raise the request on the interrupt controller line the
		// measured mask assigns to the keyboard.
		m_maincpu->set_state_int(NEC_AW, (m_maincpu->state_int(NEC_AW) & 0xff00) | 0x06);
		m_pic->ir1_w(1);
		m_pic->ir1_w(0);
	}
	emu_timer *m_kb_timer = nullptr;
	IRQ_CALLBACK_MEMBER(irq_ack)
	{
		// once the BIOS has programmed the controller its own vector
		// wins; until then fall back to the measured tick vector
		uint8_t const v = m_pic->acknowledge();
		return v ? v : 0x80;
	}
};


uint32_t wltc_state::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	// The LCD has no hardware text mode: the BIOS rasterises glyphs
	// into video memory itself (the diagnostic utility draws character
	// by character, advancing by a frame buffer row pitch), so the
	// screen is a plain 640x200 one-bit-per-pixel bitmap living in the
	// banked memory behind the window at 0xf2000. Yellow-green on dark,
	// like the real display.
	const rgb_t fg(0x30, 0x38, 0x20), bg(0xc8, 0xd4, 0x40);
	for (int y = cliprect.top(); y <= cliprect.bottom(); y++)
	{
		uint32_t *dst = &bitmap.pix(y, cliprect.left());
		for (int x = cliprect.left(); x <= cliprect.right(); x++)
		{
			uint8_t const b = m_vram[(y * 80) + (x >> 3)];
			*dst++ = BIT(b, 7 - (x & 7)) ? fg : bg;
		}
	}
	return 0;
}

uint16_t wltc_state::fseg_r(offs_t offset, uint16_t mem_mask)
{
	// The video memory window at 0xf2000-0xf3fff reads back what was
	// written, through the bank selected by the display control
	// register (the POST video RAM test depends on the readback);
	// everything else in the F segment reads the EPROM, as a running
	// machine does (verified live at F000:30E2).
	if (offset >= 0x1000 && offset < 0x2000)
	{
		uint32_t const base = (m_vram_bank[0] << 13) | ((offset - 0x1000) << 1);
		return m_vram[base] | (m_vram[base + 1] << 8);
	}
	return reinterpret_cast<const uint16_t *>(memregion("bios")->base() + 0x10000)[offset];
}

void wltc_state::fseg_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	COMBINE_DATA(&m_fram[offset]);
	if (offset >= 0x1000 && offset < 0x2000)
	{
		uint32_t const base = (m_vram_bank[0] << 13) | ((offset - 0x1000) << 1);
		if (ACCESSING_BITS_0_7)
			m_vram[base] = data & 0xff;
		if (ACCESSING_BITS_8_15)
			m_vram[base + 1] = (data >> 8) & 0xff;
	}
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

	// D71054 programmable interval timer (8254 clone) at 0x2400-0x2406,
	// one register every other address: the diagnostic utility programs
	// it with the classic control words (0x74 counter 1 mode 2, 0xb6
	// counter 2 mode 3) followed by a 16-bit divisor, LSB then MSB.
	if ((offset << 1) >= 0x2400 && (offset << 1) <= 0x2407)
		return m_pit->read((offset >> 1) & 3);

	// NCR 53C80 SCSI controller at 0x2700-0x270e, one register every
	// other address in the standard order (output data, initiator
	// command, mode, target command, current bus status, bus and
	// status, input data, reset parity). The internal Winchester and
	// the external floppy drive both live on this bus.
	if ((offset << 1) >= 0x2700 && (offset << 1) <= 0x270f)
		return m_scsi->read((offset >> 1) & 7);

	// Z8530 serial communications controller at 0x2500-0x2506, one
	// register every other address in the classic B/A control/data
	// order: the diagnostic initialises it with the textbook register
	// sequence (wr4 0x44, wr3 0xc0, wr5 0x60, wr11 0x55, wr12/13 baud,
	// wr14 0x12, wr9 0x80 channel reset).
	if ((offset << 1) >= 0x2500 && (offset << 1) <= 0x2507)
		return m_scc->dc_ab_r((offset >> 1) & 3);

	// 8250-compatible serial port at the IBM-style byte addresses
	// 0x3f8-0x3ff: a port scan on a running machine reads the classic
	// idle values there (line status 0x60, interrupt ident 0x01).
	if ((offset << 1) >= 0x3f8 && (offset << 1) <= 0x3ff)
	{
		int const reg = (offset << 1) - 0x3f8 + ((mem_mask & 0x00ff) ? 0 : 1);
		return m_uart->ins8250_r(reg & 7);
	}

	// D71059 interrupt controller (8259 clone), reachable at the
	// IBM-style byte addresses 0x20/0x21: a port scan on a running
	// machine reads a sensible mask there (0xbc = timer, keyboard and
	// floppy enabled) and the vector table shows it programmed with
	// vector base 0x80.
	if ((offset << 1) == 0x20)
		return m_pic->read((mem_mask & 0x00ff) ? 0 : 1);

	// DMA controller at 0x2300-0x230f, byte addressed (registers run
	// consecutively, odd addresses included - the diagnostic uses 0x230a
	// as the single mask register and 0x230f as the all-mask one).
	if ((offset << 1) >= 0x2300 && (offset << 1) <= 0x230f)
	{
		int const reg = (offset << 1) - 0x2300 + ((mem_mask & 0x00ff) ? 0 : 1);
		return m_dmac->read(reg & 0x0f);
	}

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
	case 0x204:  logerror("0204 stub hit\n");
	             return 0x0000; // boot-time device status, bit0 tested after read
	                            // (experiment: report clear; reads 0xff at DOS time)
	case 0x2a00:
		// Index/data device on 0x2c00 (index) / 0x2a00 (data): the gate
		// array clock and its battery-backed scratch registers. Index
		// 0x0a is a status register whose bit 7 the POST waits to
		// pulse; everything else reads back what was written, over the
		// power-on contents in m_rtc.
		if (m_index_sel == 0x0a)
			return (machine().time().as_ticks(120) & 1) ? 0x80 : 0x00;
		return m_rtc[m_index_sel & 0x3f];
	case 0x2a08: return 0x0044; // handshake status, bit7 = busy, measured idle
	case 0x2b02: return 0x00fe; // measured 0xfc idle; bit1 (ready to accept) forced high
	case 0x2e1e: return 0x00f4; // mode/config register, measured on real hardware:
	                            // 0xf4 in Wang mode, 0xfc in Industry Standard mode
	                            // (bit3 = IS mode); bit7=1 (display type, LCD) selects
	                            // the 0xde/0xbb constant set at POST
	}
	// reads of 0x2a00 must NOT return the 0xdb probe signature: the
	// optional device is absent on the reference machine too
	return m_unmapped_value;
}

void wltc_state::io_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	logerror("%06x: io_w %04x = %04x mask %04x\n", m_maincpu->pc(), offset << 1, data, mem_mask);

	if ((offset << 1) >= 0x2400 && (offset << 1) <= 0x2407)
	{
		m_pit->write((offset >> 1) & 3, data & 0xff);
		return;
	}

	if ((offset << 1) >= 0x2700 && (offset << 1) <= 0x270f)
	{
		m_scsi->write((offset >> 1) & 7, data & 0xff);
		return;
	}

	if ((offset << 1) >= 0x2500 && (offset << 1) <= 0x2507)
	{
		m_scc->dc_ab_w((offset >> 1) & 3, data & 0xff);
		return;
	}

	if ((offset << 1) >= 0x3f8 && (offset << 1) <= 0x3ff)
	{
		int const reg = (offset << 1) - 0x3f8;
		if (ACCESSING_BITS_0_7)
			m_uart->ins8250_w(reg & 7, data & 0xff);
		if (ACCESSING_BITS_8_15)
			m_uart->ins8250_w((reg + 1) & 7, (data >> 8) & 0xff);
		return;
	}

	if ((offset << 1) == 0x20)
	{
		if (ACCESSING_BITS_0_7)
			m_pic->write(0, data & 0xff);
		if (ACCESSING_BITS_8_15)
			m_pic->write(1, (data >> 8) & 0xff);
		return;
	}

	if ((offset << 1) >= 0x2300 && (offset << 1) <= 0x230f)
	{
		int const reg = (offset << 1) - 0x2300;
		if (ACCESSING_BITS_0_7)
			m_dmac->write(reg & 0x0f, data & 0xff);
		if (ACCESSING_BITS_8_15)
			m_dmac->write((reg + 1) & 0x0f, (data >> 8) & 0xff);
		return;
	}

	// Display control registers, one per screen: the low nibble selects
	// a video memory bank behind the window at 0xf2000 (the diagnostic
	// tests the banks one at a time through these ports, and the 1986
	// BIOS clears both at the end of its display init).
	if ((offset << 1) == 0x2d0a || (offset << 1) == 0x2d0c)
	{
		int const screen = ((offset << 1) == 0x2d0a) ? 0 : 1;
		if (m_vram_bank[screen] != (data & 0x0f))
			logerror("display %d: bank %x\n", screen, data & 0x0f);
		m_vram_bank[screen] = data & 0x0f;
	}

	// index register of the clock/scratch device, and its data port
	if ((offset << 1) == 0x2c00)
		m_index_sel = data & 0xff;
	if ((offset << 1) == 0x2a00)
		m_rtc[m_index_sel & 0x3f] = data & 0xff;

	// Keyboard microcontroller, as driven by the diagnostic utility:
	// 0x2c1e takes a command byte (0x2c10 gates it), 0x2a08 is the
	// status port that is read straight after a command and also takes
	// the sound commands 0x0b (tone) and 0x0c (click). A command is
	// answered with a reply interrupt on IRQ1 carrying event code 6.
	if ((offset << 1) == 0x2c1e)
	{
		m_kb_reply = 0xfa;
		if (!m_kb_timer)
			m_kb_timer = timer_alloc(FUNC(wltc_state::kb_reply_cb), this);
		m_kb_timer->adjust(attotime::from_usec(200));
		logerror("kb cmd %02x -> reply irq scheduled\n", data & 0xff);
	}
	if ((offset << 1) == 0x2a08)
	{
		switch (data & 0xff)
		{
		case 0x0b: logerror("beeper: tone\n"); break;
		case 0x0c: logerror("beeper: click\n"); break;
		}
	}

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
		// buffer directly
		const int off = 0x88 * 4;
		m_ivt_seed_rom[off] = 0x19; m_ivt_seed_rom[off + 1] = 0x00;
		m_ivt_seed_rom[off + 2] = 0x00; m_ivt_seed_rom[off + 3] = 0xe0;
		m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero);
	}
}


void wltc_state::machine_reset()
{
	m_fseg_logged.assign(0x8000, 0);
	if (!m_vram)
		m_vram = std::make_unique<uint8_t[]>(0x20000);
	std::fill_n(&m_vram[0], 0x20000, 0);

	// Power-on contents of the clock/scratch registers. The date and
	// time fields are range-checked by the POST (month 1-12 at 0x10,
	// hour 0-23 at 0x13, second 0-99 at 0x12); the values at 0x30-0x33
	// are the ones a running machine returns (measured with DEBUG).
	std::fill(std::begin(m_rtc), std::end(m_rtc), 0);
	m_rtc[0x10] = 0x01;  // month
	m_rtc[0x11] = 0x01;  // day
	m_unmapped_value = ioport("UNMAPPED")->read();
	m_rtc[0x30] = 0xbc;
	m_rtc[0x31] = 0xfb;
	m_rtc[0x32] = 0xad;
	m_rtc[0x33] = 0x2f;

	// The 1986 BIOS revision is a plain 64K image at 0xf0000 whose reset
	// vector is an ordinary far jump (ea aa 00 00 fc = jmp FC00:00AA),
	// not the int 0x88 convention: none of the gate-array scaffolding
	// below (shadow RAM, boot mirror, seeded IVT) applies to it.
	// Boot patches, applied to the ROM image before it is copied into
	// the shadow: the init routine at E4C2 starts at offset 0x63 (the
	// bytes before it are a data table, as the stack accounting and a
	// dump of a running machine both show), and its final return has
	// to be a far one, since it is reached through a far call - a near
	// return would keep CS at E4C2 and land mid-instruction at E4CA4.
	memregion("bios")->base()[0x0080] = 0x63;
	memregion("bios")->base()[0x4d56] = 0xcb;

	uint8_t const *const bios = memregion("bios")->base();
	m_legacy_bios = (bios[0] == 0xff && bios[1] == 0xff);   // E half blank: 64K image
	if (m_legacy_bios)
	{
		std::fill_n(&m_shadow[0], 0x8000, 0);
		return;
	}

	// The BIOS runs the E segment from RAM shadowed over the EPROMs: the
	// cold start patches its own dispatch stubs at 0xe0004+ and keeps
	// data and stacks in segment E35F. Preload the shadow from the
	// EPROMs; the F segment stays ROM for reads.
	memcpy(m_shadow, bios, 0x10000);

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

	// The bytes at E4C2:0000-0062 are data, not code: executing them
	// leaves five words on the stack (a push of ES, three of CS and one
	// of AX, with no matching pops - the rest of the boot balances
	// perfectly), which is exactly what makes the init chain return
	// into nowhere. The real code starts at E4C2:0063. Point the far
	// call at E007F there, which on hardware presumably comes from the
	// gate array supplying a different entry offset.
	// (patch the ROM image itself: the cold start copies its first page
	// back over the shadow from the boot mirror, so patching only the
	// shadow would be undone)

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
	// the whole seeded IVT (vectors 0x00-0x8F) is ROM-backed: the
	// intentional low-area clear must not destroy the trap vectors
	// (vector 0x63 provably survives on real hardware)
	for (int i = 0; i < 0x90; i++)
		put32(i * 4, 0xe0000384);
	for (int i = 0; i < 16; i++)
		put32((0x80 + i) * 4, ivt_seed[i]);
	m_maincpu->space(AS_PROGRAM).install_rom(0x000, 0x23f, &m_ivt_seed_rom[0x00]);
}


void wltc_state::mem_map(address_map &map)
{
	map(0x00000, 0x7ffff).ram().share("lowram");
	// CGA-style text buffer: the character output service runs with
	// DS=B800 and 80-column rows, attribute 0x07 - the standard IBM
	// text segment, kept by the BIOS as the source for the LCD refresh
	// The diagnostic utility tests a monochrome text buffer at 0xb0000
	// as well as the colour one at 0xb8000 - the machine answers on
	// both the MDA and CGA port sets on real hardware.
	map(0xb0000, 0xb7fff).ram().share("monoram");
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
	// experiment switch: what an undecoded port read returns. The 1986
	// BIOS picks its boot mode from configuration bits (bit 13 of the
	// word at 0x2b0a), and the maintenance manual describes diagnostic
	// jumpers selecting customer / repair-aid / burn-in modes, so the
	// idle bus level can change which path a BIOS takes.
	PORT_START("UNMAPPED")
	PORT_CONFNAME( 0xffff, 0xffff, "Undecoded port reads" )
	PORT_CONFSETTING(      0xffff, "0xffff (pulled up)" )
	PORT_CONFSETTING(      0x0000, "0x0000 (pulled down)" )
INPUT_PORTS_END


void wltc_state::wltc(machine_config &config)
{
	V30(config, m_maincpu, 8'000'000); // NEC D70116C-8
	m_maincpu->set_addrmap(AS_PROGRAM, &wltc_state::mem_map);
	m_maincpu->set_addrmap(AS_IO, &wltc_state::io_map);
	m_maincpu->set_irq_acknowledge_callback(FUNC(wltc_state::irq_ack));

	// NEC D71054, an 8254 clone, at 0x2400-0x2406 (one register every
	// other address); the counters are clocked from the CPU crystal
	// through the usual divider chain.
	// 8237-compatible DMA controller at 0x2300-0x230f (byte addressed):
	// the diagnostic programs channel addresses and counts there and
	// masks channels through registers 0x0a and 0x0f before starting a
	// transfer on the device at 0x2500.
	AM9517A(config, m_dmac, 8_MHz_XTAL / 2);
	m_dmac->in_memr_callback().set([this](offs_t offset) {
		return m_maincpu->space(AS_PROGRAM).read_byte(offset); });
	m_dmac->out_memw_callback().set([this](offs_t offset, uint8_t data) {
		m_maincpu->space(AS_PROGRAM).write_byte(offset, data); });

	PIT8254(config, m_pit);
	m_pit->set_clk<0>(8_MHz_XTAL / 4);
	m_pit->set_clk<1>(8_MHz_XTAL / 4);
	m_pit->set_clk<2>(8_MHz_XTAL / 4);
	// counter 0 is the system tick on IRQ0, as the measured interrupt
	// mask (0xbc) and vector table (INT 80h = the tick ISR) imply
	m_pit->out_handler<0>().set(m_pic, FUNC(pic8259_device::ir0_w));

	PIC8259(config, m_pic);
	m_pic->out_int_callback().set_inputline(m_maincpu, 0);

	// NCR 53C80 SCSI bus at 0x2700: the JVC Winchester sits on it, and
	// so does the external floppy drive
	nscsi_bus_device &scsibus(NSCSI_BUS(config, "scsi"));
	NSCSI_CONNECTOR(config, "scsi:0", default_scsi_devices, "harddisk");
	NSCSI_CONNECTOR(config, "scsi:1", default_scsi_devices, nullptr);
	NCR5380(config, m_scsi);
	scsibus.set_external_device(7, m_scsi);
	m_scsi->irq_handler().set(m_pic, FUNC(pic8259_device::ir5_w));

	// Z8530APS serial communications controller at 0x2500
	SCC8530(config, m_scc, 8_MHz_XTAL / 2);
	m_scc->out_int_callback().set(m_pic, FUNC(pic8259_device::ir3_w));

	// 8250-compatible UART at 0x3f8, with the usual 1.8432 MHz clock
	INS8250(config, m_uart, 1'843'200);
	m_uart->out_int_callback().set(m_pic, FUNC(pic8259_device::ir4_w));

	TIMER(config, "tick").configure_periodic(FUNC(wltc_state::tick), attotime::from_hz(1000));

	screen_device &screen(SCREEN(config, "screen", SCREEN_TYPE_LCD));
	// 640x200 (the Wang graphics mode from the maintenance manual);
	// the panel is 9.5 inches wide, so the pixels are far from square
	screen.set_refresh_hz(60);
	screen.set_size(640, 200);
	screen.set_visarea(0, 639, 0, 199);
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
