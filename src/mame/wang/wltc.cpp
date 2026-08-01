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
		m_screen(*this, "screen"),
		m_shadow(*this, "shadow"),
		m_lowram(*this, "lowram"),
		m_fram(*this, "fram"),
		m_textram(*this, "textram"),
		m_monoram(*this, "monoram")
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
	required_device<screen_device> m_screen;
	required_shared_ptr<uint16_t> m_shadow;
	required_shared_ptr<uint16_t> m_lowram;
	required_shared_ptr<uint16_t> m_fram;
	required_shared_ptr<uint16_t> m_textram;
	required_shared_ptr<uint16_t> m_monoram;
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
	bool m_tick_int = false;
	bool m_scsi_rst_irq = false;
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
		{
			// NMI wakes the hlt/inc-cw delay loops that run with IF
			// clear; once the POST turns interrupts on, the same tick
			// must also arrive as IRQ0, because the timer handler at
			// E00F1 is what pumps the console: it runs the device
			// poller at E119E, which sends the next queued character
			// out of port 0x2a08 through E0E65. Without it the printer
			// enqueues both boot messages and nothing ever drains them.
			m_maincpu->pulse_input_line(INPUT_LINE_NMI, attotime::zero);
			// straight to the CPU INT line: the POST has not programmed
			// the interrupt controller at this stage, so a request
			// through the PIC never reaches the CPU. The acknowledge
			// callback supplies the measured tick vector 0x80. Held off
			// until the video calibration runs - the tick handler
			// derails the early phases if it fires before the data
			// structures it walks exist.
			if (m_tick_int)
				m_maincpu->set_input_line(0, HOLD_LINE);
		}
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
	void pic_int_w(int state)
	{
		if (!m_legacy_bios)
			m_maincpu->set_input_line(0, state ? ASSERT_LINE : CLEAR_LINE);
	}
	void pit_out_w(int state)
	{
		if (m_legacy_bios && state)
		{
			logerror("PIT out edge -> INT @ %s\n", machine().time().as_string(6));
			m_maincpu->set_input_line(0, HOLD_LINE);
		}
	}
	IRQ_CALLBACK_MEMBER(irq_ack)
	{
		// once the BIOS has programmed the controller its own vector
		// wins; until then fall back to the vector the firmware family
		// expects - the 4.02.03 IVT dump shows the tick on 0x80, while
		// the 1986 POST installs its timer handler on vector 0x20
		// (F0ECB: [0080] = F000:0F67) before sti. The unprogrammed 8259
		// returns junk below either base.
		// the 1986 firmware never initialises the 8259 at all: its
		// timer interrupt is hard-vectored to 0x20
		if (m_legacy_bios)
			return 0x20;
		uint8_t const v = m_pic->acknowledge();
		return v >= 0x80 ? v : 0x80;
	}
};


uint32_t wltc_state::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	const rgb_t fg(0x30, 0x38, 0x20), bg(0xc8, 0xd4, 0x40);

	// The Wang-mode console composes text as character/attribute pairs
	// in the buffer at 0xb8000 - the tick-driven console pump delivers
	// the POST banner there - and the display micro renders it with the
	// glyph cache. Model that: when the text buffer holds anything,
	// render it as 80x25 cells of 8 scanlines, taking every other row
	// of the EPROM's 8x16 font. An empty buffer falls back to the
	// 640x200 bitmap in the banked memory behind the window at 0xf2000
	// (the diagnostic and the video test draw pixels directly).
	// The 1986 BIOS composes its POST messages in the mono buffer at
	// 0xb0000; the 4.02.03 console uses the colour buffer at 0xb8000.
	// Render whichever holds text.
	uint16_t const *tbuf = nullptr;
	for (int i = 0; i < 80 * 25 && !tbuf; i++)
		if ((m_textram[i] & 0xff) > 0x20 && (m_textram[i] & 0xff) < 0x7f)
			tbuf = m_textram;
	for (int i = 0; i < 80 * 25 && !tbuf; i++)
		if ((m_monoram[i] & 0xff) > 0x20 && (m_monoram[i] & 0xff) < 0x7f)
			tbuf = m_monoram;

	if (tbuf)
	{
		// the glyphs at ROM 0xf5d1: the BIOS's own font for 4.02.03,
		// and a stand-in for the LCD controller's internal character
		// generator on the 1986 hardware (whose BIOS only ever writes
		// character/attribute pairs - the controller renders them)
		uint8_t const *const font = memregion("bios")->base() + 0xf5d1;
		for (int y = cliprect.top(); y <= cliprect.bottom(); y++)
		{
			int const row = y >> 3, line = y & 7;
			uint32_t *dst = &bitmap.pix(y, cliprect.left());
			for (int x = cliprect.left(); x <= cliprect.right(); x++)
			{
				uint8_t const ch = tbuf[(row * 80) + (x >> 3)] & 0xff;
				uint8_t const bits = font[(ch << 4) + (line << 1) + 1];
				*dst++ = BIT(bits, 7 - (x & 7)) ? fg : bg;
			}
		}
		return 0;
	}

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
	// word offset already steps once per even port: offset & 3, not
	// (offset >> 1) & 3 - the double halving sent every control-word
	// write into counter 1 and made the POST timer test read a stale
	// counter 0, failing "too early" into error 57
	if ((offset << 1) >= 0x2400 && (offset << 1) <= 0x2407)
	{
		uint16_t const v = m_pit->read(offset & 3);
		if (m_legacy_bios && !machine().side_effects_disabled())
			logerror("PIT r[%d] = %02x @ %s\n", offset & 3, v,
					machine().time().as_string(6));
		return v;
	}

	// NCR 53C80 SCSI controller at 0x2700-0x270e, one register every
	// other address in the standard order (output data, initiator
	// command, mode, target command, current bus status, bus and
	// status, input data, reset parity). The internal Winchester and
	// the external floppy drive both live on this bus.
	if ((offset << 1) >= 0x2700 && (offset << 1) <= 0x270f)
	{
		// reading reset-parity/interrupt clears the pending interrupt
		if ((offset & 7) == 7 && !machine().side_effects_disabled())
			m_scsi_rst_irq = false;
		return m_scsi->read(offset & 7);
	}

	// Z8530 serial communications controller at 0x2500-0x2506, one
	// register every other address in the classic B/A control/data
	// order: the diagnostic initialises it with the textbook register
	// sequence (wr4 0x44, wr3 0xc0, wr5 0x60, wr11 0x55, wr12/13 baud,
	// wr14 0x12, wr9 0x80 channel reset).
	if ((offset << 1) >= 0x2500 && (offset << 1) <= 0x2507)
		return m_scc->dc_ab_r(offset & 3);

	// 8250-compatible serial port at the IBM-style byte addresses
	// 0x3f8-0x3ff: a port scan on a running machine reads the classic
	// idle values there (line status 0x60, interrupt ident 0x01).
	if ((offset << 1) >= 0x3f8 && (offset << 1) <= 0x3ff)
	{
		int const reg = (offset << 1) - 0x3f8 + ((mem_mask & 0x00ff) ? 0 : 1);
		return m_uart->ins8250_r(reg & 7);
	}

	// Configuration word. The 1986 BIOS forks on bit 13 right after
	// reset (F0012: test aw,2000h): set, it takes the burn-in path at
	// F07C4 that programs the LCD controller and deliberately powers
	// the machine down; clear, it runs the customer POST at F001A -
	// video RAM test, then the message writer that prints
	// "13 Power On Diagnostics" and the Rev 0.06 banner, matching the
	// photographs of a real machine powering up. Return the word with
	// bit 13 clear.
	// Bit 2 reflects the SCSI controller's pending interrupt: the POST
	// SCSI test at F0FAF asserts RST through the 5380's initiator
	// command register, expects this bit to rise, clears the interrupt
	// by reading the reset-parity register at 0x270e and expects it to
	// fall again.
	if ((offset << 1) == 0x2b0a)
		return 0xdffb | (m_scsi_rst_irq ? 0x0004 : 0);

	// Console status, read by the timer-tick device poller at E11C5 as
	// port (selector << 8) | 0x62 with the console's selector 0x10. The
	// poller rotates bit 0 up to bit 7 and treats it as offline/busy:
	// it has to read clear or the poller marks the console dead and
	// flushes its queue instead of transmitting.
	if ((offset << 1) == 0x1062)
		return 0x0000;

	// CGA/MDA display status register at the industry-standard addresses
	// 0x3da and 0x3ba. The POST calibrates against it at E12A7: with
	// interrupts off it counts down CX across one 0-to-1 and one 1-to-0
	// transition of bit 0, then demands that CH still reads 0xff, i.e.
	// that both edges arrived inside 256 iterations of a three
	// instruction loop. It retries five times and, when every attempt
	// overruns, calls int 0x88 function 0x0d - the service that restarts
	// the hardware init - and halts on the jmp at E12CF. So the machine
	// cannot get past video setup unless this bit really toggles with the
	// raster. Bit 0 is display enable (set while blanked), bit 3 is
	// vertical retrace, as on a CGA.
	if ((offset << 1) == 0x3da || (offset << 1) == 0x3ba)
	{
		// from here on the tick may arrive as a real interrupt: the
		// structures the handler walks are in place once the video
		// calibration runs
		if (!machine().side_effects_disabled())
			m_tick_int = true;

		uint8_t status = 0;
		bool const vblank = m_screen->vblank();
		if (vblank || m_screen->hblank())
			status |= 0x01;
		if (vblank)
			status |= 0x08;
		return status;
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
	// pipeline reads are done by then. (Dead in the current flow, which
	// never reads 0x200 - the live trigger is the 0x2d00 write in io_w.)
	if ((offset << 1) == 0x200 && m_boot_mirror && !machine().side_effects_disabled())
	{
		logerror("boot overlay reads -> RAM (0200 read)\n");
		m_maincpu->space(AS_PROGRAM).install_ram(0x00400, 0x0f7ff,
				reinterpret_cast<uint8_t *>(m_lowram.target()) + 0x400);
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

	// At the first write to 0x2d00 - the first I/O the phase-B init
	// issues once the cold start has copied itself into shadow RAM - the
	// low-memory overlay switches from the EPROM base (which the copy
	// and the E0070 constant pops needed) to the EPROM's data template
	// at ROM 0x35f0, the image of segment E35F. That one remapping is
	// what makes every phase-B read through DS=0x0040 resolve: the
	// console-ready flag at [0980] reads 0xff, the device-table pointer
	// at [00AF] reads 0x005b with its one console device, the function
	// table at [0A51] carries the entries the dispatcher jumps through.
	//
	// It must be an alias, not a copy: the init later runs the character
	// generator loader with an uninitialised source segment and sprays
	// writes across low memory - installed as RAM the template gets
	// wiped and the machine parks in the printer wait forever (measured;
	// so did installing it as RAM at any earlier point). As a read-only
	// window with writes passing to the RAM beneath, the flag survives
	// and the POST walks straight through both banner prints to the
	// boot path at 1000:000D. Whether the real gate array switches on
	// this exact port write is unproven - but the switch itself has to
	// exist, because the same bytes cannot serve as both copy source
	// and data area.
	if ((offset << 1) == 0x2d00 && m_boot_mirror && !machine().side_effects_disabled())
	{
		logerror("low overlay -> data template (2d00 write)\n");
		// The first 0x20 bytes stay on the EPROM base: they hold the
		// far-jump stub table (offsets 4/9/E/13 -> E25F/E332/E17A/E610)
		// that the vector-install path points interrupt vectors at, and
		// executing template data there is what derailed the first try.
		m_maincpu->space(AS_PROGRAM).install_rom(0x00420, 0x01a2f,
				memregion("bios")->base() + 0x35f0 + 0x20);
		m_boot_mirror = false;
	}

	// Map control for the F segment on the 1986 hardware. The customer
	// POST hops between segments through low-RAM trampolines that write
	// this port mid-flight: 0x1e (bit 0 clear) puts RAM behind F000 so
	// the POST, now running from the E alias, can pattern-test it
	// destructively; 0x1f (bit 0 set) puts the EPROM back and execution
	// returns to F000:0326 expecting its code there. Without the switch
	// the return lands in the test-wiped RAM and marches.
	if (m_legacy_bios && (offset << 1) == 0x2d02 && !machine().side_effects_disabled())
	{
		if (data & 1)
			m_maincpu->space(AS_PROGRAM).install_rom(0xf0000, 0xfffff,
					memregion("bios")->base() + 0x10000);
		else
			m_maincpu->space(AS_PROGRAM).install_ram(0xf0000, 0xfffff,
					reinterpret_cast<uint8_t *>(m_fram.target()));
	}

	if ((offset << 1) >= 0x2400 && (offset << 1) <= 0x2407)
	{
		if (m_legacy_bios && !machine().side_effects_disabled())
			logerror("PIT w[%d] = %02x @ %s\n", offset & 3, data & 0xff,
					machine().time().as_string(6));
		m_pit->write(offset & 3, data & 0xff);
		return;
	}

	if ((offset << 1) >= 0x2700 && (offset << 1) <= 0x270f)
	{
		// asserting RST through the initiator command register raises
		// the 5380 interrupt, visible in bit 2 of 0x2b0a
		if ((offset & 7) == 1 && (data & 0x80) && !machine().side_effects_disabled())
			m_scsi_rst_irq = true;
		m_scsi->write(offset & 7, data & 0xff);
		return;
	}

	if ((offset << 1) >= 0x2500 && (offset << 1) <= 0x2507)
	{
		m_scc->dc_ab_w(offset & 3, data & 0xff);
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

		// Tried and rejected: entering phase B as int 88h function 0x12
		// rather than by pointing the vector into the cold start. The
		// dispatcher gives 0x12 a preamble of its own - mov ds,cs:[2] /
		// mov sp,0x260 / mov ss,cs:[2], then the shared mov bp,sp at
		// E0677 - and driving the machine in through it does deliver
		// exactly the state the POST is missing: DS and SS come out E35F
		// and the console flag at [0980] finally reads 0xff. But
		// function 0x12 is not phase B. It lands at E0C3E, tests the
		// phase flag [0A50], restarts once through E147F, and on the
		// second pass runs a hundred-instruction routine at E0C48 and
		// returns with reti. The whole machine stops after 239
		// instructions. The full POST - video test, calibration,
		// printer - only runs when the vector points at the cold start,
		// so that entry is right and the data segment has to arrive some
		// other way.
		const int off = 0x88 * 4;
		m_ivt_seed_rom[off] = 0x19; m_ivt_seed_rom[off + 1] = 0x00;
		m_ivt_seed_rom[off + 2] = 0x00; m_ivt_seed_rom[off + 3] = 0xe0;

		// Tried and rejected: sending the CPU back to the reset vector
		// with the register file intact, on the theory that phase B
		// inherits the DS=E35F that phase A ends with and that this is
		// where the far call at E007F gets the data segment it has to
		// pass on. It makes no difference, because the cold start loads
		// DS itself - mov ax,0x40 / mov ds,ax at E005D, the source
		// segment for its own copy - before ever reaching the call.
		m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero);
	}
}


void wltc_state::machine_reset()
{
	m_fseg_logged.assign(0x8000, 0);
	m_tick_int = false;
	if (!m_vram)
		m_vram = std::make_unique<uint8_t[]>(0x20000);
	std::fill_n(&m_vram[0], 0x20000, 0);

	// The F EPROM carries the glyph cache pre-expanded behind the video
	// window: 0x2000-0x3fff holds exactly 256 characters at 32 bytes
	// each - the 8x16 font with every row doubled, the LCD's native
	// glyph format - which is precisely the window's 8K. On hardware
	// the window reads the EPROM until something writes over it, so the
	// character generator is simply there at power-on; the emulated
	// window read back RAM, which is why the POST rasterised its banner
	// with blank glyphs. Seed the first two banks with the EPROM
	// content.
	memcpy(&m_vram[0], memregion("bios")->base() + 0x10000 + 0x2000, 0x2000);

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
	uint8_t const *const bios = memregion("bios")->base();
	m_legacy_bios = (bios[0] == 0xff && bios[1] == 0xff);   // E half blank: 64K image
	if (m_legacy_bios)
	{
		// The customer POST migrates itself from segment F000 to E000
		// through a trampoline in low RAM: out 0x2d02, 0x1e, then a
		// far return to the same offset with CS=E000. The 64K EPROM
		// image therefore decodes in both segments; mirror it into the
		// E area.
		memcpy(m_shadow, memregion("bios")->base() + 0x10000, 0x10000);

		// And the reason it moves: segment F is shadow RAM on this
		// hardware. Once running from E, the POST pattern-tests
		// F000:0000-0FFF destructively (AAAA/5555 at E0225-E0240) and
		// fails onto the error/halt path if the readback comes from
		// EPROM. Back the whole segment with RAM preloaded from the
		// image.
		uint8_t *const fram = reinterpret_cast<uint8_t *>(m_fram.target());
		memcpy(fram, memregion("bios")->base() + 0x10000, 0x10000);
		m_maincpu->space(AS_PROGRAM).install_ram(0xf0000, 0xfffff, fram);
		return;
	}

	// The BIOS runs the E segment from RAM shadowed over the EPROMs: the
	// cold start patches its own dispatch stubs at 0xe0004+ and keeps
	// data and stacks in segment E35F. Preload the shadow from the
	// EPROMs; the F segment stays ROM for reads.
	memcpy(m_shadow, bios, 0x10000);

	// The data template carries runtime state frozen into the EPROM
	// image: the console descriptor at E35F:3A03 holds a pending
	// retransmit character 0x44 with nine retries left, and its ring
	// buffer at E35F:3B34 has a write index of 3 over three stale bytes.
	// On hardware these fields live in freshly initialised RAM; give
	// the shadow copy the same start.
	//
	// Where the screen text actually comes from, measured with a write
	// tap on the buffer: the status line - row 24, which holds the
	// " - BIOS 4.02.03" tail - is written directly by the driver's
	// character routine at E4E06, and the clear at E502B wipes rows
	// 0-23 only, sparing it: rows 0-23 are the scrollable screen, row
	// 24 the status line. "Wang LapTop Computer", the first message,
	// never reaches the buffer and never leaves port 0x2a08 either: its
	// delivery path (the tail of the printer at E14A6, past the wait)
	// is still unmapped.
	{
		uint8_t *const sh = reinterpret_cast<uint8_t *>(m_shadow.target());
		sh[0x35f0 + 0x3a03 + 0x16] = 0;              // pending char
		sh[0x35f0 + 0x3a03 + 0x17] = 0;              // retry counter
		for (int i = 2; i < 10; i++)                 // ring indexes
			sh[0x35f0 + 0x3b34 + i] = 0;
	}


	// Boot overlay: reads come from the EPROM mirrored at 0x400, writes
	// go through to the RAM underneath (the ES=0 clear that zeroes the
	// BDA/IVT area is intentional - it initializes the low data area,
	// and its zeros must land). The mirror spans 0x400-0xf7ff plus the
	// spill at 0x10000-0x103ff (the vector-install path ends with
	// retf 1000:000D into mirrored code at ROM 0xfc0d); 0xf800-0xffff
	// stays RAM for the manufactured interrupt frames at 0000:FFEx.
	// Reads switch to the underlying RAM at the port 0x200 read inside
	// the relocation walk (see io_r).
	//
	// The overlay is genuine hardware, not scaffolding: the cold start
	// reads 0x0fe0 bytes up from 0000:0401 through DS=0x0040 and copies
	// them over E000:0001-0fe0, taking a block of constants with it -
	// the pops at E0070-0073 come out as AX=09C0 BX=0611 CX=059C
	// DX=0200, exactly ROM 0x2c-0x33. Without the EPROM readable down
	// there the machine cannot load its own shadow.
	//
	// It does get dropped later - Wolfgang's machine reads 0040:0980 and
	// 0040:00A0 back as zeroed RAM under DOS - but nothing in the POST
	// depends on that any more, now that the BIOS data segment is known
	// to be E35F rather than 0x0040 (see the wait at E14A6 below).
	m_maincpu->space(AS_PROGRAM).install_rom(0x00400, 0x0f7ff, memregion("bios")->base());
	m_maincpu->space(AS_PROGRAM).install_writeonly(0x00400, 0x0f7ff,
			reinterpret_cast<uint8_t *>(m_lowram.target()) + 0x400);
	// The overlay wraps at 64K: physical 0x1000D reads ROM offset 0x0D,
	// not 0xFC0D. Two earlier revisions got this wrong - mirroring ROM
	// 0xfc00 there put the middle of the LCD font under the retf
	// 1000:000D at the end of the vector-install path (font bytes
	// happened to limp along for 445k instructions), and plain RAM put
	// zeros there. The real target is the stub table again: ROM 0x0D is
	// jcxz followed by jmp far E17A:01AD, the disk service entry - push
	// all, take the lock at [1128], pick one of two unit blocks at
	// [112D]/[114C]. That call is the boot read.
	m_maincpu->space(AS_PROGRAM).install_rom(0x10000, 0x103ff, memregion("bios")->base());
	m_maincpu->space(AS_PROGRAM).install_writeonly(0x10000, 0x103ff,
			reinterpret_cast<uint8_t *>(m_lowram.target()) + 0x10000);
	m_boot_mirror = true;

	// The BIOS builds dispatch stubs in its shadow RAM - the cold start
	// patches the ones at E000:0004 and up - and the entry to the init
	// segment is one of them. The ROM image holds a data table at
	// E4C2:0000-0062 (stack accounting says executing it leaves five
	// words behind, and a dump of a running machine shows a runtime
	// table there), while the code starts at E4C2:0063 and ends in a
	// near return. Both fit exactly one shape of stub:
	//
	//     E4C2:0000  call 0x0063     ; near, pushes 0x0003
	//     E4C2:0003  retf            ; pops the far frame of the caller
	//
	// so the far call at E000:007f lands on the stub, the near return
	// at the end of the init comes back to the retf, and the retf
	// returns to E000:0084 - every original instruction correct, no
	// patching of the ROM. What writes the stub on real hardware is
	// still unknown; installing it here stands in for that.
	//
	// The 4.00 image recovered from the system diskette (BIOS.SYS) calls
	// E4B0:0000 instead and holds an ordinary routine prologue there, so
	// it needs no stub: the workaround is specific to the 4.02.03 EPROM
	// layout and is keyed off that image's far-call operand.
	//
	// The stub gets the entry point right but not the entry state. The
	// body at E4C2:0063 works on a device descriptor in DS:SI and takes
	// BP-relative parameters (mov ds,[bp+0ch] at E4DF5, mov [bp+4],bx at
	// E4DCF), and it needs DS to be the BIOS data segment E35F - the
	// value the get-segment service returns for id 0x18, and the one the
	// other three callers of the POST printer load explicitly before
	// calling it (E035B: mov bx,18h / lcall E000:0009 / mov ds,ax).
	// Everything the POST reads resolves in that segment and only there:
	// the console-ready flag at [0980] is already 0xff, the device-table
	// pointer at [00AF] is 0x005b, and the table there holds one device
	// whose descriptor carries port selector 0x10 - the console, port
	// 0x2a08. The function table the phase-A dispatcher jumps through
	// pins the segment down: only DS=E35F puts 0x146a at [0A71], which
	// is where that jump actually goes.
	//
	// The cold start leaves DS as 0x0040, the source segment of its own
	// copy, and nothing between there and the far call sets it. Feeding
	// the routine DS=E35F and SI=0x3a03 by hand gets the POST to
	// rasterise its first line of text, so the rest of the machine is
	// close.
	//
	// Who establishes that state on real hardware is now known, and it
	// is not the ROM POST at all. The whole display bring-up - the BDA
	// clear at EEF25, the descriptor walk calling the setup at EF478,
	// the glyph load at EF572 - lives inside a MS-DOS device driver
	// module embedded in the EPROM at segment E718: canonical driver
	// headers (CLOCK$, COM1, LPT1) at E718:1EB0 with a thirteen-command
	// dispatch at E718:1F2C, a strategy/interrupt pair at E906E that
	// reads the DOS request header from ES:BX, and a relocatable entry
	// at EEC03 that computes its own load delta with sub ax,0x75b and
	// fixes up the stored segment constants - 0x075b being the segment
	// the boot loader puts BIOS.SYS at on a disk boot. The 4.00 BIOS.SYS
	// recovered from the system diskette is this same module as a file.
	//
	// So the banner is printed at DOS boot time, when the kernel calls
	// the console driver's INIT with a proper frame: DS comes from the
	// driver's relocated data-segment constant, not from anything the
	// cold start does. The ROM-resident cold start only initialises the
	// hardware and runs the video test, which is exactly as far as the
	// emulated machine currently gets - matching a real WLTC that fails
	// to load MSDOS320.SYS. Getting the text on screen without patching
	// therefore means booting DOS from a disk image, which needs the
	// floppy/SCSI path to work.
	//
	// It is not that E0084 is entered from somewhere else. Offset 0x0084
	// has no reference anywhere in the 128K of EPROM - no near call or
	// jump, no word in the data area, no far pointer - and walking the
	// flow from all twenty int 88h function entries never reaches it. It
	// is the return address of the far call at E007F and nothing else,
	// so this stub does hand control back to the right place.
	//
	// The routine the init calls at E4D46 shows the same missing state
	// from another side: E5904 loads the character generator with
	// mov ds,bx and reads glyphs from DS:SI, and the init never sets BX,
	// so in this flow it fills from a garbage segment and scatters its
	// writes across low memory. Entry state, again, not a missing
	// device.
	//
	// Where the glyphs are meant to live is now known, and it is not the
	// EPROM. The data template holds two character-generator segments at
	// E35F:0006 and E35F:000A - 0x1298 and 0x1398, so 0x12980 and
	// 0x13980 - and the display setup at EF478 picks between them on bit
	// 7 of port 0x2e1e, the display-type bit. From there EF572 copies a
	// kilobyte out of 1398:0000 into F000:FA6E, which is the 128-entry
	// eight-bytes-per-glyph table that the reverse lookup at E5751
	// searches, and EF4D0 and E5904 expand the same font into video
	// memory at F200:0000.
	//
	// Neither 0x1298 nor 0x1398 appears as an immediate anywhere in the
	// 128K, so nothing in the EPROM ever loads a font there: it arrives
	// from disk, which fits a BIOS whose own error text says it needs
	// MSDOS320.SYS. The EPROM does carry an 8x16 font of its own at ROM
	// 0xf5d1 - 'A', 'B' and '0' render correctly out of it - but no code
	// copies that to 0x13980 either. Until something does, the expanders
	// read empty RAM, which is why the panel fills with structure rather
	// than glyphs.
	//
	// A third shape, selectable with the config switch, enters the same
	// body with CS=E35F through a trampoline. Worth trying because the
	// module addresses its parameters CS-relative at offsets 0x13b-0x152
	// and E4C2:0000 is the same linear address as E35F:1630, which is
	// exactly where the BIOS data area ends. It derails, and the reason
	// is the more useful answer: with CS=E4C2 those offsets fall at
	// E4D5B-E4D72, inside the routine's own tail, so the cells are
	// instruction bytes and variables at once. The init stores the
	// display index and data ports into the immediates of its own code,
	// and E5959 reads one straight back with mov dx,cs:[0x13f] / out
	// dx,al. E4C2 is the right segment after all.
	{
		uint8_t *const shadow = reinterpret_cast<uint8_t *>(m_shadow.target());
		uint16_t const cfg = ioport("CONFIG")->read();
		if (shadow[0x80] == 0x00 && shadow[0x81] == 0x00
				&& shadow[0x82] == 0xc2 && shadow[0x83] == 0xe4)
		{
			if (cfg & 0x0004)
			{
				// E4C2:0000 -> jmp far E35F:1640
				shadow[0x4c20] = 0xea;
				shadow[0x4c21] = 0x40; shadow[0x4c22] = 0x16;
				shadow[0x4c23] = 0x5f; shadow[0x4c24] = 0xe3;
				// E35F:1640 (= 0xe4c30): call 0x1693, then retf
				shadow[0x4c30] = 0xe8;
				shadow[0x4c31] = 0x50; shadow[0x4c32] = 0x00;
				shadow[0x4c33] = 0xcb;
			}
			else if (!(cfg & 0x0002))
			{
				shadow[0x4c20] = 0xe8;  // call near
				shadow[0x4c21] = 0x60;  // 0x0003 + 0x0060 = 0x0063
				shadow[0x4c22] = 0x00;
				shadow[0x4c23] = 0xcb;  // retf
			}
		}
	}

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
	// 640K: the 1986 POST pattern-tests 0x00000-0x9ffff as one block
	// and reports "51 Memory Error" if any of it fails to read back
	map(0x00000, 0x9ffff).ram().share("lowram");
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

	// experiment switch: whether to install the stub at E4C2:0000 that
	// skips the init table and enters the code at 0x63, or to let the
	// far call from E007F run the table as instructions the way the ROM
	// image has it
	PORT_START("CONFIG")
	PORT_CONFNAME( 0x0006, 0x0000, "Init entry at E4C2:0000" )
	PORT_CONFSETTING(      0x0000, "stub to 0x63" )
	PORT_CONFSETTING(      0x0002, "run the table as code" )
	PORT_CONFSETTING(      0x0004, "enter the body with CS=E35F" )
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
	// 3.072 MHz: the POST timer test at F032B loads counter 0 with 2000
	// in mode 0 and demands the OUT pin rise between its 8th and 10th
	// polling attempt (~73us each as the V30 executes the delay loops) -
	// a window of roughly 580-730us that only a clock near 3 MHz
	// satisfies. 2000 / 3.072 MHz = 651us, attempt nine, and 3.072 MHz
	// is a standard crystal.
	// Calibrated against the POST's own timer tests, which poll the OUT
	// pin in a software loop and demand it rise between the 8th and
	// 10th attempt: counter 0 with a divisor of 2000, counter 1 - on a
	// quarter of the clock - with 500, its polling slowed further by
	// the timer ISR running in between. 2.7648 MHz (a standard
	// baud-rate crystal) lands counter 0 on attempt ten and counter 1
	// on attempt nine, both inside the window; 3.072 MHz leaves
	// counter 1 one attempt early.
	m_pit->set_clk<0>(2'764'800);
	m_pit->set_clk<1>(2'764'800 / 4);
	m_pit->set_clk<2>(2'764'800 / 4);   // tested identically to counter 1
	// counter 0 is the system tick on IRQ0, as the measured interrupt
	// mask (0xbc) and vector table (INT 80h = the tick ISR) imply
	m_pit->out_handler<0>().set(m_pic, FUNC(pic8259_device::ir0_w));
	// counters 1 and 2 interrupt through the gate array on the 1986
	// hardware (vectors 0x20/0x21, enabled by bits 0/1 of port 0x2202);
	// deliver their OUT edges straight to the CPU INT line there - the
	// 8259 is never initialised by that firmware
	m_pit->out_handler<1>().set(FUNC(wltc_state::pit_out_w));
	m_pit->out_handler<2>().set(FUNC(wltc_state::pit_out_w));

	PIC8259(config, m_pic);
	// through a gate: the unprogrammed 8259 answers every IR update
	// with INT low, and wired straight to the CPU that CLEAR_LINE races
	// the gate-array interrupts the 1986 firmware relies on - its timer
	// edge got erased before the CPU could take it
	m_pic->out_int_callback().set(FUNC(wltc_state::pic_int_w));

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
	// the panel is 9.5 inches wide, so the pixels are far from square.
	// Raw timings rather than a bare refresh rate: the POST calibrates
	// against the blanking intervals through the status register at
	// 0x3da, so the horizontal and vertical blanking periods have to
	// exist. CGA-like totals at 60 Hz put a scanline at 63.6us, which
	// the calibration loop at E12A7 samples roughly every 4us.
	screen.set_raw(12'576'000, 800, 0, 640, 262, 0, 200);
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
	// the 1986 BIOS only writes character/attribute pairs; the LCD
	// controller renders them from its own character generator, stood
	// in for here by the font half of the later EPROM set
	ROMX_LOAD( "mye800.bin", 0x08000, 0x8000, CRC(e25f4f8d) SHA1(f0bcaeb2120177ca023b5b8076852a1dfe4d5635), ROM_BIOS(1) )

	// BIOS 4.00 as shipped on the system diskette: the file is a shadow
	// image for segment E000 (cold start at 0x18, far call operand E4B0),
	// recovered with scantool/estrai_fat.py. It is 0x9061 bytes, so the
	// tail is filled from the 4.02.03 EPROMs.
	ROM_SYSTEM_BIOS( 2, "v400", "BIOS 4.00 (BIOS.SYS)" )
	ROMX_LOAD( "biossys400_a.bin", 0x00000, 0x8000, CRC(ddf4569f) SHA1(75b3cf71cbe7ff840b18ce0dbb04412550f4d79f), ROM_BIOS(2) )
	ROMX_LOAD( "biossys400_b.bin", 0x08000, 0x8000, CRC(7538a123) SHA1(ae92cd372dafc8b0aa90d8aa32e483544c0576ea), ROM_BIOS(2) )
	ROMX_LOAD( "myf000.bin", 0x10000, 0x8000, CRC(b0d23b9d) SHA1(c068b6c897b2ffea222ff7a38d3f39ac6fba54a4), ROM_BIOS(2) )
	ROMX_LOAD( "myf800.bin", 0x18000, 0x8000, CRC(0d458043) SHA1(5019b085fa245dd3461d8d8811d40064875b605b), ROM_BIOS(2) )
ROM_END

} // anonymous namespace


//    YEAR  NAME  PARENT  COMPAT  MACHINE  INPUT  CLASS       INIT        COMPANY              FULLNAME               FLAGS
COMP( 1987, wltc, 0,      0,      wltc,    wltc,  wltc_state, empty_init, "Wang Laboratories", "Wang LapTop Computer", MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
