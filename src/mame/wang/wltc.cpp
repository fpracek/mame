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
#include "machine/ins8250.h"
#include "machine/pic8259.h"
#include "machine/pit8253.h"
#include "machine/ncr5380.h"
#include "machine/z80scc.h"
#include "bus/nscsi/devices.h"
#include "bus/nscsi/hd.h"
#include "machine/upd765.h"
#include "imagedev/floppy.h"
#include "formats/pc_dsk.h"
#include "machine/timer.h"
#include "screen.h"

#include <deque>


// The driver's own SCSI devices, defined at the bottom of this file
extern emu::detail::device_type_impl<nscsi_harddisk_device> const WANG_WINCHESTER;
extern emu::detail::device_type_impl<nscsi_full_device> const WANG_SCSI_FLOPPY;
extern emu::detail::device_type_impl<nscsi_full_device> const WANG_SCSI_FLOPPY35;

namespace {

// ---------------------------------------------------------------------
// The Winchester drive.
//
// The POST's SCSI Winchester Command Test resets the bus, issues a
// six-byte REQUEST SENSE with an allocation length of sixteen and
// compares the sixteen bytes that come back, one for one, against a
// reference image the ROM carries at 0000:08B9:
//
//     70 00 06 00 00 00 00 08 00 00 00 00 29 00 00 00
//
// That reference is a specification of the drive, written by the people
// who shipped it, and it says two things a generic MAME hard disk does
// not do. The sense key is UNIT ATTENTION (6) with additional sense code
// 0x29, "power on, reset or bus device reset occurred" - mandatory SCSI
// behaviour on the first command after a reset, which nscsi_full_device
// never reports. And the additional sense length is 8, not the 10 of the
// SCSI-2 eighteen-byte response: this is a SCSI-1 drive, and sixteen
// bytes is its whole extended sense. Byte 0 is 0x70, so the information
// field is not valid.
// ---------------------------------------------------------------------
class wang_winchester_device : public nscsi_harddisk_device
{
public:
	wang_winchester_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0)
		: nscsi_harddisk_device(mconfig, WANG_WINCHESTER, tag, owner, clock)
	{ }

protected:
	virtual void device_reset() override ATTR_COLD
	{
		nscsi_harddisk_device::device_reset();
		m_unit_attention = true;
		// After a transfer the start-up code waits at FCA2B for REQ and
		// ACK to fall before it will look at the status, and gives up
		// after forty ticks. Turning the bus round in no time at all
		// puts REQ back up in the same instant and it never sees it low,
		// so leave the bus quiet for a moment first.
		set_status_delay(attotime::from_usec(500));
		// The sector count in a read is a maximum, not a promise: the
		// host arms its DMA for as much as it wants and stops there.
		// Hold an unwanted byte a while, then let the rest go.
		set_data_phase_timeout(attotime::from_msec(10));
	}
	virtual void scsi_command() override
	{
		if (m_scsi_cmdbuf[0] != SC_REQUEST_SENSE)
		{
			// any other command clears the condition without reporting
			// it, which is all the POST needs of the rest of the set
			m_unit_attention = false;
			nscsi_harddisk_device::scsi_command();
			return;
		}

		std::fill(std::begin(m_scsi_sense_buffer), std::end(m_scsi_sense_buffer), 0);
		m_scsi_sense_buffer[0] = 0x70;
		m_scsi_sense_buffer[7] = 8;
		if (m_unit_attention)
		{
			m_scsi_sense_buffer[2] = SK_UNIT_ATTENTION;
			m_scsi_sense_buffer[12] = 0x29;
			m_unit_attention = false;
		}
		int const alloc = m_scsi_cmdbuf[4];
		scsi_data_in(SBUF_SENSE, std::min(16, alloc ? alloc : 4));
		scsi_status_complete(SS_GOOD);
	}

private:
	bool m_unit_attention = true;
};

// The external floppy drive. The system diskettes are 368640 bytes with
// a "Wang 3.0" boot record whose BPB reads 512 bytes a sector, nine
// sectors a track, two heads - 40 cylinders of double density.
static void wltc_floppies(device_slot_interface &device)
{
	device.option_add("525dd", FLOPPY_525_DD);
	device.option_add("35dd", FLOPPY_35_DD);
}

// ---------------------------------------------------------------------
// The external floppy drive, which hangs off the SCSI bus at ID 1 - the
// maintenance manual's remedies for "won't load diagnostic diskette" are
// to check the FDD's SCSI cable and its SCSI PCB, and the start-up code's
// drive A leg selects ID 1 and gives up when nothing answers.
//
// It does not speak plain SCSI. The ROM keeps its command blocks in a
// table at FC00:2500, eight bytes each - three of header, "00 <length>
// 00", then five of CDB with a sixth implied zero - and among the
// ordinary opcodes there are four the standard does not define:
//
//     01 00 00 00 00 00    rezero unit
//     00 00 00 00 00 00    test unit ready
//     03 00 00 00 0c 00    request sense, allocation 12
//     a2 00 03 00 00 00    vendor
//     a1 00 03 nn 00 00    vendor, nn = 00, 01, 02 or 03
//
// plus an a0 built elsewhere, seen on the bus as a0 00 05 30 07 00.
// Opcodes from 0xa0 up would normally carry a twelve byte CDB; Wang
// sends six.
//
// This is a listening post, not a drive: it answers what it can and logs
// every command block so the firmware can say what the rest mean.
// ---------------------------------------------------------------------
class wang_scsi_floppy_device : public nscsi_full_device
{
public:
	wang_scsi_floppy_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0)
		: wang_scsi_floppy_device(mconfig, WANG_SCSI_FLOPPY, tag, owner, clock)
	{ }

protected:
	wang_scsi_floppy_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
		: nscsi_full_device(mconfig, type, tag, owner, clock)
		, m_fdc(*this, "fdc")
		, m_drive(*this, "fdc:0")
	{ }

protected:
	virtual void device_reset() override ATTR_COLD
	{
		nscsi_full_device::device_reset();
		m_unit_attention = true;
		// No set_floppy(): the controller has its select lines connected,
		// so it picks the drive from the unit bits of the command.
		//
		// The drive spins from the moment the enclosure is powered -
		// nothing in the command set switches a motor - but not from in
		// here. floppy_image_device's own reset puts its motor line back
		// and declares itself not ready, and the order between sibling
		// resets is not ours to choose. A moment later is safely after
		// it, and leaves room for the two index revolutions the drive
		// counts before reporting ready: starting the motor just before
		// a command, as this used to, gave it none, and every command
		// came back with the not-ready bit set.
		m_spin->adjust(attotime::zero);
		// as on the Winchester: the start-up code wants a moment of quiet
		// bus between the data and the status
		set_status_delay(attotime::from_usec(500));
		// The sector count in a read is a maximum, not a promise: the
		// host arms its DMA for as much as it wants and stops there.
		// Hold an unwanted byte a while, then let the rest go.
		set_data_phase_timeout(attotime::from_msec(10));
	}

	// the vendor opcodes come as six byte blocks, not the twelve their
	// group code would normally mean
	virtual bool scsi_command_done(uint8_t command, uint8_t length) override
	{
		// Not the twelve their group code would normally mean. Byte 2
		// is a count FCE82 builds from the floppy command it forwards,
		// and the block runs one byte past it: a0 00 05 ... is six bytes
		// and a0 00 06 ... is seven. Fixing the frame at six truncated
		// the longer ones - the seek lost its cylinder and the specify
		// its second parameter.
		if (command >= 0xa0 && command <= 0xa3)
			return length >= 3 && length == m_scsi_cmdbuf[2] + 1;
		return nscsi_full_device::scsi_command_done(command, length);
	}

	virtual void scsi_command() override
	{
		std::string cdb;
		for (int i = 0; i < m_scsi_cmdsize; i++)
			cdb += util::string_format(" %02x", m_scsi_cmdbuf[i]);
		logerror("drive A comando:%s\n", cdb);

		// A read whose data phase the host walked away from - the normal
		// case, since the sector count is a maximum - leaves the sending
		// flag up, and the next command that answers out of the main
		// buffer gets served leftover sector bytes instead. The inquiry
		// came back as two bytes of the previous track.
		m_sending = false;

		switch (m_scsi_cmdbuf[0])
		{
		case SC_TEST_UNIT_READY:
			scsi_status_complete(SS_GOOD);
			return;

		case SC_REQUEST_SENSE:
		{
			// the same SCSI-1 shape the Winchester answers with
			std::fill(std::begin(m_scsi_sense_buffer), std::end(m_scsi_sense_buffer), 0);
			m_scsi_sense_buffer[0] = 0x70;
			m_scsi_sense_buffer[7] = 8;
			if (m_unit_attention)
			{
				m_scsi_sense_buffer[2] = SK_UNIT_ATTENTION;
				m_scsi_sense_buffer[12] = 0x29;
				m_unit_attention = false;
			}
			int const alloc = m_scsi_cmdbuf[4];
			scsi_data_in(SBUF_SENSE, std::min(16, alloc ? alloc : 4));
			scsi_status_complete(SS_GOOD);
			return;
		}

		case 0x01: // rezero unit
			m_unit_attention = false;
			scsi_status_complete(SS_GOOD);
			return;

		case SC_INQUIRY:
		{
			// Asked for once, with room for 36 bytes; a plain SCSI-1
			// answer, removable direct-access.
			// Wang puts the removable bit in byte 0, not byte 1: the
			// floppy driver's init at E17A8 walks the table the device
			// scan filled and marks a unit usable - the bit 7 the "set
			// media type" request checks - only for an entry whose
			// inquiry byte 0 is exactly 0x80, and the boot scan reads a
			// byte 0 of zero as a fixed disc. The low nibble of byte 1 is
			// the unit count less one.
			int const alloc = m_scsi_cmdbuf[4];
			std::fill_n(m_scsi_cmdbuf, 36, 0);
			m_scsi_cmdbuf[0] = 0x80;    // removable direct access
			m_scsi_cmdbuf[2] = 0x01;    // SCSI-1
			m_scsi_cmdbuf[3] = 0x01;    // and its inquiry format
			m_scsi_cmdbuf[4] = 31;      // additional length
			std::memcpy(&m_scsi_cmdbuf[8],  "WANG    ", 8);
			std::memcpy(&m_scsi_cmdbuf[16], "FDD             ", 16);
			std::memcpy(&m_scsi_cmdbuf[32], "1.00", 4);
			scsi_data_in(SBUF_MAIN, std::min<int>(36, alloc ? alloc : 4));
			scsi_status_complete(SS_GOOD);
			return;
		}

		case SC_MODE_SENSE_6:
		{
			// The start-up code asks for page 4 with room for 19 bytes and
			// reads the geometry straight out of the classic layout: at
			// E66BC it takes the big-endian word at offset 15/16 (the low
			// half of the three-byte cylinder count) minus two, and the
			// byte at 17 as the head count, then works out the capacity as
			// (cylinders - 2) * heads * 16. So the four-byte header and the
			// eight-byte block descriptor both have to be there, whatever
			// the page.
			int const alloc = m_scsi_cmdbuf[4];
			int const page = m_scsi_cmdbuf[2] & 0x3f;
			floppy_image_device *const fd = m_drive->get_device();
			int const heads = (fd && fd->get_sides()) ? fd->get_sides() : 2;
			int cylinders = 40, sectors = 9;
			if (fd)
				switch (fd->get_variant())
				{
				case floppy_image::DSQD: cylinders = 80; break;
				case floppy_image::DSHD: cylinders = 80; sectors = 15; break;
				default: break;
				}
			uint32_t const blocks = uint32_t(cylinders) * heads * sectors;

			uint8_t buf[36];
			std::fill_n(buf, sizeof(buf), 0);
			buf[1] = 0;                 // medium type
			buf[2] = (fd && fd->wpt_r()) ? 0x80 : 0x00;
			buf[3] = 8;                 // block descriptor length
			buf[5] = (blocks >> 16) & 0xff;
			buf[6] = (blocks >> 8) & 0xff;
			buf[7] = blocks & 0xff;
			buf[10] = 0x02;             // 512 byte blocks
			int len = 12;
			if (page == 0x04 || page == 0x3f)
			{
				buf[len + 0] = 0x04;
				buf[len + 1] = 0x16;
				buf[len + 3] = (cylinders >> 8) & 0xff;
				buf[len + 4] = cylinders & 0xff;
				buf[len + 5] = heads;
				len += 24;
			}
			else if (page == 0x01)
			{
				buf[len + 0] = 0x01;
				buf[len + 1] = 0x0a;
				len += 12;
			}
			else
			{
				scsi_status_complete(SS_CHECK_CONDITION);
				sense(false, SK_ILLEGAL_REQUEST);
				return;
			}
			buf[0] = len - 1;
			m_unit_attention = false;
			std::copy_n(buf, len, m_scsi_cmdbuf);
			scsi_data_in(SBUF_MAIN, std::min<int>(len, alloc ? alloc : 4));
			scsi_status_complete(SS_GOOD);
			return;
		}

		case 0xa0:
		{
			// A floppy-controller command, forwarded to the controller
			// inside the drive - which is what the drive is: a SCSI
			// front end on a 765. The command bytes start at 4 and run
			// for byte 2 minus three, and the results go back as an
			// extended message; see finish_command.
			m_unit_attention = false;
			int const len = m_scsi_cmdbuf[2] - 3;
			m_fdc_command = m_scsi_cmdbuf[4];
			m_data.clear();
			m_expected = 0;
			if (m_fdc_command == 0x46 || m_fdc_command == 0x66)
			{
				// a read: the sector number, the size code and the end
				// of track say how much is coming
				int const r = m_scsi_cmdbuf[8];
				int const sz = m_scsi_cmdbuf[9];
				int const eot = m_scsi_cmdbuf[10];
				if (eot >= r)
					m_expected = (eot - r + 1) * (128 << sz);
			}
			m_fdc->tc_w(false);
			for (int i = 0; i < len; i++)
				m_fdc->fifo_w(m_scsi_cmdbuf[4 + i]);
			// specify neither interrupts nor leaves a result phase, so
			// there is nothing to wait for
			if (m_fdc_command == 0x03)
				finish_command();
			// everything else answers when the controller interrupts;
			// pushing nothing here leaves the bus phase parked until it
			// does
			return;
		}

		case 0xa1:
		case 0xa2:
		case 0xa3:
			// still unread. Good status, no data.
			m_unit_attention = false;
			scsi_status_complete(SS_GOOD);
			return;
		}

		nscsi_full_device::scsi_command();
	}

	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD
	{
		UPD765A(config, m_fdc, 8'000'000, true, true);
		m_fdc->intrq_wr_callback().set(FUNC(wang_scsi_floppy_device::fdc_int_w));
		m_fdc->drq_wr_callback().set(FUNC(wang_scsi_floppy_device::fdc_drq_w));
		FLOPPY_CONNECTOR(config, "fdc:0", wltc_floppies, drive_default(),
				floppy_formats).enable_sound(true);
	}

	// which drive sits in the enclosure: the 3.5" variant overrides it
	virtual const char *drive_default() const { return "525dd"; }

	virtual void device_start() override ATTR_COLD
	{
		nscsi_full_device::device_start();
		m_drain = timer_alloc(FUNC(wang_scsi_floppy_device::drain), this);
		m_spin = timer_alloc(FUNC(wang_scsi_floppy_device::spin_up), this);
		save_item(NAME(m_unit_attention));
		save_item(NAME(m_fdc_command));
		save_item(NAME(m_expected));
		save_item(NAME(m_drq));
	}

private:
	static void floppy_formats(format_registration &fr) { fr.add_pc_formats(); }

	virtual uint8_t scsi_get_data(int id, int pos) override
	{
		if (m_sending && id == SBUF_MAIN)
		{
			if (pos + 1 >= int(m_data.size()))
				m_sending = false;
			return (pos < int(m_data.size())) ? m_data[pos] : 0;
		}
		return nscsi_full_device::scsi_get_data(id, pos);
	}

	void fdc_int_w(int state)
	{
		if (!state)
			return;
		// seek and recalibrate leave no result phase of their own: the
		// drive has to ask the controller what happened, which is what
		// a host driver would otherwise do for itself
		if (m_fdc_command == 0x07 || m_fdc_command == 0x0f)
			m_fdc->fifo_w(0x08);
		finish_command();
	}

	void fdc_drq_w(int state)
	{
		m_drq = state;
		// Not from in here: taking the byte out of the controller's FIFO
		// from inside its own state machine leaves it unable to finish
		// the sector. One scheduler slot away costs nothing at these
		// rates.
		if (state)
			m_drain->adjust(attotime::zero);
	}

	TIMER_CALLBACK_MEMBER(spin_up)
	{
		floppy_image_device *const f = m_drive->get_device();
		if (f)
			f->mon_w(0);
		logerror("drive A: unita' %s, supporto %s\n",
				f ? "presente" : "assente",
				(f && f->exists()) ? "inserito" : "assente");
	}

	TIMER_CALLBACK_MEMBER(drain)
	{
		while (m_drq && (!m_expected || int(m_data.size()) < m_expected))
			m_data.push_back(m_fdc->dma_r());
		if (m_expected && int(m_data.size()) >= m_expected)
			m_fdc->tc_w(true);
	}

	void finish_command()
	{
		uint8_t res[8];
		int n = 0;
		while (n < 8 && (m_fdc->msr_r() & 0xd0) == 0xd0)
			res[n++] = m_fdc->fifo_r();

		std::string txt;
		for (int i = 0; i < n; i++)
			txt += util::string_format(" %02x", res[i]);
		for (int i = 0; i < 6 && i < int(m_data.size()); i++)
			txt += util::string_format(" [%02x]", m_data[i]);
		logerror("drive A: comando %02x -> risultato%s, %d byte di dati\n",
				m_fdc_command, txt, int(m_data.size()));

		if (!m_data.empty())
		{
			// Sent straight out of this device's own buffer rather than
			// through the command one: a nine sector track is 4608 bytes
			// and that only holds 4096. Copying into it lost the last
			// 512 without a word, the host's DMA sat waiting for them,
			// and its transfer never finished - which is what stopped
			// the sixth read.
			m_sending = true;
			scsi_data_in(SBUF_MAIN, m_data.size());
		}
		// the extended message: the code, the count plus three, three
		// bytes the firmware steps over, then the results
		uint8_t msg[16];
		int const len = std::max(7, 5 + n);
		std::fill_n(msg, len, 0);
		msg[0] = 0x01;
		msg[1] = n + 3;
		for (int i = 0; i < n; i++)
			msg[5 + i] = res[i];
		scsi_status_complete_msg(SS_GOOD, msg, len);
	}

	required_device<upd765a_device> m_fdc;
	required_device<floppy_connector> m_drive;
	emu_timer *m_drain = nullptr;
	emu_timer *m_spin = nullptr;
	std::vector<uint8_t> m_data;
	int m_expected = 0;
	uint8_t m_fdc_command = 0;
	bool m_drq = false;
	bool m_sending = false;
	bool m_unit_attention = true;
};

class wltc_state : public driver_device
{
public:
	wltc_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_pit(*this, "pit"),
		m_pit_ibm(*this, "pit_ibm"),
		m_pic(*this, "pic"),
		m_uart(*this, "uart"),
		m_scc(*this, "scc"),
		m_scsi(*this, "scsi5380"),
		m_fdc(*this, "fdc"),
		m_floppy(*this, "fdc:1"),
		m_screen(*this, "screen"),
		m_shadow(*this, "shadow"),
		m_lowram(*this, "lowram"),
		m_fram(*this, "fram"),
		m_textram(*this, "textram"),
		m_monoram(*this, "monoram"),
		m_keys(*this, "KB%u", 0U),
		m_layout(*this, "LAYOUT")
	{ }

	void wltc(machine_config &config);

protected:
	virtual void machine_start() override ATTR_COLD
	{
		// timers must be allocated before save-state registration closes
		m_kb_timer = timer_alloc(FUNC(wltc_state::kb_reply_cb), this);
		m_rtc_timer = timer_alloc(FUNC(wltc_state::rtc_periodic), this);
		m_kb_poll = timer_alloc(FUNC(wltc_state::kb_poll_cb), this);
		m_dma_timer = timer_alloc(FUNC(wltc_state::dma_service_cb), this);
		m_layout_timer = timer_alloc(FUNC(wltc_state::layout_cb), this);
	}
	virtual void machine_reset() override ATTR_COLD;

private:
	required_device<v30_device> m_maincpu;
	required_device<pit8254_device> m_pit;
	required_device<pit8254_device> m_pit_ibm;
	required_device<pic8259_device> m_pic;
	required_device<ins8250_device> m_uart;
	required_device<scc8530_device> m_scc;
	required_device<ncr5380_device> m_scsi;
	required_device<upd765a_device> m_fdc;
	required_device<floppy_connector> m_floppy;
	required_device<screen_device> m_screen;
	required_shared_ptr<uint16_t> m_shadow;
	required_shared_ptr<uint16_t> m_lowram;
	required_shared_ptr<uint16_t> m_fram;
	required_shared_ptr<uint16_t> m_textram;
	required_shared_ptr<uint16_t> m_monoram;
	bool m_boot_mirror = false;
	// Video window select and F-segment mapping - see screen_update and
	// the 0x2d00 / 0x2d02 writes in io_w.
	uint8_t m_video_window = 0;
	// System-mode register 0x2e1e and display-source register 0x2e06,
	// measured with logged SYSMODE runs (w/m/w/c/w). Bit 3 of 0x2e1e is
	// the Wang / Industry Standard switch: the console rewrites the
	// register from its shadow on every cursor-blink tick (bit 4 is the
	// blink phase), base 0x04 in Wang mode and 0x0c in IS mode, and the
	// switch itself goes out 2e06=30 / out 2e0a=21 / out 2e1e=5c /
	// out 2e0a=29. Each mode owns a text buffer: Wang writes the F
	// segment, IS monochrome the buffer at 0xb0000, IS CGA the one at
	// 0xb8000 - and 0x2e06 tells the last two apart (0x30 mono, 0x20
	// CGA, written twice right after the CGA switch).
	uint8_t m_mode_2e1e = 0x04;
	uint8_t m_disp_2e06 = 0x30;
	// 0x2e0a carries the video mode number in its low bits (measured
	// with GW-BASIC against the port log): 1 = 80-column text, 2 =
	// 320x200 graphics, 4 = 40-column text. Bit 3 is dropped while a
	// switch is in progress and raised again at the end; bit 5 rides
	// along on every value the loaded system writes.
	uint8_t m_disp_2e0a = 0x29;
	// IBM aliases of the gate array's Industry Standard side. PC
	// software runs on this machine unchanged - the Digger port
	// reprograms the interval timer at 0x40-0x43, sends EOI to 0x20,
	// reads its keystrokes from 0x60 and drives the CGA registers
	// directly - and the real hardware answers on those addresses (the
	// port scan on the machine shows 0x20/0x21, 0x3B5, 0x3D5 alive).
	// CGA side: 0x3d8 mode select (bit 1 = graphics, bit 4 = 640-wide),
	// 0x3d9 colour select, 0x3d4/0x3d5 = 6845 index/data, all latched.
	uint8_t m_cga_mode = 0;
	uint8_t m_cga_color = 0;
	uint8_t m_crtc_idx = 0;
	uint8_t m_crtc_reg[32] = {};
	// 0x60 = scancode latch, 0x61 = system control latch. On real
	// hardware the keyboard micro's Industry Standard set delivers IBM
	// scancodes; the translation table below is the one XLAT keeps at
	// [EBC8:0655] (Wang keycode -> IBM scancode), read out of the
	// running machine - the arrows land on 48/50/4B/4D and the left
	// shift on 2A, exactly the codes the Digger port listens for.
	uint8_t m_port60 = 0;
	uint8_t m_port61 = 0;
	static constexpr uint8_t WANG2IBM[0x80] = {
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0x51, 0x45, 0x46, 0x4a, 0x2b, 0x48, 0x49, 0x4d,
		0x29, 0x37, 0x2d, 0x2a, 0x1c, 0x2b, 0x3a, 0x00,
		0x01, 0x1b, 0x00, 0x00, 0x1d, 0x38, 0x47, 0xff,
		0x50, 0x48, 0x34, 0x0e, 0x39, 0x4b, 0x4d, 0x0c,
		0x50, 0x4f, 0x53, 0x29, 0x52, 0x36, 0xff, 0x35,
		0x33, 0x32, 0x31, 0x30, 0x2f, 0x2e, 0x2d, 0x2c,
		0x4c, 0x4b, 0x00, 0x00, 0x1c, 0x28, 0x27, 0x26,
		0x25, 0x24, 0x23, 0x22, 0x21, 0x20, 0x1f, 0x1e,
		0x47, 0x51, 0x53, 0x4f, 0x2a, 0x1a, 0x19, 0x18,
		0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11, 0x10,
		0x4e, 0x49, 0x52, 0xff, 0x0d, 0x0b, 0x0a, 0x09,
		0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x0f,
		0xff, 0xff, 0x00, 0x3f, 0x3c, 0x44, 0x43, 0x42,
		0x41, 0x40, 0x3f, 0x3e, 0x3d, 0x3c, 0x3b, 0xff,
	};
	mutable uint32_t m_scr_sum[3] = { 0, 0, 0 };
	mutable uint32_t m_scr_when[3] = { 0, 0, 0 };
	mutable uint32_t m_scr_clock = 0;
	bool m_fseg_ram = false;
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
	bool m_scsi_irq = false;
	// The 5380's own interrupt line. In the 1986 gate-array scheme it
	// shares vector 0x23 with the DMA controller: the handler reads
	// 0x2b0a and splits on bit 2 (the 5380 interrupted) against bit 5
	// (terminal count), so the line has to show up in that word as well
	// as raise the request. The loaded system's SCSI service selects the
	// target, leaves the target command register on a phase that cannot
	// match and turns on DMA mode - it is arming the phase-mismatch
	// interrupt on purpose and reads the real phase in the handler. With
	// the line going nowhere it waits forever with its busy flag set,
	// which is where INT 88h function 0x27 was hanging.
	void scsi_int_w(int state)
	{
		m_scsi_irq = bool(state);
		if (!m_legacy_bios)
		{
			m_pic->ir5_w(state);
			return;
		}
		scsi_int_update();
	}
	// The line is a level, not an edge. The service raises it while its
	// own vector is masked - it masks 0x2202 bit 3 around the phase
	// change and opens it again on the way out - so the request has to
	// survive the mask and be delivered when the mask lifts, which is
	// what the gate array does and what the firmware counts on.
	void scsi_int_update()
	{
		if (m_pic_ready)
		{
			update_line3();
		}
		else if (m_legacy_bios && m_scsi_irq && !BIT(m_int_enable_2202, 3))
		{
			m_gate_vector = m_vector_base + 3;
			m_maincpu->set_input_line(0, HOLD_LINE);
		}
	}
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
		// Only the 4.02.03 firmware takes its event code this way. The
		// 1986 console interrupt is the gate array's own vector 0x82,
		// whose handler at E000:0177 reads the reason out of 0x2b02 and
		// touches no register of the code it interrupted - see the
		// keyboard poll, which raises it. Writing AL from out here while
		// that firmware runs corrupts whatever the interrupted code was
		// holding, and since this is armed on every 0x2c1e strobe - which
		// the console handler does for every byte of every line - it lands
		// in the middle of the banner: the m of "Computer" was arriving on
		// screen as 0x06, the event code itself.
		if (m_legacy_bios)
			return;

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
	// Sources as states rather than pulses.
	//
	// Every source in this driver raises its request and never lowers it:
	// with a pulse to the CPU that was enough, because nobody ever asked
	// whether the request was still there. An 8259 does ask - the BIOS
	// programs it level triggered - and a request that never falls comes
	// straight back after each acknowledge. That is what starved the POST
	// when the controller was given charge of the interrupts.
	//
	// So a source now says what its state IS, and the line follows in both
	// directions. While the old fixed-vector path is the one delivering,
	// only the rising edge matters and set_source behaves exactly as
	// before; the moment the 8259 takes over, the falling edge is there
	// too. m_source_state is what the two paths share.
	uint8_t m_source_state = 0;
	// Line 3 is shared by the SCSI controller, the floppy controller and the
	// end of a DMA transfer: the first two are states the devices hold, the
	// third a pulse, cleared when the handler reads the status word at 0x2b0a.
	bool m_dma_done = false;

	void update_line3()
	{
		set_source(3, m_scsi_irq || m_fdc_int || m_dma_done);
	}
	bool m_pic_ready = false;

	void pic_ir(int n, int state)
	{
		switch (n & 7)
		{
		case 0: m_pic->ir0_w(state); break;
		case 1: m_pic->ir1_w(state); break;
		case 2: m_pic->ir2_w(state); break;
		case 3: m_pic->ir3_w(state); break;
		case 4: m_pic->ir4_w(state); break;
		case 5: m_pic->ir5_w(state); break;
		case 6: m_pic->ir6_w(state); break;
		case 7: m_pic->ir7_w(state); break;
		}
	}

	void set_source(int n, bool state)
	{
		bool const was = BIT(m_source_state, n);
		// When the 8259 carries the interrupts, the two counters need their
		// request latched rather than followed: their outputs are levels that
		// stay high, and the controller is programmed level triggered, so a
		// followed line is acknowledged again every few microseconds for ever.
		// The real machine latches the edge in the gate array and drops the
		// request when the handler writes its end-of-interrupt port - 2c16 and
		// 2c10 for counter 1, 2c14 and 2c10 for counter 2, which is what the
		// POST's own handlers do. Measured: with this, the two runs stay
		// together nine milliseconds longer and get past the point where they
		// used to separate.
		if (m_pic_ready)
		{
			if (n < 2)
			{
				if (state && !was) pic_ir(n, 1);
			}
			else if (state != was)
				pic_ir(n, state);
			if (state) m_source_state |= 1 << n; else m_source_state &= ~(1 << n);
			return;
		}
		if (state)
			m_source_state |= 1 << n;
		else
			m_source_state &= ~(1 << n);
		if (m_pic_ready)
		{
			pic_ir(n, state);
			return;
		}
		// old path: the request is a pulse on the rising edge, and only
		// when the source is unmasked
		if (state && !was && m_legacy_bios && !BIT(m_int_enable_2202, n))
		{
			m_gate_vector = m_vector_base + n;
			m_maincpu->set_input_line(0, HOLD_LINE);
		}
	}
	// The 1986 gate-array interrupt scheme: sources are hard-vectored
	// (counter 1 -> 0x20, counter 2 -> 0x21, keyboard -> 0x25), each
	// enabled by an active-low bit in port 0x2202 (0/1/5). The POST
	// swaps the handler behind a vector and re-runs the same test
	// against the other counter, so answering a fixed vector sends the
	// second counter's interrupt to the default handler that its
	// cleanup just installed - measured as BP never getting the status
	// word. Latch the source's vector at the OUT edge.
	uint8_t m_int_enable_2202 = 0xff;
	uint8_t m_gate_vector = 0x20;
	// Vector base of the gate array's interrupt controller. Each source
	// answers with base + its own enable bit in 0x2202, and the base is
	// what 0x2c0c holds: 0x20 out of reset, which is where the POST puts
	// its handlers, and 0x80 once F139A writes it - which is where the
	// stage after the POST puts its own. That stage installs handlers on
	// 0x81, 0x82 and 0x83 - counter 2, console and DMA, the same three
	// bits it leaves open in 0x2202 - and points all 244 other vectors at
	// the routine that reports "***42 Invalid Interrupt - Restart".
	uint8_t m_vector_base = 0x20;
	uint8_t const *m_chargen = nullptr;
	emu_timer *m_rtc_timer = nullptr;

	// Wang DMA controller state - see dma_reg_w
	uint8_t m_dma_reg[16] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xff, 0xff };
	uint32_t m_dma_addr = 0;
	uint16_t m_dma_count = 0;
	bool m_dma_go = false;
	bool m_dma_tc = false;
	bool m_dma_drq = false;
	bool m_dma_recv = true;
	bool m_dma_floppy = false;

	// Keyboard/console microcontroller. It talks over a two-way byte
	// channel: status at 0x2b02 (bit 0 ready to accept a byte, bit 1 a
	// byte waiting to be read), data both ways at 0x2a08, acknowledge
	// at 0x2c10/0x2c1e, and an interrupt on vector 0x22 whenever either
	// status bit is set and bit 2 of 0x2202 enables it.
	//
	// The POST's keyboard test at F155B drives exactly that: it sends
	// 0x1d, 0x0a and 0x1e, each on a transmit-ready interrupt, then
	// expects two bytes back - 0x01 then 0x00 - each on a receive
	// interrupt, stores them as AH and AL and demands AX == 0x0100.
	// Keyboard. The codes came off real hardware with a DEBUG capture that
	// read 0x2b02/0x2a08 with the console interrupt masked: q 5f, r 5c,
	// m 39, a s d f 4f 4e 4d 4c, 1 2 3 4 6e 6d 6c 6b, RETURN 44,
	// EXECUTE 53, and the modifiers lshift 1b, rshift 35, ctrl 24,
	// alt 25 - which are four of the five codes the firmware filters out
	// as "not a key", the reason that filter list exists.
	//
	// The matrix is regular and positional: rows 0x10 apart, descending
	// by one to the right - 0x6f the digits, 0x5f the q row, 0x4f the a
	// row, 0x3f the z row. Every measured value falls on it (m is the
	// seventh of the bottom row, 0x3f-6; r the fourth of the q row,
	// 0x5f-3), so the rest of each row is filled in from the pattern.
	// Being positional it holds for QWERTZ as well: the label changes,
	// the code does not.
	// The function row is TWELVE keys: the direct table gives them the
	// contiguous internal numbers 0x44-0x4f (keycodes 0x73-0x7e), and
	// the machine reaches F13-F24 with Shift and F25-F36 with
	// Shift+Ctrl - which is why a PC keyboard maps naturally. The four
	// codes 0x6f-0x72 that used to be wired as "F13-F16" are OTHER
	// keys: XLAT maps 0x6f to the IBM Tab, and 0x70-0x72 carry the
	// special-function numbers 0x51/0x50/0x62 (HELP/ERASE/GL family,
	// attribution pending real-hardware checks).
	//
	// Keypad, attributed by measurement (7/8/2026): the number-to-
	// character map was read out by repointing a key over the numbers
	// 0x2b-0x3c and echoing - 0x2d..0x36 are keypad 0-9, 0x37-0x3a are
	// + - * /; inverting the direct table then names every keycode.
	static inline constexpr uint8_t KB_CODE[7][16] = {
		// digits, left to right. The '-' comes from the keypad key,
		// which carries the same character; the machine's own key at the
		// PC minus position is '=' (0x64, echo-verified).
		{ 0x6e, 0x6d, 0x6c, 0x6b, 0x6a, 0x69, 0x68, 0x67,
		  0x66, 0x65, 0x13, 0x64, 0x7e, 0x7d, 0x7c, 0x7b },
		// q row. The ']' is 0x21 - internal number 0x41, second of the
		// bracket family; 0x54, extrapolated from the row pattern,
		// types nothing.
		{ 0x5f, 0x5e, 0x5d, 0x5c, 0x5b, 0x5a, 0x59, 0x58,
		  0x57, 0x56, 0x55, 0x21, 0x7a, 0x79, 0x78, 0x77 },
		// a row, with RETURN at its right-hand end
		{ 0x4f, 0x4e, 0x4d, 0x4c, 0x4b, 0x4a, 0x49, 0x48,
		  0x47, 0x46, 0x45, 0x44, 0x76, 0x75, 0x74, 0x73 },
		// z row. Echo-verified: 0x38 ',', 0x2a '.', 0x37 '/'. The high
		// bits carry TAB (0x6f, from XLAT's scancode table) and the
		// three unattributed special keys 0x70-0x72, provisionally
		// labelled HELP?/ERASE?/GL? until the real machine names them.
		{ 0x3f, 0x3e, 0x3d, 0x3c, 0x3b, 0x3a, 0x39, 0x38,
		  0x2a, 0x37, 0, 0, 0x6f, 0x70, 0x71, 0x72 },
		// EXECUTE, the modifiers, and the navigation keys. CANCEL, the
		// space bar and the up-mover were found by injection and then
		// isolated one code at a time: from a submenu 0x12 on its own
		// returns to the main menu exactly as "Press CANCEL to return to
		// MENUS" promises, 0x2c steps the item selection down the way
		// the menu's "SPACE BAR - Item Select" does, and 0x2b steps it
		// up (most likely an arrow key).
		// 0x2b is the machine's BACKSPACE: it rubs out at the DOS
		// prompt, and the menus use it to step the selection up - the
		// same code serves both. 0x1a is backslash, 0x33 backtick.
		// The backslash key answers both the PC backslash and the key
		// left of the 1 (the Italian layout puts backslash there). The
		// machine's backtick key (0x33) is left out: repointing it in
		// the layout scripts froze the console output after one echo,
		// and no layout needs it badly enough to chase that down.
		// 0x1a is the machine's ESC - DOS echoes it as a backslash and
		// discards the input line, which is why it masqueraded as a
		// backslash key for a while. 0x1e is CAPS LOCK: it is the fifth
		// entry of the modifier-filter table, the one that had no name,
		// and typing after it settles the question - abc, press, ABC,
		// press again, abc. The real machine has the key, so the host's
		// Caps Lock must reach it (right Ctrl answers as well).
		// 0x54 is a dead entry in the stock translation table; every
		// layout script repoints it at internal number 0x42, the bracket
		// family's backslash cell, which no Wang key reaches on its own.
		// That gives the PC backslash key a real backslash (and pipe on
		// shift) while the backtick key keeps its backtick.
		{ 0x53, 0x1b, 0x35, 0x24, 0x25, 0x12, 0x2c, 0x2b,
		  0x1a, 0x1e, 0x33, 0x54, 0x29, 0x28, 0x2d, 0x2e },
		// keypad, fully attributed by measurement: 0-9, + - * /, Enter
		{ 0x34, 0x31, 0x30, 0x10, 0x41, 0x40, 0x17, 0x50,
		  0x15, 0x16, 0x60, 0x13, 0x14, 0x18, 0x1c, 0 },
		// navigation and editing: the home arrow sits next to the
		// cursor cluster (its number 0x5a adjoins the arrows' 0x56-59);
		// PRINT/INSERT/PREV/DELETE come from XLAT's own scancode table
		// (PrtSc/PgUp/PgDn/Del - INSERT and PREV carry exactly the
		// "Pg Up"/"Pg Dn" their caps show); NEXT's number 0x5c adjoins
		// DELETE's 0x5d.
		{ 0x26, 0x19, 0x61, 0x51, 0x62, 0x52, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0 },
	};
	required_ioport_array<7> m_keys;
	uint16_t m_kb_seen[7] = { 0, 0, 0, 0, 0, 0, 0 };
	int m_kb_divider = 0;
	// Press sends the code, release the same code with bit 7 set, and the
	// firmware only takes a release that matches the key it has down -
	// see E000:0ED6. Queue both through the same path the micro's replies
	// take: the poll below raises the console interrupt for them.
	void scan_keyboard()
	{
		for (int row = 0; row < 7; row++)
		{
			uint16_t const now = m_keys[row]->read();
			uint16_t const diff = now ^ m_kb_seen[row];
			if (!diff)
				continue;
			for (int bit = 0; bit < 16; bit++)
				if (BIT(diff, bit) && KB_CODE[row][bit])
					m_kb_replies.push_back(BIT(now, bit)
							? KB_CODE[row][bit]
							: uint8_t(KB_CODE[row][bit] | 0x80));
			m_kb_seen[row] = now;
		}
	}
	uint8_t m_kb_status = 0x01;
	bool m_kb_ready_again = false;
	uint8_t m_kb_rx = 0;
	std::deque<uint8_t> m_kb_replies;
	emu_timer *m_kb_poll = nullptr;

	// ------------------------------------------------------------------
	// National keyboard layouts, applied from the driver.
	//
	// The real machine gets its layout from software - national XLAT and
	// BIOS.SYS builds carry different translation tables - and the
	// preserved diskettes are the American build. The driver therefore
	// patches the loaded tables the way a national build would have
	// shipped them, on both sides of the machine:
	//
	//  - Wang mode: the BIOS translation block that [E358:0002] points
	//    at (direct key table + paired base/shift character lists),
	//    found by content and repatched every couple of seconds because
	//    SYSMODE reloads it;
	//  - Industry Standard mode: XLAT's own layout archive (the segment
	//    [EBC8:0002] points at), found by the signature of its ten alias
	//    pairs and patched cell by cell, every original value acting as
	//    a guard so a different XLAT build suspends the patches instead
	//    of corrupting them.
	//
	// The recipes are the ones validated byte-by-byte with the echo
	// tests of 6/8/2026 (IT 20/20, DE 24/24, in both modes).
	//
	// The choice is the LAYOUT machine configuration setting, read at
	// reset; the wltcit / wltcde clone machines only change its default
	// so the layout can be picked straight from the command line.
	required_ioport m_layout;
	emu_timer *m_layout_timer = nullptr;
	uint8_t m_layout_active = 0;
	std::vector<std::pair<uint32_t, uint8_t>> m_kb_recipe;
	uint32_t m_kb_recipe_base = 0;
	uint32_t m_is_arch_base = 0;
	bool m_is_arch_valido = false;
	bool m_is_arch_avvisato = false;

	struct patch_is { uint16_t off; uint8_t orig; uint8_t nuovo; };
	static constexpr uint8_t FIRMA_IS[16] = {
		0x0a, 0x9b, 0x80, 0xb5, 0x81, 0x1b, 0x90, 0x35,
		0x91, 0x1e, 0xa2, 0xa4, 0x83, 0x24, 0x93, 0xa5
	};
	static constexpr patch_is PATCH_IS_IT[] = {
		{ 0x0054, 0x4e, 0x59 }, { 0x0013, 0x53, 0x50 },   // ripuntamenti indice
		{ 0x042a, 0x18, 0x27 }, { 0x042b, 0x00, 0x3f },   // riga apostrofo/? (ex ESC)
		{ 0x0455, 0x60, 0x5c }, { 0x045c, 0x7e, 0x7c },   // barra e pipe
		{ 0x045b, 0x3b, 0x40 },                           // o' -> @
		{ 0x045a, 0x27, 0x23 }, { 0x0461, 0x22, 0xf8 },   // a' -> #, shift gradi
		{ 0x0458, 0x2f, 0x2d }, { 0x045f, 0x3f, 0x5f },   // trattino, _
		{ 0x047d, 0x7b, 0x5d },                           // shift e' -> ]
		{ 0x0475, 0x5d, 0x2b }, { 0x047b, 0x7d, 0x2a },   // + e *
		{ 0x0191, 0x40, 0x22 }, { 0x0187, 0x5e, 0x26 },   // shift 2 " / 6 &
		{ 0x0207, 0x26, 0x2f }, { 0x0209, 0x2a, 0x28 },   // shift 7 / 8
		{ 0x020b, 0x28, 0x29 }, { 0x0213, 0x29, 0x3d },   // shift 9 / 0
		{ 0x045d, 0x3c, 0x3b }, { 0x045e, 0x3e, 0x3a },   // shift , ; e . :
	};
	static constexpr patch_is PATCH_IS_DE[] = {
		{ 0x003f, 0x2c, 0x2b }, { 0x005a, 0x2b, 0x2c },   // y/z scambiati
		{ 0x0013, 0x53, 0x50 }, { 0x0054, 0x4e, 0x62 },   // esszett e cancelletto
		{ 0x042a, 0x18, 0xe1 }, { 0x042b, 0x00, 0x3f },   // riga esszett/?
		{ 0x0474, 0x5d, 0x23 }, { 0x047a, 0x7d, 0x27 },   // riga #/apostrofo
		{ 0x045b, 0x3b, 0x94 }, { 0x0462, 0x3a, 0x99 },   // o dieresi
		{ 0x045a, 0x27, 0x84 }, { 0x0461, 0x22, 0x8e },   // a dieresi
		{ 0x0477, 0x5b, 0x81 }, { 0x047d, 0x7b, 0x9a },   // u dieresi
		{ 0x0458, 0x2f, 0x2d }, { 0x045f, 0x3f, 0x5f },   // trattino, _
		{ 0x0475, 0x5d, 0x2b }, { 0x047b, 0x7d, 0x2a },   // + e *
		{ 0x0455, 0x60, 0x5e }, { 0x045c, 0x7e, 0xf8 },   // circonflesso, gradi
		{ 0x01fb, 0x3d, 0x5c }, { 0x020d, 0x2b, 0x7c },   // barra e pipe (accento)
		{ 0x0191, 0x40, 0x22 }, { 0x0187, 0x5e, 0x26 },   // shift 2 " / 6 &
		{ 0x0207, 0x26, 0x2f }, { 0x0209, 0x2a, 0x28 },   // shift 7 / 8
		{ 0x020b, 0x28, 0x29 }, { 0x0213, 0x29, 0x3d },   // shift 9 / 0
		{ 0x045d, 0x3c, 0x3b }, { 0x045e, 0x3e, 0x3a },   // shift , ; e . :
	};

	void costruisci_ricetta_wang(uint32_t B)
	{
		address_space &sp = m_maincpu->space(AS_PROGRAM);
		auto rd = [&sp](uint32_t a) { return sp.read_byte(a); };
		auto put = [this](uint32_t a, uint8_t v) { m_kb_recipe.emplace_back(a, v); };
		auto trova = [&](const char *pat) -> int
		{
			int const len = int(strlen(pat));
			for (int off = 0x100; off <= 0x300; off++)
			{
				bool ok = true;
				for (int i = 0; i < len && ok; i++)
					ok = rd(B + off + i) == uint8_t(pat[i]);
				if (ok)
					return off;
			}
			return -1;
		};
		m_kb_recipe.clear();
		int const l1 = trova("357890;',");
		int const l2 = trova("=1246/-+");
		int const lb = trova("[]\\`");
		if (l1 < 0 || l2 < 0)
			return;                       // liste non ancora caricate
		int const s1 = l1 + 10, s2 = l2 + 7;
		auto coppia = [&](int basi, int shiftate, char base_ch, uint8_t nuovo)
		{
			for (int i = 0; i < 10; i++)
				if (rd(B + basi + i) == uint8_t(base_ch))
					{ put(B + shiftate + i, nuovo); return; }
		};
		// cifre e punteggiatura, comuni ai due layout continentali
		coppia(l1, s1, '7', '/'); coppia(l1, s1, '8', '(');
		coppia(l1, s1, '9', ')'); coppia(l1, s1, '0', '=');
		coppia(l2, s2, '2', '"'); coppia(l2, s2, '6', '&');
		coppia(l1, s1, ',', ';'); coppia(l1, s1, '.', ':');
		if (m_layout_active == 1)         // italiana
		{
			uint8_t const n_apice = rd(B + 0x45), n_kpmeno = rd(B + 0x13), n_btick = rd(B + 0x33);
			put(B + 0x13, n_apice);       // tasto '? -> apostrofo
			put(B + 0x37, n_kpmeno);      // tasto trattino -> meno
			put(B + 0x45, n_btick);       // tasto a' -> cella del backtick (base #)
			put(B + 0x33, 0x42);          // tasto sx dell'1 -> cella backslash
			put(B + 0x54, 0x42);          // tasto u' -> idem
			coppia(l1, s1, '\'', '?');
			for (int i = 0; i < 10; i++)
				if (rd(B + l1 + i) == ';')
					put(B + l1 + i, '@'); // tasto o' -> @
			if (lb >= 0)
			{
				put(B + lb + 3, '#'); put(B + lb + 4, ']');
				put(B + lb + 1, '+'); put(B + lb + 5, '*');
				put(B + lb + 7, 0x5c);
			}
		}
		else                              // tedesca
		{
			uint8_t const ny = rd(B + 0x5a), nz = rd(B + 0x3f);
			put(B + 0x5a, nz); put(B + 0x3f, ny);   // QWERTZ
			for (int i = 0; i < 10; i++)
			{
				uint8_t const c = rd(B + l1 + i);
				if (c == ';')  { put(B + l1 + i, 0x94); put(B + s1 + i, 0x99); }
				if (c == '\'') { put(B + l1 + i, 0x84); put(B + s1 + i, 0x8e); }
			}
			put(B + 0x13, 0x24);          // tasto esszett -> cella "="
			put(B + 0x64, 0x2a);          // tasto accento -> cella "-"
			for (int i = 0; i < 8; i++)
			{
				uint8_t const c = rd(B + l2 + i);
				if (c == '=') { put(B + l2 + i, 0xe1); put(B + s2 + i, '?'); }
				if (c == '-') { put(B + l2 + i, 0x5c); put(B + s2 + i, 0x7c); }
				if (c == '/') { put(B + l2 + i, '-');  put(B + s2 + i, '_'); }
			}
			if (lb >= 0)
			{
				put(B + lb + 0, 0x81); put(B + lb + 4, 0x9a);
				put(B + lb + 1, '+');  put(B + lb + 5, '*');
				put(B + lb + 3, 0x5e); put(B + lb + 7, 0xf8);
				put(B + lb + 2, '#');  put(B + lb + 6, '\'');
			}
			put(B + 0x54, 0x42);          // tasto # del PC -> cella backslash
		}
	}

	TIMER_CALLBACK_MEMBER(layout_cb)
	{
		if (m_layout_active == 0)
			return;
		address_space &sp = m_maincpu->space(AS_PROGRAM);

		// lato Wang: tabella del BIOS via [E358:0002]
		uint16_t const seg = sp.read_word(0xe3582);
		if (seg != 0x0000 && seg != 0xffff)
		{
			uint32_t const B = uint32_t(seg) << 4;
			if (sp.read_byte(B + 0x5f) != 0x7f)   // sanita': il tasto q esiste
			{
				if (B != m_kb_recipe_base)
				{
					costruisci_ricetta_wang(B);
					if (!m_kb_recipe.empty())
					{
						m_kb_recipe_base = B;
						logerror("layout %s (Wang) applicato, tabella a %05X\n",
								m_layout_active == 1 ? "IT" : "DE", B);
					}
				}
				if (B == m_kb_recipe_base)
					for (auto const &p : m_kb_recipe)
						if (sp.read_byte(p.first) != p.second)
							sp.write_byte(p.first, p.second);
			}
		}

		// lato I.S.: archivio di XLAT, trovato per firma (coppie alias a +0x80)
		if (!m_is_arch_base)
		{
			for (uint32_t base = 0xe0000; base < 0xeff00 && !m_is_arch_base; base += 16)
			{
				bool ok = true;
				for (int i = 0; i < 16 && ok; i++)
					ok = sp.read_byte(base + 0x80 + i) == FIRMA_IS[i];
				if (ok)
					m_is_arch_base = base;
			}
			if (m_is_arch_base)
			{
				m_is_arch_valido = true;
				auto const *patch = (m_layout_active == 1) ? PATCH_IS_IT : PATCH_IS_DE;
				int const n = (m_layout_active == 1) ? int(std::size(PATCH_IS_IT)) : int(std::size(PATCH_IS_DE));
				for (int i = 0; i < n && m_is_arch_valido; i++)
				{
					uint8_t const v = sp.read_byte(m_is_arch_base + patch[i].off);
					if (v != patch[i].orig && v != patch[i].nuovo)
						m_is_arch_valido = false;
				}
				if (!m_is_arch_valido && !m_is_arch_avvisato)
				{
					m_is_arch_avvisato = true;
					logerror("layout (I.S.): archivio XLAT diverso, patch sospese\n");
				}
			}
		}
		if (m_is_arch_base && m_is_arch_valido)
		{
			auto const *patch = (m_layout_active == 1) ? PATCH_IS_IT : PATCH_IS_DE;
			int const n = (m_layout_active == 1) ? int(std::size(PATCH_IS_IT)) : int(std::size(PATCH_IS_DE));
			for (int i = 0; i < n; i++)
				if (sp.read_byte(m_is_arch_base + patch[i].off) != patch[i].nuovo)
					sp.write_byte(m_is_arch_base + patch[i].off, patch[i].nuovo);
		}
	}
	void kb_command(uint8_t cmd)
	{
		// the 0x1d/0x0a/0x1e sequence ends with an identify, answered
		// with 0x01 0x00
		if (cmd == 0x1e)
		{
			m_kb_replies.push_back(0x01);
			m_kb_replies.push_back(0x00);
		}
		// taking the byte makes the micro busy, and ready again a moment
		// later - that re-arming is what carries the test from one byte
		// to the next
		m_kb_status &= ~0x01;
		m_kb_ready_again = true;
	}
	TIMER_CALLBACK_MEMBER(kb_poll_cb)
	{
		if (!m_legacy_bios)
			return;

		// the poll runs every 200us; scanning the keys that often is
		// pointless, every 5ms is far quicker than anyone types
		if (++m_kb_divider >= 25)
		{
			m_kb_divider = 0;
			scan_keyboard();
		}
		// The request is a level: the micro asks as long as it has a byte
		// waiting or is ready to take one. It is ready out of reset, so
		// the keyboard test gets its first transmit interrupt with no
		// preceding write; after that it is only ready again once it has
		// actually taken a byte. An acknowledge that sends nothing - all
		// the later stage's handler ever does - leaves it quiet.
		if (!(m_kb_status & 0x02) && !m_kb_replies.empty())
		{
			m_kb_rx = m_kb_replies.front();
			m_kb_replies.pop_front();
			m_kb_status |= 0x02;
			// the gate array's IBM side latches the scancode at 0x60
			// (Industry Standard set: Wang keycode -> IBM scancode,
			// break keeps bit 7); command replies fall outside the
			// table and leave the latch alone
			uint8_t const scan = WANG2IBM[m_kb_rx & 0x7f];
			if (scan != 0x00 && scan != 0xff)
				m_port60 = scan | (m_kb_rx & 0x80);
		}
		else if (!(m_kb_status & 0x03) && m_kb_ready_again)
		{
			m_kb_status |= 0x01;
			m_kb_ready_again = false;
		}
		if (m_pic_ready)
		{
			// the request is the byte waiting to be read: bit 0, ready to
			// accept, never clears and so cannot hold a level
			set_source(2, (m_kb_status & 0x02) != 0);
		}
		else if ((m_kb_status & 0x03) && !BIT(m_int_enable_2202, 2))
		{
			m_gate_vector = m_vector_base + 2;
			m_maincpu->set_input_line(0, HOLD_LINE);
		}
	}
	TIMER_CALLBACK_MEMBER(rtc_periodic)
	{
		// register C picks up the periodic flag; the interrupt goes out
		// hard-vectored to 0x25 when its gate-array enable (bit 5 of
		// 0x2202, active low) is open
		m_rtc[0x0c] |= 0x41;
		logerror("RTC tick @ %s (2202=%02x)\n",
				machine().time().as_string(6), m_int_enable_2202);
		if (m_legacy_bios)
			set_source(5, true);
	}
	template <int N>
	void pit_out_w(int state)
	{
		// Counter 0 used to be wired straight to the controller's ir0 in the
		// machine configuration, which meant it drove that line under the
		// 1986 BIOS as well - where the controller is level triggered, and a
		// counter output that is high half the time floods it. Every attempt
		// at moving the sources onto the controller failed on that wire,
		// not on the code they changed: the log showed ir0 acknowledged
		// while this driver's own shadow said no line was up.
		if (N == 0)
		{
			// Counter 0 still reaches the controller's ir0 as it always did.
			// Cutting it off under the 1986 BIOS was meant to stop it flooding
			// the controller once the sources are handed over - harmless while
			// they are not, but it is the only behavioural change among the
			// recent commits, and the DOS command processor stopped starting
			// for the machine's owner right after them. Restored until that is
			// understood; the handover work will have to gate it another way.
			m_pic->ir0_w(state);
			return;
		}
		// the counter output is a level: the request follows it
		set_source(N - 1, state != 0);
	}
	// ---------------------------------------------------------------
	// Wang DMA controller (0x2300-0x230f)
	//
	// Not an 8237. F12C8 programs it for the SCSI read and gives the
	// whole layout away:
	//     0x2300 = 0x02, 0x2301 = 0x01   two mode bytes
	//     0x2302 = word, transfer count minus one
	//     0x2304 = word, physical address bits 15:0
	//     0x2306 = byte, physical address bits 19:16
	//     0x230a = command, bit 6 runs the channel and bits 2/3 are the
	//              direction: 0x44 reads into memory, 0x48 writes out of
	//              it (0x18 in the self test at F0375, which programs a
	//              count but never transfers)
	//     0x230b = read to acknowledge the terminal count
	//     0x230f = bit 1 masks the channel: F118D clears it to let the
	//              transfer run, the ISR sets it again on the way out
	// End of transfer raises vector 0x23, whose handler F1304 installs
	// at F000:1338 before clearing bit 3 of 0x2202 to unmask it. That
	// handler reads 0x2b0a to find out why it was called and sets bit 7
	// of BP when bit 5 says the count expired - which is the one thing
	// the wait loop at F1244 tests before declaring the transfer done.
	// ---------------------------------------------------------------
	void dma_reg_w(int reg, uint8_t data)
	{
		m_dma_reg[reg & 0x0f] = data;
		switch (reg & 0x0f)
		{
		case 0x02: m_dma_count = (m_dma_count & 0xff00) | data; break;
		case 0x03: m_dma_count = (m_dma_count & 0x00ff) | (data << 8); break;
		case 0x04: m_dma_addr = (m_dma_addr & 0xfff00) | data; break;
		case 0x05: m_dma_addr = (m_dma_addr & 0xf00ff) | (data << 8); break;
		case 0x06: m_dma_addr = (m_dma_addr & 0x0ffff) | ((data & 0x0f) << 16); break;
		case 0x0a:
			if (m_dma_floppy)
				m_fdc->tc_w(false);
			// Bit 6 is the one that runs the channel; bits 2 and 3 are
			// the direction, and taking bit 2 for the go bit only looked
			// right because everything measured until now was a read.
			// The loaded system's SCSI service sends its six-byte command
			// block out the same channel and writes 0x48 for it - bit 6
			// with the other direction bit - and with bit 2 as the go bit
			// that transfer never started, which is where the command
			// phase stalled. The self test at F0375 writes 0x18, programs
			// a count and never transfers: bit 6 clear, so still no run.
			m_dma_go = BIT(data, 6);
			// the disk driver writes the command before the address and
			// the count, so there is nothing worth printing here yet
			logerror("DMA command %02x\n", data);
			dma_service();
			break;
		case 0x0f:
			// the channel mask; a clear bit lets that channel run
			dma_service();
			break;
		}
	}
	// 0x230f masks channels, one bit each, and the bit for this one is
	// bit 0. The POST's driver hides that behind a literal - F118D just
	// ands 0xfd - but the disk driver computes it: FCE57 reads 0x2301,
	// the byte that names the channel, inverts it and ands that into
	// 0x230f. With 0x2301 = 1 it is bit 0 that gets cleared, and bit 0
	// is what both drivers leave low before starting a transfer (0x0c
	// from the POST, 0x0e from the disk driver).
	bool dma_armed() const { return m_dma_go && !BIT(m_dma_reg[0x0f], 0); }
	void dma_drq_w(int state)
	{
		m_dma_drq = state;
		// Serve the request outside the requesting device's own call
		// stack. The floppy controller raises this from inside its live
		// state machine, and taking the byte straight back out of its
		// FIFO from in there leaves that machine unable to finish the
		// sector - it read all eight and then sat on the last one, with
		// terminal count asserted and nothing happening.
		if (state)
			m_dma_timer->adjust(attotime::zero);
	}
	TIMER_CALLBACK_MEMBER(dma_service_cb) { dma_service(); }
	emu_timer *m_dma_timer = nullptr;
	void dma_service()
	{
		address_space &space = m_maincpu->space(AS_PROGRAM);
		while (dma_armed() && m_dma_drq)
		{
			if (m_dma_recv)
				space.write_byte(m_dma_addr,
						m_dma_floppy ? m_fdc->dma_r() : m_scsi->dma_r());
			else if (m_dma_floppy)
				m_fdc->dma_w(space.read_byte(m_dma_addr));
			else
				m_scsi->dma_w(space.read_byte(m_dma_addr));
			m_dma_addr = (m_dma_addr + 1) & 0xfffff;
			if (m_dma_count-- == 0)
				dma_complete();
		}
	}
	void dma_complete()
	{
		m_dma_go = false;
		m_dma_tc = true;
		if (m_dma_floppy)
		{
			// terminal count goes out after the last byte and stays out
			// until the next transfer is armed
			m_fdc->tc_w(true);
		}
		else
		{
			m_scsi->eop_w(1);
			m_scsi->eop_w(0);
		}
		logerror("DMA complete, end address %05x\n", m_dma_addr);
		// vector 0x23, unmasked by bit 3 of 0x2202 (active low)
		if (m_pic_ready)
		{
			m_dma_done = true;
			update_line3();
		}
		else if (m_legacy_bios && !BIT(m_int_enable_2202, 3))
		{
			m_gate_vector = m_vector_base + 3;
			m_maincpu->set_input_line(0, HOLD_LINE);
		}
	}
	static void floppy_formats(format_registration &fr)
	{
		fr.add_pc_formats();
	}
	// The controller's interrupt line shows up in bit 5 of 0x2b02: the
	// disk driver writes 0x2818 and then spins on that bit at FCF89
	// before reading the result bytes.
	void fdc_int_w(int state)
	{
		m_fdc_int = bool(state);
		// The floppy shares the DMA source: the vector 0x83 handler at
		// FD31E checks 0x2b0a for the DMA and the SCSI, and failing both
		// reads 0x2b02 and services the controller on bit 5.
		if (m_pic_ready)
		{
			update_line3();
		}
		else if (state && m_legacy_bios && !BIT(m_int_enable_2202, 3))
		{
			m_gate_vector = m_vector_base + 3;
			m_maincpu->set_input_line(0, HOLD_LINE);
		}
	}
	bool m_fdc_int = false;
	bool m_stat_2800 = true;
	IRQ_CALLBACK_MEMBER(irq_ack)
	{
		// once the BIOS has programmed the controller its own vector
		// wins; until then fall back to the vector the firmware family
		// expects - the 4.02.03 IVT dump shows the tick on 0x80, while
		// the 1986 POST installs its timer handler on vector 0x20
		// (F0ECB: [0080] = F000:0F67) before sti. The unprogrammed 8259
		// returns junk below either base.
		// the 1986 firmware never initialises the 8259 at all: its
		// interrupts carry the vector of whichever gate-array source
		// fired last
		if (m_legacy_bios)
			return m_gate_vector;
		// Before ICW1 the 8259 cannot deliver anything (measured: asking
		// MAME's device anyway returned 0xcd, whose IVT slot is still
		// zeroed RAM - the CPU fell to 0000:0000 and marched through the
		// vector table). Until the controller is programmed the gate
		// array's own scheme is in charge, and the seeded IVT puts the
		// tick on vector 0x80.
		if (!m_pic_inited)
			return 0x80;
		uint8_t const v = m_pic->acknowledge();
		return v >= 0x80 ? v : 0x80;
	}
	bool m_pic_inited = false;
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
	//
	// The display the loaded system settles on is neither of those. Its
	// driver at E4C5:0000 takes the video segment out of the channel
	// descriptor - 0xf000 for the display channel - selects the window
	// with 0x2d00 and writes through ES, into the RAM that 0x2d02 puts
	// behind the F segment. Its cells are the other way round from the
	// two above: the attribute first and the character second, so the
	// glyph is the high byte of each word. That buffer is where the Wang
	// menu draws, and rendering only the other two is what left the
	// screen frozen on the loader banners while the menu was running.
	// Which of them is live is decided by looking at all three and
	// scoring the cells, because the first-one-that-looks-like-text rule
	// this replaces was fooled by data: the diagnostic utility keeps its
	// message table in the F segment, and read as characters it looks
	// exactly like a screen. A cell only counts when the character is
	// printable AND its companion byte is a plausible attribute - on a
	// monochrome panel that means each nibble is 0 or 7, leaving the
	// bright and blink bits free. Message-table bytes fail that at once:
	// their "attributes" are just more text. Measured: the diagnostic
	// screen scores 890 cells with attributes 00 and 08, the boot banner
	// 194 with 07, and a table of prose scores nothing.
	auto const attr_ok = [] (uint8_t a)
	{
		return ((a & 0x07) == 0x00 || (a & 0x07) == 0x07)
			&& ((a & 0x70) == 0x00 || (a & 0x70) == 0x70);
	};
	auto const score = [&attr_ok] (uint16_t const *buf, int sh)
	{
		int n = 0;
		for (int i = 0; i < 80 * 25; i++)
		{
			uint8_t const ch = (buf[i] >> sh) & 0xff;
			uint8_t const at = (buf[i] >> (8 - sh)) & 0xff;
			if (ch > 0x20 && ch < 0x7f && attr_ok(at))
				n++;
		}
		return n;
	};

	uint16_t const *tbuf = nullptr;
	int shift = 0;
	// Scoring alone picks the buffer with the most valid cells, and that is
	// wrong as soon as two of them hold something: the loader's banner is
	// five full lines that stay in the mono buffer for ever, while a DOS
	// prompt is a dozen characters. The machine's owner sat watching the
	// banner while the emulated machine was already at its prompt.
	//
	// So the score only says what is not noise; which buffer is live is
	// decided by which one was written to last, and a cheap checksum per
	// frame says that. The choice is sticky, so a screen that stops
	// changing stays on screen.
	uint16_t const *const bufs[3] = { m_fram, m_textram, m_monoram };
	int const shifts[3] = { 8, 0, 0 };
	for (int k = 0; k < 3; k++)
	{
		if (k == 0 && !m_fseg_ram)
			continue;
		uint32_t h = 0;
		for (int i = 0; i < 80 * 25; i++)
			h = (h * 31) + bufs[k][i];
		if (h != m_scr_sum[k])
		{
			m_scr_sum[k] = h;
			m_scr_when[k] = ++m_scr_clock;
		}
	}

	// Industry Standard graphics: SCREEN 1 in GW-BASIC writes mode 2
	// into 0x2e0a, and the bitmap is the standard CGA layout in the
	// buffer at 0xb8000 - even scanlines at +0000, odd at +0x2000, 80
	// bytes a row, two bits a pixel MSB first (verified by decoding a
	// CIRCLE/PAINT drawing out of the dumped buffer). The panel shows
	// CGA colour as tones, so the four pixel values become four shades,
	// and the 320 pixels are doubled across the 640-dot line.
	// Graphics also turns on when PC software drives the CGA mode
	// register directly (3d8 bit 1) while the IS source is the CGA
	// side - that is how the Digger port switches, without ever going
	// through INT 10h (XLAT's INT 10h has no graphics modes at all:
	// the only two mode writes into the BIOS data area are 3 and 7).
	bool const cga_gfx = (m_mode_2e1e & 0x08) && m_disp_2e06 == 0x20 && BIT(m_cga_mode, 1);
	if (((m_mode_2e1e & 0x08) && (m_disp_2e0a & 0x07) == 0x02) || cga_gfx)
	{
		rgb_t const tone[4] = { bg, rgb_t(0x94, 0xa4, 0x3c), rgb_t(0x60, 0x70, 0x2c), fg };
		uint8_t const *const gfx = reinterpret_cast<uint8_t const *>(m_textram.target());
		bool const hires = cga_gfx && BIT(m_cga_mode, 4);
		for (int y = cliprect.top(); y <= cliprect.bottom(); y++)
		{
			uint8_t const *const src = gfx + ((y & 1) ? 0x2000 : 0) + (y >> 1) * 80;
			uint32_t *dst = &bitmap.pix(y, cliprect.left());
			for (int x = cliprect.left(); x <= cliprect.right(); x++)
			{
				if (hires)
					*dst++ = BIT(src[x >> 3], 7 - (x & 7)) ? fg : bg;
				else
				{
					int const gx = x >> 1;
					*dst++ = tone[(src[gx >> 2] >> (6 - 2 * (gx & 3))) & 3];
				}
			}
		}
		return 0;
	}

	// The mode register decides first: bit 3 of 0x2e1e picks Wang (the F
	// segment) against Industry Standard, and 0x2e06 splits IS between
	// the mono and the CGA buffer. Only when the register points at a
	// buffer with nothing readable in it (early POST, or a BIOS that
	// never touches the register, like the 4.02.03) does the
	// written-last heuristic below keep the choice.
	// (a bare prompt is a dozen cells, so the register's pick only needs
	// the buffer to be non-empty - the >40 threshold stays with the
	// heuristic, where it tells screens from data tables)
	int pick = (m_mode_2e1e & 0x08) ? ((m_disp_2e06 == 0x20) ? 1 : 2) : 0;
	if (pick == 0 && !m_fseg_ram)
		pick = -1;
	if (pick >= 0 && score(bufs[pick], shifts[pick]) > 0)
	{
		tbuf = bufs[pick];
		shift = shifts[pick];
	}
	else
	{
		uint32_t when = 0;
		for (int k = 0; k < 3; k++)
		{
			if (k == 0 && !m_fseg_ram)
				continue;
			if (score(bufs[k], shifts[k]) > 40 && m_scr_when[k] >= when)
			{
				when = m_scr_when[k];
				tbuf = bufs[k];
				shift = shifts[k];
			}
		}
	}

	if (tbuf)
	{
		// 25 rows of 8 scanlines over the 200-line panel, one glyph row
		// per scanline, taken from the character generator's doubled
		// cells (every other byte of a 32-byte glyph is the 8x8 font).
		// In IS 40-column text (SCREEN 0 in BASIC, mode 4 in 0x2e0a)
		// the row is 40 cells and every glyph pixel is doubled.
		bool const largo = (m_mode_2e1e & 0x08) && (m_disp_2e0a & 0x07) == 0x04
				&& tbuf == static_cast<uint16_t const *>(m_textram.target());
		int const cols = largo ? 40 : 80;
		for (int y = cliprect.top(); y <= cliprect.bottom(); y++)
		{
			int const row = y >> 3, line = y & 7;
			uint32_t *dst = &bitmap.pix(y, cliprect.left());
			for (int x = cliprect.left(); x <= cliprect.right(); x++)
			{
				int const cx = largo ? (x >> 1) : x;
				uint8_t const ch = (tbuf[(row * cols) + (cx >> 3)] >> shift) & 0xff;
				// The Wang-mode display renders the high codes with the
				// display micro's own font, which has never been dumped.
				// The EPROM font we substitute is CP437-flavoured up
				// there, so the menu marker (0x87) came out as a
				// c-cedilla and the entry-field fill (0x85) as an
				// accented a. On the real panel they are a small filled
				// square and a dot (verified by the machine's owner);
				// draw those two by hand until the micro's ROM turns up.
				uint8_t bits;
				if (shift && ch == 0x87)
					bits = (line >= 2 && line <= 5) ? 0x3c : 0x00;
				else if (shift && ch == 0x85)
					bits = (line == 3 || line == 4) ? 0x18 : 0x00;
				else
					bits = m_chargen[(ch << 5) + (line << 1)];
				// bit 0 is the leftmost pixel, not bit 7: 'L' reads
				// 0x06 on its upright rows and 'J' 0x78 on its top
				// row - the other way round every glyph is mirrored
				*dst++ = BIT(bits, cx & 7) ? fg : bg;
			}
		}
		// The text cursor. No hardware register in sight carries its
		// position (measured: 0x2a04/0x2a06 are never written while it
		// moves, no F-segment register changes, no attribute marking on
		// the cell), so the panel reads it where each side keeps it:
		//
		// - Industry Standard modes: the BIOS data area, like a real PC
		//   BIOS - page at 0040:0462, column and row at 0040:0450.
		// - Wang mode: the console's current-channel pointer at
		//   [E358:1280] leads to the display channel descriptor, which
		//   carries the video segment at +2 (0xF000, the guard), cursor
		//   row at +5, column at +6, and the visibility flag at +0x12
		//   bit 0 (0x0B with the cursor shown at the access screen and
		//   the DOS prompt, 0x0A at the menu - all measured by state
		//   diffing). Guards make a moved layout fail silent.
		//
		// The blink runs on machine time: bit 4 of the mode register
		// carries the console's blink phase but its writer goes quiet
		// at an idle DOS prompt (measured), so it cannot be the clock.
		int crow = -1, ccol = -1;
		if ((m_mode_2e1e & 0x08) && tbuf != static_cast<uint16_t const *>(m_fram.target()))
		{
			uint8_t const *const bda = reinterpret_cast<uint8_t const *>(m_lowram.target());
			int const page = bda[0x462] & 7;
			ccol = bda[0x450 + page * 2];
			crow = bda[0x451 + page * 2];
		}
		else if (tbuf == static_cast<uint16_t const *>(m_fram.target()))
		{
			uint8_t const *const sh = reinterpret_cast<uint8_t const *>(m_shadow.target());
			uint16_t const chan = sh[0x4800] | (sh[0x4801] << 8);
			if (chan >= 0x1000 && chan < 0xc000)
			{
				uint32_t const cs = 0x3580 + chan;
				if ((sh[cs + 2] | (sh[cs + 3] << 8)) == 0xf000 && (sh[cs + 0x12] & 1))
				{
					crow = sh[cs + 5];
					ccol = sh[cs + 6];
				}
			}
		}
		if (crow >= 0 && crow < 25 && ccol >= 0 && ccol < cols
				&& (int(machine().time().as_double() / 0.4) & 1))
		{
			int const x0 = ccol << (largo ? 4 : 3);
			int const larghezza = largo ? 16 : 8;
			for (int y = crow * 8 + 6; y <= crow * 8 + 7; y++)
			{
				if (y < cliprect.top() || y > cliprect.bottom())
					continue;
				for (int x = x0; x < x0 + larghezza; x++)
					if (x >= cliprect.left() && x <= cliprect.right())
						bitmap.pix(y, x) = fg;
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
		return m_pit->read(offset & 3);

	// IBM-side timer at 0x40-0x43, one register per byte address: the
	// Digger port latches and reads counter 0 in its sync loop - with
	// nothing here it read 0xFF forever and never reached its video
	// setup. NOT the Wang timer: wiring these ports to the system
	// D71054 lets the game reprogram the counter the whole machine's
	// tick hangs off (measured: the BDA tick raced to ~3000/s and the
	// game starved before its calibration loop), so the alias gets a
	// timer of its own at the PC's 1.19 MHz.
	if ((offset << 1) == 0x40 || (offset << 1) == 0x42)
	{
		int const base = ((offset << 1) == 0x40) ? 0 : 2;
		uint16_t r = 0;
		if (ACCESSING_BITS_0_7)
			r |= m_pit_ibm->read(base);
		if (ACCESSING_BITS_8_15)
			r |= m_pit_ibm->read(base + 1) << 8;
		return r;
	}
	if ((offset << 1) == 0x60)
		return m_port60 | (m_port61 << 8);
	if ((offset << 1) == 0x3d4)
		return m_crtc_idx | (m_crtc_reg[m_crtc_idx] << 8);
	if ((offset << 1) == 0x3d8)
		return m_cga_mode | (m_cga_color << 8);

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
		{
			uint8_t const v = m_scsi->read(offset & 7);
			if ((offset & 7) == 0 && !machine().side_effects_disabled())
				logerror("  iniziatore legge dato %02x @ %s\n",
						v, machine().time().as_string(6));
			return v;
		}
	}

	// Floppy controller: 0x2814 main status, 0x2816 data - see the
	// machine config
	if ((offset << 1) == 0x2814 && ACCESSING_BITS_0_7)
		return m_fdc->msr_r();
	if ((offset << 1) == 0x2816 && ACCESSING_BITS_0_7)
	{
		uint8_t const v = m_fdc->fifo_r();
		if (!machine().side_effects_disabled())
			logerror("FDC risultato %02x\n", v);
		return v;
	}

	// Z8530 serial communications controller at 0x2500-0x2506, one
	// register every other address. The POST names the four for us:
	// the channel B test at F13FF opens with wr9 = 0x40, channel B
	// reset, written to 0x2500, and the channel A test with wr9 = 0x80
	// to 0x2504; each then talks data on the address two above its
	// control. So the channel selects on bit 1 of the register number
	// and data/control on bit 0 - the ab_dc order, not the dc_ab one
	// this used to ask for, which had channel A's control register
	// answering as channel B's data.
	if ((offset << 1) >= 0x2500 && (offset << 1) <= 0x2507)
		return m_scc->ab_dc_r(offset & 3);

	// 8250-compatible serial port at the IBM-style byte addresses
	// 0x3f8-0x3ff: a port scan on a running machine reads the classic
	// idle values there (line status 0x60, interrupt ident 0x01).
	if ((offset << 1) >= 0x3f8 && (offset << 1) <= 0x3ff)
	{
		int const reg = (offset << 1) - 0x3f8 + ((mem_mask & 0x00ff) ? 0 : 1);
		return m_uart->ins8250_r(reg & 7);
	}

	// Gate-array interrupt enable register, active low, read-modify-
	// written by the 1986 POST (bit 0 counter 1, bit 1 counter 2,
	// bit 5 keyboard)
	if ((offset << 1) == 0x2202)
		return 0xff00 | m_int_enable_2202;

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
	// fall again. Bits 7 and 6 are the counter 1 / counter 2 interrupt
	// flags the timer ISRs sample into BP. Bits 12 and 11 are error
	// latches the RTC test requires low - either one high sends it to
	// the failure message before the tick counting even starts.
	// Bit 5 is the DMA terminal-count flag. The vector 0x23 handler at
	// F1338 reads this word and splits on it: bit 2 set means the 5380
	// interrupted (it sets BP bit 4 and leaves), bit 5 set means the
	// transfer finished (BP bit 7, which is the only thing the wait loop
	// at F1244 ever tests). It has to be an event flag, not the constant
	// it used to be here, or every SCSI interrupt would also report a
	// completed transfer.
	if ((offset << 1) == 0x2b0a && m_pic_ready && m_dma_done
			&& !machine().side_effects_disabled())
	{
		m_dma_done = false;
		update_line3();
	}

	if ((offset << 1) == 0x2b0a)
		return 0xc7db | ((m_scsi_rst_irq || m_scsi_irq) ? 0x0004 : 0) | (m_dma_tc ? 0x0020 : 0);

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

	// Wang DMA controller at 0x2300-0x230f (see io_w for the layout)
	if ((offset << 1) >= 0x2300 && (offset << 1) <= 0x230f)
	{
		switch (offset << 1)
		{
		case 0x2302: return m_dma_count;
		case 0x2304: return m_dma_addr & 0xffff;
		case 0x2306: return (m_dma_addr >> 16) & 0x0f;
		case 0x230a:
			// reading 0x230b acknowledges the terminal count - the ISR
			// does it on the way out, after re-masking the channel
			if (ACCESSING_BITS_8_15 && !machine().side_effects_disabled())
				m_dma_tc = false;
			break;
		}
		int const reg = (offset << 1) - 0x2300;
		return m_dma_reg[reg] | (m_dma_reg[reg + 1] << 8);
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
		{
			// Bit 7 is UIP, and on the real part it is high for about two
			// milliseconds once a second, just before the update - not the
			// square wave this used to be. The POST only needs to see it
			// pulse, but the clock reader at E1639 selects this register
			// and spins until the bit is clear before every field it
			// takes, eight of them per reading: against a bit that is high
			// half the time at 120Hz a reading cannot finish inside a
			// tick, and the stage that comes up off the system diskette
			// spent the whole of its life in that loop.
			uint64_t const us = machine().time().as_ticks(1'000'000) % 1'000'000;
			return (us >= 998'000) ? 0x80 : 0x00;
		}
		// register C, MC146818-style: interrupt flags, cleared by the
		// read. The keyboard/RTC ISR counts a periodic interrupt as
		// genuine only when bits 6 and 0 are both set.
		if (m_index_sel == 0x0c && !machine().side_effects_disabled())
		{
			uint8_t const v = m_rtc[0x0c];
			m_rtc[0x0c] = 0;
			// reading register C clears the flags, and with them the request
			set_source(5, false);
			logerror("RTC regC read = %02x @ %s\n", v, machine().time().as_string(6));
			return v;
		}
		return m_rtc[m_index_sel & 0x3f];
	case 0x2a08:
		// data from the keyboard/console micro; on the 1986 hardware
		// reading it takes the byte and clears the receive flag
		if (m_legacy_bios)
		{
			if (!machine().side_effects_disabled())
				{
					m_kb_status &= ~0x02;
					if (m_pic_ready)
						set_source(2, false);
				}
			return m_kb_rx;
		}
		return 0x0044; // handshake status, bit7 = busy, measured idle
	case 0x2b02:
		// keyboard/console channel status: bit 0 ready to accept a
		// byte, bit 1 a byte waiting to be read.
		//
		// The rest of the byte is not the console's. Bit 3 says whether
		// the external floppy drive is attached: the start-up loop tries
		// drive A, drive B and the Winchester in turn, and its drive B
		// leg at FC3B8 reads this port and gives up if bit 3 is high.
		// Drive B is the 765 at 0x2814 - the routines that talk to it
		// run only when the unit flag at [0x44b3] is 1, which is what
		// the drive B leg sets, while drive A takes the SCSI path - and
		// the maintenance manual says to load the diagnostic diskette in
		// drive B. Idle on a machine with no drive fitted this port
		// reads 0xfc, bit 3 high; report the drive when there is a disk
		// in it.
		//
		// Bits 4 and 5 are the two the interrupt handler sorts on. It
		// reaches FCEDE whenever either is high and loops there until
		// both are low: bit 5 sends it to the controller, bit 4 to a
		// strobe of 0x2800 and round again - so 0x2800 is what takes bit
		// 4 down, and leaving that bit permanently high spins the loop
		// forever. Both are high in the measured idle byte, so bit 4
		// starts set and stays down once acknowledged; bit 5 is the
		// controller's own interrupt line.
		if (m_legacy_bios)
		{
			floppy_image_device *const f = m_floppy->get_device();
			return 0xc4
					| ((f && f->exists()) ? 0 : 0x08)
					| (m_fdc_int ? 0x20 : 0)
					| m_kb_status;
		}
		return 0x00fe; // measured 0xfc idle; bit1 (ready to accept) forced high
	case 0x2e1e: return 0x00f4 | (m_mode_2e1e & 0x08);
	                            // mode/config register, measured on real hardware:
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
	// 0x2d00 is the video window select. The display driver at E4C5:0000
	// writes it before every character it puts in the F segment and puts
	// the channel's own value back afterwards; the display channel holds
	// 0x0c in both places, so the window sits there for good. Everything
	// else about the register is still unknown - the boot overlay below
	// happens to hang off the same first write.
	if ((offset << 1) == 0x2d00 && ACCESSING_BITS_0_7)
		m_video_window = data & 0xff;

	// IBM aliases (see the members' comment): timer, system latch, CGA.
	if ((offset << 1) == 0x40 || (offset << 1) == 0x42)
	{
		int const base = ((offset << 1) == 0x40) ? 0 : 2;
		if (ACCESSING_BITS_0_7)
			m_pit_ibm->write(base, data & 0xff);
		if (ACCESSING_BITS_8_15)
			m_pit_ibm->write(base + 1, (data >> 8) & 0xff);
		return;
	}
	if ((offset << 1) == 0x60)
	{
		// 0x60 itself is the read-only scancode latch; 0x61 sticks
		if (ACCESSING_BITS_8_15)
			m_port61 = (data >> 8) & 0xff;
		return;
	}
	if ((offset << 1) == 0x3d4)
	{
		if (ACCESSING_BITS_0_7)
			m_crtc_idx = data & 0x1f;
		if (ACCESSING_BITS_8_15)
			m_crtc_reg[m_crtc_idx] = (data >> 8) & 0xff;
		return;
	}
	if ((offset << 1) == 0x3d8)
	{
		if (ACCESSING_BITS_0_7)
			m_cga_mode = data & 0xff;
		if (ACCESSING_BITS_8_15)
			m_cga_color = (data >> 8) & 0xff;
		return;
	}

	// Mode and display-source registers - see the members' comment.
	if ((offset << 1) == 0x2e1e && ACCESSING_BITS_0_7)
		m_mode_2e1e = data & 0xff;
	if ((offset << 1) == 0x2e06 && ACCESSING_BITS_0_7)
		m_disp_2e06 = data & 0xff;
	if ((offset << 1) == 0x2e0a && ACCESSING_BITS_0_7)
		m_disp_2e0a = data & 0xff;

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
		m_fseg_ram = !(data & 1);
		if (data & 1)
			m_maincpu->space(AS_PROGRAM).install_rom(0xf0000, 0xfffff,
					memregion("bios")->base() + 0x10000);
		else
			m_maincpu->space(AS_PROGRAM).install_ram(0xf0000, 0xfffff,
					reinterpret_cast<uint8_t *>(m_fram.target()));
	}

	if ((offset << 1) == 0x2816 && ACCESSING_BITS_0_7)
	{
		logerror("FDC comando %02x\n", data & 0xff);
		m_fdc->fifo_w(data & 0xff);
		return;
	}
	// 0x2810 picks which controller the DMA serves. FCE07 writes it
	// from the unit flag at [0x44b3] right before programming the
	// transfer: 0 for the SCSI path that drive A and the Winchester
	// take, 1 for the floppy.
	if ((offset << 1) == 0x2810 && ACCESSING_BITS_0_7)
	{
		m_dma_floppy = BIT(data, 0);
		// the 5380's start-DMA strobes are what set the direction on the
		// SCSI side; the floppy has no equivalent here, and every
		// transfer the start-up code asks it for is a read
		if (m_dma_floppy)
			m_dma_recv = true;
		return;
	}
	// 0x2818 is written once per operation at FCF7D, just before the
	// driver waits on the controller's interrupt
	if ((offset << 1) == 0x2818 && ACCESSING_BITS_0_7)
	{
		logerror("floppy select %02x\n", data & 0xff);
		return;
	}

	// End-of-interrupt for the DMA source, and with it the terminal
	// count flag in 0x2b0a. The disk driver's handler at FD31E reads
	// that word, strobes this port and reads the word again in a loop,
	// so the strobe has to take the flag down.
	if ((offset << 1) == 0x2c12)
	{
		m_dma_tc = false;
		return;
	}

	// Interrupt vector base register - see m_vector_base.
	//
	// Only bit 7. The port is not a plain base register: the ROM writes
	// it at F1394 in the middle of programming the SCC, right after
	// 0x2c0e, and the serial module keeps shadow bytes for the pair
	// (0x2c0c from es:[0x42], 0x2c0e from es:[0x43]) and ORs bits into
	// them - it writes 0x00 three times and then 0x0a while setting up a
	// channel. Reading the whole byte as a base moved it to 0x08 there
	// and sent the disk interrupt to vector 0x0b, which the stage after
	// the loader points at a bare iret; its own handlers are still on
	// 0x80-0x87, the vector 0x83 among them. Bit 7 is the only part of
	// this register we have evidence for, and only ever set.
	if ((offset << 1) == 0x2c0c && ACCESSING_BITS_0_7)
	{
		if (BIT(data, 7))
		{
			m_vector_base = 0x80;
			logerror("gate array vector base = %02x\n", m_vector_base);
		}
		return;
	}

	// The interrupt controller is an 8259, and 0x2200/0x2202 are its
	// command and data ports. The BIOS initialises it the canonical way -
	// ICW1 0x1f to 0x2200, then 0x80, 0x0d, 0x00 to 0x2202 - which is
	// where the vector base of 0x80 comes from, and the "0xe0 + n" the
	// diagnostic's handlers write is a rotate-on-specific-EOI, not a Wang
	// invention.
	//
	// The device gets the writes so its command and mask state is real.
	// The sources still reach the CPU the old way for now: moving them
	// onto the IR lines has to happen in one piece, and the machine has
	// to keep booting in between.
	// Both firmware families program it here; ICW1 (bit 4 on the command
	// port) is the moment the controller takes charge of the vectors.
	if (ACCESSING_BITS_0_7
			&& ((offset << 1) == 0x2200 || (offset << 1) == 0x2202))
	{
		m_pic->write(((offset << 1) == 0x2200) ? 0 : 1, data & 0xff);
		if ((offset << 1) == 0x2200 && BIT(data, 4))
			m_pic_inited = true;
	}

	// the gate array's end-of-interrupt ports for the two counters
	if (m_legacy_bios && m_pic_ready && ACCESSING_BITS_0_7)
	{
		if ((offset << 1) == 0x2c16 || (offset << 1) == 0x2c10)
			{ pic_ir(0, 0); m_source_state &= ~1; }
		if ((offset << 1) == 0x2c14 || (offset << 1) == 0x2c10)
			{ pic_ir(1, 0); m_source_state &= ~2; }
	}

	if ((offset << 1) == 0x2202)
	{
		m_int_enable_2202 = data & 0xff;
		// Masking a source takes its request off the line. Without this
		// a request raised just before the mask survives until the CPU
		// next enables interrupts, and by then whoever armed the source
		// has moved on.
		if (m_legacy_bios && BIT(m_int_enable_2202, m_gate_vector - m_vector_base))
			m_maincpu->set_input_line(0, CLEAR_LINE);
		scsi_int_update();
		return;
	}

	if ((offset << 1) >= 0x2400 && (offset << 1) <= 0x2407)
	{
		m_pit->write(offset & 3, data & 0xff);
		return;
	}

	if ((offset << 1) >= 0x2700 && (offset << 1) <= 0x270f)
	{
		// asserting RST through the initiator command register raises
		// the 5380 interrupt, visible in bit 2 of 0x2b0a
		if ((offset & 7) == 1 && (data & 0x80) && !machine().side_effects_disabled())
			m_scsi_rst_irq = true;
		// the write-only registers 5 and 7 are the two "start DMA"
		// strobes: 5 sends, 7 receives as initiator. They are what tells
		// the Wang DMA controller which way the bytes are going - its own
		// mode bytes are the same 0x02/0x01 pair on the read path we can
		// see, so the direction has to come from this side.
		if ((offset & 7) == 5) m_dma_recv = false;
		if ((offset & 7) == 7) m_dma_recv = true;
		m_scsi->write(offset & 7, data & 0xff);
		return;
	}

	if ((offset << 1) >= 0x2500 && (offset << 1) <= 0x2507)
	{
		m_scc->ab_dc_w(offset & 3, data & 0xff);
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
			dma_reg_w(reg, data & 0xff);
		if (ACCESSING_BITS_8_15)
			dma_reg_w(reg + 1, (data >> 8) & 0xff);
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
	{
		// Bit 7 of register A is UIP, and on the real part it is status,
		// not something you can write. The clock reader at E1639 selects
		// register A and spins until that bit is clear before taking each
		// field, so handing back whatever was last written there stops the
		// stage that comes up off the system diskette dead - which is
		// where it was sitting.
		if ((m_index_sel & 0x3f) == 0x0a)
			data &= 0x7f;
		m_rtc[m_index_sel & 0x3f] = data & 0xff;
		// register B bit 6, MC146818-style: periodic interrupt enable.
		// The 1986 POST turns it on with register A = 0x23 (122us rate)
		// and demands its keyboard/RTC ISR on vector 0x25 count between
		// 8 and 12 interrupts across a software delay loop. It enables
		// FIRST and programs the rate AFTER, so writes to either
		// register retune the timer.
		if (((m_index_sel & 0x3f) == 0x0b || (m_index_sel & 0x3f) == 0x0a)
				&& !machine().side_effects_disabled())
		{
			if (m_rtc[0x0b] & 0x40)
			{
				// rate 3 is 122us on a real MC146818. The POST demands
				// 8-12 ticks across a counting window that measures
				// ~930us as the emulated V30 executes it (the ISRs
				// lengthen the delay loop): 122us lands 7.6 ticks - one
				// short - and 61us lands 14, two over. Calibrated like
				// the PIT clocks; 100us puts 9 in the window.
				attotime const period = attotime::from_usec(88);
				m_rtc_timer->adjust(period, 0, period);
			}
			else
				m_rtc_timer->adjust(attotime::never);
		}
	}

	// Keyboard microcontroller, as driven by the diagnostic utility:
	// 0x2c1e takes a command byte (0x2c10 gates it), 0x2a08 is the
	// status port that is read straight after a command and also takes
	// the sound commands 0x0b (tone) and 0x0c (click). A command is
	// answered with a reply interrupt on IRQ1 carrying event code 6.
	if ((offset << 1) == 0x2c1e)
	{
		// This is the acknowledge for the transmit-ready condition, and
		// both console handlers turn on it. The one the POST installs
		// strobes it and then sends its byte; the one the stage after the
		// POST installs at FD35F strobes it and re-reads 0x2b02 in a
		// loop, leaving only when bit 0 has gone - so the strobe has to
		// clear that bit, and the bit must not come back before that loop
		// has read it low.
		//
		// But it does have to come back. The micro is ready for another
		// byte a moment after taking one, and equally a moment after an
		// acknowledge that sent nothing - which is what the console
		// handler does whenever its output ring happens to be empty
		// (E000:137C). Re-arming only on a byte actually written, as this
		// used to, meant the first empty acknowledge took transmit-ready
		// away for good: the ring filled and never drained again, and
		// everything the loaded system tried to print after that point
		// vanished. The poll runs every 200us, which is thousands of
		// instructions after the strobe - long enough for the wait loop
		// to have seen the bit low and left.
		if (m_legacy_bios)
		{
			m_kb_status &= ~0x01;
			m_kb_ready_again = true;
		}
		m_kb_reply = 0xfa;
		m_kb_timer->adjust(attotime::from_usec(200));
		logerror("kb cmd %02x -> reply irq scheduled\n", data & 0xff);
	}
	if ((offset << 1) == 0x2a08)
	{
		if (m_legacy_bios)
			kb_command(data & 0xff);
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
	m_pic_inited = false;
	// the layout is read once per boot: changing the setting in the UI
	// takes effect at the next soft reset
	m_layout_active = m_layout->read() & 3;
	m_kb_recipe_base = 0;
	m_kb_recipe.clear();
	m_is_arch_base = 0;
	m_is_arch_valido = false;
	m_is_arch_avvisato = false;
	m_layout_timer->adjust(attotime::from_seconds(2), 0, attotime::from_seconds(2));
	m_kb_status = 0x01;
	m_kb_ready_again = false;
	m_kb_rx = 0;
	m_kb_replies.clear();
	// nothing in the machine switches the drive motor, so spin it
	if (floppy_image_device *const f = m_floppy->get_device())
		f->mon_w(0);
	// the micro announces itself ready a few hundred microseconds after
	// each exchange, which is what raises the vector 0x22 interrupt
	m_kb_poll->adjust(attotime::from_usec(200), 0, attotime::from_usec(200));
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
	// The clock registers proper, left at zero until now. The stage that
	// comes up off the system diskette reads 6/7/8/9 to keep the date and
	// turns them into a day number at E1577, and it counts in binary from
	// a 1980 epoch: the year goes through sub cl,0x50 (80) before being
	// divided by four against 0x5b5 - 1461, the days in four years. A zero
	// month and a zero day come out of that as counters of tens of
	// thousands, and the little sum loop at E15F6 they drive is where the
	// machine was spending all of its time.
	m_rtc[0x00] = 0;     // seconds
	m_rtc[0x02] = 0;     // minutes
	m_rtc[0x04] = 12;    // hours
	m_rtc[0x06] = 4;     // day of week
	m_rtc[0x07] = 1;     // day of month
	m_rtc[0x08] = 1;     // month
	m_rtc[0x09] = 86;    // year, 1986
	m_rtc[0x0d] = 0x80;  // valid RAM and time
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

	// The character generator: 256 glyphs of 32 bytes in the F EPROM at
	// 0x2000, which is exactly the 8K of the video window at 0xf2000.
	// Each glyph is an 8x8 character with every row written twice -
	// verified across all 95 printable codes, with the upper 16 bytes
	// of the cell blank - so the display font is 8 pixels square, one
	// row per scanline of the 640x200 panel's 25 text rows.
	m_chargen = bios + (m_legacy_bios ? 0xa000 : 0x12000);

	// On hardware the video window reads the EPROM until something
	// writes over it, so the generator is simply present at power-on;
	// the emulated window read back RAM, which is why the POST first
	// rasterised its banner with blank glyphs.
	memcpy(&m_vram[0], m_chargen, 0x2000);

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
	// EXECUTE is on the right Alt as well as the keypad enter: it is
	// the key the sign-on screen and the main menu both want, a laptop
	// keyboard has no keypad, and Tab belongs to MAME's own UI. The
	// Wang Alt is on the left Alt only. RETURN is the ordinary enter.
	//
	// Keyboard, positional: see KB_CODE in the driver for where the codes
	// come from. The labels are the QWERTY ones; on a QWERTZ machine y and
	// z swap places but the codes do not.

	PORT_START("KB0")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("1") PORT_CODE(KEYCODE_1)
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("2") PORT_CODE(KEYCODE_2)
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("3") PORT_CODE(KEYCODE_3)
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("4") PORT_CODE(KEYCODE_4)
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("5") PORT_CODE(KEYCODE_5)
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("6") PORT_CODE(KEYCODE_6)
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("7") PORT_CODE(KEYCODE_7)
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("8") PORT_CODE(KEYCODE_8)
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("9") PORT_CODE(KEYCODE_9)
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("0") PORT_CODE(KEYCODE_0)
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("-") PORT_CODE(KEYCODE_MINUS)
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("=") PORT_CODE(KEYCODE_EQUALS)

	// The Wang has sixteen function keys and a PC has twelve, so
	// F13 to F16 answer shift with F1 to F4. The unshifted keys are
	// conditioned the other way round so only one of the pair can be
	// down at a time. Codes come from the positional model - the F
	// row is 0x7f, descending to the right - and are the last part of
	// the keyboard not yet confirmed against the real machine.
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F1") PORT_CODE(KEYCODE_F1)
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F2") PORT_CODE(KEYCODE_F2)
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F3") PORT_CODE(KEYCODE_F3)
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F4") PORT_CODE(KEYCODE_F4)

	PORT_START("KB1")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Q") PORT_CODE(KEYCODE_Q)
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("W") PORT_CODE(KEYCODE_W)
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("E") PORT_CODE(KEYCODE_E)
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("R") PORT_CODE(KEYCODE_R)
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("T") PORT_CODE(KEYCODE_T)
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Y") PORT_CODE(KEYCODE_Y)
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("U") PORT_CODE(KEYCODE_U)
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("I") PORT_CODE(KEYCODE_I)
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("O") PORT_CODE(KEYCODE_O)
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("P") PORT_CODE(KEYCODE_P)
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("[") PORT_CODE(KEYCODE_OPENBRACE)
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("]") PORT_CODE(KEYCODE_CLOSEBRACE)
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F5") PORT_CODE(KEYCODE_F5)
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F6") PORT_CODE(KEYCODE_F6)
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F7") PORT_CODE(KEYCODE_F7)
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F8") PORT_CODE(KEYCODE_F8)

	PORT_START("KB2")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("A") PORT_CODE(KEYCODE_A)
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("S") PORT_CODE(KEYCODE_S)
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("D") PORT_CODE(KEYCODE_D)
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F") PORT_CODE(KEYCODE_F)
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("G") PORT_CODE(KEYCODE_G)
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("H") PORT_CODE(KEYCODE_H)
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("J") PORT_CODE(KEYCODE_J)
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("K") PORT_CODE(KEYCODE_K)
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("L") PORT_CODE(KEYCODE_L)
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(";") PORT_CODE(KEYCODE_COLON)
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("'") PORT_CODE(KEYCODE_QUOTE)
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("RETURN") PORT_CODE(KEYCODE_ENTER)
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F9") PORT_CODE(KEYCODE_F9)
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F10") PORT_CODE(KEYCODE_F10)
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F11") PORT_CODE(KEYCODE_F11)
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F12") PORT_CODE(KEYCODE_F12)

	PORT_START("KB3")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Z") PORT_CODE(KEYCODE_Z)
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("X") PORT_CODE(KEYCODE_X)
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("C") PORT_CODE(KEYCODE_C)
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("V") PORT_CODE(KEYCODE_V)
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("B") PORT_CODE(KEYCODE_B)
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("N") PORT_CODE(KEYCODE_N)
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("M") PORT_CODE(KEYCODE_M)
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(",") PORT_CODE(KEYCODE_COMMA)
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(".") PORT_CODE(KEYCODE_STOP)
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("/") PORT_CODE(KEYCODE_SLASH)
	// the F row is TWELVE keys: F13-F24 = Shift+F1..F12 and F25-F36 =
	// Shift+Ctrl+F1..F12, exactly as on the machine (the modifiers
	// travel as their own make/break codes and the firmware combines
	// them). The four codes that used to sit here as "F13-F16" belong
	// to other keys: TAB and three specials still to be named.
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("TAB") PORT_CODE(KEYCODE_TAB)
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("HELP? (codice 70, da verificare)") PORT_CODE(KEYCODE_F14)
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("ERASE? (codice 71, da verificare)") PORT_CODE(KEYCODE_F15)
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("GL? (codice 72, da verificare)") PORT_CODE(KEYCODE_F16)

	PORT_START("KB4")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("EXECUTE") PORT_CODE(KEYCODE_RALT)
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Left Shift") PORT_CODE(KEYCODE_LSHIFT)
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Right Shift") PORT_CODE(KEYCODE_RSHIFT)
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Ctrl") PORT_CODE(KEYCODE_LCONTROL)
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Alt") PORT_CODE(KEYCODE_LALT)
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("CANCEL") PORT_CODE(KEYCODE_DEL)
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Space (Item Select)") PORT_CODE(KEYCODE_SPACE)
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Backspace (Item Up)") PORT_CODE(KEYCODE_BACKSPACE)
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("ESC (Wang)") PORT_CODE(KEYCODE_END)
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Caps Lock") PORT_CODE(KEYCODE_CAPSLOCK) PORT_CODE(KEYCODE_RCONTROL)
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("` (backtick)") PORT_CODE(KEYCODE_TILDE)
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("\\") PORT_CODE(KEYCODE_BACKSLASH)
	// the Wang keyboard's own arrow keys (bottom right on the real
	// unit): codes read off XLAT's scancode table, where they map to
	// the IBM arrows 48/50/4B/4D - which is what the Digger port
	// steers with. Space/Backspace stay the Wang menu keys.
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Freccia su") PORT_CODE(KEYCODE_UP)
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Freccia giu'") PORT_CODE(KEYCODE_DOWN)
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Freccia sinistra") PORT_CODE(KEYCODE_LEFT)
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Freccia destra") PORT_CODE(KEYCODE_RIGHT)

	// the numeric keypad, fully attributed by measurement (the
	// number-to-character map echoed out of the machine itself)
	PORT_START("KB5")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Tastierino 0") PORT_CODE(KEYCODE_0_PAD)
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Tastierino 1") PORT_CODE(KEYCODE_1_PAD)
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Tastierino 2") PORT_CODE(KEYCODE_2_PAD)
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Tastierino 3") PORT_CODE(KEYCODE_3_PAD)
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Tastierino 4") PORT_CODE(KEYCODE_4_PAD)
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Tastierino 5") PORT_CODE(KEYCODE_5_PAD)
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Tastierino 6") PORT_CODE(KEYCODE_6_PAD)
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Tastierino 7") PORT_CODE(KEYCODE_7_PAD)
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Tastierino 8") PORT_CODE(KEYCODE_8_PAD)
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Tastierino 9") PORT_CODE(KEYCODE_9_PAD)
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Tastierino +") PORT_CODE(KEYCODE_PLUS_PAD)
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Tastierino -") PORT_CODE(KEYCODE_MINUS_PAD)
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Tastierino *") PORT_CODE(KEYCODE_ASTERISK)
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Tastierino /") PORT_CODE(KEYCODE_SLASH_PAD)
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Tastierino Invio") PORT_CODE(KEYCODE_ENTER_PAD)

	// navigation and editing keys
	PORT_START("KB6")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Home (freccia obliqua)") PORT_CODE(KEYCODE_HOME)
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("PRINT") PORT_CODE(KEYCODE_PRTSCR)
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("INSERT (Pg Up)") PORT_CODE(KEYCODE_INSERT)
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("PREV (Pg Dn)") PORT_CODE(KEYCODE_PGDN)
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("NEXT") PORT_CODE(KEYCODE_PGUP)
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("DE LETE") PORT_CODE(KEYCODE_F13)

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

	// National layout of the HOST keyboard: the driver patches the
	// loaded translation tables (Wang lists + XLAT's IS archive) the way
	// a national software build would ship them. "US / WLTC nativa" is
	// the untouched American build - also the right choice for a real
	// WLTC replica keyboard, whose keys then do what their caps say.
	// Read at reset; the wltcit / wltcde machines preset it.
	PORT_START("LAYOUT")
	PORT_CONFNAME( 0x0003, 0x0000, "Layout tastiera (host)" )
	PORT_CONFSETTING(      0x0000, "US / WLTC nativa" )
	PORT_CONFSETTING(      0x0001, "Italiana" )
	PORT_CONFSETTING(      0x0002, "Tedesca" )
INPUT_PORTS_END

static INPUT_PORTS_START( wltcit )
	PORT_INCLUDE( wltc )
	PORT_MODIFY("LAYOUT")
	PORT_CONFNAME( 0x0003, 0x0001, "Layout tastiera (host)" )
	PORT_CONFSETTING(      0x0000, "US / WLTC nativa" )
	PORT_CONFSETTING(      0x0001, "Italiana" )
	PORT_CONFSETTING(      0x0002, "Tedesca" )
INPUT_PORTS_END

static INPUT_PORTS_START( wltcde )
	PORT_INCLUDE( wltc )
	PORT_MODIFY("LAYOUT")
	PORT_CONFNAME( 0x0003, 0x0002, "Layout tastiera (host)" )
	PORT_CONFSETTING(      0x0000, "US / WLTC nativa" )
	PORT_CONFSETTING(      0x0001, "Italiana" )
	PORT_CONFSETTING(      0x0002, "Tedesca" )
INPUT_PORTS_END


void wltc_state::wltc(machine_config &config)
{
	V30(config, m_maincpu, 8'000'000); // NEC D70116C-8
	m_maincpu->set_addrmap(AS_PROGRAM, &wltc_state::mem_map);
	m_maincpu->set_addrmap(AS_IO, &wltc_state::io_map);
	m_maincpu->set_irq_acknowledge_callback(FUNC(wltc_state::irq_ack));

	// The POST counts gate-array interrupts across software delay loops
	// a few hundred microseconds long; with the default scheduling
	// quantum the CPU runs far ahead of the timers, the interrupts all
	// land after the loop has finished counting, and the tests fail.
	// Keep the CPU and the timers interleaved tighter than the shortest
	// counting window.
	config.set_maximum_quantum(attotime::from_usec(25));

	// NEC D71054, an 8254 clone, at 0x2400-0x2406 (one register every
	// other address); the counters are clocked from the CPU crystal
	// through the usual divider chain.
	// The DMA controller at 0x2300-0x230f is a Wang part, not an 8237,
	// and is modelled in the driver itself - see dma_reg_w.

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

	// The IBM alias at 0x40-0x43 (Industry Standard side of the gate
	// array): a timer of its own at the PC's clock, free-running for
	// the software that latches and reads it (Digger's sync loop);
	// counter 2 is the speaker tone. Outputs unwired for now.
	PIT8254(config, m_pit_ibm);
	m_pit_ibm->set_clk<0>(1'193'182);
	m_pit_ibm->set_clk<1>(1'193'182);
	m_pit_ibm->set_clk<2>(1'193'182);
	// counter 0 is the system tick on IRQ0, as the measured interrupt
	// mask (0xbc) and vector table (INT 80h = the tick ISR) imply
	m_pit->out_handler<0>().set(FUNC(wltc_state::pit_out_w<0>));
	// counters 1 and 2 interrupt through the gate array on the 1986
	// hardware (vectors 0x20/0x21, enabled by bits 0/1 of port 0x2202);
	// deliver their OUT edges straight to the CPU INT line there - the
	// 8259 is never initialised by that firmware
	m_pit->out_handler<1>().set(FUNC(wltc_state::pit_out_w<1>));
	m_pit->out_handler<2>().set(FUNC(wltc_state::pit_out_w<2>));

	PIC8259(config, m_pic);
	// through a gate: the unprogrammed 8259 answers every IR update
	// with INT low, and wired straight to the CPU that CLEAR_LINE races
	// the gate-array interrupts the 1986 firmware relies on - its timer
	// edge got erased before the CPU could take it
	m_pic->out_int_callback().set(FUNC(wltc_state::pic_int_w));

	// NCR 53C80 SCSI bus at 0x2700: the JVC Winchester sits on it, and
	// so does the external floppy drive
	nscsi_bus_device &scsibus(NSCSI_BUS(config, "scsi"));
	// Seven device positions, ids 0 to 6, all free: on the real machine
	// the internal Winchester is id 0, the 3.5" enclosure is wired to
	// id 1 (drive A), and the 5.25" boxes carry a selector switch - up
	// to the full bus of drives. Only the Winchester is fitted by
	// default; add drives with e.g. -scsi:1 wangfdd35 -scsi:2 wangfdd.
	for (int id = 0; id <= 6; id++)
	{
		nscsi_connector &conn(NSCSI_CONNECTOR(config, util::string_format("scsi:%d", id).c_str()));
		conn.option_add("winchester", WANG_WINCHESTER);
		conn.option_add("wangfdd", WANG_SCSI_FLOPPY);
		conn.option_add("wangfdd35", WANG_SCSI_FLOPPY35);
		if (id == 0)
			conn.set_default_option("winchester");
	}
	NCR5380(config, m_scsi);
	scsibus.set_external_device(7, m_scsi);
	m_scsi->irq_handler().set(FUNC(wltc_state::scsi_int_w));
	m_scsi->drq_handler().set(FUNC(wltc_state::dma_drq_w));

	// Floppy controller at 0x2814/0x2816, one register every other
	// address like everything else here. The ROM names the part: FCED6
	// reads 0x2814 and spins on bit 7, then shifts bit 6 into the carry
	// - request for master and data direction, the main status register
	// of a 765 - and the command phase at FCEA1 and the result phase at
	// FCFEF push and pull bytes through 0x2816. The maintenance manual's
	// manufacturing menu calls its two floppy tests Recalibrate and
	// Seek, which are that family's command names. The drive is external
	// and the system diskettes are 360K: 40 cylinders, two heads, nine
	// sectors.
	UPD765A(config, m_fdc, 8'000'000, true, true);
	m_fdc->intrq_wr_callback().set(FUNC(wltc_state::fdc_int_w));
	m_fdc->drq_wr_callback().set(FUNC(wltc_state::dma_drq_w));
	// The drive answers as unit 1: the start-up code's drive B leg sets
	// the unit byte at [0x44f8] to 1, and that 1 is what follows the
	// recalibrate opcode into the controller. Unit 0 is drive A, which
	// is not on this controller at all - its leg takes the SCSI path -
	// so leave that connector empty.
	FLOPPY_CONNECTOR(config, "fdc:0", wltc_floppies, nullptr,
			wltc_state::floppy_formats);
	FLOPPY_CONNECTOR(config, "fdc:1", wltc_floppies, "525dd",
			wltc_state::floppy_formats).enable_sound(true);

	// Z8530APS serial communications controller at 0x2500.
	//
	// TRxC is strapped back to RTxC on the board. The POST's loopback
	// test says so: it writes wr11 = 0x15, which takes the transmit
	// clock from the baud rate generator, makes TRxC an output and puts
	// the transmit clock on it, and then takes the RECEIVE clock from
	// the RTxC pin. Nothing else on a laptop drives RTxC, and the test
	// does receive what it sends, so the two pins are tied together -
	// which is also the only reason to route the transmit clock out of
	// TRxC at all. Told statically here, since the SCC model has no
	// TRxC output line to wire, so RTxC is given the frequency the
	// generator puts on TRxC: PCLK / 2(TC + 2), with the time constant
	// of 14 the POST loads into wr12/wr13. That makes the receive and
	// transmit bit rates identical, which is the whole point of the
	// strap; a later baud constant would need this to follow, and the
	// strap on the board does follow it.
	//
	// The clock is the POST's own doing too. A time constant of 14 in
	// the x16 mode it selects divides PCLK by 512, and a diagnostic
	// that talks to itself over a serial port talks at 9600 baud, so
	// PCLK is 512 * 9600 = 4.9152 MHz - one of the standard baud
	// crystals, and not the 4 MHz guessed here before. It matters: the
	// test polls for the received character with a loop that gives up
	// after 255 turns, and at 4 MHz the character was still in the
	// shift register when the loop ran out.
	SCC8530(config, m_scc, 4'915'200);
	m_scc->configure_channels(4'915'200 / 32, 4'915'200 / 32, 4'915'200 / 32, 4'915'200 / 32);
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
	// in for here by the glyph cache of the later EPROM set
	ROMX_LOAD( "myf000.bin", 0x08000, 0x8000, CRC(b0d23b9d) SHA1(c068b6c897b2ffea222ff7a38d3f39ac6fba54a4), ROM_BIOS(1) )

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

DEFINE_DEVICE_TYPE_PRIVATE(WANG_WINCHESTER, nscsi_harddisk_device, wang_winchester_device, "wang_winchester", "Wang LapTop Winchester")
// The 3.5" enclosure: on the real machine it is the drive wired to
// SCSI ID 1 - drive A - while the 5.25" box has a selector switch.
// Same bridge, different mechanism.
class wang_scsi_floppy35_device : public wang_scsi_floppy_device
{
public:
	wang_scsi_floppy35_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0)
		: wang_scsi_floppy_device(mconfig, WANG_SCSI_FLOPPY35, tag, owner, clock)
	{ }

protected:
	virtual const char *drive_default() const override { return "35dd"; }
};

DEFINE_DEVICE_TYPE_PRIVATE(WANG_SCSI_FLOPPY, nscsi_full_device, wang_scsi_floppy_device, "wang_scsi_floppy", "Wang LapTop external floppy drive")
DEFINE_DEVICE_TYPE_PRIVATE(WANG_SCSI_FLOPPY35, nscsi_full_device, wang_scsi_floppy35_device, "wang_scsi_floppy35", "Wang LapTop external floppy drive (3.5\")")


// The clones only preset the LAYOUT machine configuration, so the
// national keyboard can be picked from the command line:
//   mamewang wltcit   (italiana)      mamewang wltcde   (tedesca)
#define rom_wltcit rom_wltc
#define rom_wltcde rom_wltc

//    YEAR  NAME    PARENT  COMPAT  MACHINE  INPUT   CLASS       INIT        COMPANY              FULLNAME                                    FLAGS
COMP( 1987, wltc,   0,      0,      wltc,    wltc,   wltc_state, empty_init, "Wang Laboratories", "Wang LapTop Computer",                      MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
COMP( 1987, wltcit, wltc,   0,      wltc,    wltcit, wltc_state, empty_init, "Wang Laboratories", "Wang LapTop Computer (tastiera italiana)",  MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
COMP( 1987, wltcde, wltc,   0,      wltc,    wltcde, wltc_state, empty_init, "Wang Laboratories", "Wang LapTop Computer (deutsche Tastatur)",  MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
