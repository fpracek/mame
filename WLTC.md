# Wang LapTop Computer (WLTC) driver

MAME driver for the Wang LapTop Computer, a V30-based portable released
by Wang Laboratories in 1987 (FCC ID B4Y8P7WLTC, made in Japan). It runs
Wang's own MS-DOS 3.20 natively and can emulate an IBM PC-XT through the
"Translator" (XLAT.SYS) shipped on the system diskettes.

Source: [`src/mame/wang/wltc.cpp`](src/mame/wang/wltc.cpp). This file
tracks the project as a whole - status, parameters, known limitations -
the source file's own header comment covers the hardware in more detail.

## Hardware modelled

- NEC D70116C-8 (V30, 8 MHz)
- NEC D71054G (8254 timer clone), D71059G (8259 interrupt controller clone)
- Zilog Z8530APS SCC (serial), 8250-compatible UART
- NCR 53C80 SCSI controller, driving the internal ~33 MB Winchester and
  an external 3.5"/5.25" floppy enclosure
- LCD 80x25 text, 320x200 / 640x200 graphics; also answers on the IBM
  mono and CGA port sets (the machine's "Industry Standard" video mode)
- Built-in thermal printer and optional modem are not modelled

## What works

- POST (1986 BIOS) passes every test the diagnostic utility exercises
- Boots from the Winchester (CHD) and from floppy (SCSI-attached
  drives, real or raw host file - see Parameters below)
- The Wang menu system, DOS 3.20 prompt and commands, GW-BASIC 3.20
  (including `SCREEN 1`)
- Wang video mode and both "Industry Standard" video modes (mono text
  and CGA, including 4-shade graphics)
- Full keyboard: numeric keypad, F1-F36 (F1-F12 direct, F13-F24 via
  Shift, F25-F36 via Shift+Ctrl), national layouts (US/IT/DE) plus a
  dedicated layout for a real WLTC keyboard (this project's USB replica),
  built into the driver as separate machine variants (`wltc`, `wltcit`,
  `wltcde`, `wltcusb`)
- IBM-compatible software through the Translator (tested with Digger)
- SCSI Winchester and floppy: reads *and* writes verified end-to-end,
  byte-exact, persistent across restarts, including hot-inserting a
  floppy into an already-booted session
- Serial (SCC) loopback and DMA transfers
- 8259-based interrupt path (the default; a fixed-vector fallback also
  exists in the driver for reference)

## Known limitations

- **BIOS 4.02.03** (the EPROM revision) does not work - its self-modifying
  threaded interpreter derails during POST. Always boot with `-bios v1986`.
- **Booting from Drive A alone** (no Winchester attached) stops at "Bad or
  missing Command Interpreter": SYSINIT opens `\COMMAND.COM` instead of
  honouring `SHELL=\MENUDRVR.COM` from CONFIG.SYS. Root cause not found
  yet; boot from the Winchester and hot-insert the floppy afterwards as
  a working alternative.
- **Any SCSI floppy slot populated at all** (even with no media, even
  the capacity-checked raw variants) makes the boot ROM attempt "01
  Start From Drive A" first. If that floppy isn't independently
  bootable, it stops at a recovery menu ("Start Failed" / "Equipment
  Malfunction" or "System Files Missing") that needs a keypress
  (`D` = Re-Direct Start) to continue to the next device - this is real
  boot ROM behaviour, not a hang, but it means a floppy device on the
  command line is not "free" even when unused.
- **RAMDISK.EXE and other "Wang Professional Computer" utilities**
  bundled on the WLTC system diskettes are generic software shared
  across Wang's DOS 3.0 product line, not written for this machine
  specifically. Running `RAMDISK.EXE` triggers a self-test that writes
  a 64K pattern starting at physical address 0x200 assuming that range
  is banked out to expansion memory; on the WLTC it's ordinary system
  RAM, so the test corrupts live BIOS/DOS state and produces a real,
  repeatable "General Failure" loop. Not a WLTC-specific driver bug -
  the utility assumes hardware this machine doesn't have.
- **Executing a program from a freshly hot-inserted floppy** (as
  opposed to just reading/listing it) has been seen to produce a
  spurious "Not ready" then "General Failure" loop while the
  underlying SCSI reads keep succeeding underneath - under
  investigation, mechanism not yet identified.
- **OPT RAM PCB** (the 512K->1M memory expansion card): the driver
  answers the card-presence I/O port (0x1026) as "not fitted" so
  software that probes for it (like RAMDISK.EXE) doesn't act on a
  phantom card, and logs the matching configuration write (0x109c).
  The card itself - the actual extra 512K and how software would
  address it once "present" - is not emulated.
- **NEC V30 core timing**: measured ~10% pessimistic on memory-access-
  heavy code compared to real hardware (108.9 clock/loop measured vs.
  120.4 emulated) - a MAME core-wide characteristic, not specific to
  this driver.
- The WLTCDIAG factory diagnostic's DMA CONTROL test stops on a
  boundary check that also fails on real hardware (confirmed by direct
  measurement) - not a driver bug, but it means the rest of that test
  suite (Winchester, Keyboard, Printer, Communications, Diskette,
  Modem, LCD) is unreachable through the diagnostic's normal flow.

## Parameters

```
mamewang.exe <machine> -bios <choice> -rompath <roms folder>
```

| machine   | keyboard |
|---|---|
| `wltc`    | American (US) |
| `wltcit`  | Italian (IT) |
| `wltcde`  | German (DE) |
| `wltcusb` | Real WLTC keyboard (USB replica) |

The four machines only preset the `Keyboard layout (host)` machine
configuration setting (Machine Configuration menu, or `-layout` core
config); everything else is identical. That setting is read once at
reset, so changing it mid-session needs a "Reset System" from the same
menu to take effect.

- `US / WLTC native` and `Italian`/`German` patch the loaded translation
  tables to make a PC keyboard (US, Italian, or German key captions)
  type the expected characters - a convenience layer for anyone using
  an ordinary PC keyboard, verified byte-by-byte against a real machine
  (IT 20/20, DE 24/24).
- `Real WLTC keyboard (USB replica)` is for the real WLTC keyboard
  layout (this project's USB replica, or any future one) rather than a
  PC keyboard: it applies none of the PC-adaptation remaps, and fixes
  the one native gap that affects a real keyboard specifically - the
  American stock BIOS never assigns a character to the driver's own
  dedicated backslash key (internal keycode 0x54), a gap no PC keyboard
  would ever notice since it has no physical key at that position.
  Fixed on both sides: the Wang-mode table (verified from the Wang
  menu/BASIC/File Spec fields) and the separate DOS/Industry-Standard
  XLAT archive (verified echoing "\" at the DOS prompt) - both use the
  same offset-by-keycode addressing, just at different base addresses.

| `-bios`   | |
|---|---|
| `v1986`   | 1986 BIOS - the only path that boots correctly, always use this |
| `v40203`  | BIOS 4.02.03 - does not work, see Known limitations |
| `v400`    | BIOS 4.00 as shipped on the system diskette - experimental |

### Hard disk (Winchester)

```
-hard <path.chd>
```

Mounts on SCSI id 0 (`winchester`, the slot's default option). Always
512 bytes/sector, CHS 512/8/16 (33,554,432 bytes):
`chdman.exe createhd -o disk.chd -chs 512,8,16 -ss 512 -c none`.
A blank disk needs `INITW` from system diskette 1, then `copy a:*.* c:`.

### Floppy drives (SCSI bus, external enclosure = Drive A)

```
-scsi:1 <option> -scsi:2 <option> ...
```

| option          | what it is | capacity | host folder as image? |
|---|---|---|---|
| `wangfdd`       | real 5.25" enclosure (UPD765 mechanism) | 360K | no |
| `wangfdd35`     | real 3.5" enclosure, same electronics | 360K | no |
| `wangfddraw`    | raw host file, any size | none | no |
| `wangfddraw525` | raw host file/folder, declared capacity | **360K** | **yes** |
| `wangfddraw35`  | raw host file/folder, declared capacity | **720K** | **yes** |

With the two capacity-checked variants, the `-flopN` argument can be a
**file** (checked against the declared capacity) or a **folder**
(packed on the fly into a fresh FAT12 image of that exact size, as a
one-time snapshot - see `wang_build_floppy_from_folder()` in the
source). Either way, if the content doesn't fit, the emulator refuses
to start rather than truncating or half-loading.

`-flop1`/`-flop2` are **not** fixed to `scsi:1`/`scsi:2`: MAME numbers
every "floppydisk" unit present in the session in order, including the
internal controller (`fdc:1`, always present, real 5.25" drive, unit
B). Populate both SCSI slots together when you need Drive A and B
addressed reliably by number; `mamewang.exe wltc -listslots wltc`
lists the options for every slot if in doubt.

### Other

| parameter | effect |
|---|---|
| `-uimodekey <key>` | hands the keyboard from the Wang to MAME's own menus and back |
| `-window` | windowed instead of fullscreen |
| `-samples -samplepath samples` / `-nosamples` | floppy motor/seek sounds on or off |

### Example

```
mamewang.exe wltcit -bios v1986 -rompath roms -hard dischi\winchester33mb.chd ^
  -scsi:1 wangfddraw35  -flop1 C:\path\to\folder_A ^
  -scsi:2 wangfddraw525 -flop2 dischi\driveB.img -window
```

## Upstream-candidate fixes in shared MAME code

Found with the WLTCDIAG factory diagnostic as an oracle; these apply to
any driver using the same shared devices, not just WLTC:

- **`nscsi_hle`** - the "abandon if the host never completes the
  nominal transfer" timeout existed only for the DATA IN phase, never
  for DATA OUT: a host that writes less than the declared maximum hung
  forever. Fixed symmetrically.
- **`ncr5380`** - initiator self-selection IRQ; phase-mismatch-on-DMA-
  enable; data bus drive during the selection phase.
- **`z80scc`** - interrupt priority order (was reversed); acknowledge
  vector taken from the wrong source; a transmit-buffer write that
  disarmed the next interrupt.
- **`nec`** (V30) - prefetch cost; confirmed against real hardware as
  the better of the two available choices.

These are isolated on local branches, not yet opened as pull requests
upstream.
