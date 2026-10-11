// license:BSD-3-Clause
// copyright-holders:Aaron Giles, AestheticEngine
/***************************************************************************

    Atari "Garfield's Quest" prototype (1984)

    Unreleased Atari coin-op.  No hardware is known to survive; the program
    was rebuilt from the original VAX source by Dutchman2000.  From what the
    program does, the board appears closely related to Atari System 2 (T-11
    CPU, and palette, alphanumerics, motion object and scroll hardware at
    the System 2 addresses), but adds a Winchester disk that DMAs graphics
    into a 128K DRAM shared by the playfield and motion objects.

    No schematics are available to this project.  The memory map, disk
    controller and graphics format below come from the program's behaviour;
    where it does not establish the hardware behaviour, System 2 behaviour
    is assumed and listed in the TODOs.

    Main CPU memory map (octal addresses, as used in the source)
    000000-007777   R/W   Program RAM (disk command block at 000400)
    010000-010777     W   Color RAM (RRRRGGGGBBBBIIII, System 2 layout)
    012000            W   IRQ0 clear
    012200            W   IRQ1 clear
    012400            W   IRQ2 (32V) clear
    012600            W   IRQ3 (VBLANK) clear
    013000            W   IRQ enable
    013200            W   Sound command
    013400            W   Playfield X scroll
    013600            W   Playfield Y scroll
    014000          R     Player inputs
    014002          R     Debug switches
    016000          R     Sound response
    020000-033777   R/W   Alphanumerics RAM (64x48)
    034000-037777   R/W   Motion object RAM (256 x 4 words, System 2 format)
    040000-047777   R/W   Playfield RAM (64x32 stamps)
    050000            W   Disk select/reset ("OUTPUT0"/"selres")
    100000-177777   R     Program ROM

    Disk controller
    The program builds a SASI-style command block in RAM and then writes
    1 to 050000 to start it (2 then 0 resets the controller):
      000400 byte   CDB byte 1 (LUN / LBA high)
      000401 byte   CDB byte 0 (opcode)
      000402 word   LBA (CDB bytes 2-3)
      000404 byte   CDB byte 5 (control)
      000405 byte   CDB byte 4 (block count, 0 = 256)
      000406 word   DMA destination
      000410 word   status: set to -1 by the program, controller writes a
                    non-negative value when done; bit 9 = error
    Opcodes used: 03 REQUEST SENSE, 04 FORMAT UNIT, 08 READ, 0A WRITE.
    LBAs in the program's load tables are in 256-byte units.

    Graphics
    Motion objects and playfield both draw 16x16 stamps from the 128K
    graphics DRAM (1024 stamps).  Each stamp row is four little-endian
    16-bit bitplane words.  The program's load lists read 64K at a time,
    one half of the DRAM.
    The program XORs color indices with 15.  The driver stores stamp pixels
    inverted to map source pen 0 to transparent pen 15.  The color RAM
    intensity bits are treated as active low, because the
    program's gray ramps and sprite palettes use intensity 0 (inferred;
    System 2 itself is active high).

    The supplied reconstruction reaches attract and accepts Start, but
    does not load graphics.  Notes below about rendered title art,
    gameplay scenes and scrolling describe diagnostic runs with modified
    program and disk inputs.  Those input modifications are not applied
    by this driver.

    TODO:
    - The program's disk load tables (pointer table at 0xFCB8) point two
      bytes before each list, so no disk read is ever issued and the
      graphics DRAM stays empty.  With the pointers corrected, the reads
      complete and fill the DRAM as expected.
    - The available disk image stores its data as LDA records rather than
      raw sectors, so it cannot be used as is.
    - The program uses two EIS DIV instructions (0xA830, 0xA83E), which the
      T-11 does not implement; the game traps as soon as the player moves.
    - Object type 50 (spawned by both object lists, 0xD856 and 0xD878) has
      a zero frame table (0xDB24), so the drawer parses low RAM as picture
      data.  That is where motion object codes 0x400 and above come from.
      In gameplay the extra objects make the VBLANK work overrun a frame,
      the 32V band counter is not cleared, and no motion objects show.
    - The DMA destination format is a guess (bit 15 = graphics DRAM,
      offset in 8-byte units).
    - The completion timing is a guess; see disk_select_w.
    - Sound program is unavailable. Hardware notes disagree on the sound chips:
      a 2020 account says Quad POKEY; the archive's SPEC.TXT says Yamaha.
    - Alphanumerics character ROM is not available.
    - Playfield entry bits 11-15 (palette, priority category) use an
      unverified System 2 decode.  Bit 10 selects a tile bank on System 2
      but is ignored here; its meaning on Garfield is unknown.  The
      examined metatile data (0xE4EE) has these upper bits clear.
    - Video timing and the alphanumerics layout are copied from System 2.
    - The inverted alphanumerics color index is inferred: the only text
      palette attract loads (list 3, raw indices 40-43) then lands on text
      color 0.  Text colors 2, 4 and 6 are not loaded by the attract
      scripts; gameplay (list 6) and the debug palette test can load them.
    - Scroll register format is copied from System 2.  The program scrolls
      in the first gameplay scene (camera code at 0xAA14 feeding the VBLANK
      copy at 0x9672), and with the System 2 format the scene stays coherent
      as it scrolls, but this is inferred, not measured.
    - Playfield/motion object priority uses the System 2 rule with the pen
      un-inverted; this matches the attract pictures, but the polarity is
      not confirmed by hardware.

***************************************************************************/

#include "emu.h"

#include "atarimo.h"

#include "cpu/t11/t11.h"
#include "machine/gen_latch.h"
#include "machine/timer.h"

#include "chd.h"
#include "emupal.h"
#include "romload.h"
#include "screen.h"
#include "tilemap.h"

#define LOG_DISK (1U << 1)

#define VERBOSE (0)
#include "logmacro.h"

#define LOGDISK(...) LOGMASKED(LOG_DISK, __VA_ARGS__)


namespace {

static constexpr XTAL MASTER_CLOCK = XTAL(20'000'000);

class garfield_state : public driver_device
{
public:
	garfield_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_gfxdecode(*this, "gfxdecode")
		, m_palette(*this, "palette")
		, m_screen(*this, "screen")
		, m_mob(*this, "mob")
		, m_playfield_tilemap(*this, "playfield")
		, m_alpha_tilemap(*this, "alpha")
		, m_soundlatch(*this, "soundlatch")
		, m_mainlatch(*this, "mainlatch")
		, m_playfield(*this, "playfield")
		, m_xscroll(*this, "xscroll")
		, m_yscroll(*this, "yscroll")
		, m_gfxram(*this, "gfxram", 0x20000, ENDIANNESS_LITTLE)
	{ }

	void garfield(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	required_device<t11_device> m_maincpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_device<screen_device> m_screen;
	required_device<atari_motion_objects_device> m_mob;
	required_device<tilemap_device> m_playfield_tilemap;
	required_device<tilemap_device> m_alpha_tilemap;
	required_device<generic_latch_8_device> m_soundlatch;
	required_device<generic_latch_8_device> m_mainlatch;
	required_shared_ptr<uint16_t> m_playfield;
	required_shared_ptr<uint16_t> m_xscroll;
	required_shared_ptr<uint16_t> m_yscroll;

	memory_share_creator<uint8_t> m_gfxram;   // 128K graphics DRAM, loaded from disk
	chd_file *m_disk = nullptr;
	emu_timer *m_disk_timer = nullptr;

	uint8_t m_interrupt_enable = 0;
	bool m_scanline_int_state = false;
	bool m_video_int_state = false;

	void update_interrupts();
	void int_enable_w(uint8_t data);
	void scanline_int_ack_w(uint16_t data);
	void video_int_ack_w(uint16_t data);
	void unused_int_ack_w(uint16_t data);
	TIMER_DEVICE_CALLBACK_MEMBER(scanline_update);
	void vblank_int(int state);

	void disk_select_w(uint16_t data);
	TIMER_CALLBACK_MEMBER(disk_command_done);
	void disk_dma_write(uint16_t dest, uint32_t offset, uint8_t data);

	void xscroll_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	void yscroll_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	void spriteram_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	void playfield_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);

	TILE_GET_INFO_MEMBER(get_alpha_tile_info);
	TILE_GET_INFO_MEMBER(get_playfield_tile_info);
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	static rgb_t RRRRGGGGBBBBIIII(uint32_t raw);

	void main_map(address_map &map) ATTR_COLD;

	static const atari_motion_objects_config s_mob_config;
};


/*************************************
 *
 *  Interrupts
 *
 *  IRQ3 (VBLANK) and IRQ2 (32V) are modeled as on System 2; the program's
 *  IRQ3 handler counts frames and runs the game, the IRQ2 handler counts
 *  32V bands and relinks motion object 0 for each band.
 *
 *************************************/

void garfield_state::update_interrupts()
{
	m_maincpu->set_input_line(t11_device::CP3_LINE, m_video_int_state ? ASSERT_LINE : CLEAR_LINE);
	m_maincpu->set_input_line(t11_device::CP2_LINE, m_scanline_int_state ? ASSERT_LINE : CLEAR_LINE);
}


void garfield_state::int_enable_w(uint8_t data)
{
	m_interrupt_enable = data;
}


void garfield_state::scanline_int_ack_w(uint16_t data)
{
	m_scanline_int_state = false;
	update_interrupts();
}


void garfield_state::video_int_ack_w(uint16_t data)
{
	m_video_int_state = false;
	update_interrupts();
}


void garfield_state::unused_int_ack_w(uint16_t data)
{
	// on System 2, IRQ0/IRQ1 come from the sound board (not emulated); assumed the same here
}


TIMER_DEVICE_CALLBACK_MEMBER(garfield_state::scanline_update)
{
	int const scanline = param;
	if (scanline <= m_screen->height())
	{
		m_scanline_int_state = BIT(m_interrupt_enable, 2);
		update_interrupts();
	}
}


void garfield_state::vblank_int(int state)
{
	if (state)
	{
		m_video_int_state = BIT(m_interrupt_enable, 3);
		update_interrupts();
	}
}


/*************************************
 *
 *  Disk controller
 *
 *************************************/

void garfield_state::disk_select_w(uint16_t data)
{
	LOGDISK("%s: disk select/reset %04x\n", machine().describe_context(), data);

	if (BIT(data, 1))
	{
		m_disk_timer->adjust(attotime::never);
		return;
	}

	if (!BIT(data, 0))
		return;

	address_space &space = m_maincpu->space(AS_PROGRAM);
	uint8_t cdb[6];
	for (int i = 0; i < 6; i += 2)
	{
		uint16_t const word = space.read_word(0x100 + i);
		cdb[i] = word >> 8;
		cdb[i + 1] = word & 0xff;
	}
	uint16_t const dest = space.read_word(0x106);
	uint32_t const lba = ((cdb[1] & 0x1f) << 16) | (cdb[2] << 8) | cdb[3];
	uint32_t const blocks = cdb[4] ? cdb[4] : 256;

	LOGDISK("disk command %02x %02x %02x %02x %02x %02x -> %06o (LBA %u, %u blocks)\n",
			cdb[0], cdb[1], cdb[2], cdb[3], cdb[4], cdb[5], dest, lba, blocks);

	uint16_t status = 0;
	switch (cdb[0])
	{
	case 0x08: // READ
		if (!m_disk)
		{
			status = 0x0200;
			break;
		}
		{
			std::vector<uint8_t> buffer(blocks * 256);
			if (m_disk->read_bytes(uint64_t(lba) * 256, buffer.data(), buffer.size()))
			{
				status = 0x0200;
				break;
			}
			for (uint32_t i = 0; i < buffer.size(); i++)
				disk_dma_write(dest, i, buffer[i]);
		}
		break;

	case 0x03: // REQUEST SENSE
		for (uint32_t i = 0; i < cdb[4]; i++)
			space.write_byte((dest + i) & 0xffff, 0);
		break;

	case 0x04: // FORMAT UNIT
	case 0x0a: // WRITE
		logerror("disk command %02x not implemented\n", cdb[0]);
		status = 0x0200;
		break;

	default:
		logerror("unknown disk command %02x\n", cdb[0]);
		status = 0x0200;
		break;
	}

	// The hardware description quotes a 246K bytes/second maximum transfer
	// rate, but the program's completion poll gives up after roughly the time
	// a 64K transfer would take at that rate, so complete well inside that.
	attotime const delay = (cdb[0] == 0x08)
			? attotime::from_usec(100) * blocks
			: attotime::from_usec(500);
	m_disk_timer->adjust(delay, status);
}


TIMER_CALLBACK_MEMBER(garfield_state::disk_command_done)
{
	LOGDISK("disk command done, status %04x\n", param);
	m_maincpu->space(AS_PROGRAM).write_word(0x108, param);
}


void garfield_state::disk_dma_write(uint16_t dest, uint32_t offset, uint8_t data)
{
	if (BIT(dest, 15))
	{
		// graphics DRAM
		uint32_t const addr = ((dest & 0x7fff) << 3) + offset;
		if (addr < m_gfxram.bytes())
		{
			// Store pixels inverted to map source pen 0 to transparent pen 15,
			// modeling the inferred color addressing described above.
			m_gfxram[addr] = ~data;
			m_gfxdecode->gfx(1)->mark_dirty(addr / 128);
			m_gfxdecode->gfx(2)->mark_dirty(addr / 128);
		}
		else
		{
			LOGDISK("DMA beyond graphics DRAM: %06o + %05x\n", dest, offset);
		}
	}
	else
	{
		m_maincpu->space(AS_PROGRAM).write_byte((dest + offset) & 0xffff, data);
	}
}


/*************************************
 *
 *  Video
 *
 *************************************/

rgb_t garfield_state::RRRRGGGGBBBBIIII(uint32_t raw)
{
	// System 2 color RAM format, except for the intensity polarity
	static constexpr int ZB = 115, Z3 = 78, Z2 = 37, Z1 = 17, Z0 = 9;

	static constexpr int intensity_table[16] =
	{
		0, ZB+Z0, ZB+Z1, ZB+Z1+Z0, ZB+Z2, ZB+Z2+Z0, ZB+Z2+Z1, ZB+Z2+Z1+Z0,
		ZB+Z3, ZB+Z3+Z0, ZB+Z3+Z1, ZB+Z3+Z1+Z0,ZB+ Z3+Z2, ZB+Z3+Z2+Z0, ZB+Z3+Z2+Z1, ZB+Z3+Z2+Z1+Z0
	};

	static constexpr int color_table[16] =
	{
		0x0, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xe, 0xf, 0xf
	};

	// The program's gray ramps and sprite palettes use intensity 0, so the
	// intensity bits are treated as active low here (inferred; System 2 is
	// active high).
	int const i = intensity_table[~raw & 15];
	uint8_t const r = (color_table[(raw >> 12) & 15] * i) >> 4;
	uint8_t const g = (color_table[(raw >> 8) & 15] * i) >> 4;
	uint8_t const b = (color_table[(raw >> 4) & 15] * i) >> 4;

	return rgb_t(r, g, b);
}


TILE_GET_INFO_MEMBER(garfield_state::get_alpha_tile_info)
{
	uint16_t const data = m_alpha_tilemap->basemem_read(tile_index);
	int const code = data & 0x3ff;
	// Apply the inferred color-index inversion to the alpha layer too
	// (pen inverted when the ROM is loaded, low two color bits here).
	int const color = ((data >> 13) & 0x07) ^ 3;
	tileinfo.set(0, code, color, 0);
}


TILE_GET_INFO_MEMBER(garfield_state::get_playfield_tile_info)
{
	uint16_t const data = m_playfield[tile_index];
	int const code = data & 0x3ff;
	int const color = (data >> 11) & 7;
	tileinfo.set(2, code, color, 0);
	tileinfo.category = (~data >> 14) & 3;
}


void garfield_state::xscroll_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	uint16_t const old = *m_xscroll;
	COMBINE_DATA(m_xscroll.target());
	if (*m_xscroll != old)
		m_screen->update_partial(m_screen->vpos());
	m_playfield_tilemap->set_scrollx(0, *m_xscroll >> 6);
}


void garfield_state::yscroll_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	uint16_t const old = *m_yscroll;
	COMBINE_DATA(m_yscroll.target());
	if (*m_yscroll != old)
		m_screen->update_partial(m_screen->vpos());
	m_playfield_tilemap->set_scrolly(0, *m_yscroll >> 6);
}


void garfield_state::spriteram_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	// the 32V handler rewrites the link of object 0 for each band
	if (offset == 0x0003)
		m_screen->update_partial(m_screen->vpos());
	COMBINE_DATA(&m_mob->spriteram()[offset]);
}


void garfield_state::playfield_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	COMBINE_DATA(&m_playfield[offset]);
	m_playfield_tilemap->mark_tile_dirty(offset);
}


uint32_t garfield_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	m_mob->draw_async(cliprect);

	bitmap_ind8 &priority_bitmap = screen.priority();
	priority_bitmap.fill(0, cliprect);

	m_playfield_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	m_playfield_tilemap->draw(screen, bitmap, cliprect, 1, 1);
	m_playfield_tilemap->draw(screen, bitmap, cliprect, 2, 2);
	m_playfield_tilemap->draw(screen, bitmap, cliprect, 3, 3);

	// System 2 priority rule; pixel data is inverted, so bit 3 is tested on the un-inverted pen
	bitmap_ind16 &mobitmap = m_mob->bitmap();
	m_mob->iterate_dirty_rects(
			cliprect,
			[&bitmap, &priority_bitmap, &mobitmap] (rectangle const &rect)
			{
				for (int y = rect.top(); y <= rect.bottom(); y++)
				{
					uint16_t const *const mo = &mobitmap.pix(y);
					uint16_t *const pf = &bitmap.pix(y);
					uint8_t const *const pri = &priority_bitmap.pix(y);
					for (int x = rect.left(); x <= rect.right(); x++)
					{
						if (mo[x] != 0xffff)
						{
							int const mopriority = mo[x] >> atari_motion_objects_device::PRIORITY_SHIFT;
							if ((mopriority + pri[x]) & 2)
							{
								if (!((pf[x] ^ 0x0f) & 0x08))
									pf[x] = mo[x] & atari_motion_objects_device::DATA_MASK;
							}
							else
								pf[x] = mo[x] & atari_motion_objects_device::DATA_MASK;
						}
					}
				}
			});

	m_alpha_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	return 0;
}


// motion objects use the System 2 format; pictures come from the 1024-stamp graphics DRAM
const atari_motion_objects_config garfield_state::s_mob_config =
{
	1,                  // index to which gfx system
	1,                  // number of motion object banks
	1,                  // are the entries linked?
	0,                  // are the entries split?
	0,                  // render in reverse order?
	0,                  // render in swapped X/Y order?
	0,                  // does the neighbor bit affect the next object?
	0,                  // pixels per SLIP entry (0 for no-slip)
	0,                  // pixel offset for SLIPs
	0,                  // maximum number of links to visit/scanline (0=all)

	0x00,               // base palette entry
	15,                 // transparent pen index

	{{ 0,0,0,0x07f8 }}, // mask for the link
	{{ 0,0x03ff,0,0 }}, // mask for the code index
	{{ 0,0,0,0x3000 }}, // mask for the color
	{{ 0,0,0xffc0,0 }}, // mask for the X position
	{{ 0x7fc0,0,0,0 }}, // mask for the Y position
	{{ 0 }},            // mask for the width, in tiles
	{{ 0,0x3800,0,0 }}, // mask for the height, in tiles
	{{ 0,0x4000,0,0 }}, // mask for the horizontal flip
	{{ 0 }},            // mask for the vertical flip
	{{ 0,0,0,0xc000 }}, // mask for the priority
	{{ 0,0x8000,0,0 }}, // mask for the neighbor
	{{ 0 }},            // mask for absolute coordinates

	{{ 0 }},            // mask for the special value
	0                   // resulting value to indicate "special"
};


// 16x16 stamps, four bitplanes, each row stored as four little-endian 16-bit plane words
static const gfx_layout stamplayout =
{
	16,16,
	1024,
	4,
	{ 0, 16, 32, 48 },
	{ STEP8(8,1), STEP8(0,1) },
	{ STEP16(0,64) },
	16*64
};


static const gfx_layout anlayout =
{
	8,8,
	RGN_FRAC(1,1),
	2,
	{ 0, 4 },
	{ STEP4(0,1), STEP4(8,1) },
	{ STEP8(0,8*2) },
	8*8*2
};


static GFXDECODE_START( gfx_garfield )
	GFXDECODE_ENTRY( "chars",  0, anlayout,   64, 8 )
	GFXDECODE_RAM( "gfxram", 0, stamplayout, 0, 4 )    // motion objects
	GFXDECODE_RAM( "gfxram", 0, stamplayout, 128, 8 )  // playfield
GFXDECODE_END


void garfield_state::machine_start()
{
	m_disk = machine().rom_load().get_disk_handle(":disk");
	m_disk_timer = timer_alloc(FUNC(garfield_state::disk_command_done), this);

	save_item(NAME(m_interrupt_enable));
	save_item(NAME(m_scanline_int_state));
	save_item(NAME(m_video_int_state));
}


void garfield_state::machine_reset()
{
	m_interrupt_enable = 0;
	m_scanline_int_state = false;
	m_video_int_state = false;
	update_interrupts();

	m_disk_timer->adjust(attotime::never);
}


/*************************************
 *
 *  Main CPU memory map
 *
 *************************************/

void garfield_state::main_map(address_map &map)
{
	map.unmap_value_high();
	map(0000000, 0007777).ram();
	map(0010000, 0010777).ram().w(m_palette, FUNC(palette_device::write16)).share("palette");
	map(0012000, 0012001).w(FUNC(garfield_state::unused_int_ack_w));
	map(0012200, 0012201).w(FUNC(garfield_state::unused_int_ack_w));
	map(0012400, 0012401).w(FUNC(garfield_state::scanline_int_ack_w));
	map(0012600, 0012601).w(FUNC(garfield_state::video_int_ack_w));
	map(0013000, 0013000).w(FUNC(garfield_state::int_enable_w));
	map(0013200, 0013200).w(m_soundlatch, FUNC(generic_latch_8_device::write));
	map(0013400, 0013401).w(FUNC(garfield_state::xscroll_w)).share(m_xscroll);
	map(0013600, 0013601).w(FUNC(garfield_state::yscroll_w)).share(m_yscroll);
	map(0014000, 0014001).portr("IN0");
	map(0014002, 0014003).portr("IN1");
	map(0016000, 0016000).r(m_mainlatch, FUNC(generic_latch_8_device::read));
	map(0020000, 0033777).ram().w(m_alpha_tilemap, FUNC(tilemap_device::write16)).share("alpha");
	map(0034000, 0037777).ram().w(FUNC(garfield_state::spriteram_w)).share("mob");
	map(0040000, 0047777).ram().w(FUNC(garfield_state::playfield_w)).share(m_playfield);
	map(0050000, 0050001).w(FUNC(garfield_state::disk_select_w));
	map(0100000, 0177777).rom();
}


/*************************************
 *
 *  Input ports
 *
 *************************************/

static INPUT_PORTS_START( garfield )
	PORT_START("IN0")
	// joystick nibble feeds a 16-direction lookup table; orientation inferred from that table
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN )
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_JOYSTICK_UP )
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT )
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT )
	PORT_BIT( 0x0030, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_BUTTON2 )
	PORT_BIT( 0xff00, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("IN1")
	PORT_DIPNAME( 0x0001, 0x0001, "Debug: IRQ timing colors" )
	PORT_DIPSETTING(      0x0001, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0002, 0x0002, "Debug: freeze" )
	PORT_DIPSETTING(      0x0002, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0004, 0x0004, "Debug: unknown" )
	PORT_DIPSETTING(      0x0004, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0008, 0x0008, "Debug: clear colors every frame" )
	PORT_DIPSETTING(      0x0008, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_BIT( 0xfff0, IP_ACTIVE_LOW, IPT_UNKNOWN )
INPUT_PORTS_END


/*************************************
 *
 *  Machine driver
 *
 *************************************/

void garfield_state::garfield(machine_config &config)
{
	constexpr XTAL VIDEO_CLOCK = XTAL(32'000'000);

	T11(config, m_maincpu, MASTER_CLOCK/2);
	m_maincpu->set_initial_mode(0x36ff);
	m_maincpu->set_addrmap(AS_PROGRAM, &garfield_state::main_map);

	TIMER(config, "scantimer").configure_scanline(FUNC(garfield_state::scanline_update), m_screen, 0, 64);

	GFXDECODE(config, m_gfxdecode, m_palette, gfx_garfield);
	PALETTE(config, m_palette).set_format(2, &garfield_state::RRRRGGGGBBBBIIII, 256);

	TILEMAP(config, m_playfield_tilemap, m_gfxdecode, 2, 16,16, TILEMAP_SCAN_ROWS, 64,32).set_info_callback(FUNC(garfield_state::get_playfield_tile_info));
	TILEMAP(config, m_alpha_tilemap, m_gfxdecode, 2, 8,8, TILEMAP_SCAN_ROWS, 64,48, 3).set_info_callback(FUNC(garfield_state::get_alpha_tile_info));

	ATARI_MOTION_OBJECTS(config, m_mob, m_screen, garfield_state::s_mob_config);
	m_mob->set_gfxdecode(m_gfxdecode);

	SCREEN(config, m_screen);
	m_screen->set_video_attributes(VIDEO_UPDATE_BEFORE_VBLANK);
	m_screen->set_raw(VIDEO_CLOCK/2, 640, 0, 512, 416, 0, 384);
	m_screen->set_screen_update(FUNC(garfield_state::screen_update));
	m_screen->set_palette(m_palette);
	m_screen->screen_vblank().set(FUNC(garfield_state::vblank_int));

	// Sound hardware is unresolved (Quad POKEY in a 2020 account, Yamaha in SPEC.TXT).
	// No sound program is available; keep the latches.
	GENERIC_LATCH_8(config, m_soundlatch);
	GENERIC_LATCH_8(config, m_mainlatch);
}


/*************************************
 *
 *  ROM definitions
 *
 *************************************/

ROM_START( garfield )
	// rebuilt from source, not dumped from hardware
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "garfield_n1.bin", 0x8000, 0x2000, BAD_DUMP CRC(80ad484c) SHA1(17bd0893769586fd90cee28190f80927ffaab2f5) )
	ROM_LOAD16_BYTE( "garfield_l1.bin", 0x8001, 0x2000, BAD_DUMP CRC(0393655b) SHA1(910670644a0cab40bf513ca39bad225d96e283b4) )
	ROM_LOAD16_BYTE( "garfield_j1.bin", 0xc000, 0x2000, BAD_DUMP CRC(71060f8d) SHA1(1d1d93cf0f8d94832aaf8b64f04c2aace0dd67c0) )
	ROM_LOAD16_BYTE( "garfield_h1.bin", 0xc001, 0x2000, BAD_DUMP CRC(b0caaea5) SHA1(08a31f49f55bc37c4827a5a29540d118bf49ff6d) )

	// SPEC.TXT lists 1024 8x8, 2bpp alpha stamps (16K); ROM packaging is unknown.
	// Invert alpha pixels to model the same inferred color addressing.
	ROM_REGION( 0x4000, "chars", ROMREGION_ERASE00 | ROMREGION_INVERT )
	ROM_LOAD( "garfield_alpha.bin", 0x0000, 0x4000, NO_DUMP )

	DISK_REGION( "disk" )
	DISK_IMAGE_READONLY( "garfield", 0, NO_DUMP )
ROM_END

} // anonymous namespace


GAME( 1984, garfield, 0, garfield, garfield, garfield_state, empty_init, ROT0, "Atari", "Garfield's Quest (prototype)", MACHINE_NOT_WORKING | MACHINE_IS_INCOMPLETE | MACHINE_NO_SOUND | MACHINE_IMPERFECT_GRAPHICS )
