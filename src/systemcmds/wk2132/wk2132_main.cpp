/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file wk2132_main.cpp
 *
 * Vypis stavu kanalu WK2132 (I2C na dva UARTy, viz
 * platforms/nuttx/.../drivers/serial/wk2132.c).
 *
 * Proc samostatny prikaz: u pollovaneho I2C mustku nejde stav zmerit
 * zvenku. Registry FSR/RFCNT sice jdou precist i rucne nastrojem "i2c",
 * jenze poll smycka driveru cte tytez registry kazdych
 * CONFIG_WK2132_POLL_MS a prichozi data hned vysype - rucni cteni tak
 * temer vzdy ukaze nulu bez ohledu na to, jestli prijem funguje.
 * Rozhodujici je kumulativni "rx", ktere vidi jen driver sam.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <drivers/drv_hrt.h>

#include <nuttx/serial/wk2132.h>

#include <string.h>

extern "C" __EXPORT int wk2132_main(int argc, char *argv[]);

static void print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Show the state of each WK2132 I2C-to-UART channel: the chip's configuration
registers plus the driver's cumulative byte and error counters.

The counters are the useful part. Reading FSR/RFCNT by hand with the `i2c`
tool almost always shows zero, because the driver's poll loop drains incoming
data before anyone can look at it - only the cumulative `rx` tells you whether
the chip is receiving anything at all.

### Examples
$ wk2132
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME_SIMPLE("wk2132", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print channel status (default)");
}

static void print_channel(const wk2132_status_s &s)
{
	PX4_INFO("%s  ch%u  %s  %lu baud",
		 s.devpath ? s.devpath : "?",
		 (unsigned)s.channel,
		 s.opened ? "otevreny" : "zavreny",
		 (unsigned long)s.baud);

	PX4_INFO("  GENA=%02x SPAGE=%02x SCR=%02x LCR=%02x FCR=%02x",
		 s.gena, s.spage, s.scr, s.lcr, s.fcr);

	PX4_INFO("  FSR=%02x LSR=%02x RFCNT=%02x TFCNT=%02x",
		 s.fsr, s.lsr, s.rfcnt, s.tfcnt);

	// polls + t: spustit prikaz dvakrat a vydelit rozdily mezi sebou - to
	// da skutecnou frekvenci pollovani. Kdyz vyjde vyrazne min nez
	// 1000/CONFIG_WK2132_POLL_MS za sekundu, vlakno se nedostava ke slovu a
	// nastaveny interval je jen zbozne prani - viz WK2132_POLL_PRIORITY.
	//
	// Cas se tiskne prave proto, aby to sla spocitat: perf citace v "dshot
	// status" k tomu pouzit nejdou, jejich "elapsed" je soucet zmerenych
	// intervalu, ne uplynuly cas.
	PX4_INFO("  polls=%lu  t=%llu ms",
		 (unsigned long)s.polls,
		 (unsigned long long)(hrt_absolute_time() / 1000));

	// regerr je tu proto, aby sla "cip nic neprijima" odlisit od "nedari
	// se precist RFCNT" - obojim vyjde rx=0 a jinak zadna chyba.
	PX4_INFO("  rx=%lu tx=%lu drop=%lu rxerr=%lu regerr=%lu txerr=%lu rxint=%d txint=%d",
		 (unsigned long)s.rxbytes,
		 (unsigned long)s.txbytes,
		 (unsigned long)s.rxdropped,
		 (unsigned long)s.rxerrors,
		 (unsigned long)s.regerrors,
		 (unsigned long)s.txerrors,
		 (int)s.rxint,
		 (int)s.txint);

	// Vyhodnoceni toho, co je na tech cislech spatne. Bez nej se dva
	// naprosto ruzne problemy - "cip neprijima" a "cipu se nepovedlo ani
	// nastavit" - tvari uplne stejne: same nuly.

	if (s.gena == 0xff) {
		PX4_ERR("  -> registr GENA se nepodarilo precist: cip neodpovida na I2C");

	} else if ((s.gena & 0x03) == 0) {
		PX4_ERR("  -> GENA=%02x: hodiny sub-UARTu jsou vypnute, zapis do GENA se neulozil.",
			s.gena);
		PX4_ERR("     S vypnutymi hodinami je cely kanalovy blok mrtvy a vsechny");
		PX4_ERR("     registry nize ctou nuly. Typicka pricina je prilis vysoka");
		PX4_ERR("     rychlost I2C na dane kabelaz - viz WK2132_I2C_FREQUENCY.");

	} else if (s.opened && s.scr != (WK2132_SCR_RXEN | WK2132_SCR_TXEN)) {
		PX4_ERR("  -> SCR=%02x misto 03: konfigurace kanalu se neulozila", s.scr);

	} else if (s.opened && s.polls == 0) {
		PX4_ERR("  -> polls=0: poll vlakno tenhle kanal neobsluhuje");
	}
}

int wk2132_main(int argc, char *argv[])
{
	if (argc > 1 && strcmp(argv[1], "status") != 0) {
		print_usage();
		return 1;
	}

	int found = 0;

	for (int i = 0; ; i++) {
		wk2132_status_s status;

		if (wk2132_get_status(i, &status) != 0) {
			break;
		}

		print_channel(status);
		++found;
	}

	if (found == 0) {
		PX4_WARN("zadny WK2132 kanal neni zaregistrovany");
		return 1;
	}

	return 0;
}
