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
 * @file wk2132.c
 *
 * Bring-up pro DFRobot Gravity DFR0627 (I2C to Dual UART, cip WK2132) na
 * Cube Orange. NAVRH/DRAFT - nebylo overeno na realnem hardwaru, viz
 * caveaty v include/nuttx/serial/wk2132.h a drivers/serial/wk2132.c.
 *
 * Predpoklady k overeni/upraveni podle skutecneho zapojeni:
 *   - Modul je na I2C2 (viz src/i2c.cpp: initI2CBusExternal(2)) - zmen
 *     WK2132_I2C_BUS, pokud je jinde.
 *   - DIP prepinace IA1/IA0 na modulu: podle fyzicke kontroly oba na "1"
 *     (WK2132_ADDR_BASE = 0x60 = (1<<6)|(1<<5)). Pokud "i2cdetect -b 2"
 *     ukaze neco jineho nez 0x70-0x73, uprav podle rozboru adresovani v
 *     include/nuttx/serial/wk2132.h.
 *   - Zadny IRQ pin nikam nejde (jen standardni 4pinovy I2C konektor) -
 *     driver bezi v POLLED rezimu, viz komentare v jeho zdrojaku.
 */

#include "board_config.h"

#include <errno.h>
#include <syslog.h>

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/serial/wk2132.h>

#include <px4_arch/micro_hal.h>

#ifdef CONFIG_WK2132_UART

#define WK2132_I2C_BUS          2         /* I2C2 (externi) - viz src/i2c.cpp */
#define WK2132_I2C_FREQUENCY    400000

/* DIP prepinace na modulu: IA1=1, IA0=1 (fyzicky zkontrolovano na desce).
 * addr_base = (IA1<<6)|(IA0<<5) = (1<<6)|(1<<5) = 0x60.
 * Vysledne I2C adresy (viz WK2132_I2C_ADDR v wk2132.h): registr/FIFO kanalu
 * 1 = 0x70/0x71, registr/FIFO kanalu 2 = 0x72/0x73 - tyhle 4 by mel ukazat
 * "i2cdetect -b 2". */

#define WK2132_ADDR_BASE        0x60

/****************************************************************************
 * Name: board_wk2132_initialize
 *
 * Description:
 *   Ziska I2C2 sbernici, nainicializuje cip WK2132 na DFR0627 modulu a
 *   zaregistruje oba jeho kanaly jako /dev/ttyWK1 a /dev/ttyWK2. Vola se
 *   z board_app_initialize() (viz init.c) po px4_platform_init(), aby uz
 *   I2C sbernice bezela.
 *
 ****************************************************************************/

int board_wk2132_initialize(void)
{
	/* 'lower' musi mit staticky/trvaly zivotni cyklus - wk2132_instantiate()
	 * si jen uklada UKAZATEL na ni, nekopiruje obsah (stejny vzor jako u
	 * ostatnich NuttX "lower half" konfiguraci). */

	static struct wk2132_lower_s lower;
	FAR struct i2c_master_s *i2c;
	FAR struct wk2132_dev_s *chip;
	int ret;

	i2c = px4_i2cbus_initialize(WK2132_I2C_BUS);

	if (i2c == NULL) {
		syslog(LOG_ERR, "[boot] WK2132: I2C%d init selhalo\n", WK2132_I2C_BUS);
		return -ENODEV;
	}

	lower.i2c           = i2c;
	lower.addr_base     = WK2132_ADDR_BASE;
	lower.i2c_frequency = WK2132_I2C_FREQUENCY;
	lower.xtal_freq     = WK2132_FOSC_DFR0627_HZ;
	lower.reset         = NULL; /* DFR0627 nevyvadi externi reset pin */

	chip = wk2132_instantiate(&lower);

	if (chip == NULL) {
		syslog(LOG_ERR, "[boot] WK2132: cip neodpovida na I2C%d "
		       "(over DIP prepinace IA1/IA0 na modulu a WK2132_ADDR_BASE)\n",
		       WK2132_I2C_BUS);
		return -ENODEV;
	}

	ret = wk2132_register("/dev/ttyWK1", chip, WK2132_CHAN_1);

	if (ret < 0) {
		syslog(LOG_ERR, "[boot] WK2132: /dev/ttyWK1 registrace selhala: %d\n", ret);
	}

	ret = wk2132_register("/dev/ttyWK2", chip, WK2132_CHAN_2);

	if (ret < 0) {
		syslog(LOG_ERR, "[boot] WK2132: /dev/ttyWK2 registrace selhala: %d\n", ret);
	}

	syslog(LOG_INFO, "[boot] WK2132: cip nalezen na I2C%d, /dev/ttyWK1 a "
	       "/dev/ttyWK2 zaregistrovany\n", WK2132_I2C_BUS);

	return OK;
}

#endif /* CONFIG_WK2132_UART */
