/*
 *  jack_cod3035_sysfs_cb.c
 *  Copyright (c) Samsung Electronics
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/input.h>
#include <linux/extcon/extcon-madera.h>
#include <linux/mfd/madera/core.h>
#include <sound/soc.h>
#include <sound/samsung/sec_audio_sysfs.h>
#include "jack_cod3035_sysfs_cb.h"
#include "../codecs/cod3035x.h"

static struct snd_soc_codec *cod3035_codec;
int cod3035_jack_det;
int cod3035_ear_mic;

static int get_jack_status(void)
{
	struct snd_soc_codec *codec = cod3035_codec;
	struct cod3035x_priv *cod3035x = dev_get_drvdata(codec->dev);
	struct cod3035x_jack_det *jackdet = &cod3035x->jack_det;
	int status = jackdet->jack_det;
	int report = 0;
	
	if (status) {
	    report = 1;
	}

	dev_info(codec->dev, "%s: %d\n", __func__, report);

	return report;
}

static int get_key_status(void)
{
	struct snd_soc_codec *codec = cod3035_codec;
	struct cod3035x_priv *cod3035x = dev_get_drvdata(codec->dev);
	struct cod3035x_jack_det *jackdet = &cod3035x->jack_det;
	int report = 0;

	report = jackdet->button_det ? true : false;

	dev_info(codec->dev, "%s: %d\n", __func__, report);

	return report;
}

static int get_mic_adc(void)
{
	struct snd_soc_codec *codec = cod3035_codec;
	struct cod3035x_priv *cod3035x = dev_get_drvdata(codec->dev);
	struct cod3035x_jack_det *jackdet = &cod3035x->jack_det;
	int adc = 0;

	adc = jackdet->adc_val;

	dev_info(codec->dev, "%s: %d\n", __func__, adc);

	return adc;
}

void register_cod3035_jack_cb(struct snd_soc_codec *codec)
{
	cod3035_codec = codec;

	audio_register_jack_state_cb(get_jack_status);
	audio_register_key_state_cb(get_key_status);
	audio_register_mic_adc_cb(get_mic_adc);
}
EXPORT_SYMBOL_GPL(register_cod3035_jack_cb);