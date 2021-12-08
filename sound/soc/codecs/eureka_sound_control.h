/*
 * eureka_sound_control.h  --  Sound Control for SMA1301 sound driver inspired from MoroSound driver
 *
 * Author: @Chatur27 - https://github.com/chatur27
 *
 */

#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <sound/soc.h>

#include <sound/cod3035x.h>
#include "cod3035x.h"
#include "sma1301.h"

/* External function declarations */
void eureka_sound_control_hook_probe(struct regmap *pmap);
int _regmap_write_nohook(struct regmap *map, unsigned int reg, unsigned int val);

/*
 * Default Vars
 */

#define EUREKA_SOUND_ON			0
#define EUREKA_SOUND_VERSION		"1.0"

/* Headphone */
#define HEADPHONE_DEFAULT_BALANCE	94
#define HEADPHONE_MIN_BALANCE		94
#define HEADPHONE_MAX_BALANCE		124
#define HEADPHONE_DEFAULT_GAIN		0
#define HEADPHONE_MIN_GAIN		0
#define HEADPHONE_MAX_GAIN		20

/* Mono Mix Mode */
#define MONO_MIX_DEFAULT		0
#define MONO_MIX_MIN_LIMIT		0
#define MONO_MIX_MAX_LIMIT		5
