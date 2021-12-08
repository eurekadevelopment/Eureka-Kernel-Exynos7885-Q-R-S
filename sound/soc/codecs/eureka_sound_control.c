/*
 * eureka_sound_control.c  --  Sound Control for SMA1301 & COD3035X sound drivers inspired from MoroSound driver
 *
 * Author: @Chatur27 - https://github.com/chatur27
 *
 * Date: 18 December 2021 - v1.0
 *
 * Based on MoroSound 2.1.1 for Galaxy S8 and Boeffla Sound 1.6 for Galaxy S3
 *
 * Credits: 	andip71, author of Boeffla Sound
 *		Supercurio, Yank555 and Gokhanmoral.
 *		AndreiLux, for his Madera control sound mod
 *		Flar2, for his speaker gain mod
 *		morogoku, author of MoroSound
 *
 */

#include "eureka_sound_control.h"

/* Variables */
static struct regmap *map;
static int hp_balance_l, hp_balance_r, hp_lr_gain;
static int mono_mix_mode;
static int hp_mute_reg, hp_mute_off;

static int eureka_sound_control_on = 0;

static void init_audio_hub(void);
static void reset_audio_hub(void);
static void update_audio_hub(void);

/* Internal helper functions */
#define eureka_regmap_write(reg, val) _regmap_write_nohook(map, reg, val)
#define eureka_regmap_read(reg, val) regmap_read(map, reg, val)

static void headphone_balance_l(int gain)
{
	unsigned int val;

	eureka_regmap_read(COD3035X_51_DVOLL, &val);
	val &= ~CTVOL_HP_AVCBYPASS_MASK;
	val |= (gain << CTVOL_HP_AVCBYPASS_SHIFT);
	eureka_regmap_write(COD3035X_51_DVOLL, val);
}

static void headphone_balance_r(int gain)
{
	unsigned int val;

	eureka_regmap_read(COD3035X_52_DVOLR, &val);
	val &= ~CTVOL_HP_AVCBYPASS_MASK;
	val |= (gain << CTVOL_HP_AVCBYPASS_SHIFT);
	eureka_regmap_write(COD3035X_52_DVOLR, val);
}

static void headphone_lr_gain(int gain)
{
	unsigned int val;

	eureka_regmap_read(COD3035X_E5_PRESET_AVC, &val);
	val &= ~CTVOL_HP_AVCBYPASS_MASK;
	val |= (gain << CTVOL_HP_AVCBYPASS_SHIFT);
	eureka_regmap_write(COD3035X_E5_PRESET_AVC, val);
}

static void mono_mix_control(int mode)
{
	unsigned int val;

	eureka_regmap_read(COD3035X_50_DAC1, &val);
	val &= ~DAC1_MONOMIX_MASK;
	val |= mode << DAC1_MONOMIX_SHIFT;
	eureka_regmap_write(COD3035X_50_DAC1, val);
}

static void headphone_analog_mute(int data)
{
	unsigned int val;

	eureka_regmap_read(COD3035X_37_MIX_DA1, &val);
	if (data == 1) {
		val &= ~EN_HP_MIXL_DCTL_MASK;
		val |= (hp_mute_off << EN_HP_MIXL_DCTL_SHIFT);
	} else if (data == 2) {
		val &= ~EN_HP_MIXL_DCTR_MASK;
		val |= (hp_mute_off << EN_HP_MIXL_DCTR_SHIFT);
	} else if (data == 3) {
		val &= ~EN_HP_MIXR_DCTR_MASK;
		val |= (hp_mute_off << EN_HP_MIXR_DCTR_SHIFT);
	}
	eureka_regmap_write(COD3035X_37_MIX_DA1, val);
}

/* Sound hook functions */
void eureka_sound_control_hook_probe(struct regmap *pmap)
{
	map = pmap;
	eureka_sound_control_on = EUREKA_SOUND_ON;

	if (eureka_sound_control_on)
		init_audio_hub();
}

unsigned int eureka_sound_control_write_hook(unsigned int reg, unsigned int val)
{
	if (!eureka_sound_control_on)
		return val;

	switch (reg) {
		case COD3035X_51_DVOLL:
			val &= ~CTVOL_HP_AVCBYPASS_MASK;
			val |= (hp_balance_l << CTVOL_HP_AVCBYPASS_SHIFT);
			break;
		case COD3035X_52_DVOLR:
			val &= ~CTVOL_HP_AVCBYPASS_MASK;
			val |= (hp_balance_r << CTVOL_HP_AVCBYPASS_SHIFT);
			break;
		case COD3035X_E5_PRESET_AVC:
			val &= ~CTVOL_HP_AVCBYPASS_MASK;
			val |= (hp_lr_gain << CTVOL_HP_AVCBYPASS_SHIFT);
			break;
		case COD3035X_50_DAC1:
			val &= ~DAC1_MONOMIX_MASK;
			val |= (mono_mix_mode << DAC1_MONOMIX_SHIFT);
			break;
		case COD3035X_37_MIX_DA1:
			if (hp_mute_reg == 1) {
				val &= ~EN_HP_MIXL_DCTL_MASK;
				val |= (hp_mute_off << EN_HP_MIXL_DCTL_SHIFT);
			} else if (hp_mute_reg == 2) {
				val &= ~EN_HP_MIXL_DCTR_MASK;
				val |= (hp_mute_off << EN_HP_MIXL_DCTR_SHIFT);
			} else if (hp_mute_reg == 3) {
				val &= ~EN_HP_MIXR_DCTR_MASK;
				val |= (hp_mute_off << EN_HP_MIXR_DCTR_SHIFT);
			}
			break;
		default:
			break;
	}

	return val;
}

/* Initialization functions */
static void init_audio_hub(void)
{
	hp_balance_l = HEADPHONE_DEFAULT_BALANCE;
	hp_balance_r = HEADPHONE_DEFAULT_BALANCE;
	hp_lr_gain = HEADPHONE_DEFAULT_GAIN;
	mono_mix_mode = MONO_MIX_DEFAULT;
	hp_mute_reg = 1;	// Start at first mute reg by default
	hp_mute_off = 1;	// HP is unmuted by default
}

static void reset_audio_hub(void)
{
	headphone_balance_l(HEADPHONE_DEFAULT_BALANCE);
	headphone_balance_r(HEADPHONE_DEFAULT_BALANCE);
	headphone_lr_gain(HEADPHONE_DEFAULT_GAIN);
	mono_mix_control(MONO_MIX_DEFAULT);
	headphone_analog_mute(HEADPHONE_DEFAULT_GAIN);
}

static void update_audio_hub(void)
{
	headphone_balance_l(hp_balance_l);
	headphone_balance_r(hp_balance_r);
	headphone_lr_gain(hp_lr_gain);
	mono_mix_control(mono_mix_mode);
	headphone_analog_mute(hp_mute_reg);
}

/* sysfs interface functions */
static ssize_t eureka_sound_control_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", eureka_sound_control_on);
}

static ssize_t eureka_sound_control_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%d", &val) < 1)
		return -EINVAL;

	if (((val == 0) || (val == 1))) {
		if (eureka_sound_control_on != val) {
			eureka_sound_control_on = val;

			if (val == 1)
				update_audio_hub();
			else if (val == 0)
				reset_audio_hub();
		}
	}

	return count;
}

/* Headphone volume */
static ssize_t headphone_gain_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d %d %d\n", hp_balance_l, hp_balance_r, hp_lr_gain);
}


static ssize_t headphone_gain_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	int val_l;
	int val_r;
	int val_lr;

	if (!eureka_sound_control_on)
		return count;

	if (sscanf(buf, "%d %d %d", &val_l, &val_r, &val_lr) < 3)
		return -EINVAL;

	if (val_l < HEADPHONE_MIN_BALANCE)
		val_l = HEADPHONE_MIN_BALANCE;

	if (val_l > HEADPHONE_MAX_BALANCE)
		val_l = HEADPHONE_MAX_BALANCE;

	if (val_r < HEADPHONE_MIN_BALANCE)
		val_r = HEADPHONE_MIN_BALANCE;

	if (val_r > HEADPHONE_MAX_BALANCE)
		val_r = HEADPHONE_MAX_BALANCE;

	if (val_lr < HEADPHONE_MIN_GAIN)
		val_lr = HEADPHONE_MIN_GAIN;

	if (val_lr > HEADPHONE_MAX_GAIN)
		val_lr = HEADPHONE_MAX_GAIN;

	hp_balance_l = val_l;
	hp_balance_r = val_r;
	hp_lr_gain = val_lr;

	headphone_balance_l(hp_balance_l);
	headphone_balance_r(hp_balance_r);
	headphone_lr_gain(hp_lr_gain);

	return count;
}

static ssize_t mono_mix_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", mono_mix_mode);
}

static ssize_t mono_mix_mode_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	int mode;

	if (!eureka_sound_control_on)
		return count;

	if (sscanf(buf, "%d", &mode) < 1)
		return -EINVAL;

	if (mode < MONO_MIX_MIN_LIMIT)
		mode = MONO_MIX_MIN_LIMIT;

	if (mode > MONO_MIX_MAX_LIMIT)
		mode = MONO_MIX_MAX_LIMIT;

	/* 0 = DISABLE, 1 = R_MONO, 2 = L_MONO, 3 = LR_SWAP, 4 = (L+R)/2_MIX_MONO, 5 = L+R_MIX_MONO */
	mono_mix_mode = mode;
	mono_mix_control(mono_mix_mode);

	return count;
}

static ssize_t headphone_mute_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d %d\n", hp_mute_reg, hp_mute_off);
}

static ssize_t headphone_mute_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	int data1;
	int data2;

	if (!eureka_sound_control_on)
		return count;

	if (sscanf(buf, "%d %d", &data1, &data2) < 2)
		return -EINVAL;

	if (data1 < 1)
		data1 = 1;

	if (data1 > 3)
		data1 = 3;

	if (((data2 == 0) || (data2 == 1))) {
		if (hp_mute_off != data2) {
			hp_mute_off = data2;
		}
	}

	hp_mute_reg = data1;
	headphone_analog_mute(hp_mute_reg);

	return count;
}

static ssize_t version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", EUREKA_SOUND_VERSION);
}

static ssize_t reg_dump_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned int hpl, hpr, hplr, mono_mix, hp_unmuted, hp_unmuted1, hp_unmuted2, hp_unmuted3;

	eureka_regmap_read(COD3035X_51_DVOLL, &hpl);
		hpl = hpl >> DA_DVOL_SHIFT;

	eureka_regmap_read(COD3035X_52_DVOLR, &hpr);
		hpr = hpr >> DA_DVOL_SHIFT;

	eureka_regmap_read(COD3035X_E5_PRESET_AVC, &hplr);
		hplr = (hplr & CTVOL_HP_AVCBYPASS_MASK) >> CTVOL_HP_AVCBYPASS_SHIFT;

	eureka_regmap_read(COD3035X_50_DAC1, &mono_mix);
		mono_mix = (mono_mix & DAC1_MONOMIX_MASK) >> DAC1_MONOMIX_SHIFT;

	eureka_regmap_read(COD3035X_37_MIX_DA1, &hp_unmuted1);
		hp_unmuted1 = (hp_unmuted1 & EN_HP_MIXL_DCTL_MASK) >> EN_HP_MIXL_DCTL_SHIFT;

	eureka_regmap_read(COD3035X_37_MIX_DA1, &hp_unmuted2);
		hp_unmuted2 = (hp_unmuted2 & EN_HP_MIXL_DCTR_MASK) >> EN_HP_MIXL_DCTR_SHIFT;

	eureka_regmap_read(COD3035X_37_MIX_DA1, &hp_unmuted3);
		hp_unmuted3 = (hp_unmuted3 & EN_HP_MIXR_DCTR_MASK) >> EN_HP_MIXR_DCTR_SHIFT;

	if ((hp_unmuted1 == 1) && (hp_unmuted2 == 1) && (hp_unmuted3 == 1))
		hp_unmuted = 1;
	else
		hp_unmuted = 0;

	return sprintf(buf, " Headphone Left Balance: %d\n Headphone Right Balance: %d\n Headphone Boost Level: %d\n \
Mono Mix Mode: %d\n Headphone Unmuted: %d\n \
",
hpl, hpr, hplr, mono_mix, hp_unmuted);
}

/* Sysfs permissions */
static DEVICE_ATTR(eureka_sound_control, 0664, eureka_sound_control_show, eureka_sound_control_store);
static DEVICE_ATTR(headphone_gain, 0664, headphone_gain_show, headphone_gain_store);
static DEVICE_ATTR(mono_mix_mode, 0664, mono_mix_mode_show, mono_mix_mode_store);
static DEVICE_ATTR(headphone_mute, 0664, headphone_mute_show, headphone_mute_store);
static DEVICE_ATTR(version, 0664, version_show, NULL);
static DEVICE_ATTR(reg_dump, 0664, reg_dump_show, NULL);

static struct attribute *eureka_sound_control_attributes[] = {
	&dev_attr_eureka_sound_control.attr,
	&dev_attr_headphone_gain.attr,
	&dev_attr_mono_mix_mode.attr,
	&dev_attr_headphone_mute.attr,
	&dev_attr_version.attr,
	&dev_attr_reg_dump.attr,
	NULL
};

static struct attribute_group eureka_sound_control_group = {
	.attrs = eureka_sound_control_attributes,
};

static struct miscdevice eureka_sound_control_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "eureka_sound_control",
};

static int eureka_sound_control_init(void)
{
	misc_register(&eureka_sound_control_device);

	if (sysfs_create_group(&eureka_sound_control_device.this_device->kobj,
				&eureka_sound_control_group) < 0) {
		return 0;
	}

	init_audio_hub();

	return 0;
}

static void eureka_sound_control_exit(void)
{
	sysfs_remove_group(&eureka_sound_control_device.this_device->kobj,
                           &eureka_sound_control_group);
}

/* Driver init and exit functions */
module_init(eureka_sound_control_init);
module_exit(eureka_sound_control_exit);
