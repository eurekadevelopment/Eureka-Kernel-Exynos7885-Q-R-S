#ifndef _JACK_COD3035_SYSFS_CB_H
#define _JACK_COD3035_SYSFS_CB_H

extern int cod3035_jack_det;
extern int cod3035_ear_mic;

void register_cod3035_jack_cb(struct snd_soc_codec *codec);

#endif /*_JACK_COD3035_SYSFS_CB_H */
