#ifndef __SOUND_COD3034X_H__
#define __SOUND_COD3034X_H__

#define COD3034X_MICBIAS1	0
#define COD3034X_MICBIAS2	1

extern void cod3034x_call_notifier(int irq1, int irq2, int irq3, int irq4, int status1);
int cod3034x_mic_bias_ev(struct snd_soc_codec *codec,int mic_bias, int event);

#endif /* __SOUND_COD3034X_H__ */
