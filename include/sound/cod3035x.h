#ifndef __SOUND_COD3035X_H__
#define __SOUND_COD3035X_H__

#define COD3035X_MICBIAS1	0
#define COD3035X_MICBIAS2	1

extern void cod3035x_call_notifier(int irq1, int irq2, int irq3, int irq4, int status1,
		int param1, int param2, int param3, int param4, int param5);
int cod3035x_mic_bias_ev(struct snd_soc_codec *codec,int mic_bias, int event);

#endif /* __SOUND_COD3035X_H__ */
