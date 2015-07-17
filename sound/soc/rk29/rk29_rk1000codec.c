/*
 * rk29_wm8988.c  --  SoC audio for rockchip
 *
 * Driver for rockchip wm8988 audio
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include "../codecs/rk1000_codec.h"
#include "rk29_pcm.h"
#include "rk29_i2s.h"

#if 0
#define	DBG(x...)	printk(KERN_INFO x)
#else
#define	DBG(x...)
#endif

static int rk29_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;
	unsigned int rate, mclk;
	  
	DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);    

		/* set codec DAI configuration */
		#if defined (CONFIG_SND_RK29_CODEC_SOC_SLAVE) 
		ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS); 
		#endif	
		#if defined (CONFIG_SND_RK29_CODEC_SOC_MASTER) 
		ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM ); 
		#endif
		if (ret < 0)
			return ret; 
		/* set cpu DAI configuration */
		#if defined (CONFIG_SND_RK29_CODEC_SOC_SLAVE) 
		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
                	SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
		#endif	
		#if defined (CONFIG_SND_RK29_CODEC_SOC_MASTER) 
		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
                	SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);	
		#endif		
		if (ret < 0)
			return ret;

	rate = params_rate (params);
	switch (rate) {
		case 8000:
		case 16000:
		case 24000:
		case 32000:
		case 48000:
		case 64000:
		case 96000:
			mclk = 12288000;
			break;
		case 11025:
		case 22050:
		case 44100:
		case 88200:
			mclk = 11289600;
			break;
		default:
			return -EINVAL;
	}

	snd_soc_dai_set_sysclk(cpu_dai, 0, mclk,  SND_SOC_CLOCK_OUT);
	/* A value of 3 here means the mclk gets divided by 4, which in turns
	 * means for 96k samplerate the bit clock gets its minimal divisor of
	 * 32 */
	snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_MCLK, 3);
	snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_BCLK,
		((mclk/4)/rate) - 1);

	return 0;
}

/*
 * Logic for a rk1000 codec as connected on a rockchip board.
 */
static int rk29_rk1000_codec_init(struct snd_soc_pcm_runtime *rtd)
{
	return 0;
}

static struct snd_soc_ops rk29_ops = {
	  .hw_params = rk29_hw_params,
};

static struct snd_soc_dai_link rk29_dai[] = {
	{
	  .name = "RK1000",
	  .stream_name = "RK1000 CODEC PCM",
	  .platform_name = "rockchip-audio",
	  .codec_name = "RK1000_CODEC.0-0060",
	  .codec_dai_name = "rk1000_codec",
#if defined(CONFIG_SND_RK29_SOC_I2S_8CH)        
          .cpu_dai_name = "rk29_i2s.0",
#elif defined(CONFIG_SND_RK29_SOC_I2S_2CH)
          .cpu_dai_name = "rk29_i2s.1",
#endif
	  .init = rk29_rk1000_codec_init,
	  .ops = &rk29_ops,
	}
};

static struct snd_soc_card snd_soc_card_rk29 = {
	.name = "RK29_RK1000",
	.dai_link = rk29_dai,
	.num_links = 1,
};


static struct platform_device *rk29_snd_device;

static int __init audio_card_init(void)
{
	int ret =0;	
	
	DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
	
	rk29_snd_device = platform_device_alloc("soc-audio", -1);
	if (!rk29_snd_device) {
		  printk("platform device allocation failed\n");
		  ret = -ENOMEM;
		  return ret;
	}
 
	platform_set_drvdata(rk29_snd_device, &snd_soc_card_rk29);
	ret = platform_device_add(rk29_snd_device);
	if (ret) {
		printk("platform device add failed\n");
		platform_device_put(rk29_snd_device);
	}
	printk("audio_card_init end....\n");
	return ret;
}

static void __exit audio_card_exit(void)
{
	platform_device_unregister(rk29_snd_device);
}

module_init(audio_card_init);
module_exit(audio_card_exit);
/* Module information */
MODULE_AUTHOR("rockchip");
MODULE_DESCRIPTION("ROCKCHIP i2s ASoC Interface");
MODULE_LICENSE("GPL");
