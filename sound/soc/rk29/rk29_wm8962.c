/*
 * rk29_WM8962.c  --  SoC audio for rockchip
 *
 * Driver for rockchip WM8962 audio
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
#include <mach/gpio.h>
#include "../codecs/wm8962.h"
#include "rk29_pcm.h"
#include "rk29_i2s.h"
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#if 1
//#define	DBG(x...)	printk(KERN_INFO x)
#define	DBG(x...)	printk(x)
#else
#define	DBG(x...)
#endif

#define WM8962_POW RK30_PIN0_PA5

static int rk29_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
    	struct snd_soc_dai *codec_dai = rtd->codec_dai;
    	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
#else
        struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
        struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
#endif
        unsigned int pll_out = 0;
		int div_bclk,div_mclk;
        int ret;
		struct clk	*general_pll;

        DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
        /*by Vincent Hsiung for EQ Vol Change*/
        #define HW_PARAMS_FLAG_EQVOL_ON 0x21
        #define HW_PARAMS_FLAG_EQVOL_OFF 0x22
        if ((params->flags == HW_PARAMS_FLAG_EQVOL_ON)||(params->flags == HW_PARAMS_FLAG_EQVOL_OFF))
        {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
    		ret = codec_dai->driver->ops->hw_params(substream, params, codec_dai); //by Vincent
#else
        	ret = codec_dai->ops->hw_params(substream, params, codec_dai); //by Vincent
#endif
        	DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
        }
        else
        {

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

        }


        switch(params_rate(params)) {
        case 8000:
        case 16000:
        case 24000:
        case 32000:
        case 48000:
                pll_out = 12288000;
                break;
        case 11025:
        case 22050:
        case 44100:
                pll_out = 11289600;
                break;
        default:
                DBG("Enter:%s, %d, Error rate=%d\n",__FUNCTION__,__LINE__,params_rate(params));
                return -EINVAL;
                break;
        }
        DBG("Enter:%s, %d, rate=%d\n",__FUNCTION__,__LINE__,params_rate(params));

        //pll_out = 12000000;
        //snd_soc_dai_set_pll(codec_dai, NULL, 12000000, pll_out);
       // snd_soc_dai_set_clkdiv(codec_dai, WM8962_LRCLK_MODE, 0x000);

        #if defined (CONFIG_SND_RK29_CODEC_SOC_MASTER)
        snd_soc_dai_set_clkdiv(codec_dai, WM8962_BCLK_DIV, WM8962_BCLK_DIV_4);
        snd_soc_dai_set_clkdiv(codec_dai, WM8962_DAC_LRCLK,(pll_out/4)/params_rate(params));
        snd_soc_dai_set_clkdiv(codec_dai, WM8962_ADC_LRCLK,(pll_out/4)/params_rate(params));
        #endif

        #if defined (CONFIG_SND_RK29_CODEC_SOC_SLAVE)
		general_pll=clk_get(NULL, "general_pll");
		if(clk_get_rate(general_pll)>260000000)
		{
			div_bclk=(pll_out/4)/params_rate(params)-1;
			div_mclk=3;
		}
		else if(clk_get_rate(general_pll)>130000000)
		{
			div_bclk=(pll_out/2)/params_rate(params)-1;
			div_mclk=1;
		}
		else
		{
			pll_out=pll_out/4;
			div_bclk=(pll_out)/params_rate(params)-1;
			div_mclk=0;
		}
		DBG("func is%s,gpll=%ld,pll_out=%u,div_mclk=%d\n",
			__FUNCTION__,clk_get_rate(general_pll),pll_out,div_mclk);
		snd_soc_dai_set_sysclk(cpu_dai, 0, pll_out, 0);
        snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_BCLK,div_bclk);
        snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_MCLK, div_mclk);
        #endif
	//snd_soc_dai_set_sysclk(cpu_dai, 0, 11289600, 0);
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8962_SYSCLK_MCLK,
					11289600, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;
        DBG("Enter:%s, %d, LRCK=%d\n",__FUNCTION__,__LINE__,(pll_out/4)/params_rate(params));

        return 0;
}

/*
static const struct snd_soc_dapm_widget WM8962_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Audio Out", NULL),
	SND_SOC_DAPM_LINE("Line in", NULL),
	SND_SOC_DAPM_MIC("Micn", NULL),
	SND_SOC_DAPM_MIC("Micp", NULL),
};

static const struct snd_soc_dapm_route audio_map[]= {

	{"Audio Out", NULL, "HP_L"},
	{"Audio Out", NULL, "HP_R"},
	{"Line in", NULL, "RINPUT1"},
	{"Line in", NULL, "LINPUT1"},
	{"Micn", NULL, "RINPUT2"},
	{"Micp", NULL, "LINPUT2"},
};
*/

/*
 * Logic for a WM8962 as connected on a rockchip board.
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
static int rk29_WM8962_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

        DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);

        /* Add specific widgets */
//	snd_soc_dapm_new_controls(dapm, WM8962_dapm_widgets,
//				  ARRAY_SIZE(WM8962_dapm_widgets));
//	DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
        /* Set up specific audio path audio_mapnects */
//        snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));
//        DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
//        snd_soc_dapm_nc_pin(dapm, "HP_L");
//        DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
//	snd_soc_dapm_nc_pin(dapm, "HP_R");
//	DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
//        snd_soc_dapm_sync(dapm);
//        DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);

	return 0;
}
#else
static int rk29_WM8962_init(struct snd_soc_codec *codec)
{
        DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);

        /* Add specific widgets */
	snd_soc_dapm_new_controls(codec, WM8962_dapm_widgets,
				  ARRAY_SIZE(WM8962_dapm_widgets));
	DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
        /* Set up specific audio path audio_mapnects */
        snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));
        DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
        snd_soc_dapm_nc_pin(codec, "HP_L");
        DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
	snd_soc_dapm_nc_pin(codec, "HP_R");
	DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
        snd_soc_dapm_sync(codec);
        DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
        return 0;
}
#endif

static struct snd_soc_ops rk29_ops = {
	  .hw_params = rk29_hw_params,
	//.shutdown =
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
static struct snd_soc_dai_link rk29_dai = {
	.name = "WM8962",
	.stream_name = "WM8962 PCM",
	.codec_name = "wm8962-codec.1-001a",
	.platform_name = "rockchip-audio",
#if defined(CONFIG_SND_RK29_SOC_I2S_8CH)
          .cpu_dai_name = "rk29_i2s.0",
#elif defined(CONFIG_SND_RK29_SOC_I2S_2CH)
          .cpu_dai_name = "rk29_i2s.1",
#endif
	.codec_dai_name = "wm8962",
	.init = rk29_WM8962_init,
	.ops = &rk29_ops,
};
#else
static struct snd_soc_dai_link rk29_dai = {
	  .name = "WM8962",
	  .stream_name = "WM8962 PCM",
	  .cpu_dai = &rk29_i2s_dai[0],
	  .codec_dai = &WM8962_dai,
	  .init = rk29_WM8962_init,
	  .ops = &rk29_ops,
};
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
static struct snd_soc_card snd_soc_card_rk29 = {
	.name = "RK29_WM8962",
	.dai_link = &rk29_dai,
	.num_links = 1,
};
#else
static struct snd_soc_card snd_soc_card_rk29 = {
	  .name = "RK29_WM8962",
	  .platform = &rk29_soc_platform,
	  .dai_link = &rk29_dai,
	  .num_links = 1,
};


static struct snd_soc_card rk29_snd_devdata = {
	  .card = &snd_soc_card_rk29,
	  .codec_dev = &soc_codec_dev_WM8962,
};
#endif

static struct platform_device *rk29_snd_device;

static int __init audio_card_init(void)
{
	int ret =0;

    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);

    //wm8962 pow on
    ret = gpio_request(WM8962_POW, NULL);
    if(ret)
    {
        DBG("request wm8962 pow gpio failed\n");
        return ret;
    }
    gpio_direction_output(WM8962_POW, GPIO_HIGH);
    msleep(100);

	rk29_snd_device = platform_device_alloc("soc-audio", -1);
	if (!rk29_snd_device) {
		  DBG("platform device allocation failed\n");
		  ret = -ENOMEM;
		  return ret;
	}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
	platform_set_drvdata(rk29_snd_device, &snd_soc_card_rk29);
#else
	platform_set_drvdata(rk29_snd_device, &rk29_snd_devdata);
	rk29_snd_devdata.dev = &rk29_snd_device->dev;
#endif
	ret = platform_device_add(rk29_snd_device);
	if (ret) {
	        DBG("platform device add failed\n");
	        platform_device_put(rk29_snd_device);
	}
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
