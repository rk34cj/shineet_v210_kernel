# S3c24XX Platform Support
#snd-soc-s3c24xx-objs := s3c24xx-pcm.o
snd-soc-s3c-dma-objs := s3c-dma.o
snd-soc-s3c-idma-objs := s3c-idma.o
snd-soc-s3c-dma-wrapper-objs := s3c-dma-wrapper.o
snd-soc-s3c24xx-i2s-objs := s3c24xx-i2s.o
snd-soc-s3c2412-i2s-objs := s3c2412-i2s.o
snd-soc-s3c64xx-i2s-objs := s3c64xx-i2s.o
snd-soc-s3c2443-ac97-objs := s3c2443-ac97.o
snd-soc-s5p-i2s_sec-objs := s5p-i2s_sec.o
snd-soc-s3c-i2s-v2-objs := s3c-i2s-v2.o
snd-soc-s3c-ac97-objs := s3c-ac97.o
snd-soc-s3c-pcm-objs := s3c-pcm.o
snd-soc-s5p-spdif-objs := s5p-spdif.o

obj-$(CONFIG_SND_S3C24XX_SOC) += snd-soc-s3c-dma.o
obj-$(CONFIG_SND_S3C64XX_SOC_I2S) += snd-soc-s3c-idma.o
obj-$(CONFIG_SND_S3C64XX_SOC_I2S) += snd-soc-s3c-dma-wrapper.o
obj-$(CONFIG_SND_S3C24XX_SOC_I2S) += snd-soc-s3c24xx-i2s.o
obj-$(CONFIG_SND_S3C2443_SOC_AC97) += snd-soc-s3c2443-ac97.o
obj-$(CONFIG_SND_S3C2412_SOC_I2S) += snd-soc-s3c2412-i2s.o
obj-$(CONFIG_SND_S3C64XX_SOC_I2S) += snd-soc-s3c64xx-i2s.o
obj-$(CONFIG_SND_S3C64XX_SOC_I2S) += snd-soc-s5p-i2s_sec.o
obj-$(CONFIG_SND_S3C_I2SV2_SOC) += snd-soc-s3c-i2s-v2.o
obj-$(CONFIG_SND_S3C_SOC_AC97) += snd-soc-s3c-ac97.o
obj-$(CONFIG_SND_S3C_SOC_PCM) += snd-soc-s3c-pcm.o
obj-$(CONFIG_SND_S5P_SOC_SPDIF) += snd-soc-s5p-spdif.o

# S3C24XX Machine Support
snd-soc-jive-wm8750-objs := jive_wm8750.o
snd-soc-neo1973-wm8753-objs := neo1973_wm8753.o
snd-soc-neo1973-gta02-wm8753-objs := neo1973_gta02_wm8753.o
snd-soc-smdk2443-wm9710-objs := smdk2443_wm9710.o
snd-soc-ln2440sbc-alc650-objs := ln2440sbc_alc650.o
snd-soc-s3c24xx-uda134x-objs := s3c24xx_uda134x.o
snd-soc-smdk64xx-wm8580-objs := smdk64xx_wm8580.o
ifeq ($(CONFIG_WM8580_MASTER),y)
snd-soc-smdkv2xx-wm8580-objs := smdkv2xx_wm8580mst.o
else
snd-soc-smdkv2xx-wm8580-objs := smdkv2xx_wm8580slv.o
endif
snd-soc-s3c24xx-simtec-objs := s3c24xx_simtec.o
snd-soc-s3c24xx-simtec-hermes-objs := s3c24xx_simtec_hermes.o
snd-soc-s3c24xx-simtec-tlv320aic23-objs := s3c24xx_simtec_tlv320aic23.o
snd-soc-smdkv2xx-wm8580-pcm-objs := smdkv2xx_wm8580pcm.o
snd-soc-smdk-wm9713-objs := smdk_wm9713.o
snd-soc-smdkv2xx-spdif-objs := smdkv2xx_spdif.o

###
snd-soc-smdkv2xx-wm8976-objs := smdkv2xx_wm8976.o
obj-$(CONFIG_SND_S5PV2XX_SOC_WM8976) += snd-soc-smdkv2xx-wm8976.o

obj-$(CONFIG_SND_S3C24XX_SOC_JIVE_WM8750) += snd-soc-jive-wm8750.o
obj-$(CONFIG_SND_S3C24XX_SOC_NEO1973_WM8753) += snd-soc-neo1973-wm8753.o
obj-$(CONFIG_SND_S3C24XX_SOC_NEO1973_GTA02_WM8753) += snd-soc-neo1973-gta02-wm8753.o
obj-$(CONFIG_SND_S3C24XX_SOC_SMDK2443_WM9710) += snd-soc-smdk2443-wm9710.o
obj-$(CONFIG_SND_S3C24XX_SOC_LN2440SBC_ALC650) += snd-soc-ln2440sbc-alc650.o
obj-$(CONFIG_SND_S3C24XX_SOC_S3C24XX_UDA134X) += snd-soc-s3c24xx-uda134x.o
obj-$(CONFIG_SND_S3C64XX_SOC_WM8580) += snd-soc-smdk64xx-wm8580.o
obj-$(CONFIG_SND_S5PV2XX_SOC_WM8580) += snd-soc-smdkv2xx-wm8580.o
obj-$(CONFIG_SND_S3C24XX_SOC_SIMTEC) += snd-soc-s3c24xx-simtec.o
obj-$(CONFIG_SND_S3C24XX_SOC_SIMTEC_HERMES) += snd-soc-s3c24xx-simtec-hermes.o
obj-$(CONFIG_SND_S3C24XX_SOC_SIMTEC_TLV320AIC23) += snd-soc-s3c24xx-simtec-tlv320aic23.o
obj-$(CONFIG_SND_S5PV2XX_SOC_WM8580_PCM) += snd-soc-smdkv2xx-wm8580-pcm.o
obj-$(CONFIG_SND_SOC_SMDK_WM9713) += snd-soc-smdk-wm9713.o
obj-$(CONFIG_SND_SOC_SMDK_SPDIF) += snd-soc-smdkv2xx-spdif.o


