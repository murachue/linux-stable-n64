#
# Makefile for ALSA
#

snd-au1x00-objs := au1x00.o
snd-sgi-o2-objs := sgio2audio.o ad1843.o
snd-sgi-hal2-objs := hal2.o
snd-n64ai-objs := n64ai.o

# Toplevel Module Dependency
obj-$(CONFIG_SND_AU1X00) += snd-au1x00.o
obj-$(CONFIG_SND_SGI_O2) += snd-sgi-o2.o
obj-$(CONFIG_SND_SGI_HAL2) += snd-sgi-hal2.o
obj-$(CONFIG_SND_NINTENDO64_AI) += snd-n64ai.o
