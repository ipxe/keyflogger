GPASM		:= gpasm
PK2CMD		:= pk2cmd
HIDRD_CONVERT	:= hidrd-convert

PICMODEL	:= 16f1454
DEBUG		?= 1

all : keyflogger.hex

install : keyflogger.install

keyflogger.hex : keyflogger.asm usb.inc keyboard.xml.inc mouse.xml.inc Makefile
	$(GPASM) -p p$(PICMODEL) -S 2 -D DEBUG=$(DEBUG) -o $@ $<

%.xml.hex : %.xml Makefile
	$(HIDRD_CONVERT) -i xml -o hex $< $@

%.xml.inc : %.xml.hex Makefile
	for x in $$(cat $<) ; do echo " dt 0x$${x}" ; done > $@

%.install : %.hex Makefile
	$(PK2CMD) -PPIC$(PICMODEL) -F $< -M -R -W

clean :
	$(RM) *.hex *.lst *.cod *.err *.xml.inc
