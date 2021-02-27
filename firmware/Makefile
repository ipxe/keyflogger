GPASM		:= gpasm
PK2CMD		:= pk2cmd
HIDRD_CONVERT	:= hidrd-convert

PICMODEL	:= 16f1454

all : keyflogger.hex

install : keyflogger.install

keyflogger.hex : keyflogger.asm usb.inc kb.xml.inc Makefile
	$(GPASM) -p p$(PICMODEL) -S 2 -o $@ $<

%.xml.hex : %.xml Makefile
	$(HIDRD_CONVERT) -i xml -o hex $< $@

%.xml.inc : %.xml.hex Makefile
	for x in $$(cat $<) ; do echo " dt 0x$${x}" ; done > $@

%.install : %.hex Makefile
	$(PK2CMD) -PPIC$(PICMODEL) -F $< -M -R -W

clean :
	$(RM) *.hex *.lst *.cod *.err *.xml.inc
