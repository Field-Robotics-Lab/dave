drawings = \
	joy_uuv_controller.png joy_arm_controller.png

.SUFFIXES: .svg .pdf
.SUFFIXES: .svg .png
.SUFFIXES: .m   .svg

.m.svg:
	octave -q $<

.svg.pdf:
	inkscape --without-gui -D --export-pdf=$@ $<

.svg.png:
	inkscape --without-gui -D --export-png=$@ $<

all: $(drawings)

clean:
	rm -f *.pdf *.png

