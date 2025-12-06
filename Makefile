PROGRAM_COUNT := 19
PROGRAM_NUMBERS := $(shell seq 1 $(PROGRAM_COUNT))
PROGRAMS := $(patsubst %,program-%.uf2,$(PROGRAM_NUMBERS))

all: $(PROGRAMS)

program-%.uf2: main.go go.mod go.sum
	tinygo build -target=nicenano -o $@ -ldflags="-X 'main.DeviceIdString=$*'"

flash: program-1.uf2
	udisksctl mount -b /dev/disk/by-label/NICENANO
	cp $< /run/media/*/NICENANO

flash-monitor: flash
	tinygo monitor

.PHONY: flash flash-monitor
