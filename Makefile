PROGRAM_COUNT := 19
PROGRAM_NUMBERS := $(shell seq 1 $(PROGRAM_COUNT))
PROGRAMS := $(patsubst %,program-%.uf2,$(PROGRAM_NUMBERS))

all: $(PROGRAMS)

program-%.uf2: main.go generator/generate.go go.mod go.sum
	rm -rf ".build-$*"
	mkdir ".build-$*"
	cp main.go ".build-$*"
	go run generator/generate.go "$*" "$(SECRETUUID)" > ".build-$*/constants.go"
	( cd ".build-$*" && tinygo build -target=nicenano -o $@ && mv $@ .. )

flash: program-1.uf2
	udisksctl mount -b /dev/disk/by-label/NICENANO
	cp $< /run/media/*/NICENANO

flash-monitor: flash
	tinygo monitor

clean:
	rm -rf program-*.uf2 .build-*

.PHONY: flash flash-monitor
