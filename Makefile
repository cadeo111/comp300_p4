build: hardware.bin firmware.bin

upload:
ifdef port
		tinyprog -c /dev/ttyS$(port) -p hardware.bin -u firmware.bin
else
		tinyprog -p hardware.bin -u firmware.bin
endif

hardware.blif: hardware.v spimemio.v simpleuart.v picosoc.v picorv32.v
	yosys -ql hardware.log -p 'synth_ice40 -top hardware -blif hardware.blif' $^

hardware.asc: hardware.pcf hardware.blif
	arachne-pnr -d 8k -P cm81 -o hardware.asc -p hardware.pcf hardware.blif

hardware.bin: hardware.asc
#	icetime -C ~/.apio/packages/toolchain-icestorm/share/icebox/chipdb-8k.txt -d hx8k -c 12 -mtr hardware.rpt hardware.asc
	icepack hardware.asc hardware.bin


firmware.elf: sections.lds start.S firmware.c
	riscv32-unknown-elf-gcc -march=rv32imc -nostartfiles -Wl,-Bstatic,-T,sections.lds,--strip-debug,-Map=firmware.map,--cref  -ffreestanding -nostdlib -o firmware.elf start.S firmware.c

firmware.bin: firmware.elf
	riscv32-unknown-elf-objcopy -O binary firmware.elf /dev/stdout > firmware.bin


clean:
	rm -f firmware.elf firmware.hex firmware.bin firmware.o firmware.map \
	      hardware.blif hardware.log hardware.asc hardware.rpt hardware.bin
