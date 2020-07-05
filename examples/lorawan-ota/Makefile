
%.o: %.c
	gcc -std=gnu11 -c $< -o $@

generate-metadata: generate-metadata.o
	gcc -o $@ generate-metadata.o
	rm -f *.o

.PHONY: bootloader ota-image-example master-hex

bootloader:
	rm -f firmware.hex;
	printf "Making OTA Bootloader\n";
	cd ./bootloader; 				\
		make clean;						\
		make bootloader.bin;	\

ota-image-example: generate-metadata
	rm -f firmware*.hex firmware*.bin;
	printf "\nMaking OTA Example Image\n"
	cd ./ota-image-example;		\
		make; 		\
	printf "\nAdding metadata to OTA Example Image\n";
	./generate-metadata ota-image-example/build/loramac/ota-image-example.bin 0x1 0xdeadbeef 0 # create golden image
	#./generate-metadata ota-image-example/ota-image-example.bin 0x1 0xdeadbeef 0 # create ota update
	srec_cat firmware-metadata.bin -binary ota-image-example/build/loramac/ota-image-example.bin -binary -offset 0x4000 -o ota-image.bin -binary

master-hex:
	rm -f firmware.hex;
	printf "\nMerging OTA Example Image with Bootloader\n"
	srec_cat bootloader/bootloader.bin -binary  ota-image.bin -binary -offset 0x4000 -o firmware.bin -binary

flash: master-hex
	stm32flash -w firmware.bin -S 0x8000000 -v -g 0x0 -b 115200 /dev/ttyUSB0
