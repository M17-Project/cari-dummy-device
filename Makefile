all:
	gcc -O2 -Wall -Wextra -Werror -Wshadow cari-dummy-device.c -o cari-dummy-device -lzmq

clean:
	rm cari-dummy-device.exe
