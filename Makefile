all:
	gcc -O2 -Wall -Wextra cari-dummy-device.c -o cari-dummy-device -lzmq

clean:
	rm cari-dummy-device.exe
