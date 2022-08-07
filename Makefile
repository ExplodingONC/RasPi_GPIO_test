all: gpio

gpio:
	cc -fPIC -shared -o direct_GPIO.so direct_GPIO.c

clean:
	rm direct_GPIO.so