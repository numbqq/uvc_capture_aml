all:
	gcc -O3 main.c -o uvc_capture_aml -lvpcodec -lamcodec -lamadec -lamavutils -lrt -lpthread -lge2d -lion

clean:
	rm -rf *.o uvc_capture_aml
