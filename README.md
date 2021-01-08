# UVC Capture With Hardware Decoding & Encoding

### Build

```
$ make
```

### Hardware Decoding & Preview on OSD0

```
$ ./uvc_capture_aml -d /dev/videoX -w 1920 -h 1080 -p
```

### Hardware Decoding & Capture One RGB24 Image 'test.rgb'

```
$ ./uvc_capture_aml -d /dev/videoX -w 1920 -h 1080
```

### Hardware Decoding & H264 Hardware Encoding

```
$ ./uvc_capture_aml -d /dev/videoX -w 1920 -h 1080 -e -o test.h264
```

**Note: Please replace /dev/videoX with correct video node.**

### Get Help Messages

```
$ ./uvc_capture_aml -H
./uvc_capture_aml [-d device] [-w width] [-h height] [-f fps] [-b bitrate] [-e] [-o output] [-p] [-H]
```
