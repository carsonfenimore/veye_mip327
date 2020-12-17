# veye_mip327
Fork of some code from https://github.com/veyeimaging/raspberrypi.git.  The veyeimaging repo only provided H264 OR YUV streaming - but not both simultenaouesly.

This code allows simultaneous H264 and YUV frames capture using the IMX327 sensor as integrated by veye.  On a raspberry pi 4 this enables full 1080p NAL and YUV capture.  Even with YUV->rgb conversion, this process only consumes a small fraction (<10%) of total cpu - leaving the rest for vision / AI processing tasks.

Note that while it is possible to stream RGB, the internal bus of the raspberry pi is not able to push full 1080p @ 30fps.  For this reason YUV is preferred for frame formats.

Note2: you can easily convert the yuv frames using opencv as follows:

```
cv::Mat yuv(frameHeight+frameHeight/2, frameWidth, CV_8UC1,(uchar *)yuvData);
```

## To use this code 
```
git clone the https://github.com/veyeimaging/raspberrypi.git
git clone https://github.com/carsonfenimore/veye_mip327.git
cp veye_mip327/* raspberrypi/veye_raspcam/source
cd raspberrypi/veye_raspcam/source
./build.sh
./mipi327cap
```
  

