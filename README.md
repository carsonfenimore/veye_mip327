# veye_mip327
Fork of some code from https://github.com/veyeimaging/raspberrypi.git

This code allows simultaneous streaming of H264 NALUs and YUV frames.  On a raspberry pi 4 this enables streaming of full frame 1080p NAL and YUV.  Even with YUV->rgb conversion, this process only consumes a small fraction (<10%) of total cpu - leaving the rest for vision / AI processing tasks.

Note that while it is possible to stream RGB, the internal bus of the raspberry pi is not able to push full 1080p @ 30fps.  For this reason YUV is preferred for frame formats.

Note2: you can easily convert the yuv frames using opencv as follows:
