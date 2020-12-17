#include <stdio.h>

#include "mipi327_cam_stream.h"

void frameCallback(unsigned int width, unsigned int height, unsigned char * frame){
    printf("Frame cb!\n");
}

void nalFrameCallback(unsigned int bytesLen, unsigned char * bytes){
    printf("NAL cb!\n");
}

int main(int argc, char** argv){
    CamSettings camSettings; 
    camSettings.requestedWidth = 1920;
    camSettings.requestedHeight = 1088;
    camSettings.bitrate = 4000000;
    camSettings.frameRate = 30;
    mipi327_cam_stream_run(camSettings, &frameCallback, &nalFrameCallback);
    return 0;
}
