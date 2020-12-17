#ifndef mipi327_cam_stream_h_
#define mipi327_cam_stream_h_

#include "cam_settings.h"

int mipi327_cam_stream_run(CamSettings camset, void (*frameCallback)(unsigned int width, unsigned int height, unsigned char * frame),
	       		   void (*nalFrameCallback)(unsigned int len, unsigned char* nal)	);

#endif //mipi327_cam_stream_h_
