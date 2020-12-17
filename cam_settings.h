#ifndef cam_settings_h_
#define cam_settings_h_ 1


typedef struct {
	int flip;
	int cameraMode;
	int requestedWidth;
	int requestedHeight;
	int bitrate;
	int frameRate;
} CamSettings;

#endif
