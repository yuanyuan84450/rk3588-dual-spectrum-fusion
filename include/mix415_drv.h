#ifndef __MIX415_DRV_H
#define __MIX415_DRV_H

#define CAM_DEVICE "/dev/video11"
#define MIX_WIDTH 1920
#define MIX_HEIGHT 1080

#define ROI_X 720
#define ROI_Y 360
#define ROI_W 480
#define ROI_H 480

void* camera_thread(void *arg);

#endif // 