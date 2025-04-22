#ifndef __MIX415_DRV_H
#define __MIX415_DRV_H

#define CAM_DEVICE "/dev/video11"
#define MIX_WIDTH 3840
#define MIX_HEIGHT 2160

#define ROI_X 1000
#define ROI_Y 800
#define ROI_W 640
#define ROI_H 480

int cam_main(int argc, char *argv[]) ;

#endif // 