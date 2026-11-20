#ifndef __VISION_ALL_H__
#define __VISION_ALL_H__

#define SCUT_IMAGE_H 120
#define SCUT_IMAGE_W 188
#define MAX_STEP 2000
#define LINE_LENGTH (SCUT_IMAGE_H+SCUT_IMAGE_W + 50)


extern int experience_value;    //10
extern int ww , bw;             //2,2
extern int max_length;          //90
extern int offset_w;            //5
extern int target_dist;         //65

void camera_init(void);
void camera_process(void);

extern float reality_x1,reality_x2,reality_x3,reality_y1,reality_y2,reality_y3;
extern int state ,flag;
extern unsigned char origin_image[SCUT_IMAGE_H][SCUT_IMAGE_W];
// for (uint8 i = 0; i < 120; i++) {
//     for (uint8 j = 0; j < 128; j++) {
//         origin_image[i][j] = mt9v03x_image[i][j];
//     }
// }
// 注：在屏幕上显示上述变量(除了origin_image)，在调用摄像头主程序之后，要进行将读到的图片传给origin_image()
#endif