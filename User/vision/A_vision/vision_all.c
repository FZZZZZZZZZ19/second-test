


#define SCUT_IMAGE_H 120
#define SCUT_IMAGE_W 188
#define MAX_STEP 2000
#define LINE_LENGTH (SCUT_IMAGE_H+SCUT_IMAGE_W + 50)

unsigned char g_binary[SCUT_IMAGE_H][SCUT_IMAGE_W];

/**
 * @brief state: 0 正常情况 1 直角弯 2 识别到1->2的跳变 3 十字路口 4 环岛
 * @brief flag: 0 正常情况 1 识别到直角弯 2 识别1到Y 3 进入Y 4 识别到十字路口 5 识别到环岛 6 开始进入环岛 
 * @brief flag: 7 进入1 8 进入2 9 识别到退出 10 开始推出 11 推出1  
*/
int state,flag;

void reset_state_flag(void)
{state = 0;flag = 0;}

int lx,ly,rx,ry;

void start_point_reset(void)
{
    int lx = 0;
    int ly = SCUT_IMAGE_H - 1;
    int rx = SCUT_IMAGE_W - 1;
    int ry = SCUT_IMAGE_H - 1;
}

int l_points[MAX_STEP][2];
int r_points[MAX_STEP][2];
int l_points_dir[MAX_STEP];
int r_points_dir[MAX_STEP];
int l_statics, r_statics;
int min_statics;
int target_l_line[MAX_STEP][2];
int target_r_line[MAX_STEP][2];
int target_mid_line[MAX_STEP][2];
int target_statics,target_l_statics,target_r_statics;