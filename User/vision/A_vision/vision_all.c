


#define SCUT_IMAGE_H 120
#define SCUT_IMAGE_W 188
#define MAX_STEP 2000
#define LINE_LENGTH (SCUT_IMAGE_H+SCUT_IMAGE_W + 50)

unsigned char mt9v03x_image[SCUT_IMAGE_H][SCUT_IMAGE_W];
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
int l_points_dir[MAX_STEP]; //这两个发现没用到，有空可以删了
int r_points_dir[MAX_STEP];
int l_statics, r_statics;
int min_statics;
int target_l_line[MAX_STEP][2];
int target_r_line[MAX_STEP][2];
int target_mid_line[MAX_STEP][2];
int target_statics,target_l_statics,target_r_statics;

void getmin_statics(void)
{
    min_statics = l_statics >= r_statics ? r_statics : l_statics; 
}

void reset_points(void)
{
    int i;
    for (i = 0; i < MAX_STEP; i++) {
        l_points[i][0] = -1;
        l_points[i][1] = -1;
        r_points[i][0] = -1;
        r_points[i][1] = -1;
        l_points_dir[i] = -1;
        r_points_dir[i] = -1;
    }
    l_statics = 0;
    r_statics = 0;
}

void reset_target_line(void)
{
    for(int i = 0;i<LINE_LENGTH;i++)
    {
        target_l_line[i][0] = -1;
        target_r_line[i][0] = -1;
        target_mid_line[i][0] = -1;
        target_l_line[i][1] = -1;
        target_r_line[i][1] = -1;
        target_mid_line[i][1] = -1;
    }
    target_statics = 0;
    target_l_statics = 0;
    target_r_statics = 0;
}

void reset_target_l(void)
{
    for(int i = 0;i<LINE_LENGTH;i++)
    {
        target_l_line[i][0] = -1;
        target_l_line[i][1] = -1;
    }
    target_l_statics = 0;
}

void reset_target_r(void)
{
    for(int i = 0;i<LINE_LENGTH;i++)
    {
        target_r_line[i][0] = -1;
        target_r_line[i][1] = -1;
    }
    target_r_statics = 0;
}

void put_point_info(int x, int y, int dir, int lr)
{
    if (lr == 0) {
        if (l_statics >= MAX_STEP) {
            return;
        }
        l_points[l_statics][0] = x;
        l_points[l_statics][1] = y;
        l_points_dir[l_statics] = dir;
        l_statics++;
    } else {
        if (r_statics >= MAX_STEP) {
            return;
        }
        r_points[r_statics][0] = x;
        r_points[r_statics][1] = y;
        r_points_dir[r_statics] = dir;
        r_statics++;
    }
}

/**
 * @brief 这个只在初始化摄像头的时候运行一次，其余时间不运行
*/
void camera_init(void)
{
    // 具体还没想好怎么写
}

/**
 * @brief 大津法（Otsu）阈值。
 * @param[in] value 经验偏移量；正常传 0。
 */
int GetThre(int value)
{
    int amount[256] = {0};          /* 计数可达 22560，不能用 unsigned char */
    int area = SCUT_IMAGE_W * SCUT_IMAGE_H;
    int area_bg = 0;
    int thre = 128;                 /* 全图同灰度时的兜底阈值 */
    float sigma_max = -1.0f;
    unsigned long total_pixel = 0;
    unsigned long sum_bg = 0;
    int x, y;

    for (y = 0; y < SCUT_IMAGE_H; y++) {
        for (x = 0; x < SCUT_IMAGE_W; x++) {
            amount[mt9v03x_image[y][x]]++;
        }
    }

    for (x = 0; x < 256; x++) {
        total_pixel += (unsigned long)amount[x] * x;
    }

    for (x = 0; x < 256; x++) {
        int area_fg;
        float w0, w1, mu0, mu1, sigma;

        sum_bg += (unsigned long)amount[x] * x;
        area_bg += amount[x];
        area_fg = area - area_bg;

        if (area_bg == 0 || area_fg == 0) {
            continue;               /* 空类均值无定义，也避免除零 */
        }

        w0 = (float)area_bg / (float)area;
        w1 = (float)area_fg / (float)area;
        mu0 = (float)sum_bg / (float)area_bg;
        mu1 = (float)(total_pixel - sum_bg) / (float)area_fg;

        sigma = w0 * w1 * (mu0 - mu1) * (mu0 - mu1);   /* Otsu 类间方差 */
        if (sigma > sigma_max) {
            sigma_max = sigma;
            thre = x;
        }
    }
    thre = thre + value;
    if (thre < 0)   { thre = 0; }
    if (thre > 255) { thre = 255; }
    return thre;
}

/**
 * @brief 常规的二值化函数。
 * @param[in] thre 阈值
 */
void mybinary(int thre)
{
    int x, y;
    for (y = 0; y < SCUT_IMAGE_H; y++) {
        for (x = 0; x < SCUT_IMAGE_W; x++) {
            g_binary[y][x] = (mt9v03x_image[y][x] > thre ? 255 : 0);
        }
    }
}

/** 
 * @brief 给屏幕一个黑色的边框，在一定情况下很好用
 * @param[in] pad 边框的长度
*/
int mypadding(int pad)
{
    if(pad >= 10)
    {
        return 0;
    }
    else{
        for(int i = 0;i<pad;i++)
        {
            for(int x = 0;x<SCUT_IMAGE_W;x++)
            {
                g_binary[0+i][x] = 0;
                g_binary[SCUT_IMAGE_H-1-i][x] = 0;
            }
            for(int y = 0;y < SCUT_IMAGE_H;y++)
            {
                g_binary[y][0+i] = 0;
                g_binary[y][SCUT_IMAGE_W-1-i] = 0;
            }
        }
        return 1;
    }
}