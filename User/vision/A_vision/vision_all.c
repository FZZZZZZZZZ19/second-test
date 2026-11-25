
//// 注意：reality_x1,reality_x2,reality_x3,reality_y1,reality_y2,reality_y3,state,flag是要暴露出来的量
#include "vision_all.h"

#include <string.h>

#define SCUT_IMAGE_H ((int)APP_HEIGHT)
#define SCUT_IMAGE_W ((int)APP_WIDTH)
enum { MAX_STEP = (SCUT_IMAGE_H + SCUT_IMAGE_W)*3, LINE_LENGTH = 200 };
static const FrameView *source_frame;
static uint8_t g_binary[SCUT_IMAGE_H][(SCUT_IMAGE_W+7)/8];
static const int experience_value=20;

static int Offset_x = 6;
static int Offset_y = 10;

static int lx,ly,rx,ry;

/* 这些数组都放在 .bss 里，芯片只有 64 KiB SRAM（可用约 61 KiB），
 * 所以按实际取值范围选最小类型：
 *   坐标 x∈[0,187]、y∈[0,119]  -> uint8_t
 *   8 邻域方向 0..7             -> int8_t
 * 补连续后的中线只由 <=LINE_LENGTH 个点插值得到，容量不需要 MAX_STEP。 */
static uint8_t l_points[MAX_STEP][2], r_points[MAX_STEP][2];
static int l_statics, r_statics;
static int min_statics;
static int16_t target_l_line[LINE_LENGTH][2];
static int16_t target_r_line[LINE_LENGTH][2];
static int8_t l_points_dir[MAX_STEP];
static int8_t r_points_dir[MAX_STEP];
static int16_t target_mid_line[LINE_LENGTH][2];
static int target_statics,target_l_statics,target_r_statics;
enum { FINAL_LINE_MAX = 512 };             /* 连续中线容量：单侧轨迹约 120~300 点 */
static int16_t final_line[FINAL_LINE_MAX][2];  /* 补连续后的中线 */
static int final_statics = 0;              /* final_line 的有效点数 */
static void getmin_statics(void)
{
    min_statics = l_statics >= r_statics ? r_statics : l_statics; 
}

static void reset_points(void)
{
    // int i;
    // for (i = 0; i < MAX_STEP; i++) {
    //     l_points[i][0] = -1;
    //     l_points[i][1] = -1;
    //     r_points[i][0] = -1;
    //     r_points[i][1] = -1;
    //     l_points_dir[i] = -1;
    //     r_points_dir[i] = -1;
    // }
    l_statics = 0;
    r_statics = 0;
}

static void reset_lrxy(void)
{
    lx = -1;
    rx = -1;
    ly = -1;
    ry = -1;
}

static void reset_target_line(void)
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

static void reset_target_l(void)
{
    for(int i = 0;i<LINE_LENGTH;i++)
    {
        target_l_line[i][0] = -1;
        target_l_line[i][1] = -1;
    }
    target_l_statics = 0;
}

/**
 * @brief 大津法（Otsu）阈值。
 * @param[in] value 经验偏移量；正常传 0。
 */
static int GetThre(int value)
{
    uint16_t amount[256] = {0};          /* 计数可达 22560，不能用 unsigned char */
    int area = SCUT_IMAGE_W * SCUT_IMAGE_H;
    int area_bg = 0;
    int thre = 128;                 /* 全图同灰度时的兜底阈值 */
    float sigma_max = -1.0f;
    unsigned long total_pixel = 0;
    unsigned long sum_bg = 0;
    int x, y;

    for (y = 0; y < SCUT_IMAGE_H; y++) {
        for (x = 0; x < SCUT_IMAGE_W; x++) {
            amount[source_frame->pixels[y*source_frame->stride+x]]++;
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
    if (sigma_max < 0) return -1;
    thre = thre + value;
    if (thre < 0)   { thre = 0; }
    if (thre > 255) { thre = 255; }
    return thre;
}

/**
 * @brief 常规的二值化函数。
 * @param[in] thre 阈值
 */
static void mybinary(int threshold)
{
    memset(g_binary,0,sizeof g_binary);
    for (int y=2; y<SCUT_IMAGE_H-2; y++) {
        const uint8_t *row=source_frame->pixels+y*source_frame->stride;
        for (int x=2; x<SCUT_IMAGE_W-2; x++)
            if (row[x]>threshold)
                g_binary[y][x/8] |= (uint8_t)(1u<<(x%8));
    }
}

/* g_binary 是位压缩的（每个字节存 8 个像素），清像素必须按位清，
 * 不能像以前那样把 x 当成字节下标写 —— 那样每行只占 (W+7)/8 字节，
 * x 一超过这个范围就会写到别的行甚至数组外面（未定义行为）。 */
static void clear_binary_pixel(int x, int y)
{
    g_binary[y][x >> 3] &= (uint8_t)~(1u << (x & 7));
}

int mypadding(int pad)
{
    if(pad >= 10)
    {
        return 0;
    }
    else{
        for(int i = 0;i<pad;i++)
        {
            int top = i, bottom = SCUT_IMAGE_H - 1 - i;
            int left = i, right = SCUT_IMAGE_W - 1 - i;
            for(int x = 0;x<SCUT_IMAGE_W;x++)
            {
                clear_binary_pixel(x,top);
                clear_binary_pixel(x,bottom);
            }
            for(int y = 0;y < SCUT_IMAGE_H;y++)
            {
                clear_binary_pixel(left,y);
                clear_binary_pixel(right,y);
            }
        }
        return 1;
    }
}

/**
 * @brief 防止访问出界的一种方式。
 * @param[in] x,y 要检查的点的坐标
 */
static int is_white(int x, int y)
{
    if (x < 0 || x >= SCUT_IMAGE_W || y < 0 || y >= SCUT_IMAGE_H) {
        return 0;
    }
    return (g_binary[y][x/8] >> (x%8)) & 1u;
}

/**
 * @brief 绝对值函数
*/
static int myabs(int v)
{
    return (v < 0) ? -v : v;
}

/**
 * @brief 返回小的值
*/
static int mymin(int a,int b)
{
    return a>=b ? b : a;
}

/**
 * @brief 在某一行里找一段宽度合适的白线，作为起始行。
 *
 * 只接受宽度 3..45 且不贴左右边界的白段：
 *   - 太宽的是横向赛道/大面积反光，不是当前要跟的线；
 *   - 贴边界的段无法在白线两侧各留一个黑点作起点。
 * 在候选段里选中点离画面中心最近的一段。
 *
 * @param[in] start_y 要检查的行
 * @return 1 = 找到；0 = 本行没有合适的白段
 */
static int find_y_start(int start_y)
{
    int center = SCUT_IMAGE_W / 2;
    int x = 0;
    int best_left = -1;
    int best_right = -1;
    int best_dist = SCUT_IMAGE_W * 2;

    if (start_y < 0 || start_y >= SCUT_IMAGE_H) {
        return 0;
    }

    while (x < SCUT_IMAGE_W) {
        int left, right, width, mid, dist;

        /* 跳过黑像素，找到一段白像素 */
        while (x < SCUT_IMAGE_W && !is_white(x, start_y)) {
            x++;
        }
        if (x >= SCUT_IMAGE_W) {
            break;
        }
        left = x;
        while (x < SCUT_IMAGE_W && is_white(x, start_y)) {
            x++;
        }
        right = x - 1;
        width = right - left + 1;

        if (width < 3 || width > 40) {
            continue;
        }
        if (left <= 0 || right >= SCUT_IMAGE_W - 1) {
            continue;
        }

        mid = (left + right) / 2;
        dist = myabs(mid - center);
        if (dist < best_dist) {
            best_dist = dist;
            best_left = left;
            best_right = right;
        }
    }

    if (best_left < 0) {
        lx = ly = rx = ry = -1;
        return 0;
    }

    lx = best_left - 1;             /* 黑点：白线左侧 */
    ly = start_y;
    rx = best_right + 1;            /* 黑点：白线右侧 */
    ry = start_y;
    return 1;
}

/**
 * @brief 从 start_y 往上最多找 max_line 行，找到合适的起始白段。
 * @param[in] start_y 开始的行
 * @param[in] max_line 最多检查的行数
 */
static int find_start(int start_y, int max_line)
{
    int i;
    for (i = 0; i < max_line; i++) {
        int y = start_y - i;
        if (y < 0) {
            break;
        }
        if (find_y_start(y)) {
            return 1;
        }
    }
    lx = ly = rx = ry = -1;
    return 0;
}

int flx = 0;
int fly = SCUT_IMAGE_H - 1;
int frx = SCUT_IMAGE_W - 1;
int fry = SCUT_IMAGE_H - 1;

int first_point_x,first_point_y;
void get_first_point(void)
{
    flx = lx;
    fly = ly;
    frx = rx;
    fry = ry;
    first_point_x = (flx + frx)/2;
    first_point_y = (fly + fry)/2;
}

void reset_first_point(void)
{
    flx = -1;
    fly = -1;
    frx = -1;
    fry = -1;
    first_point_x = -1;
    first_point_y = -1;
}

/* 8 邻域方向，按屏幕坐标顺时针排列：
 * 0=E 1=SE 2=S 3=SW 4=W 5=NW 6=N 7=NE */
static const int DIR8[8][2] = {
    { 1,  0}, { 1,  1}, { 0,  1}, {-1,  1},
    {-1,  0}, {-1, -1}, { 0, -1}, { 1, -1}
};

void put_info(int x, int y, int dir, int lr)
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
 * @brief 沿白线边界走一步（Moore 邻域跟踪）。
 *
 * 当前点 (x,y) 是白线边界上的白点，(bx,by) 是上一步记录的黑点（回溯点）。
 * lr=0 走左边线：从回溯方向开始顺时针找下一个白点；
 * lr=1 走右边线：逆时针找。这样左右两条边会朝相反方向绕过同一个白区。
 *
 * @param[in,out] x,y  当前白点
 * @param[out]    dir  本次移动的方向索引 0..7
 * @param[in,out] bx,by 回溯点
 * @param[in]     lr   0=左边线，1=右边线
 * @return 1=走了一步；0=找不到下一个白点
 */
int one_step(int *x, int *y, int *dir, int *bx, int *by, int lr)
{
    int i, di = -1;
    int k, nd, bd, nx, ny;
    int dx = *bx - *x;
    int dy = *by - *y;

    for (i = 0; i < 8; i++) {
        if (DIR8[i][0] == dx && DIR8[i][1] == dy) {
            di = i;
            break;
        }
    }
    if (di < 0) {
        return 0;
    }

    for (k = 1; k <= 8; k++) {
        nd = (lr == 0) ? (di + k) % 8 : (di - k + 8) % 8;
        nx = *x + DIR8[nd][0];
        ny = *y + DIR8[nd][1];

        if (!is_white(nx, ny)) {
            continue;
        }

        /* 新回溯点 = 搜索到白点之前最后一次检查的黑点 */
        bd = (lr == 0) ? (di + k - 1) % 8 : (di - k + 1 + 8) % 8;
        *bx = *x + DIR8[bd][0];
        *by = *y + DIR8[bd][1];
        *x = nx;
        *y = ny;
        *dir = nd;
        return 1;
    }

    return 0;
}
/**
 * @brief 从底部起点同时跟踪白线的左右两条边。
 *
 * 起点由 find_start 给出：lx/rx 是白线外侧的黑点。
 * 左边界从 (lx+1,ly) 出发、以 (lx,ly) 为回溯点；
 * 右边界从 (rx-1,ry) 出发、以 (rx,ry) 为回溯点。
 * 两边各自走到图像边界、回到起点或互相交叉为止。
 *
 * @return 1 = 两边都正常结束（到边界/闭合/相遇）；0 = 起点无效或中途卡住
 */
int forward(void)
{
    int lxs, lys, lxb, lyb, ldir;
    int rxs, rys, rxb, ryb, rdir;
    int lstart_x, lstart_y, rstart_x, rstart_y;
    int ldone = 0, rdone = 0;
    int lfail = 0, rfail = 0;
    int met = 0;
    int step = 0;

    reset_points();

    if (lx < 0 || ly < 0 || rx < 0 || ry < 0) {
        return 0;
    }

    /* 白点 + 外侧黑点（回溯点） */
    lxs = lx + 1;  lys = ly;  lxb = lx;  lyb = ly;
    rxs = rx - 1;  rys = ry;  rxb = rx;  ryb = ry;

    if (lxs < 0 || lxs >= SCUT_IMAGE_W || lys >= SCUT_IMAGE_H ||
        rxs < 0 || rxs >= SCUT_IMAGE_W || rys >= SCUT_IMAGE_H) {
        return 0;
    }
    if (!is_white(lxs, lys) || !is_white(rxs, rys)) {
        return 0;
    }
    if (rxs <= lxs) {
        return 0;
    }

    lstart_x = lxs; lstart_y = lys;
    rstart_x = rxs; rstart_y = rys;
    ldir = 6;                       /* 初始朝北（DIR8[6]） */
    rdir = 6;

    put_info(lxs, lys, ldir, 0);
    put_info(rxs, rys, rdir, 1);

    while (step < MAX_STEP - 1 && (!ldone || !rdone)) {
        if (!ldone) {
            if (!one_step(&lxs, &lys, &ldir, &lxb, &lyb, 0)) {
                ldone = 1;
                lfail = 1;
            } else {
                put_info(lxs, lys, ldir, 0);
                if (lys <= 0 || lxs <= 0 || lxs >= SCUT_IMAGE_W - 1) {
                    ldone = 1;          /* 到达图像边界，正常结束 */
                } else if (lxs == lstart_x && lys == lstart_y) {
                    ldone = 1;          /* 闭合成环 */
                }
            }
        }

        if (!rdone) {
            if (!one_step(&rxs, &rys, &rdir, &rxb, &ryb, 1)) {
                rdone = 1;
                rfail = 1;
            } else {
                put_info(rxs, rys, rdir, 1);
                if (rys <= 0 || rxs <= 0 || rxs >= SCUT_IMAGE_W - 1) {
                    rdone = 1;
                } else if (rxs == rstart_x && rys == rstart_y) {
                    rdone = 1;
                }
            }
        }

        step++;

        /* 两条边交叉，说明线已经走到头/走丢，停止 */
        if (!ldone && !rdone && lxs >= rxs) {
            met = 1;
            ldone = rdone = 1;
        }
    }

    if (met) {
        return 1;
    }
    if (!ldone || !rdone) {
        return 0;                   /* 步数上限 */
    }
    if (lfail || rfail) {
        return 0;                   /* 有一边中途卡住 */
    }
    return 1;
}

/**
 * @brief 预瞄点的寻找
 * @param[in]  distance 寻找的预瞄点的距离，尽量大于两倍的路宽
 * @param[out] near_mid 近段中点（预瞄距离 1/4 处），可传 0 不要
 */
static int X1 = -1,Y1 = -1,X2 = -1,Y2 = -1;

void pred_points(int distance, TeamPixel *near_mid)
{
    if (final_statics <= 0) 
    {
        X1 = Y1 = X2 = Y2 = -1;
        if (near_mid != 0) {
            *near_mid = (TeamPixel){-1, -1};
        }
        return;
    }   
    
    int mid_x = SCUT_IMAGE_W/2;
    int mid_y = SCUT_IMAGE_H-3;
    int dx,dy,dist;
    int is_have = 0,index = -1;
    for(int i = 0;i<final_statics;i++)
    {
        dx = final_line[i][0] - mid_x;
        dy = final_line[i][1] - mid_y;
        dist = myabs(dx)+myabs(dy);
        if(dist>=distance)
        {
            is_have = 1;
            index = i;
            break;
        }
    }
    if(is_have == 0)
    {
        index = final_statics - 1;
    }
    /* 近段只取预瞄距离 1/4 处的点，转弯时远端折点不会影响近处前馈 */
    if (near_mid != 0) {
        *near_mid = (TeamPixel){final_line[index/4][0], final_line[index/4][1]};
    }
    X1 = final_line[index/2][0];
    Y1 = final_line[index/2][1];
    X2 = final_line[index][0];
    Y2 = final_line[index][1];
    // dx = target_mid_line[index][0] - mid_x;
    // dy = target_mid_line[index][1] = mid_y;
    // angle = (float)(atan2(dx,dy+theta)*180.0/3.1415926535); // 防止报错
    // target_lr = dx >= 0 ? 1 : 0; 
}

static int LR = 0;
static int out_y_lr = 0;
static int bottom = -1;                 /* 十字路口底边 y，越大越靠下 */
static int top    = -1;                 /* 十字路口顶边 y，越小越靠上 */
static int cx     = -1;                 /* Y/环岛分叉点 x */
static int cy     = -1;                 /* Y/环岛分叉点 y */
// static int elem_lr = 0;                 /* 0=左, 1=右 */
// static int elem_lr_manual = 0;          /* Y型人工给定：0=左Y, 1=右Y（只改这个） */

static void reset_info(void)
{
    bottom = -1;                 /* 十字路口底边 y，越大越靠下 */
    top    = -1;                 /* 十字路口顶边 y，越小越靠上 */
    cx     = -1;                 /* Y/环岛分叉点 x */
    cy     = -1;                 /* Y/环岛分叉点 y */
}

#define LINE_SEG_STEP   10      /* 每段包含的巡线点数，按需要改 */
/* 最坏情况：MAX_STEP 个点、每 LINE_SEG_STEP 点一段。以前写死 256 段，
 * 但实际最多只有 93 段，多出来的 163*4 字节白占 SRAM。 */
#define LINE_SEG_MAX    ((MAX_STEP + LINE_SEG_STEP - 1) / LINE_SEG_STEP)

int8_t l_seg_dir[LINE_SEG_MAX];    /* 左折线每一段的方向 0..7，-1=无效 */
int8_t r_seg_dir[LINE_SEG_MAX];    /* 右折线每一段的方向 0..7，-1=无效 */
int l_seg_statics = 0;          /* 左折线实际分出来的段数 */
int r_seg_statics = 0;          /* 右折线实际分出来的段数 */

/* 实际用的"每段点数"，由 get_line_seg_dir 更新。
 * 后面要把段号换算成巡线点号（段号 × 每段点数）时用它，别再写死 LINE_SEG_STEP。 */
int g_seg_step = LINE_SEG_STEP;

/**
 * @brief 把一个方向向量量化到 8 个方向之一。
 * @param[in] vx,vy 合向量（屏幕坐标：x 向 E，y 向 S）
 * @return 0=E 1=SE 2=S 3=SW 4=W 5=NW 6=N 7=NE；向量为零（方向无定义）时返回 -1
 * @note  以 22.5° 为分界取 8 个方向里投影最大的那个（严格 8 等分，各 45°）。
 *        斜方向的单位向量 DIR8[1]=(1,1) 的模是 √2，所以乘 707/1000 ≈ 1/√2
 *        归一化后再比较，8 个方向的权重才一致；全程整数运算，不用浮点和 atan2。
 */
int dir8_of_vector(int vx, int vy)
{
    int  i;
    int  best = -1;
    long best_score = 0;

    if (vx == 0 && vy == 0) {
        return -1;
    }

    for (i = 0; i < 8; i++) {
        long dot = (long)vx * DIR8[i][0] + (long)vy * DIR8[i][1];
        long score = (i & 1) ? (dot * 707) : (dot * 1000);   /* 奇数下标是斜方向 */
        if (score > best_score) {
            best_score = score;
            best = i;
        }
    }
    return best;
}

/**
 * @brief 把一段折线按每 seg_step 个点切开，求每一段的方向（8 等分）。
 * @param[in]  dirs     逐步走向数组（l_points_dir / r_points_dir）
 * @param[in]  n        有效点数（l_statics / r_statics）
 * @param[in]  seg_step 每段多少个点，<= 0 时用 LINE_SEG_STEP
 * @param[out] out      段方向结果，out[0] 是最下面（最先走到）的一段
 * @param[in]  out_max  out 的容量，防止越界
 * @return 分出来的段数；0 = 没有点
 * @note   末尾不满 seg_step 的一段也算一段（剩下几个点就用几个点）；
 *         连续 -1 的段记为 -1，这样 out 的下标始终等于段号，
 *         不会因为某段没数据而错位。
 */
int line_seg_dir(const int8_t *dirs, int n, int seg_step, int8_t *out, int out_max)
{
    int seg = 0;
    int start, end, i;
    int vx, vy, cnt;

    if (dirs == 0 || out == 0 || out_max <= 0 || n <= 0) {
        return 0;
    }
    if (n > MAX_STEP) {
        n = MAX_STEP;               /* 防止调用方传大了越界 */
    }
    if (seg_step <= 0) {
        seg_step = LINE_SEG_STEP;
    }

    for (start = 0; start < n && seg < out_max; start += seg_step) {
        end = start + seg_step;
        if (end > n) {
            end = n;                /* 末尾不满一段，按剩下的点算 */
        }

        vx = 0;
        vy = 0;
        cnt = 0;
        for (i = start; i < end; i++) {
            int d = dirs[i];
            if (d < 0 || d > 7) {
                continue;           /* 未填/非法：这一步没有方向信息 */
            }
            vx += DIR8[d][0];
            vy += DIR8[d][1];
            cnt++;
        }

        out[seg] = (cnt > 0) ? dir8_of_vector(vx, vy) : -1;
        seg++;
    }

    return seg;
}

/**
 * @brief 求左右边线每一段的方向，分别存进 l_seg_dir[] / r_seg_dir[]。
 * @param[in] seg_step 每段多少个点，<= 0 时用 LINE_SEG_STEP
 * @return 左右两边里段数较少的那个（方便按下标成对使用）
 * @note  每帧巡线之后调用一次即可（camera_process 里 forward() /
 *        getmin_statics() 之后加一句 get_line_seg_dir(0); ）。
 *        实际段数分别在 l_seg_statics / r_seg_statics 里；两条线点数
 *        常常不一样，所以可能一边比另一边多出几段。
 *        想固定分成 N 段：传 l_statics / N 就行（如 get_line_seg_dir(l_statics/5)）。
 */
int get_line_seg_dir(int seg_step)
{
    if (seg_step <= 0) {
        seg_step = LINE_SEG_STEP;   /* 和 line_seg_dir 里的默认值保持一致 */
    }
    g_seg_step = seg_step;          /* 记下实际段长，段号↔点号换算要用 */

    l_seg_statics = line_seg_dir(l_points_dir, l_statics, seg_step,
                                 l_seg_dir, LINE_SEG_MAX);
    r_seg_statics = line_seg_dir(r_points_dir, r_statics, seg_step,
                                 r_seg_dir, LINE_SEG_MAX);

    return mymin(l_seg_statics, r_seg_statics);
}

/* ============= 段方向滤波（众数 + 去掉没连续出现两次的值） =============
 * l_seg_dir[] / r_seg_dir[] 是每段的方向，段与段之间难免有跳变（弯道边界
 * 抖动、某段点太少等）。这里做两步滤波，结果放进 l_seg_dir_filt[] /
 * r_seg_dir_filt[]，数组长度和段数都不变，段号仍然一一对应：
 *   1) 众数滤波：以该段为中心取 LINE_FILT_WIN 个段，统计有效方向出现次数，
 *      取次数最多的那个；票数并列时保留原来的值，避免方向来回跳；
 *   2) 去掉孤立值：某段方向和前后都不一样（没连续出现两次）就认为不可信，
 *      用前一个保留的方向补上（开头没有前值时参考后一个有效值），
 *      这样不会留下空洞。
 * -1（该段没数据）不参与投票，也不会被当成方向补出去。
 */

#define LINE_FILT_WIN   5               /* 众数滤波窗口，取奇数：3 轻、5 中、7 重 */
#define LINE_FILT_MAX   LINE_SEG_MAX    /* 滤波后的数组大小（和段数组一样大） */

int8_t l_seg_dir_filt[LINE_FILT_MAX];   /* 左折线滤波后的段方向 0..7，-1=无效 */
int8_t r_seg_dir_filt[LINE_FILT_MAX];   /* 右折线滤波后的段方向 0..7，-1=无效 */
int l_seg_filt_statics = 0;             /* 左折线滤波后的段数 */
int r_seg_filt_statics = 0;             /* 右折线滤波后的段数 */

static int8_t s_seg_filt_tmp[LINE_FILT_MAX];   /* 两步滤波的中间结果 */

/**
 * @brief 众数滤波：逐段取窗口内出现次数最多的方向。
 * @param[in]  src,dst    输入/输出段方向数组
 * @param[in]  n          输入段数
 * @param[in]  dst_max    dst 的容量
 * @param[in]  win        窗口长度，<= 0 时用 LINE_FILT_WIN（奇数）
 * @return 输出段数（= n，滤波不改变段数）
 * @note   窗口边缘自动截断；整个窗口都没有有效方向时输出 -1；
 *         票数并列时保留 src[i] 本身，防止方向在 6/7 之间来回跳。
 */
int dir_mode_filter(const int8_t *src, int n, int8_t *dst, int dst_max, int win)
{
    int i, d, j;
    int half, lo, hi;
    int vote[8];
    int best, best_cnt;

    if (src == 0 || dst == 0 || dst_max <= 0 || n <= 0) {
        return 0;
    }
    if (n > LINE_SEG_MAX) {
        n = LINE_SEG_MAX;
    }
    if (n > dst_max) {
        n = dst_max;
    }
    if (win <= 0) {
        win = LINE_FILT_WIN;
    }
    half = win / 2;
    if (half < 1) {
        half = 1;               /* 至少和左右邻段比一比，否则等于没滤波 */
    }

    for (i = 0; i < n; i++) {
        lo = i - half;
        hi = i + half;
        if (lo < 0) {
            lo = 0;
        }
        if (hi > n - 1) {
            hi = n - 1;
        }

        for (d = 0; d < 8; d++) {
            vote[d] = 0;
        }
        for (j = lo; j <= hi; j++) {
            int v = src[j];
            if (v >= 0 && v < 8) {
                vote[v]++;
            }
        }

        best = -1;
        best_cnt = 0;
        for (d = 0; d < 8; d++) {
            if (vote[d] > best_cnt) {
                best_cnt = vote[d];
                best = d;
            }
        }
        if (best_cnt <= 0) {
            dst[i] = -1;        /* 整个窗口都没有有效方向 */
            continue;
        }
        if (src[i] >= 0 && src[i] < 8 && vote[src[i]] == best_cnt) {
            best = src[i];      /* 并列：保留原值 */
        }
        dst[i] = best;
    }

    return n;
}

/**
 * @brief 去掉"没有连续出现两次"的孤立值。
 * @param[in]  src,dst  输入/输出段方向数组
 * @param[in]  n        输入段数
 * @param[in]  dst_max  dst 的容量
 * @return 输出段数（= n，段号不变）
 * @note   某段和相邻段（前或后）至少有一边相同才保留；否则用前一个保留的
 *         方向补上；开头没有前值时用后一个有效值；全程没有可参考的值则为 -1。
 *         这样滤波后既没有孤立跳变，也不会出现需要再处理的空洞。
 */
int dir_drop_unrepeated(const int8_t *src, int n, int8_t *dst, int dst_max)
{
    int i, k, prev, v;

    if (src == 0 || dst == 0 || dst_max <= 0 || n <= 0) {
        return 0;
    }
    if (n > LINE_SEG_MAX) {
        n = LINE_SEG_MAX;
    }
    if (n > dst_max) {
        n = dst_max;
    }

    prev = -1;                  /* 上一个保留下来（有连续重复）的方向 */
    for (i = 0; i < n; i++) {
        v = src[i];
        if (v < 0 || v > 7) {
            dst[i] = -1;        /* 本来就没数据，保持 -1 */
            continue;
        }
        if ((i > 0 && src[i - 1] == v) || (i + 1 < n && src[i + 1] == v)) {
            dst[i] = v;         /* 前后至少有一边相同：确实连着出现，保留 */
            prev = v;
            continue;
        }
        /* 孤立值（前后都不同）：用前一个保留值补，段号不变 */
        if (prev >= 0) {
            dst[i] = prev;
        } else {
            k = i + 1;          /* 开头没有前值：参考后一个有效方向 */
            while (k < n && (src[k] < 0 || src[k] > 7)) {
                k++;
            }
            dst[i] = (k < n) ? src[k] : -1;
            if (dst[i] >= 0) {
                prev = dst[i];
            }
        }
    }

    return n;
}

/**
 * @brief 对一段段方向数组做两步滤波：先众数，再去孤立值。
 * @param[in]  src,dst  输入/输出段方向数组
 * @param[in]  n        输入段数
 * @param[in]  dst_max  dst 的容量
 * @param[in]  win      众数窗口，<= 0 时用 LINE_FILT_WIN
 * @return 输出段数（= n）
 */
int line_seg_filter(const int8_t *src, int n, int8_t *dst, int dst_max, int win)
{
    int m;

    m = dir_mode_filter(src, n, s_seg_filt_tmp, LINE_FILT_MAX, win);
    return dir_drop_unrepeated(s_seg_filt_tmp, m, dst, dst_max);
}

/**
 * @brief 对左右边线的段方向数组滤波，结果存进 l_seg_dir_filt[] / r_seg_dir_filt[]。
 * @param[in] win 众数窗口，<= 0 时用 LINE_FILT_WIN
 * @return 左右两边滤波后段数较少的那个
 * @note  调用顺序：先 get_line_seg_dir(0) 求出 l_seg_dir/r_seg_dir，
 *        再调 get_line_seg_dir_filt(0)（或直接调 get_line_seg_dir_filt(0)，
 *        它会用上一次的段数组）。滤波后的段数在 l_seg_filt_statics /
 *        r_seg_filt_statics 里，和滤波前一样，段号一一对应。
 */
int get_line_seg_dir_filt(int win)
{
    l_seg_filt_statics = line_seg_filter(l_seg_dir, l_seg_statics,
                                         l_seg_dir_filt, LINE_FILT_MAX, win);
    r_seg_filt_statics = line_seg_filter(r_seg_dir, r_seg_statics,
                                         r_seg_dir_filt, LINE_FILT_MAX, win);

    return mymin(l_seg_filt_statics, r_seg_filt_statics);
}

/* ============ 段方向压缩（删短 run + 连续相同的合并成一个） ============
 * 滤波版（l_seg_dir_filt）是用前值补、长度不变；压缩版是"真删再合并"：
 * 把连续相同的方向看成一个 run，只有长度 >= min_run 的 run 才认为可信，
 * 短于 min_run 的（只出现一次的孤立方向、以及整段没数据的 -1 run）直接删掉；
 * 剩下的按原顺序压紧，并且连续相同的方向只留一个（跨过被删掉的 run 之后
 * 方向相同的也合并）。所以 l_seg_dir_pack 里相邻两个值一定不同，
 * 段数也一定比原来少，段号不再和原来的段对应。
 * 压缩结果之后的槽位统一填 -1（0 是真正的 E 方向，不能拿 0 当空槽）。
 */

#define LINE_COMPACT_N  10      /* 压缩只处理前多少段（最下面、离车最近的那几段） */

int8_t l_seg_dir_pack[LINE_SEG_MAX];    /* 左折线压缩后的段方向 0..7，空槽为 -1 */
int8_t r_seg_dir_pack[LINE_SEG_MAX];    /* 右折线压缩后的段方向 0..7，空槽为 -1 */
int l_seg_pack_statics = 0;             /* 左折线压缩后的段数 */
int r_seg_pack_statics = 0;             /* 右折线压缩后的段数 */

int8_t l_seg_pack_start[LINE_SEG_MAX];  /* 压缩后每一项对应的原始段号（第几段） */
int8_t r_seg_pack_start[LINE_SEG_MAX];

/* 压缩后第一个折点的纵坐标（图像坐标 y，越大越靠下）；-1 = 没有折点 */
int l_first_bend_y = -1;
int r_first_bend_y = -1;

/**
 * @brief 按 run 压缩段方向数组：只处理最下面（最先走到）的前
 *        LINE_COMPACT_N 段，删掉长度不足 min_run 的 run，连续相同的
 *        方向合并成一个值；压缩结果之后的槽位全部置 -1。
 * @param[in]  src      输入段方向数组
 * @param[in]  n        输入段数
 * @param[out] dst      压缩结果（相邻元素一定不同），未用到的槽位填 -1
 * @param[in]  dst_max  dst 容量，写满就停
 * @param[in]  min_run  一个方向至少要连续出现几段才保留，<= 0 时用 2
 * @param[out] starts   每一项对应的原始段号（可传 0 不要），未用槽位填 -1
 * @return 压缩后的段数；0 = 全被删掉或参数无效（此时 dst 全 -1）
 * @note   只看前 LINE_COMPACT_N 段：巡线是从下往上走的，最下面几段离车最近、
 *         最可信；更上面的段不参与压缩（dst 里对应位置就是 -1）。
 *         先删短 run（含 -1 的 run），再把相邻相同的方向合并成一个，
 *         所以 dst 里不会出现两个连续相同的值；段号不再和原来对应。
 *         参数都做了保护：空指针/非正数安全返回 0。
 */
int line_seg_compact(const int8_t *src, int n, int8_t *dst, int dst_max,
                     int min_run, int8_t *starts)
{
    int i, k, run_len, out;

    if (dst == 0 || dst_max <= 0) {
        return 0;
    }
    /* 未用到的槽位先全部填 -1，避免残留上一帧的结果 */
    for (k = 0; k < dst_max; k++) {
        dst[k] = -1;
        if (starts != 0) {
            starts[k] = -1;
        }
    }
    if (src == 0 || n <= 0) {
        return 0;
    }
    if (n > LINE_SEG_MAX) {
        n = LINE_SEG_MAX;
    }
    if (n > LINE_COMPACT_N) {
        n = LINE_COMPACT_N;     /* 只压缩最下面（最先走到）的 LINE_COMPACT_N 段 */
    }
    if (min_run < 1) {
        min_run = 2;            /* 默认：必须连续出现两次以上才算可信 */
    }

    out = 0;
    i = 0;
    while (i < n && out < dst_max) {
        /* 找出和 src[i] 相同的一整段 run */
        run_len = 0;
        while (i + run_len < n && src[i + run_len] == src[i]) {
            run_len++;
        }

        /* 只保留够长的有效 run；和上一个结果相同就不再写，
         * 这样跨过被删掉的短 run / -1 run 之后相同的方向也会合并 */
        if (src[i] >= 0 && src[i] <= 7 && run_len >= min_run &&
            (out == 0 || dst[out - 1] != src[i])) {
            dst[out] = src[i];
            if (starts != 0) {
                starts[out] = i;    /* 这一项的方向是从第 i 段开始的 */
            }
            out++;
        }
        i += run_len;
    }

    return out;
}

/**
 * @brief 求压缩结果里"第一个折点"的纵坐标。
 * @param[in] pts    原折线的点数组（l_points / r_points）
 * @param[in] pt_cnt 原折线点数（l_statics / r_statics）
 * @param[in] starts 压缩结果的原始段号数组（l_seg_pack_start / r_seg_pack_start）
 * @param[in] cnt    压缩后的段数（l_seg_pack_statics / r_seg_pack_statics）
 * @return 折点的图像坐标 y（越大越靠下）；-1 = 不足两段方向（没有折点）
 * @note  折点就是第一段方向变成第二段方向的那个点，即 starts[1] 对应原始段的
 *        起点；段号 × g_seg_step 就得到它在原折线点列里的下标。
 */
int first_bend_y(const uint8_t (*pts)[2], int pt_cnt, const int8_t *starts, int cnt)
{
    int seg, pt;

    if (pts == 0 || starts == 0 || cnt < 2) {
        return -1;                  /* 只有一种方向，没有折点 */
    }

    seg = starts[1];
    pt = seg * g_seg_step;
    if (seg < 0 || pt < 0 || pt >= pt_cnt) {
        return -1;                  /* 段号/点号越界（数据不一致） */
    }

    return pts[pt][1];
}

/**
 * @brief 对左右边线的段方向数组压缩并合并相同方向，
 *        结果存进 l_seg_dir_pack[] / r_seg_dir_pack[]，同时算出
 *        压缩后第一个折点的纵坐标 l_first_bend_y / r_first_bend_y。
 * @param[in] min_run 一个方向至少连续出现几段才保留，<= 0 时用 2
 * @return 左右两边压缩后段数较少的那个
 * @note  先 get_line_seg_dir(0) 求出段方向，再调这个函数。
 *        压缩只处理前 LINE_COMPACT_N 段，压缩结果之后的槽位都是 -1。
 *        想压缩"众数滤波后"的数组，直接调
 *        line_seg_compact(l_seg_dir_filt, l_seg_filt_statics,
 *                         dst, LINE_SEG_MAX, min_run, starts) 即可。
 */
int get_line_seg_dir_compact(int min_run)
{
    l_seg_pack_statics = line_seg_compact(l_seg_dir, l_seg_statics,
                                          l_seg_dir_pack, LINE_SEG_MAX,
                                          min_run, l_seg_pack_start);
    r_seg_pack_statics = line_seg_compact(r_seg_dir, r_seg_statics,
                                          r_seg_dir_pack, LINE_SEG_MAX,
                                          min_run, r_seg_pack_start);

    /* 第一段方向变成第二段方向的位置，就是压缩后的第一个折点 */
    l_first_bend_y = first_bend_y(l_points, l_statics,
                                  l_seg_pack_start, l_seg_pack_statics);
    r_first_bend_y = first_bend_y(r_points, r_statics,
                                  r_seg_pack_start, r_seg_pack_statics);

    return mymin(l_seg_pack_statics, r_seg_pack_statics);
}


/**
 * restate 0:正常的线 1:十字路口 2: Y型 3: 环岛入口 4:环岛出口
 * reflag  0:正常的线 1:识别到路口 2:识别到Y 3
*/
int l1=-1,r1=-1,l2=-1,r2=-1;
int restate =0,reflag = 0;

void reset_restate(void)
{
    restate = 0;reflag = 0;
}

void analyze_dir(void)
{

    if(restate ==  0)
    {
        // 这个基于车得正
        if(l1 == r1 && ((r2 == (r1-2)%8 && l2 == (l1-2)%8) || (r2 == (r1+2)%8 && l2 == (l1+2)%8)))
        {restate = 6;reflag = 6;return;}
        if(l1 == r1 && l2 == (l1 -2)%8 && r2 == (r1 + 2)%8)
        {restate = 1;reflag = 1;return;}
        if(l1 == r1 && l2 == (l1 -1)%8 && r2 == (r1 + 1)%8)
        {restate = 2;reflag = 2;return ;}
        if(l1 == r1 && (l2 == (l1-3)%8 || l2 == (l1-4)%8) && r2 == -1)
        {
            // 给一个方向的控制
            LR = 0;
            restate = 3;reflag = 3;return;
        }
        if(l1 == r1 && (r2 == (r1+3)%8 || r2 == (r1+4)%8) && l2 == -1)
        {
            // 给一个方向的控制
            LR = 1;
            restate = 3;reflag = 3;return;
        }
        if(l1 == r1 && (l2 == (l1-3)%8 || l2 == (l1-4)%8) && (r2 == (r1+1)%8 || r2 == (r1+2)%8 ) )
        {
            // 给一个方向的控制
            LR = 1;
            restate = 4;reflag = 4;return;
        }
        if(l1 == r1 && (r2 == (r1+3)%8 || r2 == (r1+4)%8)&& (l2 == (l1-1)%8 || l2 == (l1-2)%8 ))
        {
            // 给一个方向的控制
            LR = 0;
            restate = 4;reflag = 4;return;
        }
        if(l1 == r1 && (l2 == (l1 - 2)%8) && r2 == (r1+1)%8)
        {
            out_y_lr = 1;
            restate = 5;reflag = 5;return;
        }
        if(l1 == r1 && l2 == (l1-1)%8 && r2 == (r1 + 2)%8)
        {
            out_y_lr = 0;
            restate = 5;reflag = 5;return;
        }
        restate = 0;reflag = 0;
        return;
    }
    if(restate == 1)
    {
        if(l1 == r1 && l2==r2)
        {restate = 0;reflag = 0;return;}
        restate = 1;reflag = 1;return ;
    }
    if(restate == 2)
    {
        if(l1 == r1 && l2==r2)
        {restate = 0;reflag = 0;return;}
        restate = 2;reflag = 2;return ;
    }
    if(restate == 3)
    {
        if(l1 == r1 && l2==r2)
        {restate = 0;reflag = 0;return;}
        restate = 3;reflag = 3;return ;
    }
    if(restate == 4)
    {
        if(l1 == r1 && l2==r2)
        {restate = 0;reflag = 0;return;}
        restate = 4;reflag = 4;return ;
    }
    if(restate == 5)
    {
        if(l1 == r1 && l2==r2)
        {restate = 0;reflag = 0;return;}
        restate = 5;reflag = 5;return;
    }
    if(restate == 6)
    {
        if(l1 == r1 && ((l_first_bend_y>=SCUT_IMAGE_H/6 && r_first_bend_y >= SCUT_IMAGE_H/6) || (l_first_bend_y == -1 && r_first_bend_y == -1)))
        {
            restate = 0;reflag = 0;return;
        }
        restate = 6;reflag = 6;
    }
}

void fill_Y_dir(int lr,int offsetx)
{
    int n = (lr == 0) ? l_statics : r_statics;
    if (n > LINE_LENGTH) {
        n = LINE_LENGTH;            /* target_mid_line 容量只有 LINE_LENGTH */
    }
    reset_target_line();
    if(lr == 0)
    {
        reset_target_line();
        for(int i = 0;i<n;i++)
        {
            target_mid_line[i][0] = l_points[i][0] + offsetx;
            target_mid_line[i][1] = l_points[i][1];
        }
        target_statics = n;
    }
    else{
        reset_target_l();
        for(int i = 0;i<n;i++)
        {
            target_mid_line[i][0] = r_points[i][0] - offsetx;
            target_mid_line[i][1] = r_points[i][1];
        }
        target_statics = n;
    }
}

void fill_crossing_dir(int offsety)
{
    int limit = mymin(l_statics, r_statics);
    if (limit > LINE_LENGTH) {
        limit = LINE_LENGTH;
    }
    reset_target_line();
    int i = 0;
    int y_start = SCUT_IMAGE_H-3-offsety-8;
    if(l_first_bend_y !=-1 && r_first_bend_y !=-1)
    {
        for(i=0;i<limit;i++)
        {
            if(l_points[i][1] <= l_first_bend_y && r_points[i][1] <= r_first_bend_y){
                break;
            }
            target_l_line[i][0] = l_points[i][0];
            target_l_line[i][1] = l_points[i][1];
            target_r_line[i][0] = r_points[i][0];
            target_r_line[i][1] = r_points[i][1];
        }
        y_start = (l_first_bend_y + r_first_bend_y) / 2 - offsety-3;
    }
    if (find_start(y_start, 20)) {
        forward();
        getmin_statics();          /* 这一句现在缺失，必须补 */
    } else {
        reset_points();
    }
    for(int j = 0;j<mymin(l_statics,r_statics) && i < LINE_LENGTH;j++)
    {
        target_l_line[i][0] = l_points[j][0];
        target_l_line[i][1] = l_points[j][1];
        target_r_line[i][0] = r_points[j][0];
        target_r_line[i][1] = r_points[j][1];
        i++;
    }
    target_statics = i;
    for(int j = 0;j<target_statics;j++)
    {
        target_mid_line[j][0] = (target_l_line[j][0] + target_r_line[j][0])/2;
        target_mid_line[j][1] = (target_l_line[j][1] + target_r_line[j][1])/2;
    }
}

void fill_roundabout_in_dir(int offsetx,int offsety)
{
    reset_target_line();
    if(LR == 0)
    {
        int i = 0;
        int start_y = SCUT_IMAGE_H-3 - offsety-5;
        if(l_first_bend_y != -1 && l_first_bend_y >= SCUT_IMAGE_H / 3)
        {
            for(i = 0;i<l_statics && i < LINE_LENGTH;i++)
            {
                if(l_points[i][1] <= l_first_bend_y)
                {break;}
                target_mid_line[i][0] = l_points[i][0] + offsetx;
                target_mid_line[i][1] = l_points[i][1];
            }
            start_y = l_first_bend_y - offsety -5 ;
        }
        if (find_start(start_y, 10)) {
            forward();
            getmin_statics();          /* 这一句现在缺失，必须补 */
        } else {
            reset_points();
        }
        for(int j = 0;j<l_statics && i < LINE_LENGTH;j++)
        {
            target_mid_line[i][0] = l_points[j][0]+ offsetx;
            target_mid_line[i][1] = l_points[j][1];
            i++;
        }
        target_statics = i;
    }
    else{
        int i = 0;
        int start_y = SCUT_IMAGE_H-3 - offsety-5;
        if(r_first_bend_y != -1 && r_first_bend_y >= SCUT_IMAGE_H / 3)
        {
            for(i = 0;i<r_statics && i < LINE_LENGTH;i++)
            {
                if(r_points[i][1] <= r_first_bend_y)
                {break;}
                target_mid_line[i][0] = r_points[i][0] - offsetx;
                target_mid_line[i][1] = r_points[i][1];
            }
            start_y = r_first_bend_y - offsety-5;
        }
        if (find_start(start_y, 10)) {
            forward();
            getmin_statics();          /* 这一句现在缺失，必须补 */
        } else {
            reset_points();
        }
        for(int j = 0;j<r_statics && i < LINE_LENGTH;j++)
        {
            target_mid_line[i][0] = r_points[j][0] - offsetx;
            target_mid_line[i][1] = r_points[j][1];
            i++;
        }
        target_statics = i;
    }
}

void fill_roundabout_out_dir(int offsetx,int offsety)
{
    reset_target_line();
    if(LR == 1)
    {
        int i = 0;
        int start_y = SCUT_IMAGE_H-3 - offsety-5;
        if(l_first_bend_y != -1)
        {
            for(i = 0;i<l_statics && i < LINE_LENGTH;i++)
            {
                if(l_points[i][1] <= l_first_bend_y)
                {break;}
                target_mid_line[i][0] = l_points[i][0] + offsetx;
                target_mid_line[i][1] = l_points[i][1];
            }
            start_y = l_first_bend_y - offsety-5;
        }
        if (find_start(start_y, 10)) {
            forward();
            getmin_statics();          /* 这一句现在缺失，必须补 */
        } else {
            reset_points();
        }
        for(int j = 0;j<l_statics && i < LINE_LENGTH;j++)
        {
            target_mid_line[i][0] = l_points[j][0]+ offsetx;
            target_mid_line[i][1] = l_points[j][1];
            i++;
        }
        target_statics = i;
    }
    else{
        int i = 0;
        int start_y = SCUT_IMAGE_H-3 - offsety-5;
        if(r_first_bend_y != -1)
        {
            for(i = 0;i<r_statics && i < LINE_LENGTH;i++)
            {
                if(r_points[i][1] <= r_first_bend_y)
                {break;}
                target_mid_line[i][0] = r_points[i][0] - offsetx;
                target_mid_line[i][1] = r_points[i][1];
            }
            start_y = r_first_bend_y - offsety-5;
        }
        if (find_start(start_y, 10)) {
            forward();
            getmin_statics();          /* 这一句现在缺失，必须补 */
        } else {
            reset_points();
        }
        for(int j = 0;j<r_statics && i < LINE_LENGTH;j++)
        {
            target_mid_line[i][0] = r_points[j][0] - offsetx;
            target_mid_line[i][1] = r_points[j][1];
            i++;
        }
        target_statics = i;
    }
}

void fill_none_dir(void)
{
    /* target_*_line 只有 LINE_LENGTH 个槽位，巡线点数可能更多，必须截断，
     * 否则会写到数组外面。 */
    int ln = l_statics > LINE_LENGTH ? LINE_LENGTH : l_statics;
    int rn = r_statics > LINE_LENGTH ? LINE_LENGTH : r_statics;
    reset_target_line();
    for(int i = 0;i<ln;i++)
    {
        target_l_line[i][0] = l_points[i][0];
        target_l_line[i][1] = l_points[i][1];
    }
    target_l_statics = ln;
    for(int i = 0;i<rn;i++)
    {
        target_r_line[i][0] = r_points[i][0];
        target_r_line[i][1] = r_points[i][1];
    }
    target_r_statics = rn;
    int j;
    for(j = 0;j<mymin(target_l_statics,target_r_statics);j++)
    {
        target_mid_line[j][0] = (target_l_line[j][0] + target_r_line[j][0])/2;
        target_mid_line[j][1] = (target_l_line[j][1] + target_r_line[j][1])/2;
    }
    target_statics = j;
}


void update_dir_info(void)
{
    if(restate == 0)
    {
        fill_none_dir();LR=0;out_y_lr = 0;
    }
    if(restate == 1)
    {
        fill_crossing_dir(Offset_y);
    }
    if(restate == 2)
    {
        fill_Y_dir(0, Offset_x);
    }
    if(restate == 3)
    {
        fill_roundabout_in_dir(Offset_x, Offset_y);
    }
    if(restate == 4)
    {
        fill_roundabout_out_dir(Offset_x, Offset_y);
    }
    if(restate == 5)
    {
        fill_Y_dir(out_y_lr, Offset_x);
    }
    if(restate == 6)
    {
        fill_none_dir();
    }
}

/* ========== 把 target_mid_line 补成一条连续的中线（final_line） ==========
 * target_mid_line 里可能有两种"断"：
 *   1) 线的开头不是 first_point（十字/环岛补线时会从弯点上方重新巡线，
 *      下面那一段没接上）——先把 first_point 写进去，再用直线连到线的开头；
 *   2) 中间相邻两点的间距超过 1（缺行 / 跳变）——用直线把中间的点补出来。
 * 结果放进 final_line[]，点数在 final_statics 里；补完之后相邻两点一定是
 * 8 邻接的（|dx|<=1 且 |dy|<=1），可以直接拿去算预瞄/拟合。
 */



/**
 * @brief 把一段可能间断的折线补成连续折线。
 * @param[in]  src    输入点列（target_mid_line）
 * @param[in]  n      输入点数（target_statics）
 * @param[in]  fx,fy  起始点（first_point）；任一为负表示没有起始点
 * @param[out] dst    输出点列（final_line）
 * @param[in]  dst_max dst 容量，写满就停
 * @return 输出点数
 * @note   先把 (fx,fy) 放进 dst；之后每读一个点，如果它和上一个写入的点
 *         不相邻（|dx|>1 或 |dy|>1），就先在两点之间按直线补点，再写该点。
 *         重复点会跳过，所以不管线的开头是不是 (fx,fy)，结果都是连续的。
 *         中线上坐标 < 0 的无效点会被跳过，它前后的点照样连起来。
 */
int connect_line(const int16_t (*src)[2], int n, int fx, int fy,
                 int16_t (*dst)[2], int dst_max)
{
    int i, out = 0;
    int px = 0, py = 0;         /* 上一个写进 dst 的点 */
    int have = 0;               /* dst 里是否已经有上一个点 */

    if (dst == 0 || dst_max <= 0) {
        return 0;
    }
    if (src == 0) {
        n = 0;
    }
    if (n > LINE_LENGTH) {
        n = LINE_LENGTH;
    }

    /* 起始点先放进去；线的第一个有效点会和它用直线连上 */
    if (fx >= 0 && fy >= 0) {
        dst[0][0] = fx;
        dst[0][1] = fy;
        px = fx;
        py = fy;
        have = 1;
        out = 1;
    }

    for (i = 0; i < n && out < dst_max; i++) {
        int x = src[i][0];
        int y = src[i][1];
        int dx, dy, adx, ady, steps, t;

        if (x < 0 || y < 0) {
            continue;               /* 无效点：跳过，前后两点直接连 */
        }
        if (have && x == px && y == py) {
            continue;               /* 和上一个点重复，不写第二遍 */
        }

        if (have) {
            dx = x - px;
            dy = y - py;
            adx = (dx < 0) ? -dx : dx;
            ady = (dy < 0) ? -dy : dy;
            steps = (adx > ady) ? adx : ady;

            /* 两点不相邻：按直线补中间的点（不含两端） */
            for (t = 1; t < steps && out < dst_max; t++) {
                dst[out][0] = px + (dx * t) / steps;
                dst[out][1] = py + (dy * t) / steps;
                out++;
            }
        }

        if (out < dst_max) {
            dst[out][0] = x;
            dst[out][1] = y;
            out++;
            px = x;
            py = y;
            have = 1;
        }
    }

    return out;
}

/**
 * @brief 用 first_point 和 target_mid_line 生成连续中线 final_line[]。
 * @param[in] fx,fy 起始点（一般就是 first_point_x, first_point_y）
 * @return final_line 的点数（同时放在 final_statics 里）
 * @note   update_dir_info() 之后调用；只保证 0..final_statics-1 这些点有效。
 */
int make_final_line(int fx, int fy)
{
    final_statics = connect_line(target_mid_line, target_statics,
                                 fx, fy, final_line, FINAL_LINE_MAX);
    return final_statics;
}


void team_vision_reset(void)
{
   reset_restate();
}

int px1=-1,px2=-1,px3=-1,py1=-1,py2=-1,py3=-1;

int is_have_line = 0;
void get_ppoints(void)
{
    px1 = first_point_x;
    px2 = X1;
    px3 = X2;
    py1 = first_point_y;
    py2 = Y1;
    py3 = Y2;
}

void set_ppoints(void)
{
    first_point_x = px1;
    X1=px2;           
    X2=px3;           
    first_point_y=py1;
    Y1=py2;           
    Y2=py3;           
}

void reset_XY(void)
{
    first_point_x = -1;
    X1 = -1;
    X2 = -1;
    first_point_y = -1;
    Y1 = -1;
    Y2 =-1;
}

bool team_vision_points(const FrameView *frame, int preview_px, TeamPixel points[3],
                        TeamPixel *near_mid, int *element, int *stage)
{
    *element = *stage = -1;
    reset_lrxy();
    reset_points();
    reset_XY();
    if (!frame || !frame->pixels || frame->width != APP_WIDTH ||
        frame->height != APP_HEIGHT || frame->stride < frame->width)
        return false;
    source_frame=frame;
    int threshold=GetThre(experience_value);
    if (threshold < 0) return false;
    mybinary(threshold);
    mypadding(2);
    if (find_start(SCUT_IMAGE_H-3,10) && forward())
    {
        getmin_statics();
        reset_target_line();
        get_first_point();
        get_line_seg_dir(0); 
        get_line_seg_dir_filt(0);
        get_line_seg_dir_compact(2);
        l1=l_seg_dir_pack[0],r1=r_seg_dir_pack[0],l2=l_seg_dir_pack[1],r2=r_seg_dir_pack[1];
        analyze_dir();
        update_dir_info();
        make_final_line(first_point_x, first_point_y);
        if(final_statics <= 5)
        {
            is_have_line = 0;
            set_ppoints();
            
        }
        else{
            is_have_line = 1;
            pred_points(preview_px, near_mid);
            get_ppoints();
        }
        
    }
    else{
        is_have_line = 0;
        set_ppoints();
    }
    *element=restate;
    *stage=is_have_line;
    points[0]=(TeamPixel){first_point_x,first_point_y};
    points[1]=(TeamPixel){X1,Y1};
    points[2]=(TeamPixel){X2,Y2};
    return true;
}
