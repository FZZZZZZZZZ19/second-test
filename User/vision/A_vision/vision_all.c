
//// 注意：reality_x1,reality_x2,reality_x3,reality_y1,reality_y2,reality_y3,state,flag是要暴露出来的量
#include "vision_all.h"

// #define SCUT_IMAGE_H 120
// #define SCUT_IMAGE_W 188
// #define MAX_STEP 2000
// #define LINE_LENGTH (SCUT_IMAGE_H+SCUT_IMAGE_W + 50)

unsigned char origin_image[SCUT_IMAGE_H][SCUT_IMAGE_W];
unsigned char g_binary[SCUT_IMAGE_H][SCUT_IMAGE_W];
int g_threholds;
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
int white_period[SCUT_IMAGE_H] = {0};
int white_count[SCUT_IMAGE_H] = {0};
int white_change = 0;

void reset_white(void)
{
    for(int i = 0;i<SCUT_IMAGE_H;i++)
    {
        white_period[i] = 0;
        white_count[i] = 0;
    }
    white_change = 0;
}


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
    int i ;
    for (i = 0; i < SCUT_IMAGE_H; i++) {
        int x;
        for (x = 0; x < SCUT_IMAGE_W; x++) {
            g_binary[i][x] = 0;
            origin_image[i][x] = 0;
        }
    }
    reset_state_flag();
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
            amount[origin_image[y][x]]++;
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
            g_binary[y][x] = (origin_image[y][x] > thre ? 255 : 0);
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

/**
 * @brief 防止访问出界的一种方式。
 * @param[in] x,y 要检查的点的坐标
 */
int is_white(int x, int y)
{
    if (x < 0 || x >= SCUT_IMAGE_W || y < 0 || y >= SCUT_IMAGE_H) {
        return 0;
    }
    return g_binary[y][x] != 0;
}

/**
 * @brief 绝对值函数
*/
int myabs(int v)
{
    return (v < 0) ? -v : v;
}

/**
 * @brief 返回小的值
*/
int mymin(int a,int b)
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
int find_y_start(int start_y)
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

        if (width < 3 || width > 45) {
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
int find_start(int start_y, int max_line)
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

/* 8 邻域方向，按屏幕坐标顺时针排列：
 * 0=E 1=SE 2=S 3=SW 4=W 5=NW 6=N 7=NE */
static const int DIR8[8][2] = {
    { 1,  0}, { 1,  1}, { 0,  1}, {-1,  1},
    {-1,  0}, {-1, -1}, { 0, -1}, { 1, -1}
};

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

    put_point_info(lxs, lys, ldir, 0);
    put_point_info(rxs, rys, rdir, 1);

    while (step < MAX_STEP - 1 && (!ldone || !rdone)) {
        if (!ldone) {
            if (!one_step(&lxs, &lys, &ldir, &lxb, &lyb, 0)) {
                ldone = 1;
                lfail = 1;
            } else {
                put_point_info(lxs, lys, ldir, 0);
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
                put_point_info(rxs, rys, rdir, 1);
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
 * @brief 还能用吧只能说是
 * @param[in] white_width 不被忽略的白色的最小长度
 * @param[in] black_width 被认为是连续的黑色的最大长度
*/
void count_white(int white_width,int black_width)
{
    reset_white();
    int l = -1,r = -1;
    int count = 0;
    int state = 0;
    for(int y = SCUT_IMAGE_H-1;y>=0;y--)
    {
        count = 0;
        state = 0;
        l = -1,r = -1;
        for(int x = 0;x<SCUT_IMAGE_W;x++)
        {
            if(g_binary[y][x] == 255 && state == 0)
            {
                if(r != -1 && (x-r) <= black_width)
                {
                    state = 1;
                }
                else if(r != -1 && (x-r) > black_width)
                {
                    state = 1;
                    if((r-l) >= white_width){
                        count++;
                        white_count[y] += (r-l);
                        // SCUT_DrawPoint(r,y, SCUT_COLOR_RED);
                        // SCUT_DrawPoint(l,y, SCUT_COLOR_RED);
                    }
                    l = x;
                }
                else if(r == -1)
                {
                    l = x;state = 1;
                }
            }
            if(g_binary[y][x] == 0 && state == 1)
            {
                r = x;state = 0;
            }
        }
        if((r-l) > white_width)
        {
            count++;
            white_count[y] += (r-l);
            // SCUT_DrawPoint(r,y, SCUT_COLOR_RED);
            // SCUT_DrawPoint(l,y, SCUT_COLOR_RED);
        }
        white_period[y] = count;
    }
}

/**
 * @brief 用于修补十字路口的函数
 * @param[in] down 这个是视觉上的下面，对应在坐标系中应该是y大的那一个
 * @param[in] up 这个是视觉上的上面，对应出来的y小的那个
 * @param[in] theta 最多给1，2
*/
void fill_crossing(int Down,int Up,int theta,int width)
{
    int down = Down + theta;
    int up = Up - theta;
    int pair_count, n, i, start = -1, above = -1;
    int idx0, idx1, index;
    int l_x0, l_y0, l_x1, l_y1;
    int r_x0, r_y0, r_x1, r_y1;
    float lk, rk, ldy, rdy;
    int step;
    int use_below;

    if(down < 0)
        down = 0;
    if(up < 0)
        up = 0;
    if(down > SCUT_IMAGE_H - 1)
        down = SCUT_IMAGE_H - 1;
    if(up > SCUT_IMAGE_H - 1)
        up = SCUT_IMAGE_H - 1;
    if(down < up)
    {
        int t = down;
        down = up;
        up = t;
    }

    /* 无论本次能否补线，都不把上一帧的目标线留给调用者。 */
    target_l_statics = 0;
    target_r_statics = 0;

    /* getmin_statics() 可能还没被调用；这里自己取左右点数的较小值。 */
    pair_count = mymin(l_statics, r_statics);
    n = (min_statics > 0) ? mymin(min_statics, pair_count) : pair_count;
    n = mymin(n, LINE_LENGTH);
    if(n <= 0)
        return;

    step = (width > 0) ? width : 1;

    /* 先抄写 down 以下（y 更大）的点，并找到第一个 y <= down 的点。 */
    for(i = 0;i < n;i++)
    {
        if(l_points[i][1] <= down || r_points[i][1] <= down)
        {
            start = i;
            break;
        }
        target_l_line[i][0] = l_points[i][0];
        target_l_line[i][1] = l_points[i][1];
        target_r_line[i][0] = r_points[i][0];
        target_r_line[i][1] = r_points[i][1];
    }
    if(start < 0)
        return;

    /* 找到 up 以上（y 更小）的第一个点，补线后从这里继续抄。 */
    for(i = start;i < n;i++)
    {
        if(l_points[i][1] <= up && r_points[i][1] <= up)
        {
            above = i;
            break;
        }
    }

    /*
     * 下面长度足够时，用 down 以下的两点算斜率；
     * 否则用 up 以上的两点算斜率。两种方式都要保证取点不越界。
     */
    use_below = (SCUT_IMAGE_H - down > step + 3);
    idx0 = -1;
    idx1 = -1;

    if(use_below && start > 0)
    {
        idx0 = start - 1;
        idx1 = (idx0 - step >= 0) ? (idx0 - step) : 0;
        if(idx1 == idx0)
            idx1 = -1;
    }
    if(idx0 < 0 || idx1 < 0)
    {
        int base = (above >= 0) ? above : start;
        idx0 = base;
        idx1 = base + step;
        if(idx1 >= n)
            idx1 = n - 1;
        if(idx1 == idx0)
        {
            if(idx0 > 0)
                idx1 = idx0 - 1;
            else if(idx0 + 1 < n)
                idx1 = idx0 + 1;
            else
                idx1 = -1;
        }
    }
    if(idx0 < 0 || idx1 < 0)
        return;

    l_x0 = l_points[idx0][0];
    l_y0 = l_points[idx0][1];
    l_x1 = l_points[idx1][0];
    l_y1 = l_points[idx1][1];
    r_x0 = r_points[idx0][0];
    r_y0 = r_points[idx0][1];
    r_x1 = r_points[idx1][0];
    r_y1 = r_points[idx1][1];

    ldy = (float)(l_y1 - l_y0);
    rdy = (float)(r_y1 - r_y0);
    if(ldy == 0.0f)
        ldy = 0.5f;
    if(rdy == 0.0f)
        rdy = 0.5f;
    lk = (float)(l_x1 - l_x0) / ldy;
    rk = (float)(r_x1 - r_x0) / rdy;

    /* down..up 这一段必须放得进 target 数组，否则宁可不补。 */
    index = start;
    if(index + (down - up + 1) > LINE_LENGTH)
        return;

    for(int y = down;y >= up;y--)
    {
        int plx = (int)(lk * (y - l_y0) + l_x0);
        int prx = (int)(rk * (y - r_y0) + r_x0);

        if(plx < 0)
            plx = 0;
        else if(plx >= SCUT_IMAGE_W)
            plx = SCUT_IMAGE_W - 1;
        if(prx < 0)
            prx = 0;
        else if(prx >= SCUT_IMAGE_W)
            prx = SCUT_IMAGE_W - 1;

        target_l_line[index][0] = plx;
        target_r_line[index][0] = prx;
        target_l_line[index][1] = y;
        target_r_line[index][1] = y;
        index++;
    }

    /* 从 up 以上的点继续接上，避免重复抄已经补过的 up 行。 */
    if(above >= 0)
    {
        if(l_points[above][1] == up && r_points[above][1] == up)
            above++;
        for(i = above;i < n && index < LINE_LENGTH;i++)
        {
            target_l_line[index][0] = l_points[i][0];
            target_r_line[index][0] = r_points[i][0];
            target_l_line[index][1] = l_points[i][1];
            target_r_line[index][1] = r_points[i][1];
            index++;
        }
    }

    target_l_statics = index;
    target_r_statics = index;
}

/**
 * @brief 用于补Y型的分岔口的补线代码，用的方式是直接取一边的线
 * @param[in] lr 向左还是向右 
*/
void fill_Y(int lr)
{
    if(lr == 0)
    {
        reset_target_r();
        for(int i = 0;i<l_statics;i++)
        {
            target_l_line[i][0] = l_points[i][0];
            target_l_line[i][1] = l_points[i][1];
        }
        target_l_statics = l_statics;
    }
    else{
        reset_target_l();
        for(int i = 0;i<r_statics;i++)
        {
            target_r_line[i][0] = r_points[i][0];
            target_r_line[i][1] = r_points[i][1];
        }
        target_r_statics = r_statics;
    }
}

/**
 * @brief 这是环岛的第一步判断(看情况可能用不到)
 * @param[in] cx,cy 这个是环岛上面那个分叉点的位置
 * @param[in] lr 左环岛还是右环岛
*/
void fill_roundaboutI(int cx,int cy, int lr)
{
    if(lr == 0)
    {
        reset_target_l();
        int index = 0;
        int x,y;
        x = r_points[0][0];
        y = r_points[0][1];
        float k = (float)(cx-x)/(float)(cy-y == 0 ? cy - y + 0.5 : cy -y);
        for(int yy = SCUT_IMAGE_H-1;yy>=cy;yy--)
        {
            target_r_line[index][0] = (int)(k*(yy-cy)+cx);
            target_r_line[index][1] = yy;
            index++;
        }
        target_r_statics = index;
        // 可以进行一些代码的补充
    }
    else {
        reset_target_r();
        int index = 0;
        int x = l_points[0][0];
        int y = l_points[0][1];
        float k = (float)(cx -x)/(float)(cy-y == 0 ? cy-y + 0.5 : cy-y);
        for(int yy = SCUT_IMAGE_H -1;yy >= cy;yy--)
        {
            target_l_line[index][0] = (int)(k*(yy-cy)+cx);
            target_l_line[index][1] = yy;
            index++;
        }
        target_l_statics = index;
        // 这里仍然可以进行补充
    }
}

/**
 * @brief 这个是环岛的判断的第二步
 * @param[in] lr 依旧是左右环岛
*/
void fill_roundaboutII(int lr)
{
    fill_Y(lr);
}

/**
 * @brief 这个是环岛判断的第三步(刚开始出环岛的情况)
*/
void fill_roundaboutIII(int cx,int cy,int lr)
{
    if(lr == 0)
    {
        reset_target_r();
        int x,y,index = 0;
        x = l_points[0][0];
        y = l_points[0][1];
        float k = (float)(cx-x)/(float)(cy-y == 0 ? cy - y + 0.5 : cy-y);
        for(int yy = SCUT_IMAGE_H - 1;yy>=cy;yy--)
        {
            target_l_line[index][0] = (int)(k*(yy-cy)+cx);
            target_l_line[index][1] = yy;
            index++;
        }
        target_l_statics = index;
        //
    }
    else{
        reset_target_l();
        int x,y,index= 0;
        x = r_points[0][0];
        y = r_points[0][1];
        float k = (float)(cx-x)/(float)(cy-y == 0? cy - y + 0.5 : cy-y);
        for(int yy = SCUT_IMAGE_H-1;yy>=cy;yy--)
        {
            target_r_line[index][0] = (int)(k*(yy-cy)+cx);
            target_r_line[index][1] = yy;
            index++;
        }
        target_r_statics = index;
        //
    }
}

/**
 * @brief 这个是环岛的最后一步(但是依旧是出环岛的情况)
*/
void fill_roundaboutIV(int lr)
{
    fill_Y(!lr);
}

void fill_none(void)
{
    reset_target_line();
    for(int i = 0;i<l_statics;i++)
    {
        target_l_line[i][0] = l_points[i][0];
        target_l_line[i][1] = l_points[i][1];
    }
    target_l_statics = l_statics;
    for(int i = 0;i<r_statics;i++)
    {
        target_r_line[i][0] = r_points[i][0];
        target_r_line[i][1] = r_points[i][1];
    }
    target_r_statics = r_statics;
}

/**
 * @brief 拟合中线的
*/
int match_target_line(int theta)
{
    if(target_l_statics != 0 && target_r_statics != 0)
    {
        for(int i = 0;i<mymin(target_l_statics,target_r_statics);i++)
        {
            target_mid_line[i][0] = (target_l_line[i][0] + target_r_line[i][0])/2;
            target_mid_line[i][1] = (target_l_line[i][1] + target_r_line[i][1])/2;
        }
        target_statics = mymin(target_l_statics,target_r_statics);
        return 1;
    }
    else if(target_l_statics == 0 && target_r_statics != 0)
    {
        for(int i = 0;i<target_r_statics;i++)
        {
            target_mid_line[i][0] = target_r_line[i][0] - theta; // 这里需要进行防出界的东西
            if(target_mid_line[i][0] < 0)
            {target_mid_line[i][0] = 0;}
            else if(target_mid_line[i][0] > 188)
            {target_mid_line[i][0] = 188;}
            target_mid_line[i][1] = target_r_line[i][1];
        }
        target_statics = target_r_statics;
        return 1;
    }
    else if(target_l_statics != 0 && target_r_statics == 0)
    {
        for(int i = 0;i<target_l_statics;i++)
        {
            target_mid_line[i][0] = target_l_line[i][0] + theta; // 这里需要进行防出界的东西
            if(target_mid_line[i][0] < 0)
            {target_mid_line[i][0] = 0;}
            else if(target_mid_line[i][0] > 188)
            {target_mid_line[i][0] = 188;}
            target_mid_line[i][1] = target_l_line[i][1];
        }
        target_statics = target_l_statics;
        return 1;
    }
    return 0;
}



/**
 * @brief 预瞄点的寻找
 * @param[in] distance 寻找的预瞄点的距离，尽量大于两倍的路宽
 * @param[in] threta 防止除数为零的手段，越大误差越大
*/

int X1 = -1,Y1 = -1,X2 = -1,Y2 = -1;

void pred_points(int distance)
{
    if (target_statics <= 0) 
    {
        X1 = Y1 = X2 = Y2 = -1;
        return;
    }   
    int distance2 = distance*distance;
    int mid_x = SCUT_IMAGE_W/2;
    int mid_y = SCUT_IMAGE_H-3;
    int dx,dy,dist;
    int is_have = 0,index = -1;
    for(int i = 0;i<target_statics;i++)
    {
        dx = target_mid_line[i][0] - mid_x;
        dy = target_mid_line[i][1] - mid_y;
        dist = dx*dx+dy*dy;
        if(dist>=distance2)
        {
            is_have = 1;
            index = i;
            break;
        }
    }
    if(is_have == 0)
    {
        index = target_statics - 1;
    }
    X1 = target_mid_line[index/2][0];
    Y1 = target_mid_line[index/2][1];
    X2 = target_mid_line[index][0];
    Y2 = target_mid_line[index][1];
    // dx = target_mid_line[index][0] - mid_x;
    // dy = target_mid_line[index][1] = mid_y;
    // angle = (float)(atan2(dx,dy+theta)*180.0/3.1415926535); // 防止报错
    // target_lr = dx >= 0 ? 1 : 0; 
}

/**
 * @brief 用来约束白块的数量
*/
void limit_white(void)
{
    for(int i = 0;i<SCUT_IMAGE_H;i++)
    {
        if(white_period[i] >= 3)
            white_period[i] = 2;
    }
}

/**
 * @brief 获得一个中位数
*/
int get_midnum(int a,int b,int c,int d,int e)
{
    int t;

    if (a > b) { t = a; a = b; b = t; }

    if (c > d) { t = c; c = d; d = t; }

    if (a > c) {
        t = a; a = c; c = t;
        t = b; b = d; d = t;
    }

    if (b > e) { t = b; b = e; e = t; }

    if (b > c) {
        return (b < d) ? b : d;
    } else {
        return (c < e) ? c : e;
    }
}

/**
 * @brief 这个算法好用奥，用中位数来代替这个的连续的白块的值
*/

int white_deal[SCUT_IMAGE_H] = {-1};

void putin(void)
{
    for(int i = 0;i<SCUT_IMAGE_H;i++)
    {
        white_deal[i] = white_period[i];
    }
}

void deal_white(void)
{
    putin();
    for(int i = 2;i<SCUT_IMAGE_H-2;i++)
    {
        white_period[i] = get_midnum(white_deal[i-2],white_deal[i-1],white_deal[i],white_deal[i+1], white_deal[i+2]);
    }
}

int minus_to_y[30] = {-1};
int minus_white[30] = {-1};
int minus_statics = 0;

void reset_minuswhite(void)
{
    for(int i = 0;i<30;i++)
    {
        minus_white[i] = -1;
        minus_to_y[i] = -1;
    }
    minus_statics = 0;
}
/**
 * @brief 重进下进行整理
*/
void collate_white(int collate_length)
{
    reset_minuswhite();
    minus_white[0] = 0;
    int index = 1;
    for(int i = SCUT_IMAGE_H-2;i>=SCUT_IMAGE_H-collate_length;i--)
    {
        if(white_period[i] != minus_white[index-1])
        {
            minus_white[index] = white_period[i];
            minus_to_y[index] = i;
            index++;
        }
        if (index >= 29) break;
    }
    minus_statics = index;
    minus_white[index] = 0;
    minus_statics ++;
}

/**
 * @brief 用于查找一个数组中是否有另一个数组
*/
int containsSubarray(const int *text, int n, const int *pat, int m) {
    if (m == 0) return 1;      /* 空数组视为包含 */
    if (m > n)  return 0;

    for (int i = 0; i + m <= n; i++) {
        int j = 0;
        while (j < m && text[i + j] == pat[j])
            j++;
        if (j == m)
            return 1;          /* 找到 */
    }
    return 0;
}

int all_white = SCUT_IMAGE_W - 30;
int half_white = SCUT_IMAGE_W/2 - 15;

int have_all_white(void)
{
    int count = 0;
    for(int i = 0;i<SCUT_IMAGE_H;i++)
    {
        if(white_count[i] >= all_white)
        {count++;}
    }
    if(count >= 5)
        return 1;
    else
        return 0;
}

int have_half_white(void)
{
    int count = 0;
    for(int i = 0;i<SCUT_IMAGE_H;i++)
    {
        if(white_count[i] >= half_white && white_count[i] <= all_white)
        {count++;}
    }
    if(count >= 5)
        return 1;
    else
        return 0;
}


/**
 * @brief state: 0 正常情况 1 直角弯 2 识别到1->2的跳变 3 十字路口 4 环岛
 * @brief flag: 0 正常情况 1 识别到直角弯 2 识别1到2 3 进入Y 4 识别到十字路口 5 识别到环岛 6 开始进入环岛 
 * @brief flag: 7 进入1 8 进入2 9 识别到退出 10 开始推出 11 推出1  
*/
void analyze(void)
{
    if(state == 4)
    {
        if(flag == 5)
        {
            for(int i = 2;i<minus_statics;i++)
            {
                if(minus_white[i-2] == 2 && minus_white[i-1] == 1 &&minus_white[i] == 2)
                {return;}
            }
            if(minus_white[0] == 0 && minus_white[1] == 1 && minus_white[2] == 2)
            {flag++;return;}
            if(minus_white[0] == 0 && minus_white[1] == 2)
            {flag+=2;return;}
            return;
        }
        if(flag == 6)
        {
            if(minus_white[0] == 0 && minus_white[1] == 2)
            {flag++;return;}
            if(minus_white[0] == 0 && minus_white[1] == 1)
            {flag+=2;return;}
            return;
        }
        if(flag == 7)
        {
            if(minus_white[0] == 0 && minus_white[1] == 1)
            {flag++;return;}
            return;
        }
        if(flag == 8)
        {
            for(int i = 2;i<minus_statics;i++)
            {
                if(minus_white[i-2] == 2 && minus_white[i-1] == 1 &&minus_white[i] == 2)
                {flag++;return;}
            }
        }
        if(flag == 9)
        {
            if(minus_white[0] == 0 && minus_white[1] == 1 && minus_white[2] == 2)
            {flag++;return;}
            if(minus_white[0] == 0 && minus_white[1] == 2)
            {flag+=2;return;}
            return;
        }
        if(flag == 10)
        {
            if(minus_white[0] == 0 && minus_white[1] == 2)
            {flag++;return;}
            if(minus_white[0] == 0 && minus_white[1] == 1)
            {flag = 0;state = 0;return;}
            return;
        }
        if(flag == 11)
        {
            if(minus_white[0] == 0 && minus_white[1] == 1)
            {flag = 0;state = 0;return;}
            return;
        }
    }
    if(state == 2)
    {
        if(flag == 2)
        {
            for(int i = 2;i<minus_statics;i++)
            {
                if(minus_white[i-2] == 2 && minus_white[i-1] == 1 &&minus_white[i] == 2)
                {state = 4;flag = 5;return;}
            }
            if(minus_white[0] == 0 && minus_white[1] == 2)
            {
                flag++;return;
            }
            if(minus_white[0] == 0 && minus_white[1] == 1 && minus_white[2] != 2)
            {state = 0;flag = 0;return;}
            return;
        }
        if(flag == 3)
        {
            if(minus_white[0] == 0 && minus_white[1] == 1)
            {state = 0;flag = 0;return;}
            return;
        }
    }
    if(state == 3)
    {
        if(flag == 4)
        {
            if(have_all_white())
            {return;}
            if(!have_all_white())
            {state = 0;flag = 0;return;}
        }
    }
    if(state == 1)
    {
        if(flag == 1)
        {
            if(have_half_white())
            {return;}
            if(!have_half_white())
            {state = 0;flag = 0;return;}
        }
    }
    if(state == 0)
    {
        if(have_all_white())
        {state = 3;flag = 4;return;}
        if(have_half_white())
        {state = 1;flag = 1;return;}
        if(minus_white[0] == 0 && minus_white[1] == 1 &&minus_white[2] == 2 && minus_white[3] != 1)
        {state = 2;flag = 2;return;}
        for(int i = 2 ;i<minus_statics ;i++)
        {
            if(minus_white[i-2] == 2 && minus_white[i-1] == 1 && minus_white[i] == 2)
            {state = 4;flag = 5;return;}
        }
        return;
    }
}


/**
 * @brief 要不一个太远的话不做偏移
*/


int LR = 0;

int bottom = -1;                 /* 十字路口底边 y，越大越靠下 */
int top    = -1;                 /* 十字路口顶边 y，越小越靠上 */
int cx     = -1;                 /* Y/环岛分叉点 x */
int cy     = -1;                 /* Y/环岛分叉点 y */
int elem_lr = 0;                 /* 0=左, 1=右 */
int elem_lr_manual = 0;          /* Y型人工给定：0=左Y, 1=右Y（只改这个） */

void reset_info(void)
{
    bottom = -1;                 /* 十字路口底边 y，越大越靠下 */
    top    = -1;                 /* 十字路口顶边 y，越小越靠上 */
    cx     = -1;                 /* Y/环岛分叉点 x */
    cy     = -1;                 /* Y/环岛分叉点 y */
    elem_lr = 0;                 /* 0=左, 1=右 */
    elem_lr_manual = 0;          /* Y型人工给定：0=左Y, 1=右Y（只改这个） */
}


int findClosestIndex(const int* arr, int n, int target) {
    if (n <= 0) return -1;

    int bestIndex = 0;
    long long bestDist = (arr[0] > target) ? (long long)arr[0] - target
                                           : (long long)target - arr[0];

    for (int i = 1; i < n; ++i) {
        long long d = (arr[i] > target) ? (long long)arr[i] - target
                                        : (long long)target - arr[i];
        if (d < bestDist) {   // 严格小于 → 相等时保留靠前的
            bestDist = d;
            bestIndex = i;
        }
    }
    return bestIndex;
}

int errors = 0;
static void find_fork(void)
{
    int index = -1;
    for(int i = 2;i<minus_statics;i++)
    {
        if(minus_white[i-2] == 2 && minus_white[i-1] == 1 && minus_white[i] == 2)
        {
            index = i;break;
        }
    }
    if(index == -1)
    {
        if(minus_white[0] == 0 && minus_white[1] == 1 && minus_white[2] == 2)
        {index = 2;}
        else{
            // 补救措施
            errors = 1;
        }
    }
    int real_y = minus_to_y[index];
    int l[10] = {-1};
    int r[10] = {-1};
    int nl = 0,nr = 0;
    for(int i = 1;i<SCUT_IMAGE_W;i++)
    {
        if(g_binary[real_y][i] == 255 && g_binary[real_y][i-1] == 0)
        {l[nl] = i;nl++;}
        if(g_binary[real_y][i] == 0 && g_binary[real_y][i-1] == 255)
        {r[nr] = i;nr ++;}
    }
    if(nl == 0 && nr == 0)
    {
        // 补救措施
        errors = 1;
    }
    if(nl >= 2 && nr >= 2)
    {
        int mid[10] = {-1};int nmid = 0;
        for(int i = 0;i<mymin(nl,nr)-1;i++)
        {
            mid[nmid] = (r[i]+l[i+1])/2;nmid++;
        }
        int k = findClosestIndex(mid, nmid, SCUT_IMAGE_H/2);
        cx = mid[k];
        cy = real_y;
    }
    if(nl == 1 && nr == 1)
    {
        cx = (l[0]+r[0])/2;
        cy = real_y;
    }
}

static int find_lr(void)
{
    return LR;
}

/* 十字路口：从下往上找最近的、至少两行宽的横向白带。 */
static void geo_cross_edges(void)
{
    int y;
    int start = -1, last = -1;
    int thr = all_white - 10;

    if (thr < SCUT_IMAGE_W / 2) thr = SCUT_IMAGE_W / 2;

    for (y = SCUT_IMAGE_H - 3; y >= 2; y--) {
        if (white_count[y] >= thr) {
            if (start < 0) start = y;   /* 底边，y 大 */
            last = y;                   /* 顶边，y 小 */
        } else if (start >= 0) {
            if (start - last + 1 >= 2) break;
            start = last = -1;
        }
    }

    if (start >= 0 && last >= 0 && start - last + 1 >= 2) {
        bottom = start;
        top = last;
    }
}

int pstate = -1;
int pflag = -1;
int manul_lock = 0;
int auto_lock = 0;

void update_info(void)
{
    if(pstate != state)
    {
        manul_lock = 0;
        auto_lock = 0;
    }
    if(state == 0)
    {fill_none();pstate = 0;pflag = 0;}
    if(state == 1)
    {fill_none();pstate = 1;pflag = 1;}
    if(state == 3)
    {
        geo_cross_edges();
        fill_crossing(bottom, top, 2, 5);
        pstate = 3;
        pflag = 4;
    }
    if(state == 2)
    {
        if(manul_lock == 0)
        {
            elem_lr_manual = LR;
            manul_lock = 1;
        }
        if(flag == 2){
            fill_Y(elem_lr_manual);
            pstate = 2;
            pflag = 2;
        }      
        if(flag == 3)
        {
            fill_none();
            pstate = 2;
            pflag = 3;
        }
    }
    if(state == 4)
    {
        if(flag == 5)
        {
            if(auto_lock == 0)
            {
                elem_lr = find_lr();
                auto_lock = 1;
            }
            find_fork();
            fill_roundaboutI(cx,cy,elem_lr);
            pstate = 4;
            pflag = 5;
        }
        if(flag == 6)
        {fill_roundaboutII(elem_lr);pstate = 4;pflag = 6;}
        if(flag == 7)
        {fill_none();pstate = 4;pflag = 7;}
        if(flag == 8)
        {fill_none();pstate = 4;pflag = 8;}
        if(flag == 9)
        {
            find_fork();
            fill_roundaboutIII(cx, cy, elem_lr);
            pstate = 4;
            pflag = 9;
        }
        if(flag == 10)
        {
            fill_roundaboutIV(elem_lr);pstate = 4;pflag = 10;
        }
        if(flag == 11)
        {
            fill_none();pstate = 4;pflag = 11;
        }
    }
}

// w = h6·u + h7·v + 1
// x = (h0·u + h1·v + h2) / w      ← 车体右方向
// z = (h3·u + h4·v + h5) / w      ← 车头前方向
#define CAL_GROUND_H0 0.0045366634f
#define CAL_GROUND_H1 -0.000223055189f
#define CAL_GROUND_H2 -0.439942106f
#define CAL_GROUND_H3 -0.000210106974f
#define CAL_GROUND_H4 -0.0030013693f
#define CAL_GROUND_H5 0.561728764f
#define CAL_GROUND_H6 0.000546354536f
#define CAL_GROUND_H7 0.0137539169f

float reality_x1,reality_x2,reality_x3,reality_y1,reality_y2,reality_y3;

void pixel2reality(void)
{
    float w1 = CAL_GROUND_H6*X1 + CAL_GROUND_H7*Y1 + 1;
    float w2 = CAL_GROUND_H6*X2 + CAL_GROUND_H7*Y2 + 1;
    float w3 = CAL_GROUND_H6*(lx+rx)/2 + CAL_GROUND_H7*(ly+ry)/2 + 1;
    reality_x1 = (CAL_GROUND_H0 * X1 + CAL_GROUND_H1 * Y1 + CAL_GROUND_H2) / w1;
    reality_x2 = (CAL_GROUND_H0 * X2 + CAL_GROUND_H1 * Y2 + CAL_GROUND_H2) / w2;
    reality_x3 = (CAL_GROUND_H0 * (lx+rx)/2 + CAL_GROUND_H1 * (ly+ry)/2 + CAL_GROUND_H2) / w3;
    reality_y1 = (CAL_GROUND_H3 * X1 + CAL_GROUND_H4 * Y1 + CAL_GROUND_H5) / w1;
    reality_y2 = (CAL_GROUND_H3 * X2 + CAL_GROUND_H4 * Y2 + CAL_GROUND_H5) / w2;
    reality_y3 = (CAL_GROUND_H3 * (lx+rx)/2 + CAL_GROUND_H4 * (ly+ry)/2 + CAL_GROUND_H5) / w3;
}

void reality_reset(void)
{
    reality_x1 = 0.0f; 
    reality_x2 = 0.0f;
    reality_x3 = 0.0f;
    reality_y1 = 0.0f;
    reality_y2 = 0.0f;
    reality_y3 = 0.0f;
}

// int experience_value;//10
// int ww , bw;//2,2
// int max_length;//90
// int offset_w;//5
// int target_dist;//65

void camera_process(void)
{
    int  thre;
    reset_info();
    reality_reset();
    /* 1) 直方图 + 大津法阈值 */


    thre = GetThre(experience_value);
    g_threholds = thre;
    /* 2) 二值化。必须在找起点/巡线之前完成，is_white 读的就是 g_binary。 */
    mybinary(thre);
    mypadding(2);
    count_white(ww,bw);
    limit_white();
    deal_white();
    collate_white(max_length);
    if (find_start(SCUT_IMAGE_H - 3, 10)) {
        forward();
        getmin_statics();          /* 这一句现在缺失，必须补 */
    } else {
        reset_points();
    }
    analyze();
    update_info();
    match_target_line(offset_w);
    pred_points(target_dist);
    pixel2reality();
}

