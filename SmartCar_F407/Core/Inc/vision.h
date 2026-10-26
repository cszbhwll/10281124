/**
 * vision.h —— 和视觉模块（OpenMV / K210 / MaixCam 等）的串口通信
 *
 * 摄像头型号还没定，所以这里只规定"通信协议"，相机端按这个协议发数据即可。
 * 串口 USART3（PB10 TX→相机RX，PB11 RX←相机TX），115200，8N1，一行一条消息，以 '\n' 结尾。
 *
 * 单片机 → 相机：
 *   $MODE,<n>\n          切换识别模式：0空闲 1二维码 2转盘颜色 3放置区圆环 4障碍物
 * 相机 → 单片机：
 *   $QR,156+123+516+231\n                    识别到的任务码
 *   $TT,<颜色1~6或0>,<稳定0/1>,<dx>,<dy>\n   转盘取料点上的物料颜色；dx/dy 为物料中心相对
 *                                            "理想取料点"的偏差（车体坐标 mm，前为+x，左为+y）
 *   $RING,<找到0/1>,<dx>,<dy>[,<dth>]\n       当前对准的圆环中心相对理想放置点的偏差（mm）；
 *                                            可选 dth：由区域边线算出的车头朝向误差（度，逆时针为正），
 *                                            有了它程序会顺便校正朝向
 *   $OBS,<距离mm或0>\n                        前方车道上的障碍物距离（0=没有）
 * 可选校验：行尾可加 *XX（XX 为 $ 和 * 之间所有字符异或值的十六进制），有就校验，没有就不校验。
 */
#ifndef VISION_H
#define VISION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { VMODE_IDLE = 0, VMODE_QR = 1, VMODE_TURNTABLE = 2, VMODE_RING = 3, VMODE_OBSTACLE = 4 } vision_mode_t;

typedef struct {
    int      color;        /* 0 = 没看到物料 */
    bool     stable;       /* 转盘已停稳 */
    float    dx, dy;       /* mm */
    uint32_t stamp;        /* 收到这条数据的时间 */
} vision_tt_t;

typedef struct { bool found; float dx, dy; bool has_dth; float dth_deg; uint32_t stamp; } vision_ring_t;

void vision_init(void);
void vision_poll(void);                         /* 由 sys_poll 调用，解析收到的数据 */
void vision_set_mode(vision_mode_t m);

bool vision_get_qr(char *out, int maxlen, uint32_t max_age_ms);
bool vision_get_tt(vision_tt_t *out, uint32_t max_age_ms);
bool vision_get_ring(vision_ring_t *out, uint32_t max_age_ms);
int  vision_obstacle_mm(uint32_t max_age_ms);   /* 0 = 没有或数据过期 */
void vision_clear(void);
uint32_t vision_last_rx_ms(void);

#endif
