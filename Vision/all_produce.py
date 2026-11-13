# -*- coding: utf-8 -*-
# ============================================================
#  工创赛·智能搬运  视觉终端 v6（完整版）
#  适配：OpenMV4 H7 Plus / v5.0 固件（csi 模块）
#
#  串口：UART3   P4 = TX  →  STM32 PB11
#                P5 = RX  ←  STM32 PB10
#                115200 8N1，三线共地
#
#  协议（与主控 Core/Src/vision.c 对齐）：
#    主控 → 视觉
#      $MODE,<0-3>       0=空闲 1=二维码 2=颜色 3=圆环
#      $TARGET,<0-6>     只找指定颜色，0=任何颜色（可选，默认 0）
#      $PING             保活
#    视觉 → 主控
#      $QR,<任务码>                       例：$QR,156+123+516+231
#      $TT,<颜色>,<稳定>,<dx>,<dy>        颜色 1~6，dx/dy 单位 mm
#      $RING,<found>,<dx>,<dy>,<dth>,<num>  环心偏差 mm + 环号 1~6
#      $ALIGN,<OK|LEFT|RIGHT|UP|DOWN>     微调提示（主控可选解析）
#      $ACK,<mode>                        命令应答
#      $HB,<mode>,<fps>                   心跳
#
#  用法：
#    1. 上电自动运行本脚本
#    2. 主控发 $MODE,2 → 本脚本持续回 $TT
#    3. 主控发 $MODE,3 → 本脚本持续回 $RING
#    4. 主控发 $MODE,1 → 本脚本持续回 $QR
# ============================================================

import csi
import time
import image
from pyb import UART
from image import SEARCH_EX

# ============================================================
# 1. 配置区（现场只改这里）
# ============================================================
MOUNT = "DOWN"     # "DOWN"=朝下找物料/圆环   "GIMBAL"=前视找二维码
UART_ID = 3
BAUD = 115200
ENABLE_QR = True

# ---- 像素→毫米标定（现场用 100mm 标尺实测后改）----
CAL = {
    "DOWN":   {"kx": 0.50, "ky": 0.50, "ox": 0, "oy": 0},
    "GIMBAL": {"kx": 0.35, "ky": 0.35, "ox": 0, "oy": 0},
}

# ---- 夹爪投影点在画面里的像素坐标（标定后填）----
GRIPPER_PX = {"DOWN": (160, 120), "GIMBAL": (160, 120)}

# ---- 物料颜色阈值（LAB），现场必须重新调 ----
thresholds = [
    (32, 57, 24, 54, -3, 28),      # 1 红
    (75, 98, -16, 6, 22, 49),      # 2 黄
    (20, 45, 5, 37, -67, -38),     # 3 蓝
    (49, 75, -50, -24, 10, 37),    # 4 绿
    (0, 19, -15, 26, -8, 11),      # 5 黑
    (65, 90, -17, 10, -32, -14),   # 6 浅蓝
]
COLOR_NAME = ["?", "红", "黄", "蓝", "绿", "黑", "浅蓝"]

# ---- 数字模板（圆环里的数字 1~6）----
# 注意：模板文件要放在 SD 卡根目录，命名必须和这里一致
templates = [
    "1_small.pgm", "1_A_small.pgm",
    "2_small.pgm", "2_A_small.pgm",
    "3_small.pgm", "3_A_small.pgm",
]
TPL_THRESHOLD = 0.55   # 模板匹配阈值：认得松就调小，认得乱就调大

# ---- 算法参数 ----
LOST_FRAMES = 5        # 连续 N 帧没目标才报 found=0（防抖）
WINDOW = 5        # 滑动平均窗口长度
STABLE_NEED = 8        # 连续 N 帧位置不跳才判稳定
MIN_AREA = 100      # 色块最小面积（像素）
CENTER_BAND = 160      # 只认画面中心 ±160 像素内的色块
DIST_CORR = None     # 畸变系数，如 1.8；None = 不做校正（省时间）

# ---- 微调提示参数 ----
ALIGN_DEAD = 20        # 偏差小于 20 像素算"对准了"
ALIGN_MS = 200       # 同一方向最多每 200ms 发一次

# ============================================================
# 2. 工具函数
# ============================================================


def _v(obj, name):
    """兼容"属性"和"方法"两种写法（不同固件不一样）"""
    a = getattr(obj, name)
    return a() if callable(a) else a


_frame_no = 0


def now_ms():
    """毫秒时间戳；固件没有 ticks_ms 时用帧号粗略兜底"""
    try:
        return time.ticks_ms()
    except Exception:
        return _frame_no * 66


def ms_diff(a, b):
    """时间差，兼容有/没有 ticks_diff 的固件"""
    try:
        return time.ticks_diff(a, b)
    except Exception:
        return a - b


# ============================================================
# 3. 一维卡尔曼滤波（位置 + 速度）
#    状态向量 = [位置, 速度]
# ============================================================
class KF1D:
    def __init__(self, dt=1.0 / 15.0, q=0.05, r=6.0):
        self.dt = dt
        self.q = q          # 过程噪声：模型多不可信
        self.r = r          # 观测噪声：测量多不准
        self.x = 0.0
        self.v = 0.0
        self.p00, self.p01 = 1.0, 0.0
        self.p10, self.p11 = 0.0, 1.0

    def reset(self, z=0.0):
        self.x, self.v = float(z), 0.0
        self.p00, self.p01 = 1.0, 0.0
        self.p10, self.p11 = 0.0, 1.0

    def update(self, z):
        dt = self.dt

        # ---------- 预测 ----------
        x = self.x + self.v * dt
        v = self.v
        p00 = self.p00 + dt * (self.p01 + self.p10) + dt * dt * self.p11 + self.q
        p01 = self.p01 + dt * self.p11
        p10 = self.p10 + dt * self.p11
        p11 = self.p11 + self.q

        # ---------- 更新 ----------
        s = p00 + self.r
        k0 = p00 / s
        k1 = p10 / s
        y = z - x

        self.x = x + k0 * y
        self.v = v + k1 * y

        self.p00 = p00 - k0 * p00
        self.p01 = p01 - k0 * p01
        self.p10 = p10 - k1 * p00
        self.p11 = p11 - k1 * p01

        return self.x, self.v


kf_x = KF1D()
kf_y = KF1D()

# ============================================================
# 4. 摄像头初始化
# ============================================================
cam = csi.CSI()
cam.reset()
cam.pixformat(csi.RGB565)
cam.framesize(csi.QVGA)          # 320x240

cam.vflip(True)                  # 画面倒着就打开
cam.hmirror(True)                # 画面镜像就打开

# 预热（v5 固件 snapshot 不接受 time 参数，用循环代替）
for _ in range(30):
    cam.snapshot()

# 自动曝光/白平衡跑一会儿，然后锁死，保证颜色阈值稳定
cam.auto_gain(True)
cam.auto_whitebal(True)
for _ in range(20):
    cam.snapshot()

try:
    gains = cam.get_rgb_gain_db()
    cam.auto_whitebal(False, rgb_gain_db=gains)
except Exception:
    cam.auto_whitebal(False)

try:
    cam.auto_gain(False)
except Exception:
    pass

uart = UART(UART_ID, BAUD)

try:
    clock = time.clock()
except Exception:
    clock = None

# ============================================================
# 5. 模板预加载（绝不能在循环里 image.Image）
# ============================================================
tpl_loaded = []
for t in templates:
    try:
        n = int("".join(c for c in t if c.isdigit()))
        tpl_loaded.append((n, image.Image(t)))
    except Exception:
        pass          # 文件不存在就跳过，不影响其他功能

# ============================================================
# 6. 通信层
# ============================================================


def send(msg):
    try:
        uart.write(msg + "\n")
    except Exception:
        pass


rxbuf = bytearray()

current_mode = 0     # 0空闲 1二维码 2颜色 3圆环
target_color = 0     # 0=任何颜色，1~6=只找这个颜色

xs, ys = [], []
stable_count = 0
last_ax, last_ay = -1, -1
lost_cnt = 0

last_align_ms = 0
last_align_dir = ""


def handle_cmd(line):
    global current_mode, target_color
    global xs, ys, stable_count, last_ax, last_ay, lost_cnt

    if not line or line[0] != "$":
        return

    body = line.split("*")[0]          # 兼容带校验的写法
    p = body.split(",")
    cmd = p[0].upper()

    if cmd == "$MODE" and len(p) >= 2:
        try:
            m = int(p[1])
        except Exception:
            return
        if 0 <= m <= 3 and m != current_mode:
            current_mode = m
            xs, ys = [], []
            stable_count = 0
            last_ax, last_ay = -1, -1
            lost_cnt = 0
            kf_x.reset()
            kf_y.reset()
            send("$ACK,%d" % m)
            print("mode -> %d" % m)

    elif cmd == "$TARGET" and len(p) >= 2:
        try:
            target_color = int(p[1])
        except Exception:
            return
        print("target -> %d" % target_color)

    elif cmd == "$PING":
        send("$HB,%d,0" % current_mode)


def poll_cmd():
    global rxbuf
    try:
        n = uart.any()
        if n:
            rxbuf.extend(uart.read(n))
    except Exception:
        rxbuf = bytearray()
        return

    if len(rxbuf) > 256:               # 防内存无限增长
        rxbuf = rxbuf[-128:]

    while True:
        i = rxbuf.find(b"\n")
        if i < 0:
            break
        line = bytes(rxbuf[:i]).decode("utf-8", "ignore").strip()
        rxbuf = rxbuf[i + 1:]
        try:
            handle_cmd(line)
        except Exception:
            pass

# ============================================================
# 7. 坐标换算 + 微调提示
# ============================================================


def to_mm(px, py):
    c = CAL[MOUNT]
    g = GRIPPER_PX[MOUNT]
    return (int((px - g[0]) * c["kx"] + c["ox"]),
            int((py - g[1]) * c["ky"] + c["oy"]))


def send_align(img, cx, cy):
    """判断目标偏离画面中心的方向，节流后发给主控"""
    global last_align_ms, last_align_dir

    now = now_ms()
    if ms_diff(now, last_align_ms) < ALIGN_MS:
        return

    mx = img.width() // 2          # 用实际分辨率，不写死 160/120
    my = img.height() // 2
    dx = cx - mx
    dy = cy - my

    if abs(dx) <= ALIGN_DEAD and abs(dy) <= ALIGN_DEAD:
        d = "OK"
    elif abs(dx) >= abs(dy):
        d = "RIGHT" if dx > 0 else "LEFT"
    else:
        d = "DOWN" if dy > 0 else "UP"

    if d == last_align_dir and d != "OK":
        return                     # 方向没变就少发

    last_align_dir = d
    last_align_ms = now
    send("$ALIGN,%s" % d)

# ============================================================
# 8. 模式 1：二维码
# ============================================================


def task_qr(img):
    if not ENABLE_QR:
        return

    codes = img.find_qrcodes()
    if not codes:
        send("$QR,")
        return

    q = max(codes, key=lambda c: _v(c, "w") * _v(c, "h"))

    payload = _v(q, "payload")
    if isinstance(payload, (bytes, bytearray)):
        payload = payload.decode("utf-8", "ignore")
    payload = (payload or "").strip()

    send("$QR,%s" % payload)
    print("QR:%s" % payload)

    cx = _v(q, "x") + _v(q, "w") // 2
    cy = _v(q, "y") + _v(q, "h") // 2

    send_align(img, cx, cy)
    img.draw_rectangle(_v(q, "rect"), color=(0, 255, 0))
    img.draw_cross((cx, cy), color=(255, 0, 0), size=6)

# ============================================================
# 9. 模式 2：物料颜色（滑动窗口 + 稳定判定 + 防抖）
# ============================================================


def task_color(img):
    global xs, ys, stable_count, last_ax, last_ay, lost_cnt

    # 指定了目标色 → 只送那一个阈值，merge=True 合并碎片
    # 没指定 → 送全部 6 组，merge=False（否则不同颜色会被粘成一坨）
    if 1 <= target_color <= 6:
        ths = [thresholds[target_color - 1]]
        merge = True
    else:
        ths = thresholds
        merge = False

    blobs = img.find_blobs(ths, pixels_threshold=MIN_AREA,
                           area_threshold=MIN_AREA, merge=merge)

    # ---- 只保留画面中心区域的色块 ----
    valid = []
    if blobs:
        mx = img.width() // 2
        my = img.height() // 2
        for b in blobs:
            if abs(_v(b, "cx") - mx) < CENTER_BAND and \
               abs(_v(b, "cy") - my) < CENTER_BAND:
                valid.append(b)

    if not valid:
        lost_cnt += 1
        if lost_cnt >= LOST_FRAMES:
            send("$TT,0,0,0,0")
            xs, ys = [], []
            stable_count = 0
            last_ax, last_ay = -1, -1
        return

    lost_cnt = 0

    t = max(valid, key=lambda b: _v(b, "area"))

    # ---- 判断颜色（位掩码，不能直接比 code）----
    if 1 <= target_color <= 6:
        color = target_color
    else:
        code = _v(t, "code")
        color = 0
        for i in range(len(thresholds)):
            if code & (1 << i):
                color = i + 1
                break
    if color == 0:
        return

    cx, cy = _v(t, "cx"), _v(t, "cy")

    # ---- 滑动窗口平滑 ----
    xs.append(cx)
    ys.append(cy)
    if len(xs) > WINDOW:
        xs.pop(0)
        ys.pop(0)
    ax = sum(xs) // len(xs)
    ay = sum(ys) // len(ys)

    # ---- 稳定判定 ----
    if last_ax >= 0 and abs(ax - last_ax) < 10 and abs(ay - last_ay) < 10:
        stable_count += 1
    else:
        stable_count = 0
    last_ax, last_ay = ax, ay

    stable = 1 if stable_count >= STABLE_NEED else 0

    dx, dy = to_mm(ax, ay)
    send("$TT,%d,%d,%d,%d" % (color, stable, dx, dy))

    if stable:
        print("TT color=%d(%s) dx=%d dy=%d" % (color, COLOR_NAME[color], dx, dy))

    send_align(img, ax, ay)
    img.draw_rectangle(_v(t, "rect"), color=(0, 255, 0))
    img.draw_cross((ax, ay), color=(255, 0, 0), size=6)

# ============================================================
# 10. 模式 3：圆环 + 数字（卡尔曼 + 模板匹配）
# ============================================================


def task_ring(img):
    global lost_cnt

    # 找圆在灰度图上做（省一半时间），画图仍画在彩色原图上
    gray = img.to_grayscale()

    circles = gray.find_circles(threshold=2500,
                                x_margin=10, y_margin=10, r_margin=10,
                                r_min=15, r_max=60, r_step=2)

    # ---- 只保留画面中心区域的圆 ----
    valid = []
    if circles:
        mx = gray.width() // 2
        my = gray.height() // 2
        for c in circles:
            if abs(_v(c, "x") - mx) < 60 and abs(_v(c, "y") - my) < 60:
                valid.append(c)

    if not valid:
        kf_x.reset()
        kf_y.reset()
        lost_cnt += 1
        if lost_cnt >= LOST_FRAMES:
            send("$RING,0,0,0,0,0")
        return

    lost_cnt = 0

    c = max(valid, key=lambda o: _v(o, "r"))
    raw_x, raw_y, r = _v(c, "x"), _v(c, "y"), _v(c, "r")

    fx, _ = kf_x.update(raw_x)
    fy, _ = kf_y.update(raw_y)
    fx, fy = int(fx), int(fy)

    # ---- 在圆环内部找数字（模板已预加载，不重复读卡）----
    half = max(24, r)                 # ROI 半边长，至少比模板大
    rx = max(0, fx - half)
    ry = max(0, fy - half)
    rw = min(half * 2, gray.width() - rx)
    rh = min(half * 2, gray.height() - ry)

    num = 0
    if rw > 20 and rh > 20:
        roi = (rx, ry, rw, rh)
        for n, tpl in tpl_loaded:
            try:
                hit = gray.find_template(tpl, TPL_THRESHOLD, roi=roi,
                                         step=4, search=SEARCH_EX)
            except Exception:
                hit = None
            if hit:
                num = n
                break

    dx, dy = to_mm(fx, fy)

    # 第 5 个字段是角度 dth（暂时填 0），第 6 个才是环号
    send("$RING,1,%d,%d,0,%d" % (dx, dy, num))
    print("RING dx=%d dy=%d r=%d num=%d" % (dx, dy, r, num))

    send_align(img, fx, fy)

    img.draw_circle((fx, fy, r), color=(255, 0, 0))
    img.draw_cross((fx, fy), color=(255, 0, 0), size=6)
    if num:
        img.draw_string((rx, max(0, ry - 12)), str(num), color=(255, 0, 0))

# ============================================================
# 11. 主循环
# ============================================================


print("vision v6 start, mount=%s, uart=%d@%d" % (MOUNT, UART_ID, BAUD))

while True:
    try:
        poll_cmd()

        img = cam.snapshot()

        # 去畸变：很吃性能，按需打开
        if DIST_CORR:
            img = img.lens_corr(DIST_CORR)

        if current_mode == 1:
            task_qr(img)
        elif current_mode == 2:
            task_color(img)
        elif current_mode == 3:
            task_ring(img)

        _frame_no += 1

        # 每 20 帧发一次心跳
        if _frame_no % 20 == 0:
            fps = 0
            try:
                fps = int(clock.fps())
            except Exception:
                pass
            send("$HB,%d,%d" % (current_mode, fps))

    except Exception as e:
        try:
            print("ERR", e)
        except Exception:
            pass
        time.sleep(0.05)
