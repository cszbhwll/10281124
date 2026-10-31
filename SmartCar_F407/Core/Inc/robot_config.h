/**
 * robot_config.h —— 小车所有"需要实测/标定"的参数集中在这里
 *
 * 规则：凡是标了【标定】的数，上车后都要按 README 第 4 节的方法实测后改掉。
 * 单位约定：长度 mm，时间 ms，角度在代码里用弧度（这里写度数的会注明 _DEG）。
 *
 * 坐标约定（非常重要，全程统一）：
 *   车体坐标系：原点在车体几何中心，X 轴指向车头（摄像头那一侧），Y 轴指向车的左侧，Z 轴向上，地面 Z=0。
 *   场地坐标系：原点在场地左下角，X 向右，Y 向上（按规则图1摆放），车头朝向角 θ 从 +X 逆时针为正。
 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* ======================= 1. 底盘 ======================= */
#define WHEEL_DIAMETER_MM        80.0f     /* 麦轮直径（80mm 麦轮） */
#define ENC_COUNTS_PER_WHEEL_REV 1496.0f   /* 【标定】轮子转一圈驱动板计数（暂按 11线×4倍频×减速比34 估算） */
#define CHASSIS_HALF_TRACK_MM    133.0f    /* 【标定】左右两轮中心距离的一半（a） */
#define CHASSIS_HALF_BASE_MM     78.0f     /* 【标定】前后两轮中心距离的一半（b） */
#define ROT_SCALE                1.00f     /* 【标定】原地转 10 圈的角度修正系数，见 README */
#define FWD_SCALE                1.00f     /* 【标定】前进 1000mm 实际走了多少 / 1000 */
#define STRAFE_SCALE             1.00f     /* 【标定】横移 1000mm 实际走了多少 / 1000（麦轮横移打滑更多，常见 0.85~0.95） */

/* 车体外形的一半（用于判断有没有压到禁行区；出发包络动画实测约 233×296） */
#define ROBOT_HALF_LEN_MM        117.0f    /* 车头-车尾方向（X）的一半 */
#define ROBOT_HALF_WID_MM        148.0f    /* 左右方向（Y，含麦轮）的一半 */

/* 幻尔四路编码电机驱动板 */
#define MOTOR_DRV_I2C_ADDR       0x34      /* 7 位地址 */
#define MOTOR_DRV_TYPE           3         /* 【确认】电机类型寄存器值：0无编码器 1TT 2N20 3JGB；25GA370 霍尔编码器先按 3 试 */
#define MOTOR_DRV_ENC_POLARITY   0         /* 【标定】编码器极性 0/1：电机越转越快停不下来时改成另一个值 */
#define MOTOR_SPEED_LIMIT        45        /* 速度寄存器单位是"脉冲/10ms"，限制最大绝对值 */

/*
 * 驱动板通道 M1..M4 分别接哪个轮子、正转方向是否相反——【标定】
 * 轮子编号：0=左前 FL，1=右前 FR，2=左后 RL，3=右后 RR
 * WHEEL_CH[i]  : 第 i 个轮子接在驱动板哪个通道（0..3 对应 M1..M4）
 * WHEEL_SIGN[i]: 让该轮"向前滚"所需的速度符号（+1 或 -1）
 * 用串口命令 `wheel` 逐个转动来确认，详见 README。
 */
#define WHEEL_CH_INIT    { 0, 1, 2, 3 }
#define WHEEL_SIGN_INIT  { +1, -1, +1, -1 }

/* 陀螺仪 MPU6050（GY-521 模块，见 imu_mpu6050.c）。默认开启；没接模块时会自动退回只用轮子 */
#ifndef USE_MPU6050
#define USE_MPU6050              1         /* 1 = 启用陀螺仪（GY-521/MPU6050）；没接时自动退回只用轮子 */
#endif
#define IMU_YAW_SIGN             (+1)      /* 【标定】芯片朝上安装 = +1；芯片朝下（倒装）= -1。用 imu 命令检查 */
#define IMU_SCALE                1.000f    /* 【标定】陀螺仪比例：spin 2 后 IMU_SCALE = 旧值 × 实际角度 ÷ 720 */

/* 舵机单独测试模式：1 = 上电后机械臂不动（不上力、不收折叠姿态），只等串口命令，
 * 用来第一次接舵机时小幅度试转（sscan / snudge 命令，见 README 5.5）。比赛前一定改回 0。 */
#ifndef SERVO_TEST_MODE
#define SERVO_TEST_MODE          0
#endif

/* ======================= 2. 导航（走位） ======================= */
#define CTRL_PERIOD_MS           20        /* 底盘控制周期：每 20ms 读一次编码器、发一次速度 */
#define NAV_VMAX_MMPS            534.0f    /* 直线最大速度（无载理论约 534，留余量） */
#define NAV_ACC_MMPS2            700.0f    /* 加速度上限 */
#define NAV_WMAX_DPS             150.0f    /* 最大角速度（度/秒） */
#define NAV_WACC_DPS2            360.0f    /* 角加速度上限 */
#define NAV_POS_TOL_MM           6.0f      /* 到点位置容差 */
#define NAV_ANG_TOL_DEG          1.5f      /* 到点角度容差 */
#define NAV_PASS_TOL_MM          40.0f     /* 途经点（不停车）容差 */
#define NAV_LANE_MARGIN_MM       30.0f     /* 规划路线时车身四周额外留的安全距离 */
#define NAV_TIMEOUT_MS           8000      /* 单段走位超时 */
#define NAV_KP_POS               3.0f      /* 位置比例系数：误差 1mm → 3mm/s */
#define NAV_KP_ANG               3.5f      /* 角度比例系数 */

/* ======================= 3. 机械臂（3×ST3215 + 1×SC09） ======================= */
/* 舵机 ID（出厂多为 1，需要先用 `sid` 命令逐个改 ID，一次只接一个舵机） */
#define SERVO_ID_YAW             1         /* 底座回转  ST3215 */
#define SERVO_ID_SHOULDER        2         /* 肩        ST3215 */
#define SERVO_ID_ELBOW           3         /* 肘        ST3215 */
/* 夹爪 SC09 接在哪条总线上：
 *   1 = 和 3 个 ST3215 同一条总线（现在的接法）：舵机驱动板接口 2 → 6V 降压模块 → SC09。
 *       同一条总线上 ID 不能重复，所以夹爪用 ID 4（出厂是 1，要先用 sid sc 1 4 改）
 *   0 = 单独一块舵机驱动板（6V 供电），接 USART6 PC6/PC7，ID 可以也是 1 */
#define GRIP_ON_ST_BUS           1
#if GRIP_ON_ST_BUS
#define SERVO_ID_GRIP            4         /* 夹爪      SC09 */
#define PORT_UART_GRIP           PORT_UART_SERVO_ST
#else
#define SERVO_ID_GRIP            1         /* 夹爪      SC09（单独一条总线） */
#define PORT_UART_GRIP           PORT_UART_SERVO_SC
#endif

/* 机械臂几何（来自 10月7日动画模型，换算到车体坐标系；上车后用尺子复核） */
#define ARM_BASE_X_MM            45.0f     /* 回转轴在车体坐标中的位置：偏车头 */
#define ARM_BASE_Y_MM           -45.0f     /* 偏车的右侧 */
#define ARM_SHOULDER_R0_MM      -10.4f     /* 肩轴离回转轴的水平距离：负号 = 肩轴在回转轴"后面"（与小臂伸出方向相反），按 CAD 核对 */
#define ARM_TOOL_SIDE_MM         16.0f     /* 【标定】夹持中心偏离大臂/小臂摆动平面的距离（朝小臂伸出方向的左侧为正），按 CAD 为 16mm；
                                              如果在肩、肘加了隔套，要按实物重新量 */
#define ARM_SHOULDER_Z_MM        164.7f    /* 肩轴离地高度 */
#define ARM_L1_MM                90.0f     /* 大臂：肩轴→肘轴 */
#define ARM_L2_MM                165.9f    /* 小臂+夹爪：肘轴→夹持中心 */

/*
 * 关节角 → 舵机原始位置 的换算：raw = ZERO + DIR * 角度(弧度) * 计数每弧度
 * ST3215: 4096 计数 / 360°；SC09: 1024 计数 / 300°
 * ZERO 的含义：
 *   回转 YAW=0      → 小臂水平指向车头正前方
 *   肩   SHOULDER=0 → 大臂水平向前
 *   肘   ELBOW=0    → 小臂与大臂成一条直线
 * 【标定】三个 ZERO 和 DIR 必须用 `steach` 命令实测（README 第 4.5 节）
 */
#define ST_COUNTS_PER_RAD        651.8986f
/* 2026-10-24 实测（ID 1 回转、ID 2 肩做过中位校准 scal，校准后读数如下）：
 *   标准姿势（整条手臂水平拉直、指向车头）：回转 3819  肩 1588  肘 211
 *   收起上电姿势（出发姿态）：              回转 2178  肩 2098  肘 1331
 *     → 换算成角度：回转 +144°（指向左后方）、大臂抬起 45°、小臂相对大臂往下折 98°
 *   转动方向（看着舵机动确认过）：回转读数变大 = 从上往下看顺时针；肩读数变大 = 抬起；肘读数变大 = 小臂往下伸开
 * 能表示的角度范围（读数 0~4095 内）：回转 -24°~+335°，肩 -139°~+220°，肘 -341°~+18°；
 * 仿真整场实际用到：回转 -10°~173°，肩 16°~138°，肘 -145°~-69°，都在范围内。
 * 标准姿势是手摆的，可能差几度，装好夹爪后用 arm 命令实测位置再微调这三个数。 */
#define YAW_ZERO_RAW             3819
#define YAW_DIR                  (-1)
#define SHOULDER_ZERO_RAW        1588
#define SHOULDER_DIR             (+1)
#define ELBOW_ZERO_RAW           211
#define ELBOW_DIR                (-1)
/*
 * 关节软限位（角度，度）——【标定】按实物能转到、且不撞车体的范围填写
 * 注意：小臂(166)比大臂(90)长很多，够车上近处的载物槽时，大臂要向后仰过竖直(仿真整场最大约140°)，
 *       小臂向前下方折回来(最小约-145°)。装配时要确认肩、肘能转到这么大的角度而不干涉。
 */
#define YAW_MIN_DEG             (-20.0f)   /* 读数 4095 在 -24°，留 4° 余量 */
#define YAW_MAX_DEG              260.0f
#define SHOULDER_MIN_DEG        (-20.0f)
#define SHOULDER_MAX_DEG         170.0f
#define ELBOW_MIN_DEG           (-170.0f)
#define ELBOW_MAX_DEG            12.0f     /* 读数 0 在 +18°，留 6° 余量 */

/* 夹爪 SC09 原始位置——【标定】 */
#define GRIP_RAW_OPEN            700       /* 张开到能套住 Ø30 顶部 */
#define GRIP_RAW_CLOSE           420       /* 夹紧（命令值，会被物料挡住） */
#define GRIP_RAW_EMPTY_CLOSED    400       /* 空夹时实际能走到的位置：读回接近它说明没夹到 */
#define GRIP_EMPTY_THRESHOLD     25        /* 读回值与空夹位置相差小于它 → 判定没夹到 */

/* 舵机运动速度（计数/秒）与加速度 */
#define ARM_SPEED_RAW_PER_S      2800      /* 约 246°/s；ST3215 12V 空载最快约 3070（0.222s/60°） */
#define ARM_ACC                  200       /* 加速度，单位 100步/s²（最大 254）：200 → 约 0.14s 加到最快 */
#define GRIP_TIME_MS             350       /* 夹爪开合等待时间 */

/* 安全高度：机械臂在车上方转动时，夹持中心不低于这个高度，避免碰屏幕和载物槽 */
#define ARM_SAFE_Z_MM            235.0f
#define ARM_APPROACH_DZ_MM       35.0f     /* 取放前先停在目标上方这么高 */

/* 折叠出发姿态（原始计数）——【标定】必须保证出发时整车 ≤300×300×400 */
/* 2026-10-24 实测：你们收起上电时的姿势（回转 +144°、肩 +45°、肘 -98°） */
#define FOLD_YAW_RAW             2178
#define FOLD_SHOULDER_RAW        2098
#define FOLD_ELBOW_RAW           1331

/* ======================= 4. 物料与高度 ======================= */
#define MAT_HEIGHT_MM            60.0f     /* 初赛物料总高 */
#define MAT_GRIP_FROM_BASE_MM    56.0f     /* 夹持点：顶部 Ø30 直段中心，离物料底面 56mm */
#define CAR_TRAY_FLOOR_Z_MM      92.0f     /* 车上载物槽底面离地高度 */
#define TURNTABLE_TOP_Z_MM       90.0f     /* 【现场】原料转盘顶面高度（规则 80~100） */
#define FLOOR_Z_MM               0.0f

/* 三个车载槽中心（车体坐标，来自装配模型：槽1右后、槽2左前、槽3左后） */
#define SLOT0_X_MM              -50.0f
#define SLOT0_Y_MM              -50.0f
#define SLOT1_X_MM               45.0f
#define SLOT1_Y_MM               50.0f
#define SLOT2_X_MM              -50.0f
#define SLOT2_Y_MM               50.0f

/* ======================= 5. 比赛时间管理 ======================= */
#define MATCH_TIME_LIMIT_MS      180000    /* 每轮运行 3 分钟 */
#define MATCH_RETURN_RESERVE_MS  6000      /* 剩余时间少于"预估返程时间+这个余量"就放弃剩下任务、直接回启停区 */
#define TURNTABLE_WAIT_MAX_MS    20000     /* 转盘等某个颜色最多等多久（规则：停滞 15s+转盘 8s） */
#define QR_TIMEOUT_MS            6000      /* 读二维码超时 */

/* ======================= 6. 避障：VL53L0X 激光测距（940nm 红外）×8，经 TCA9548A 多路开关接 I2C ======================= */
/*
 * 规则：车道上有黑色障碍物 φ50×100mm，数量和位置随机抽签放置。
 * 麦轮车走路线时车头朝向不变、经常横着/倒着走，所以四面都要能看：每面 2 个，共 8 个。
 * 8 个 VL53L0X 地址都是 0x29，各接 TCA9548A 的一个通道（第 i 个传感器接 SD[i]/SC[i]），
 * TCA9548A 接在电机驱动板那条 I2C 上（PB8/PB9），不另占单片机引脚。
 */
#ifndef USE_TOF
#define USE_TOF                  1         /* 1 = 启用测距避障；没接传感器时自动退回“不避障” */
#endif
#define TOF_MUX_ADDR             0x70      /* TCA9548A 地址：A0/A1/A2 都接 GND */
#define TOF_MOUNT_Z_MM           50.0f     /* 【安装】传感器离地高度：障碍物高 100，装在 40~70 之间，水平朝外 */
#define TOF_FOV_DEG              25.0f     /* VL53L0X 视场角 */
#define TOF_MAX_RANGE_MM         600       /* 【标定】只信这个距离以内的读数（黑色障碍物要实测能看多远） */
/*
 * 每个传感器的安装位置和朝向（车体坐标：x 向前，y 向左；朝向 0°=车头方向，90°=左，-90°=右，180°=车尾）
 * 【安装】按实际装的位置量好后改这里；侧面的要避开轮子（轮子在 x=±48~108 之间）。
 * 第 i 项接多路开关第 i 通道。
 */
#ifndef TOF_LAYOUT_INIT
#define TOF_COUNT                8
#define TOF_LAYOUT_INIT  { { 117.0f,  70.0f,    0.0f }, { 117.0f, -70.0f,    0.0f },   /* 车头 左、右 */ \
                           {-117.0f,  70.0f,  180.0f }, {-117.0f, -70.0f,  180.0f },   /* 车尾 左、右 */ \
                           {  30.0f, 125.0f,   90.0f }, { -30.0f, 125.0f,   90.0f },   /* 左侧 前、后 */ \
                           {  30.0f,-125.0f,  -90.0f }, { -30.0f,-125.0f,  -90.0f } }  /* 右侧 前、后 */
#endif

#define OBS_RADIUS_MM            25.0f     /* 障碍物半径（φ50） */
#define OBS_CLEAR_MM             35.0f     /* 规划路线时车身离障碍物边缘至少留多远（另加 NAV_LANE_MARGIN_MM） */
#define OBS_MERGE_MM             70.0f     /* 两次看到的位置相距小于这个，算同一个障碍物 */
#define OBS_CONFIRM              2         /* 同一位置连续看到几次才确认（防止偶尔一次误读） */

#endif
