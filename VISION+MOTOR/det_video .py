"""
K230 钢球检测 + LCD 显示 + 低延迟 MJPEG 图传。

数据链路：
  Sensor CHN2 (RGB888P) -> 320x320 AnchorBaseDet -> LCD OSD
  Sensor CHN0 (YUV420SP) -> LCD video layer
  LCD 最终合成画面 -> WBC -> JPEG 硬编码 -> HTTP-MJPEG

浏览器或 VLC 打开：http://<开发板IP>:8080/
原 H.264/RTSP 脚本保持独立，不受本文件影响。
"""

import aicube
import image
import nncase_runtime as nn
import ulab.numpy as np

from media.sensor import *
from media.display import Display
from media.media import *
from media.vencoder import Encoder, ChnAttrStr, StreamData
from _media import Display as RawDisplay
from mpp import *
from machine import UART, FPIOA

try:
    from machine import TOUCH
except ImportError:
    TOUCH = None

try:
    import _thread
except ImportError:
    _thread = None
import gc
import math
import network
import os
import socket
import sys
import time
import uctypes
import ujson


# ======================== 配置区 ========================

WIFI_SSID = "XXX"
WIFI_PASSWORD = "XXXXXX"

# 使用第二个文件原有的模型和部署配置。
ROOT_PATH = "/sdcard/mp_deployment_source/"
CONFIG_PATH = ROOT_PATH + "deploy_config.json"
EXPECTED_MODEL_INPUT_SIZE = [320, 320]
DEBUG_MODE = 0

# CHN2 保持参考程序的 4:3 画面，再由 AI2D letterbox 到 320x320。
RGB888P_SIZE = [640, 480]

DISPLAY_MODE = "lcd"
DISPLAY_SIZE = [800, 480]
# 水管无法与画面几何中心重合时，在这里整体移动逻辑中心和三条水平线。
# 负数向上、正数向下；根据用户截图拟合黑线，球心比原绿线高约 5 px，
# 因此由 y=264 上移到 y=259。
CENTER_Y_OFFSET_PX = 19
# 物理中心实测时下位机稳定显示 +0.800 mm。按 160 px = 5 cm 换算，
# 将 X 零点向右移动 2.56 px，使该物理位置发送为约 0.000 mm。
# 正数向右、负数向左；装置重新固定后只需调整这一项。
CENTER_X_OFFSET_PX = 12
# OSD/模型检测框均使用显示坐标；X/Y 分别使用上面的物理安装偏移。
SCREEN_CENTER_X = DISPLAY_SIZE[0] * 0.5 + CENTER_X_OFFSET_PX
SCREEN_CENTER_Y = DISPLAY_SIZE[1] * 0.5 + CENTER_Y_OFFSET_PX
# 脱机实时模式：关闭 CanMV IDE Frame Buffer 同步，减少额外图像拷贝；
# 关键是 IDE 预览与图传 WBC 都从 VO 抓帧会互相抢资源导致两者卡死，关掉 IDE 预览让图传独占。
# 需要 IDE 看画面时再改回 True（但那时别同时开图传）。
ENABLE_IDE_PREVIEW = False
# 启用 WiFi + WBC/JPEG/HTTP MJPEG 图传。
ENABLE_MJPEG_STREAM = True
# 诊断开关：False = 只连 WiFi 不建编码器，测 WiFi 单独对帧率的影响。
ENABLE_MJPEG_ENCODER = True
# 关闭左上角三行调试文字，只保留导轨、锁球框和左下角距离。
SHOW_DEBUG_OVERLAY = False

# 远距离小球置信度会明显降低。若误检增多可调回 0.25~0.35；
# 若仍有少量漏检可继续降到 0.15。
CONFIDENCE_THRESHOLD = 0.12
NMS_THRESHOLD = 0.45
# 降低阈值后保留更多候选，再由下方管道 ROI 筛出真正的小球。
MAX_DETECTION_BOXES = 20
# 远处检测框只有几像素时，长宽比抖动会比近距离明显。
BALL_ASPECT_RATIO_MIN = 0.30
BALL_ASPECT_RATIO_MAX = 3.00

# 首次运行会自动执行三点标定，并保存到 SD 卡。需要重新标定时，
# 将 FORCE_RECALIBRATE 改为 True；标定成功后再改回 False。
CALIBRATION_FILE = "/sdcard/ball_calibration.json"
FORCE_RECALIBRATE = False
# 装置固定前使用临时像素坐标：画面中心为 0，左负右正；固定后再改为 True 标定厘米。
ENABLE_CALIBRATION = True

# ========= 临时像素/厘米换算：装置固定后只需修改下面两个宏 =========
# 例：实测球在 +5 cm 时距离画面中心 142 px，就保持 5.0 并把 160.0 改为 142.0。
DISTANCE_REFERENCE_CM = 5.0
DISTANCE_REFERENCE_PX = 148.0
PROVISIONAL_PIXELS_PER_CM = DISTANCE_REFERENCE_PX / DISTANCE_REFERENCE_CM
# ================================================================

CALIBRATION_POSITIONS_CM = (
    0.0,
    DISTANCE_REFERENCE_CM,
    -DISTANCE_REFERENCE_CM,
)
CALIBRATION_SETTLE_MS = 6000
CALIBRATION_TARGET_GATE_CM = 1.0
CALIBRATION_SAMPLE_COUNT = 20
CALIBRATION_MIN_VALID_COUNT = 10

ROD_LENGTH_CM = 25.0
BALL_CLASS_ID = 0
# 水管外径 20 mm、内径 3.4 mm。上/下引导线对应外壁，内径用于候选球
# 靠近中心轴的优先级；控制量始终只使用水平方向 X。
PIPE_OUTER_DIAMETER_CM = 2.0
PIPE_INNER_DIAMETER_CM = 0.34
# 上下引导线各向外扩 8 px；钢球检测 ROI 与屏幕上可见的蓝线完全一致。
PIPE_ROI_MARGIN_PX = 8.0
# 0 = 球心必须严格位于上下蓝线之间。通常保持 0；改大会在蓝线外
# 增加不可见的识别余量。
BALL_DETECTION_EXTRA_ROI_MARGIN_PX = 4.0
# 真钢球的检测框必须穿过绿色中心线。5 px 用于包容运动时检测框的纵向抖动；
# 若还要更严可改成 0.0。
BALL_GREEN_LINE_TOLERANCE_PX = 10.0
# 尚未完成标定时用于显示水管边界的临时半高；与 8 px ROI 余量相加后，
# 蓝色上下线距当前 y=259 中心约 31 px，即 y=228 和 y=290。
PIPE_PREVIEW_HALF_HEIGHT_PX = 23.0
# 640x480 AI 画面显示到 800x480 OSD 后，X/Y 像素缩放比例不同。
DISPLAY_Y_TO_X_SCALE = (
    (DISPLAY_SIZE[1] / RGB888P_SIZE[1])
    / (DISPLAY_SIZE[0] / RGB888P_SIZE[0])
)
ROD_END_MARGIN_CM = 1.0

# 钢球运动状态估计参数。位置和速度都使用真实帧间隔，不依赖固定 FPS。
# 控制路径不用三点中值，避免单调运动引入约一帧延迟；创新门和低通负责抑噪。
FILTER_MEDIAN_WINDOW = 1
FILTER_POSITION_ALPHA = 0.60
FILTER_VELOCITY_TAU_S = 0.12
FILTER_MAX_ACCELERATION_CM_S2 = 200.0
# 钢球快速滚动时允许较大瞬时速度，但输出仍经过加速度限幅和低通。
MAX_BALL_SPEED_CM_S = 100.0
OUTLIER_BASE_ALLOWANCE_CM = 0.7
PREDICTION_TIMEOUT_MS = 300
TRACK_LOST_TIMEOUT_MS = 500
REACQUIRE_RESET_MS = 300
REACQUIRE_CONFIRM_FRAMES = 2
REACQUIRE_GATE_CM = 2.5
REACQUIRE_CONFIRM_TIMEOUT_MS = 400

# ========= LCD 实时锁球预测参数 =========
# CHN0 是实时视频层，AI 框来自较早的 CHN2 帧；按速度向前预测补偿延迟。
VISUAL_TRACK_PREDICTION_ENABLED = True
# 高帧率(55FPS)下 AI 帧延迟小，前馈 20ms 补偿 CHN0/VO/LCD 显示延迟。
# 前馈太大会放大速度噪声导致红框左右摆，保持小值。红框落后就增大、超前就减小。
# 只影响红框显示，不改变 UART 实测位置。
VISUAL_TRACK_EXTRA_LEAD_MS = 20
VISUAL_TRACK_MAX_LEAD_MS = 190
# 限制单次速度前馈的最大位移，防止异常速度把红框甩出很远。
VISUAL_TRACK_MAX_PREDICTION_OFFSET_PX = 120.0
# 320 模型检测球心抖动较大，低速时会被预测放大成红框左右摆；低于此速度不做前馈。
VISUAL_TRACK_PREDICTION_MIN_SPEED_PX_S = 250.0
# 模型短暂漏检时跨过更多帧（~5 帧），快球漏几帧不至于立即 LOST。
VISUAL_TRACK_HOLD_MS = 700
# 越大越紧跟速度变化，越小越平滑。320 模型噪声大，降到 0.5 抑制红框抖动；跟不住再调高。
VISUAL_TRACK_VELOCITY_ALPHA = 0.5
VISUAL_TRACK_MAX_SPEED_PX_S = 2400.0
# 检测短暂丢失后保留速度预测锁定，避免使用过期球心排斥真实候选。
BALL_TARGET_LOCK_MEMORY_MS = 1500
# 锁球门限放宽到 320px，快球冲远后仍允许回到候选，不被直接拒绝。
BALL_TARGET_LOCK_GATE_PX = 320.0
# 同一锁定范围内候选越靠近上一球心，排序权重越高。
BALL_TARGET_PROXIMITY_PENALTY = 0.30
# 检测框尺寸每帧采用新值的比例；调小可减少红框大小抖动。
VISUAL_TRACK_BOX_SIZE_ALPHA = 0.35
# 球心死区：移动 ≤5px 时红框冻结，滤掉检测抖动。去掉中值后靠死区稳住。
VISUAL_TRACK_DEADBAND_PX = 2.0
# 红框比检测框每侧扩大若干像素，降低快速运动时出框概率。
VISUAL_TRACK_BOX_MARGIN_PX = 7
# ========================================

# K230 直接控制 Emm42，不再经过 MSPM0G3507。
# GPIO11(TX) -> Emm42 RX，GPIO12(RX) <- Emm42 TX，双方必须共地；
# Emm42 必须设为 TTL 串口、115200 8N1、地址 0x01、固定 0x6B 校验。
DIRECT_MOTOR_CONTROL_ENABLED = True
CONTROL_BUILD_ID = "HOLD0_DIRECT_V27_STATIC_GATE_20260802"
MOTOR_UART_BAUDRATE = 115200
MOTOR_UART_TX_PIN = 11
MOTOR_UART_RX_PIN = 12
MOTOR_CONTROL_HZ = 10
MOTOR_TARGET_CM = 0.0
MOTOR_AUTO_ARM_VALID_FRAMES = 8
MOTOR_AUTO_ARM_WINDOW_CM = 1.0
MOTOR_AUTO_ARM_MAX_SPEED_CM_S = 1.0
MOTOR_VISION_SAFE_AGE_MS = 160
MOTOR_VISION_STOP_AGE_MS = 900
MOTOR_ACK_TIMEOUT_MS = 500
MOTOR_ACK_MISS_LIMIT = 3
MOTOR_UART_WRITE_FAIL_LIMIT = 2
MOTOR_STOP_WRITE_RETRIES = 2
MOTOR_CONTROL_OVERRUN_MS = 250
MOTOR_REARM_COOLDOWN_MS = 1000

# 平衡任务模式配置；SEQUENCE 模式下才执行 0 cm -> +5 cm -> -5 cm。
BALANCE_TASK_ENABLED = True
# 可选任务模式：
#   HOLD_ZERO：持续以 0 cm 为平衡点，受到外力干扰后仍回到零点。
#   HOLD_POSITION：持续以管内指定坐标为平衡点。
#   SEQUENCE_0_PLUS5_MINUS5：保留原来的 0 -> +5 cm -> -5 cm 动作任务。
TASK_MODE_HOLD_ZERO = "HOLD_ZERO"
TASK_MODE_HOLD_POSITION = "HOLD_POSITION"
TASK_MODE_SEQUENCE = "SEQUENCE_0_PLUS5_MINUS5"
TASK_MODE_CALIBRATION = "CALIBRATION"
BALANCE_TASK_MODE = TASK_MODE_HOLD_ZERO

CALIBRATION_DURATION_MS = 6000
CALIBRATION_MIN_SAMPLES = 40
CALIBRATION_MAX_DEVIATION_CM = 0.35

# HOLD_ZERO/HOLD_POSITION must be able to re-arm after switching from the
# +/-5 cm sequence or the +3 cm hold target. Vision and speed gates below
# still prevent a restart while the ball is moving quickly.
TASK_HOLD_ZERO_TARGET_CM = 0.0
TASK_HOLD_POSITION_TARGET_CM = 3.0
TASK_HOLD_REARM_WINDOW_CM = 6.0

# 持续平衡模式使用“剩余距离 -> 安全回中速度”的速度包络。
# 球离目标远时先加速；接近目标时安全速度自动下降并提前制动。
TASK_HOLD_MAX_RETURN_SPEED_CM_S = 0.60
TASK_HOLD_BRAKE_ACCEL_CM_S2 = 0.80
TASK_HOLD_APPROACH_ZONE_CM = 7.0
TASK_HOLD_APPROACH_SPEED_GAIN_CM_S_PER_CM = 0.18
TASK_HOLD_MAX_ACCELERATING_ANGLE_DEG = 1.20
TASK_HOLD_OVERSPEED_COMP_GAIN_DEG_S_PER_CM = 0.35
TASK_HOLD_HARD_SPEED_LIMIT_CM_S = 1.20
TASK_HOLD_HARD_OVERSPEED_CM_S = 0.90
TASK_HOLD_HARD_BRAKE_ANGLE_DEG = 1.20
TASK_HOLD_HARD_BRAKE_SLEW_DEG_S = 12.0
TASK_HOLD_PROFILE_ACCEL_CM_S2 = 4.0
TASK_HOLD_PROFILE_DECEL_CM_S2 = 8.0
TASK_HOLD_INTEGRAL_MAX_SPEED_CM_S = 0.8
# 视觉帧、UART 发送和位置模式执行均有延迟；速度包络按该延迟预留制动距离。
TASK_PROFILE_ACTUATOR_DELAY_S = 0.060
TASK_START_POSITION_CM = 0.0
# True: the detected launch position is treated as task zero, so each run
# travels exactly +5 cm and then -5 cm relative to that launch position.
# False: use the fixed physical +5/-5 cm calibration marks.
TASK_SEQUENCE_RELATIVE_TO_START = False
TASK_PLUS_TARGET_OFFSET_CM = 5.0
TASK_MINUS_TARGET_OFFSET_CM = -5.0
TASK_PLUS_TARGET_CM = TASK_PLUS_TARGET_OFFSET_CM
TASK_MINUS_TARGET_CM = TASK_MINUS_TARGET_OFFSET_CM
TASK_POSITION_TOLERANCE_CM = 0.25
TASK_START_POSITION_TOLERANCE_CM = 0.80
TASK_START_MAX_SPEED_CM_S = 0.50
TASK_START_SETTLE_MS = 200
TASK_PLUS_POSITION_TOLERANCE_CM = 0.50
TASK_PLUS_MAX_SPEED_CM_S = 0.90
TASK_PLUS_BRAKE_ACCEL_CM_S2 = 0.85
TASK_SEQUENCE_PROFILE_ACCEL_CM_S2 = 2.5
TASK_SEQUENCE_PROFILE_DECEL_CM_S2 = 6.0
TASK_PLUS_APPROACH_SPEED_ZONE_CM = 1.1
TASK_PLUS_APPROACH_SPEED_GAIN_CM_S_PER_CM = 0.80
TASK_PLUS_APPROACH_MAX_ANGLE_ZONE_CM = 1.1
TASK_PLUS_APPROACH_MAX_ANGLE_DEG = 1.85
TASK_PLUS_BREAKAWAY_MAX_SPEED_CM_S = 0.45
TASK_PLUS_BREAKAWAY_ANGLE_DEG = 1.55
TASK_PLUS_CRUISE_MIN_SPEED_CM_S = 1.00
TASK_PLUS_CRUISE_MIN_ANGLE_DEG = 2.00
TASK_PLUS_OVERSPEED_BRAKE_GAIN_DEG_S_PER_CM = 1.10
TASK_PLUS_SWITCH_TOLERANCE_CM = 0.65
TASK_PLUS_SWITCH_MAX_SPEED_CM_S = 0.25
TASK_PLUS_SETTLE_MS = 0
TASK_PLUS_FINAL_PUSH_ZONE_CM = 1.00
TASK_PLUS_FINAL_PUSH_MAX_SPEED_CM_S = 0.25
TASK_PLUS_FINAL_PUSH_MIN_ANGLE_DEG = 1.00
TASK_PLUS_FINAL_PUSH_STUCK_ZONE_CM = 0.60
TASK_PLUS_FINAL_PUSH_STUCK_MAX_SPEED_CM_S = 0.45
TASK_PLUS_FINAL_PUSH_STUCK_ANGLE_DEG = 1.35
TASK_PLUS_DRIVE_POSITION_GAIN_SCALE = 1.00
TASK_PLUS_DRIVE_INTEGRAL_GAIN_SCALE = 1.00
TASK_PLUS_DRIVE_VELOCITY_GAIN_SCALE = 1.00
TASK_PLUS_DRIVE_STATIC_ANGLE_SCALE = 1.25
TASK_PLUS_DRIVE_MAX_ANGLE_DEG = 2.20
TASK_PLUS_DRIVE_ANGLE_SLEW_DEG_S = 6.0
# 0 -> +5 起步破静摩擦补偿：只在视觉确认序列进入 MOVE_PLUS 后启用。
TASK_SEQUENCE_START_FEEDFORWARD_ENABLED = True
TASK_SEQUENCE_START_FEEDFORWARD_ANGLE_DEG = 1.20
TASK_SEQUENCE_START_FEEDFORWARD_HOLD_MS = 120
TASK_SEQUENCE_START_FEEDFORWARD_FADE_MS = 280
TASK_SEQUENCE_START_FEEDFORWARD_RELEASE_SPEED_CM_S = 0.60
TASK_SEQUENCE_START_FEEDFORWARD_RELEASE_PROGRESS_CM = 0.35
TASK_SEQUENCE_START_FEEDFORWARD_SLEW_DEG_S = 16.0
# +5 cm 切到 -5 cm 后，小球仍向正方向滑动时加快卸掉正倾角，限制左侧惯性过冲。
TASK_PLUS_TO_MINUS_RELEASE_SLEW_DEG_S = 40.0
TASK_FINAL_MAX_SPEED_CM_S = 0.20
TASK_FINAL_SETTLE_MS = 800
TASK_PERFORMANCE_BUDGET_MS = 5000
TASK_DEADLINE_MS = 60000
TASK_TIMEOUT_ENABLED = False
# 启动时若球不在 0 cm 附近，先自动使能并用独立回中 PID 拉回任务起点。
TASK_RETURN_ENABLED = False
TASK_RETURN_AUTO_ARM_WINDOW_CM = 8.0
TASK_RETURN_POSITION_KP_DEG_PER_CM = 0.50
TASK_RETURN_INTEGRAL_KI_DEG_PER_CM_S = 0.0
TASK_RETURN_VELOCITY_KD_DEG_S_PER_CM = 0.40
TASK_RETURN_STATIC_ANGLE_DEG = 0.50
TASK_RETURN_STATIC_APPLY_DELAY_MS = 250
TASK_RETURN_STATIC_SLEW_DEG_S = 1.5
TASK_RETURN_MAX_ANGLE_DEG = 3.5
TASK_RETURN_ANGLE_SLEW_DEG_S = 12.0
# 回中最后 3.5 cm 提前制动，使用低增益、无积分、无静摩擦补偿。
TASK_RETURN_BRAKE_ZONE_CM = 3.5
TASK_RETURN_BRAKE_POSITION_KP_DEG_PER_CM = 0.42
TASK_RETURN_BRAKE_INTEGRAL_KI_DEG_PER_CM_S = 0.0
TASK_RETURN_BRAKE_VELOCITY_KD_DEG_S_PER_CM = 0.40
TASK_RETURN_BRAKE_STATIC_ERROR_CM = 0.8
TASK_RETURN_BRAKE_STATIC_ANGLE_DEG = 0.45
TASK_RETURN_BRAKE_STATIC_APPLY_DELAY_MS = 300
TASK_RETURN_BRAKE_STATIC_SLEW_DEG_S = 1.0
TASK_RETURN_BRAKE_MAX_ANGLE_DEG = 2.0
TASK_RETURN_BRAKE_ANGLE_SLEW_DEG_S = 8.0
TASK_RETURN_POSITION_DEADBAND_CM = 0.35
TASK_RETURN_SPEED_DEADBAND_CM_S = 0.90
# -5 cm 最终收敛区单独增强低速修正，避免全程增益过大造成 +5 cm 过冲。
TASK_MINUS_DRIVE_POSITION_GAIN_SCALE = 0.50
TASK_MINUS_DRIVE_INTEGRAL_GAIN_SCALE = 0.0
TASK_MINUS_DRIVE_VELOCITY_GAIN_SCALE = 1.00
TASK_MINUS_DRIVE_STATIC_ANGLE_SCALE = 1.50
TASK_MINUS_DRIVE_MAX_ANGLE_DEG = 4.00
TASK_MINUS_DRIVE_ANGLE_SLEW_DEG_S = 6.0
TASK_MINUS_BRAKE_ANGLE_SLEW_DEG_S = 30.0
TASK_MINUS_COAST_ANGLE_DEG = -0.30
TASK_MINUS_CRUISE_MIN_ANGLE_DEG = -1.20
TASK_MINUS_REBOUND_BRAKE_START_CM = -1.00
TASK_MINUS_REBOUND_BRAKE_MIN_SPEED_CM_S = 0.40
TASK_MINUS_REBOUND_BRAKE_ANGLE_DEG = -3.0
TASK_MINUS_MAX_SPEED_CM_S = 2.5
TASK_MINUS_BRAKE_ACCEL_CM_S2 = 1.35
TASK_MINUS_APPROACH_SPEED_ZONE_CM = 1.1
TASK_MINUS_APPROACH_SPEED_GAIN_CM_S_PER_CM = 0.90
TASK_MINUS_OVERSPEED_BRAKE_GAIN_DEG_S_PER_CM = 2.80
TASK_MINUS_HARD_OVERSPEED_CM_S = 0.35
TASK_MINUS_HARD_BRAKE_ANGLE_DEG = 3.30
TASK_MINUS_APPROACH_BRAKE_ZONE_CM = 1.8
TASK_MINUS_APPROACH_BRAKE_MAX_ANGLE_DEG = 1.0
TASK_MINUS_TARGET_HOLD_ZONE_CM = 0.60
TASK_MINUS_OVERSHOOT_TRIGGER_CM = 0.50
TASK_MINUS_OVERSHOOT_CORRECTION_MAX_ANGLE_DEG = -3.30
TASK_MINUS_OVERSHOOT_ANGLE_SLEW_DEG_S = 6.0
TASK_MINUS_DEEP_OVERSHOOT_TRIGGER_CM = 1.00
TASK_MINUS_DEEP_OVERSHOOT_CORRECTION_ANGLE_DEG = -4.00
TASK_MINUS_TARGET_POSITION_GAIN = 0.00
TASK_MINUS_TARGET_VELOCITY_BRAKE_GAIN = 0.80
TASK_MINUS_TARGET_REST_BIAS_ANGLE_DEG = 0.00
TASK_MINUS_TARGET_MIN_ANGLE_DEG = -1.40
TASK_MINUS_TARGET_MAX_ANGLE_DEG = 3.30
TASK_MINUS_TARGET_HOLD_ANGLE_SLEW_DEG_S = 16.0
TASK_MINUS_FINAL_HOLD_MIN_CM = -6.0
TASK_MINUS_FINAL_HOLD_MAX_CM = -5.5
TASK_MINUS_FINAL_HOLD_ANGLE_DEG = -1.00
TASK_MINUS_FINAL_HOLD_ANGLE_SLEW_DEG_S = 1.0
TASK_MINUS_STUCK_PUSH_MAX_SPEED_CM_S = 0.60
TASK_MINUS_STUCK_PUSH_ANGLE_DEG = -3.30
TASK_MINUS_CORRECTION_ZONE_CM = 2.5
TASK_MINUS_POSITION_GAIN_SCALE = 1.80
TASK_MINUS_INTEGRAL_GAIN_SCALE = 1.00
TASK_MINUS_STATIC_ANGLE_SCALE = 1.00
TASK_MINUS_STATIC_APPLY_DELAY_MS = 120
TASK_MINUS_STATIC_SLEW_DEG_S = 2.5

# Emm42 快速绝对位置模式参数，来自原 3507 工程的实机配置。
EMM42_ADDRESS = 0x01
EMM42_FIXED_CHECKSUM = 0x6B
EMM42_SPEED_RPM = 60
EMM42_ACCELERATION = 80
# V25 稳态残差约 -0.34 cm 时，平均仍需 +0.167°（约 8 脉冲）维持；
# 由 pulse = center - angle*50 反推，将机械水平中心从 -5 微调到 -13。
EMM42_CENTER_PULSE = -13
EMM42_MIN_PULSE = -270
EMM42_MAX_PULSE = 430
# 坐标固定为屏幕右侧=正；该符号只描述“正摆杆角度”对应的电机脉冲方向。
# 空载用 ±0.5° 验证：正角应使小球加速度朝右；若相反，只把此项改为 -1。
EMM42_ANGLE_TO_PULSE_SIGN = -1

# 持续定点平衡 PID：只用于 HOLD_ZERO / HOLD_POSITION。
HOLD_BALANCE_POSITION_KP_DEG_PER_CM = 0.34
HOLD_BALANCE_INTEGRAL_KI_DEG_PER_CM_S = 0.050
HOLD_BALANCE_VELOCITY_KD_DEG_S_PER_CM = 0.35
HOLD_BALANCE_STATIC_ANGLE_DEG = 0.24
HOLD_POSITION_DEADBAND_CM = 0.08
HOLD_SPEED_DEADBAND_CM_S = 0.25
# 正坐标侧回零需要负角，实机负向机构阻力更大，因此只增强这一侧。
HOLD_POSITIVE_SIDE_VELOCITY_GAIN_SCALE = 1.00
HOLD_POSITIVE_SIDE_STATIC_ANGLE_DEG = 0.25
HOLD_POSITIVE_SIDE_MAX_ANGLE_DEG = 0.80
# The opposite side needs its own friction compensation and angle limit.
HOLD_NEGATIVE_SIDE_STATIC_ANGLE_DEG = 0.38
HOLD_NEGATIVE_SIDE_MAX_ANGLE_DEG = 2.00

# The cart starts along the pipe and initially drives the ball toward +X.
# Apply one negative pre-tilt when the balance motor first becomes ready.
CART_START_FEEDFORWARD_ENABLED = False
CART_START_FEEDFORWARD_ANGLE_DEG = 0.0
CART_START_FEEDFORWARD_HOLD_MS = 450
CART_START_FEEDFORWARD_FADE_MS = 350

# K230 GPIO3/4 接蓝牙串口模块，用 UART1 与 Emm42 的 UART2 分离。
# 若蓝牙模块出厂波特率不同，只需修改这一项后重新上传脚本。
BLUETOOTH_UART_BAUDRATE = 9600
BLUETOOTH_UART_TX_PIN = 3
BLUETOOTH_UART_RX_PIN = 4
BLUETOOTH_UART_MAX_LINE_LENGTH = 96
BLUETOOTH_UART_TX_CHUNK_BYTES = 32
BLUETOOTH_UART_TX_INTERVAL_MS = 40
BLUETOOTH_UART_TX_BUFFER_LIMIT = 768

# ST7701 LCD touch mode selector. This firmware reports display coordinates.
LCD_MODE_TOUCH_POLL_MS = 30
LCD_MODE_TOUCH_DEBOUNCE_MS = 350
LCD_MODE_BUTTONS = (
    (TASK_MODE_HOLD_ZERO, "HOLD 0", 400, 50, 120, 56),
    (TASK_MODE_HOLD_POSITION, "HOLD POS", 525, 50, 120, 56),
    (TASK_MODE_SEQUENCE, "0/+5/-5", 650, 50, 140, 56),
)

# 动作序列 PID：只用于 0 -> +5 cm -> -5 cm，后续仍叠加各阶段缩放参数。
SEQUENCE_BALANCE_POSITION_KP_DEG_PER_CM = 0.65
SEQUENCE_BALANCE_INTEGRAL_KI_DEG_PER_CM_S = 0.08
SEQUENCE_BALANCE_VELOCITY_KD_DEG_S_PER_CM = 0.45
SEQUENCE_BALANCE_STATIC_ANGLE_DEG = 0.60

# 两套控制器共享脉冲换算、积分范围、限角和斜率保护。
BALANCE_PULSES_PER_DEG = 50.0
BALANCE_INTEGRAL_ZONE_CM = 2.5
BALANCE_INTEGRAL_LIMIT_CM_S = 5.0
# 首次实机方向验证临时限角；确认正角使球向右后恢复为 2.5。
BALANCE_MAX_ANGLE_DEG = 1.4
BALANCE_ANGLE_SLEW_DEG_S = 8.0
BALANCE_BRAKE_ANGLE_SLEW_DEG_S = 22.0
BALANCE_BRAKE_MIN_SPEED_CM_S = 2.0
BALANCE_POSITION_DEADBAND_CM = 0.35
BALANCE_SPEED_DEADBAND_CM_S = 0.35
# 真正低速进入目标区后锁存当时的实际平衡角；超出更宽的滞回区才恢复动态闭环。
# 这避免稳住后把角度强制归零，也避免视觉量化噪声反复唤醒 PID。
BALANCE_SETTLED_CONFIRM_MS = 150
BALANCE_SETTLED_POSITION_RELEASE_SCALE = 2.0
BALANCE_SETTLED_SPEED_RELEASE_SCALE = 2.0
BALANCE_SETTLED_MIN_POSITION_RELEASE_CM = 0.20
BALANCE_SETTLED_MIN_SPEED_RELEASE_CM_S = 0.50
# HOLD0 实机最后约 0.35 cm 会因静摩擦提前停住；低于该误差也允许缓慢
# 建立静摩擦补偿，避免在零点外卡死。
BALANCE_STATIC_ERROR_CM = 0.15
BALANCE_STATIC_APPLY_DELAY_MS = 900
BALANCE_STATIC_MAX_SPEED_CM_S = 0.35
# 已经建立的静摩擦补偿允许短时保留到更高速度，避免检测噪声反复撤销补偿。
BALANCE_STATIC_RELEASE_SPEED_CM_S = 0.60
BALANCE_STATIC_SLEW_DEG_S = 0.35
# 球开始滚动后快速撤销静摩擦补偿，避免补偿继续推动造成过冲。
BALANCE_STATIC_RELEASE_SLEW_DEG_S = 8.00

# 有界在线调参：高速过零增加阻尼；长时间偏离且低速则增加驱动力。
PID_AUTO_TUNE_ENABLED = False
PID_TUNING_FILE = "/sdcard/vision_pid.json"
PID_TUNING_VERSION = 27
PID_AUTO_TUNE_MIN_INTERVAL_MS = 3000
PID_AUTO_TUNE_SAVE_INTERVAL_MS = 30000
PID_AUTO_TUNE_CROSSING_SPEED_CM_S = 6.0
PID_AUTO_TUNE_STUCK_ERROR_CM = 0.8
PID_AUTO_TUNE_STUCK_SPEED_CM_S = 0.5
PID_AUTO_TUNE_STUCK_TIME_MS = 1500
PID_POSITION_GAIN_MIN = 0.10
PID_POSITION_GAIN_MAX = 1.05
PID_VELOCITY_GAIN_MIN = 0.04
PID_VELOCITY_GAIN_MAX = 1.20
PID_STATIC_ANGLE_MIN = 0.05
PID_STATIC_ANGLE_MAX = 1.00

# ========= MJPEG 图传参数：后续只需在这里调整 =========
MJPEG_PORT = 8080
# 由独立线程 _mjpeg_loop 按此频率发送；不受 AI 推理帧率（~12.5fps）限制。
MJPEG_FPS = 20
# JPEG 质量范围 1~99；越高越清晰、带宽越大。建议先用 50。
MJPEG_QUALITY = 40
# 非阻塞 TCP 分块发送，网络堵塞时尽快丢掉客户端，避免拖慢控制循环。
MJPEG_SEND_CHUNK_BYTES = 16384
# VLC 偶发会超过 100 ms 不读取数据；稳定阶段允许 300 ms 网络抖动。
# 手机 Wi-Fi 偶发调度停顿会超过 300 ms，过短会误判断线并停在最后一帧。
MJPEG_SEND_STALL_TIMEOUT_MS = 1500
# WBC/VENC 在脱机冷启动时偶尔不会立即产出下一帧，禁止无限等待卡死主循环。
MJPEG_ENCODER_TIMEOUT_MS = 300
# 新连接建立和解码器预热更慢，仅前几帧给予更长宽限。
MJPEG_STARTUP_STALL_TIMEOUT_MS = 1000
MJPEG_STARTUP_FRAME_COUNT = 5
# K230 上 socket.accept() 很重，每帧都调会把 FPS 从 55 拖到 ~8；限频轮询客户端连接。
MJPEG_ACCEPT_POLL_MS = 500
# ========================================================

# image 绘图接口使用 RGB；单类别强制为红色。
RED = (255, 0, 0)
MAGENTA = (255, 0, 255)
YELLOW = (255, 255, 0)
CYAN = (0, 255, 255)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
BLACK = (0, 0, 0)
MODE_BUTTON_ACTIVE = (0, 150, 80)
MODE_BUTTON_INACTIVE = (35, 35, 35)
MODE_BUTTON_BORDER = (255, 255, 255)

# 小球中心到画面中心的实时距离线宽。
CENTER_DISTANCE_LINE_THICKNESS = 4

# ========================================================


def read_deploy_config(config_path):
    with open(config_path, "r") as json_file:
        config = ujson.load(json_file)
    return config


def validate_deploy_config(config):
    required_keys = (
        "kmodel_path",
        "categories",
        "confidence_threshold",
        "nms_threshold",
        "img_size",
        "num_classes",
        "nms_option",
        "model_type",
        "anchors",
    )
    for key in required_keys:
        if key not in config:
            raise ValueError("deploy_config missing key: %s" % key)

    if config["model_type"] != "AnchorBaseDet":
        raise ValueError(
            "Only AnchorBaseDet is supported by this det_video.py: %s"
            % config["model_type"]
        )
    if list(config["img_size"]) != EXPECTED_MODEL_INPUT_SIZE:
        raise ValueError(
            "Expected 320x320 model input, got %s" % config["img_size"]
        )
    if int(config["num_classes"]) != len(config["categories"]):
        raise ValueError("num_classes does not match categories")
    if not config["categories"] or str(config["categories"][BALL_CLASS_ID]).lower() != "ball":
        raise ValueError("class 0 must be the ball class")
    if len(config["anchors"]) != 3:
        raise ValueError("AnchorBaseDet requires three anchor groups")
    for group in config["anchors"]:
        if len(group) == 0 or len(group) % 6 != 0:
            raise ValueError("invalid AnchorBaseDet anchor group")
    return config


def convert_detections_to_display(det_boxes, source_size, display_size):
    """Convert aicube xyxy boxes into the reference xywh result format."""
    if not det_boxes:
        return [[], [], []]

    source_width = float(source_size[0])
    source_height = float(source_size[1])
    display_width = float(display_size[0])
    display_height = float(display_size[1])
    scale_x = display_width / source_width
    scale_y = display_height / source_height
    boxes = []
    class_ids = []
    scores = []

    for det_box in det_boxes:
        if len(det_box) < 6:
            continue
        class_id = int(det_box[0])
        score = float(det_box[1])
        x1 = float(det_box[2]) * scale_x
        y1 = float(det_box[3]) * scale_y
        x2 = float(det_box[4]) * scale_x
        y2 = float(det_box[5]) * scale_y
        x1 = max(0.0, min(x1, display_width))
        y1 = max(0.0, min(y1, display_height))
        x2 = max(x1, min(x2, display_width))
        y2 = max(y1, min(y2, display_height))
        boxes.append([x1, y1, x2 - x1, y2 - y1])
        class_ids.append(class_id)
        scores.append(score)

    return [boxes, class_ids, scores]


class NativeAnchorBaseDetector:
    """Native nncase/aicube detector for the deployed 320x320 kmodel."""

    def __init__(self, config_path=CONFIG_PATH, rgb888p_size=RGB888P_SIZE):
        self.config = validate_deploy_config(read_deploy_config(config_path))
        self.labels = {
            index: label
            for index, label in enumerate(self.config["categories"])
        }
        self.model_input_size = list(self.config["img_size"])
        self.rgb888p_size = list(rgb888p_size)
        self.frame_size = list(rgb888p_size)
        self.strides = [8, 16, 32]
        self.anchors = (
            self.config["anchors"][0]
            + self.config["anchors"][1]
            + self.config["anchors"][2]
        )
        self.confidence_threshold = self.config["confidence_threshold"]
        self.nms_threshold = self.config["nms_threshold"]
        self.num_classes = self.config["num_classes"]
        self.nms_option = self.config["nms_option"]
        self.kmodel_path = ROOT_PATH + self.config["kmodel_path"]

        width = self.model_input_size[0]
        height = self.model_input_size[1]
        source_width = self.rgb888p_size[0]
        source_height = self.rgb888p_size[1]
        ratio = min(
            float(width) / source_width,
            float(height) / source_height,
        )
        new_width = int(ratio * source_width)
        new_height = int(ratio * source_height)
        dw = float(width - new_width) / 2.0
        dh = float(height - new_height) / 2.0
        top = int(round(dh - 0.1))
        bottom = int(round(dh + 0.1))
        left = int(round(dw - 0.1))
        right = int(round(dw + 0.1))

        self.kpu = None
        self.ai2d = None
        self.ai2d_builder = None
        self.ai2d_output_tensor = None
        try:
            self.kpu = nn.kpu()
            self.ai2d = nn.ai2d()
            self.kpu.load_kmodel(self.kmodel_path)
            self.ai2d.set_dtype(
                nn.ai2d_format.NCHW_FMT,
                nn.ai2d_format.NCHW_FMT,
                np.uint8,
                np.uint8,
            )
            self.ai2d.set_pad_param(
                True,
                [0, 0, 0, 0, top, bottom, left, right],
                0,
                [114, 114, 114],
            )
            self.ai2d.set_resize_param(
                True,
                nn.interp_method.tf_bilinear,
                nn.interp_mode.half_pixel,
            )
            self.ai2d_builder = self.ai2d.build(
                [1, 3, source_height, source_width],
                [1, 3, height, width],
            )
            output_data = np.ones(
                (1, 3, height, width),
                dtype=np.uint8,
            )
            self.ai2d_output_tensor = nn.from_numpy(output_data)
        except Exception:
            self.deinit()
            raise

    def config_preprocess(self):
        """Compatibility no-op for calibration helpers."""

    def run(self, frame):
        if frame is None or frame.format() != image.RGBP888:
            return [[], [], []]

        input_tensor = None
        results = []
        try:
            input_tensor = nn.from_numpy(frame.to_numpy_ref())
            self.ai2d_builder.run(input_tensor, self.ai2d_output_tensor)
            self.kpu.set_input_tensor(0, self.ai2d_output_tensor)
            self.kpu.run()
            for index in range(self.kpu.outputs_size()):
                output_tensor = self.kpu.get_output_tensor(index)
                try:
                    result = output_tensor.to_numpy()
                    result = result.reshape(
                        (
                            result.shape[0]
                            * result.shape[1]
                            * result.shape[2]
                            * result.shape[3],
                        )
                    )
                    results.append(result)
                finally:
                    del output_tensor

            det_boxes = aicube.anchorbasedet_post_process(
                results[0],
                results[1],
                results[2],
                self.model_input_size,
                self.frame_size,
                self.strides,
                self.num_classes,
                self.confidence_threshold,
                self.nms_threshold,
                self.anchors,
                self.nms_option,
            )
            return convert_detections_to_display(
                det_boxes,
                self.frame_size,
                DISPLAY_SIZE,
            )
        finally:
            if input_tensor is not None:
                del input_tensor
            gc.collect()

    def deinit(self):
        self.ai2d_output_tensor = None
        self.ai2d_builder = None
        self.ai2d = None
        self.kpu = None
        gc.collect()


class NativeDisplayPipeline:
    """Single Sensor/Display owner matching the reference pipeline interface."""

    def __init__(self):
        self.sensor = None
        self.osd_img = None
        self.display_initialized = False
        self.media_initialized = False
        self.sensor_running = False

    def create(self):
        try:
            self.sensor = Sensor(id=2)
            self.sensor.reset()
            self.sensor.set_hmirror(False)
            self.sensor.set_vflip(False)
            self.sensor.set_framesize(
                width=DISPLAY_SIZE[0],
                height=DISPLAY_SIZE[1],
                chn=CAM_CHN_ID_0,
            )
            self.sensor.set_pixformat(
                PIXEL_FORMAT_YUV_SEMIPLANAR_420,
                chn=CAM_CHN_ID_0,
            )
            self.sensor.set_framesize(
                width=RGB888P_SIZE[0],
                height=RGB888P_SIZE[1],
                chn=CAM_CHN_ID_2,
            )
            self.sensor.set_pixformat(
                PIXEL_FORMAT_RGB_888_PLANAR,
                chn=CAM_CHN_ID_2,
            )
            sensor_bind_info = self.sensor.bind_info(
                x=0,
                y=0,
                chn=CAM_CHN_ID_0,
            )
            Display.bind_layer(
                **sensor_bind_info,
                layer=Display.LAYER_VIDEO1,
            )
            Display.init(
                Display.ST7701,
                to_ide=ENABLE_IDE_PREVIEW,
            )
            self.display_initialized = True
            self.osd_img = image.Image(
                DISPLAY_SIZE[0],
                DISPLAY_SIZE[1],
                image.ARGB8888,
            )
            MediaManager.init()
            self.media_initialized = True
            self.sensor.run()
            self.sensor_running = True
        except BaseException:
            self.destroy()
            raise

    def get_display_size(self):
        return list(DISPLAY_SIZE)

    def get_frame(self):
        if not self.sensor_running:
            raise RuntimeError("Sensor is not running")
        return self.sensor.snapshot(chn=CAM_CHN_ID_2)

    def show_image(self):
        Display.show_image(
            self.osd_img,
            0,
            0,
            Display.LAYER_OSD3,
        )

    def destroy(self):
        if self.sensor_running and self.sensor is not None:
            try:
                self.sensor.stop()
            except BaseException:
                pass
            self.sensor_running = False
        if self.display_initialized:
            try:
                Display.deinit()
            except BaseException:
                pass
            self.display_initialized = False
        if self.media_initialized:
            try:
                MediaManager.deinit()
            except BaseException:
                pass
            self.media_initialized = False
        self.sensor = None
        self.osd_img = None
        gc.collect()


def align_up(value, alignment):
    """向上对齐，避免依赖不同 CanMV 固件是否导出 ALIGN_UP。"""
    return (value + alignment - 1) // alignment * alignment


def median(values):
    """MicroPython 兼容的中值函数，仅用于标定采样。"""
    ordered = list(values)
    ordered.sort()
    count = len(ordered)
    if count == 0:
        return None
    middle = count // 2
    if count & 1:
        return float(ordered[middle])
    return (float(ordered[middle - 1]) + float(ordered[middle])) * 0.5


def calibration_target_ready(target_cm, detection_x):
    """球进入目标刻度附近后才允许开始标定倒计时，防止同一点被连续误采。"""
    if detection_x is None:
        return False
    expected_x = (
        SCREEN_CENTER_X + float(target_cm) * PROVISIONAL_PIXELS_PER_CM
    )
    gate_px = CALIBRATION_TARGET_GATE_CM * PROVISIONAL_PIXELS_PER_CM
    return abs(float(detection_x) - expected_x) <= gate_px


class BallStateEstimator:
    """钢球一维位置/速度估计器，输出可直接用于平衡控制的状态量。"""

    def __init__(self):
        self.initialized = False
        self.position_cm = 0.0
        self.velocity_cm_s = 0.0
        self.raw_cm = None
        self.median_cm = None
        self.valid = False
        self.status = "LOST"
        self.last_update_ms = None
        self.last_measurement_ms = None
        self.last_filtered_measurement_cm = None
        self.accepted_this_frame = False
        self.accepted_measurement_ms = None
        self.reacquire_candidate_cm = None
        self.reacquire_candidate_ms = None
        self.reacquire_confirm_count = 0
        self.raw_history = []
        self.rejected_count = 0
        self.missed_count = 0

    @staticmethod
    def _clamp(value, minimum, maximum):
        if value < minimum:
            return minimum
        if value > maximum:
            return maximum
        return value

    def _append_raw(self, value):
        self.raw_history.append(float(value))
        while len(self.raw_history) > FILTER_MEDIAN_WINDOW:
            self.raw_history.pop(0)
        return median(self.raw_history)

    def _initialize(self, measurement_cm, measurement_ms, status="ACQUIRE"):
        self.raw_history = []
        filtered_measurement = self._append_raw(measurement_cm)
        self.position_cm = filtered_measurement
        self.velocity_cm_s = 0.0
        self.raw_cm = float(measurement_cm)
        self.median_cm = filtered_measurement
        self.last_filtered_measurement_cm = filtered_measurement
        self.valid = True
        self.status = status
        self.last_update_ms = measurement_ms
        self.last_measurement_ms = measurement_ms
        self.accepted_this_frame = True
        self.accepted_measurement_ms = measurement_ms
        self.reacquire_candidate_cm = None
        self.reacquire_candidate_ms = None
        self.reacquire_confirm_count = 0
        self.initialized = True
        return True

    def predict_to(self, now_ms):
        """把已接受的视觉状态按匀速模型预测到控制器当前时刻。"""
        if not self.initialized or self.last_update_ms is None:
            return self
        elapsed_ms = time.ticks_diff(now_ms, self.last_update_ms)
        if elapsed_ms > 0:
            elapsed_seconds = elapsed_ms / 1000.0
            self.position_cm += self.velocity_cm_s * elapsed_seconds
            rod_limit = ROD_LENGTH_CM * 0.5 + ROD_END_MARGIN_CM
            self.position_cm = self._clamp(
                self.position_cm, -rod_limit, rod_limit
            )
            self.last_update_ms = now_ms

        since_measurement_ms = time.ticks_diff(now_ms, self.last_measurement_ms)
        if since_measurement_ms > PREDICTION_TIMEOUT_MS:
            self.valid = False
            self.status = "LOST"
            if since_measurement_ms > TRACK_LOST_TIMEOUT_MS:
                self.velocity_cm_s *= 0.5
        elif not self.accepted_this_frame:
            self.valid = True
            self.status = "PREDICT"
        return self

    def observe(self, measurement_cm, measurement_ms):
        """在 AI 帧时间戳处吸收一次测量，返回本帧测量是否通过创新门。"""
        self.accepted_this_frame = False
        if not self.initialized:
            if measurement_cm is None:
                self.raw_cm = None
                self.valid = False
                self.status = "LOST"
                self.missed_count += 1
                return False
            return self._initialize(measurement_cm, measurement_ms)

        self.predict_to(measurement_ms)
        predicted_position = self.position_cm
        since_measurement_ms = time.ticks_diff(
            measurement_ms, self.last_measurement_ms
        )

        if measurement_cm is not None:
            measurement_cm = float(measurement_cm)
            self.raw_cm = measurement_cm

            # 长时间丢球后允许在任意合法位置重新捕获，避免旧预测阻碍恢复。
            if since_measurement_ms > REACQUIRE_RESET_MS:
                candidate_expired = (
                    self.reacquire_candidate_ms is None
                    or time.ticks_diff(
                        measurement_ms, self.reacquire_candidate_ms
                    ) > REACQUIRE_CONFIRM_TIMEOUT_MS
                )
                candidate_changed = (
                    self.reacquire_candidate_cm is None
                    or abs(measurement_cm - self.reacquire_candidate_cm)
                    > REACQUIRE_GATE_CM
                )
                if candidate_expired or candidate_changed:
                    self.reacquire_candidate_cm = measurement_cm
                    self.reacquire_confirm_count = 1
                else:
                    self.reacquire_candidate_cm += 0.5 * (
                        measurement_cm - self.reacquire_candidate_cm
                    )
                    self.reacquire_confirm_count += 1
                self.reacquire_candidate_ms = measurement_ms
                if self.reacquire_confirm_count >= REACQUIRE_CONFIRM_FRAMES:
                    return self._initialize(
                        self.reacquire_candidate_cm,
                        measurement_ms,
                        "REACQUIRE",
                    )
                self.missed_count += 1
                return False

            # 允许位移由基础误差和物理最大速度共同决定，过滤远处误检。
            measurement_dt_s = max(0.001, since_measurement_ms / 1000.0)
            allowance_cm = (
                OUTLIER_BASE_ALLOWANCE_CM
                + MAX_BALL_SPEED_CM_S * measurement_dt_s
            )
            if abs(measurement_cm - predicted_position) > allowance_cm:
                self.rejected_count += 1
                self.missed_count += 1
                return False
        else:
            self.raw_cm = None
            self.reacquire_candidate_cm = None
            self.reacquire_candidate_ms = None
            self.reacquire_confirm_count = 0
            self.missed_count += 1
            return False

        filtered_measurement = self._append_raw(measurement_cm)
        residual = filtered_measurement - predicted_position
        self.position_cm = (
            predicted_position + FILTER_POSITION_ALPHA * residual
        )

        # 速度由滤波后位置的真实帧间差分得到，再做低通和加速度限幅。
        measurement_dt_s = since_measurement_ms / 1000.0
        if (
            self.last_filtered_measurement_cm is not None
            and measurement_dt_s >= 0.005
        ):
            measured_velocity_cm_s = (
                filtered_measurement - self.last_filtered_measurement_cm
            ) / measurement_dt_s
            measured_velocity_cm_s = self._clamp(
                measured_velocity_cm_s,
                -MAX_BALL_SPEED_CM_S,
                MAX_BALL_SPEED_CM_S,
            )
            maximum_velocity_step = (
                FILTER_MAX_ACCELERATION_CM_S2 * measurement_dt_s
            )
            measured_velocity_cm_s = self._clamp(
                measured_velocity_cm_s,
                self.velocity_cm_s - maximum_velocity_step,
                self.velocity_cm_s + maximum_velocity_step,
            )
            velocity_alpha = measurement_dt_s / (
                FILTER_VELOCITY_TAU_S + measurement_dt_s
            )
            self.velocity_cm_s += velocity_alpha * (
                measured_velocity_cm_s - self.velocity_cm_s
            )
        else:
            self.velocity_cm_s = 0.0
        self.last_filtered_measurement_cm = filtered_measurement
        self.velocity_cm_s = self._clamp(
            self.velocity_cm_s,
            -MAX_BALL_SPEED_CM_S,
            MAX_BALL_SPEED_CM_S,
        )
        self.median_cm = filtered_measurement
        self.last_measurement_ms = measurement_ms
        self.accepted_measurement_ms = measurement_ms
        self.accepted_this_frame = True
        self.reacquire_candidate_cm = None
        self.reacquire_candidate_ms = None
        self.reacquire_confirm_count = 0
        self.valid = True
        self.status = "TRACK"
        return True

    def update(self, measurement_cm, now_ms):
        """兼容旧调用：测量和控制时间相同时直接更新。"""
        accepted = self.observe(measurement_cm, now_ms)
        self.predict_to(now_ms)
        return accepted


class VisualBallTracker:
    """根据AI帧时间戳和水平速度，把检测框预测到LCD当前视频时刻。"""

    def __init__(self):
        self.valid = False
        self.status = "LOST"
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_measurement_ms = None
        self.velocity_x_px_s = 0.0
        self.box_width = 0.0
        self.box_height = 0.0
        self.predicted_x = 0.0
        self.predicted_y = 0.0
        self.prediction_lead_ms = 0
        self.pos_history_x = []
        self.pos_history_y = []

    @staticmethod
    def _clamp(value, minimum, maximum):
        if value < minimum:
            return minimum
        if value > maximum:
            return maximum
        return value

    def candidate_reference_x(self, measurement_ms):
        """短时漏检后仍返回上一真球 X，防止远处假候选抢框。"""
        if self.last_measurement_ms is None:
            return None
        age_ms = time.ticks_diff(measurement_ms, self.last_measurement_ms)
        if age_ms < 0 or age_ms > BALL_TARGET_LOCK_MEMORY_MS:
            return None
        predicted_x = (
            self.last_x + self.velocity_x_px_s * age_ms / 1000.0
        )
        return self._clamp(predicted_x, 0.0, DISPLAY_SIZE[0] - 1.0)

    def update(self, detection, measurement_ms, render_ms):
        """返回预测后的 (中心x, 中心y, 框宽, 框高, 是否预测帧)。"""
        has_measurement = detection is not None

        if has_measurement:
            raw_x = float(detection[0])
            raw_y = float(detection[1])
            width = float(detection[5])
            height = float(detection[6])
            # 不做中值滤波（会引入延迟导致红框追在球后面）；靠死区冻结小幅抖动。
            if self.valid and abs(raw_x - self.last_x) <= VISUAL_TRACK_DEADBAND_PX:
                center_x = self.last_x
            else:
                center_x = raw_x
            if self.valid and abs(raw_y - self.last_y) <= VISUAL_TRACK_DEADBAND_PX:
                center_y = self.last_y
            else:
                center_y = raw_y

            if self.last_measurement_ms is not None:
                elapsed_ms = time.ticks_diff(
                    measurement_ms, self.last_measurement_ms
                )
                if 1 <= elapsed_ms <= 500:
                    instant_velocity = (
                        (center_x - self.last_x) * 1000.0 / elapsed_ms
                    )
                    instant_velocity = self._clamp(
                        instant_velocity,
                        -VISUAL_TRACK_MAX_SPEED_PX_S,
                        VISUAL_TRACK_MAX_SPEED_PX_S,
                    )
                    # 均匀平滑，不再反向瞬切（瞬切会因检测噪声让红框左右甩）。
                    self.velocity_x_px_s += (
                        VISUAL_TRACK_VELOCITY_ALPHA
                        * (instant_velocity - self.velocity_x_px_s)
                    )
                else:
                    self.velocity_x_px_s = 0.0
            else:
                self.velocity_x_px_s = 0.0

            # 用 min(width,height) 做正方形框：运动模糊让检测框沿运动方向拉长，
            # 取短边可去掉向后的拉伸，框始终是贴球的正方形。
            # 尺寸每帧采用新值的比例调到 0.1，几乎锁定固定大小，减少框抖动。
            box_size = min(width, height)
            if self.valid:
                self.box_width += 0.1 * (box_size - self.box_width)
                self.box_height += 0.1 * (box_size - self.box_height)
            else:
                self.box_width = box_size
                self.box_height = box_size

            self.last_x = center_x
            self.last_y = center_y
            self.last_measurement_ms = measurement_ms
            self.valid = True
            self.status = "TRACK"

        if self.last_measurement_ms is None:
            self.valid = False
            self.status = "LOST"
            return None

        age_ms = time.ticks_diff(render_ms, self.last_measurement_ms)
        if age_ms < 0:
            age_ms = 0
        if not has_measurement and age_ms > VISUAL_TRACK_HOLD_MS:
            self.valid = False
            self.status = "LOST"
            self.velocity_x_px_s = 0.0
            return None

        if VISUAL_TRACK_PREDICTION_ENABLED:
            lead_ms = age_ms + VISUAL_TRACK_EXTRA_LEAD_MS
            lead_ms = int(
                self._clamp(lead_ms, 0, VISUAL_TRACK_MAX_LEAD_MS)
            )
        else:
            lead_ms = 0

        prediction_velocity_x = self.velocity_x_px_s
        if abs(prediction_velocity_x) < VISUAL_TRACK_PREDICTION_MIN_SPEED_PX_S:
            prediction_velocity_x = 0.0
        prediction_offset_x = prediction_velocity_x * lead_ms / 1000.0
        prediction_offset_x = self._clamp(
            prediction_offset_x,
            -VISUAL_TRACK_MAX_PREDICTION_OFFSET_PX,
            VISUAL_TRACK_MAX_PREDICTION_OFFSET_PX,
        )
        predicted_x = self.last_x + prediction_offset_x
        half_width = max(1.0, self.box_width * 0.5)
        half_height = max(1.0, self.box_height * 0.5)
        predicted_x = self._clamp(
            predicted_x,
            half_width,
            DISPLAY_SIZE[0] - 1 - half_width,
        )
        predicted_y = self._clamp(
            self.last_y,
            half_height,
            DISPLAY_SIZE[1] - 1 - half_height,
        )

        self.predicted_x = predicted_x
        self.predicted_y = predicted_y
        self.prediction_lead_ms = lead_ms
        if not has_measurement:
            self.status = "PREDICT"
        return (
            predicted_x,
            predicted_y,
            self.box_width,
            self.box_height,
            not has_measurement,
        )


class VisionPidController:
    """用小球位置和视觉速度直接计算摆杆角度，并执行有界在线调参。"""

    def __init__(self):
        # sequence_* 参数只属于 0 -> +5 -> -5 动作任务。
        self.position_gain = SEQUENCE_BALANCE_POSITION_KP_DEG_PER_CM
        self.integral_gain = SEQUENCE_BALANCE_INTEGRAL_KI_DEG_PER_CM_S
        self.velocity_gain = SEQUENCE_BALANCE_VELOCITY_KD_DEG_S_PER_CM
        self.static_angle_deg = SEQUENCE_BALANCE_STATIC_ANGLE_DEG
        # hold_* 参数只属于零点/任意点持续平衡任务。
        self.hold_position_gain = HOLD_BALANCE_POSITION_KP_DEG_PER_CM
        self.hold_integral_gain = HOLD_BALANCE_INTEGRAL_KI_DEG_PER_CM_S
        self.hold_velocity_gain = HOLD_BALANCE_VELOCITY_KD_DEG_S_PER_CM
        self.hold_static_angle_deg = HOLD_BALANCE_STATIC_ANGLE_DEG
        # active_* 记录本控制周期实际使用的参数，供 LCD/串口显示。
        self.active_profile = "--"
        self.active_position_gain = 0.0
        self.active_integral_gain = 0.0
        self.active_velocity_gain = 0.0
        self.active_static_angle_deg = 0.0
        self.last_tune_ms = None
        self.last_save_ms = None
        self.previous_error_cm = None
        self.stuck_since_ms = None
        self.static_error_since_ms = None
        self.static_comp_angle_deg = 0.0
        self.tuning_dirty = False
        self._load_tuning()
        self.reset()

    @staticmethod
    def _clamp(value, minimum, maximum):
        if value < minimum:
            return minimum
        if value > maximum:
            return maximum
        return value

    @staticmethod
    def _safe_speed_for_distance(distance_cm, brake_accel_cm_s2):
        distance_cm = max(0.0, float(distance_cm))
        brake_accel_cm_s2 = max(0.001, float(brake_accel_cm_s2))
        delay_s = max(0.0, TASK_PROFILE_ACTUATOR_DELAY_S)
        # distance = speed * delay + speed^2 / (2 * deceleration)
        delayed_speed_cm_s = brake_accel_cm_s2 * delay_s
        return max(
            0.0,
            math.sqrt(
                delayed_speed_cm_s * delayed_speed_cm_s
                + 2.0 * brake_accel_cm_s2 * distance_cm
            ) - delayed_speed_cm_s,
        )

    def _load_tuning(self):
        global TASK_HOLD_POSITION_TARGET_CM
        global TASK_HOLD_MAX_RETURN_SPEED_CM_S
        global TASK_HOLD_APPROACH_ZONE_CM
        global TASK_HOLD_APPROACH_SPEED_GAIN_CM_S_PER_CM
        global TASK_HOLD_OVERSPEED_COMP_GAIN_DEG_S_PER_CM
        global TASK_HOLD_HARD_SPEED_LIMIT_CM_S
        global TASK_HOLD_HARD_BRAKE_ANGLE_DEG
        global HOLD_POSITIVE_SIDE_STATIC_ANGLE_DEG
        global HOLD_POSITIVE_SIDE_MAX_ANGLE_DEG
        global HOLD_NEGATIVE_SIDE_STATIC_ANGLE_DEG
        global HOLD_NEGATIVE_SIDE_MAX_ANGLE_DEG
        global CART_START_FEEDFORWARD_ANGLE_DEG
        global CART_START_FEEDFORWARD_HOLD_MS
        global CART_START_FEEDFORWARD_FADE_MS
        global TASK_PLUS_DRIVE_MAX_ANGLE_DEG
        global TASK_PLUS_DRIVE_ANGLE_SLEW_DEG_S
        global TASK_PLUS_FINAL_PUSH_MIN_ANGLE_DEG
        global TASK_PLUS_FINAL_PUSH_STUCK_ANGLE_DEG
        global TASK_SEQUENCE_START_FEEDFORWARD_ANGLE_DEG
        global TASK_SEQUENCE_START_FEEDFORWARD_HOLD_MS
        global TASK_SEQUENCE_START_FEEDFORWARD_FADE_MS
        global TASK_SEQUENCE_START_FEEDFORWARD_RELEASE_SPEED_CM_S
        global TASK_SEQUENCE_START_FEEDFORWARD_RELEASE_PROGRESS_CM
        global TASK_SEQUENCE_START_FEEDFORWARD_SLEW_DEG_S
        global TASK_MINUS_DRIVE_MAX_ANGLE_DEG
        global TASK_MINUS_DRIVE_ANGLE_SLEW_DEG_S
        global TASK_MINUS_COAST_ANGLE_DEG
        global TASK_MINUS_APPROACH_BRAKE_ZONE_CM
        global TASK_MINUS_APPROACH_BRAKE_MAX_ANGLE_DEG
        global TASK_MINUS_TARGET_HOLD_ZONE_CM
        global TASK_MINUS_TARGET_REST_BIAS_ANGLE_DEG
        global TASK_MINUS_TARGET_MIN_ANGLE_DEG
        global TASK_MINUS_STUCK_PUSH_ANGLE_DEG
        global TASK_MINUS_TARGET_POSITION_GAIN
        global TASK_MINUS_TARGET_VELOCITY_BRAKE_GAIN
        try:
            with open(PID_TUNING_FILE, "r") as file_obj:
                data = ujson.loads(file_obj.read())
            if int(data.get("version", 0)) != PID_TUNING_VERSION:
                raise ValueError("PID tuning version mismatch")
            self.position_gain = self._clamp(
                float(data.get("sequence_position_gain_deg_per_cm", self.position_gain)),
                PID_POSITION_GAIN_MIN,
                PID_POSITION_GAIN_MAX,
            )
            self.integral_gain = float(
                data.get("sequence_integral_gain_deg_per_cm_s", self.integral_gain)
            )
            self.velocity_gain = self._clamp(
                float(data.get("sequence_velocity_gain_deg_s_per_cm", self.velocity_gain)),
                PID_VELOCITY_GAIN_MIN,
                PID_VELOCITY_GAIN_MAX,
            )
            self.static_angle_deg = self._clamp(
                float(data.get("sequence_static_angle_deg", self.static_angle_deg)),
                PID_STATIC_ANGLE_MIN,
                PID_STATIC_ANGLE_MAX,
            )
            self.hold_position_gain = self._clamp(
                float(data.get("hold_position_gain_deg_per_cm", self.hold_position_gain)),
                PID_POSITION_GAIN_MIN,
                PID_POSITION_GAIN_MAX,
            )
            self.hold_integral_gain = float(
                data.get("hold_integral_gain_deg_per_cm_s", self.hold_integral_gain)
            )
            self.hold_velocity_gain = self._clamp(
                float(data.get("hold_velocity_gain_deg_s_per_cm", self.hold_velocity_gain)),
                PID_VELOCITY_GAIN_MIN,
                PID_VELOCITY_GAIN_MAX,
            )
            self.hold_static_angle_deg = self._clamp(
                float(data.get("hold_static_angle_deg", self.hold_static_angle_deg)),
                PID_STATIC_ANGLE_MIN,
                PID_STATIC_ANGLE_MAX,
            )
            TASK_HOLD_POSITION_TARGET_CM = self._clamp(
                float(data.get("hold_position_target_cm", TASK_HOLD_POSITION_TARGET_CM)),
                -(ROD_LENGTH_CM * 0.5 - ROD_END_MARGIN_CM),
                ROD_LENGTH_CM * 0.5 - ROD_END_MARGIN_CM,
            )
            TASK_HOLD_MAX_RETURN_SPEED_CM_S = self._clamp(
                float(data.get("hold_max_return_speed_cm_s", TASK_HOLD_MAX_RETURN_SPEED_CM_S)),
                0.2,
                10.0,
            )
            TASK_HOLD_APPROACH_ZONE_CM = self._clamp(
                float(data.get("hold_approach_zone_cm", TASK_HOLD_APPROACH_ZONE_CM)),
                0.5,
                20.0,
            )
            TASK_HOLD_APPROACH_SPEED_GAIN_CM_S_PER_CM = self._clamp(
                float(data.get("hold_approach_gain_cm_s_per_cm", TASK_HOLD_APPROACH_SPEED_GAIN_CM_S_PER_CM)),
                0.02,
                3.0,
            )
            TASK_HOLD_OVERSPEED_COMP_GAIN_DEG_S_PER_CM = self._clamp(
                float(data.get("hold_overspeed_gain_deg_s_per_cm", TASK_HOLD_OVERSPEED_COMP_GAIN_DEG_S_PER_CM)),
                0.0,
                4.0,
            )
            TASK_HOLD_HARD_SPEED_LIMIT_CM_S = self._clamp(
                float(data.get("hold_hard_speed_limit_cm_s", TASK_HOLD_HARD_SPEED_LIMIT_CM_S)),
                0.5,
                10.0,
            )
            TASK_HOLD_HARD_BRAKE_ANGLE_DEG = self._clamp(
                float(data.get("hold_hard_brake_angle_deg", TASK_HOLD_HARD_BRAKE_ANGLE_DEG)),
                0.2,
                4.0,
            )
            HOLD_POSITIVE_SIDE_STATIC_ANGLE_DEG = self._clamp(
                float(data.get("hold_positive_static_angle_deg", HOLD_POSITIVE_SIDE_STATIC_ANGLE_DEG)),
                0.0,
                2.5,
            )
            HOLD_POSITIVE_SIDE_MAX_ANGLE_DEG = self._clamp(
                float(data.get("hold_positive_max_angle_deg", HOLD_POSITIVE_SIDE_MAX_ANGLE_DEG)),
                0.2,
                4.0,
            )
            HOLD_NEGATIVE_SIDE_STATIC_ANGLE_DEG = self._clamp(
                float(data.get("hold_negative_static_angle_deg", HOLD_NEGATIVE_SIDE_STATIC_ANGLE_DEG)),
                0.0,
                2.5,
            )
            HOLD_NEGATIVE_SIDE_MAX_ANGLE_DEG = self._clamp(
                float(data.get("hold_negative_max_angle_deg", HOLD_NEGATIVE_SIDE_MAX_ANGLE_DEG)),
                0.2,
                4.0,
            )
            CART_START_FEEDFORWARD_ANGLE_DEG = self._clamp(
                float(data.get("cart_start_ff_angle_deg", CART_START_FEEDFORWARD_ANGLE_DEG)),
                -4.0,
                4.0,
            )
            CART_START_FEEDFORWARD_HOLD_MS = int(self._clamp(
                float(data.get("cart_start_ff_hold_ms", CART_START_FEEDFORWARD_HOLD_MS)),
                0.0,
                5000.0,
            ))
            CART_START_FEEDFORWARD_FADE_MS = int(self._clamp(
                float(data.get("cart_start_ff_fade_ms", CART_START_FEEDFORWARD_FADE_MS)),
                0.0,
                5000.0,
            ))
            TASK_PLUS_DRIVE_MAX_ANGLE_DEG = self._clamp(
                TASK_PLUS_DRIVE_MAX_ANGLE_DEG,
                0.2,
                8.0,
            )
            TASK_PLUS_DRIVE_ANGLE_SLEW_DEG_S = self._clamp(
                float(data.get("sequence_plus_slew_deg_s", TASK_PLUS_DRIVE_ANGLE_SLEW_DEG_S)),
                0.5,
                40.0,
            )
            TASK_PLUS_FINAL_PUSH_MIN_ANGLE_DEG = self._clamp(
                float(data.get("sequence_plus_push_angle_deg", TASK_PLUS_FINAL_PUSH_MIN_ANGLE_DEG)),
                0.0,
                8.0,
            )
            TASK_PLUS_FINAL_PUSH_STUCK_ANGLE_DEG = self._clamp(
                float(data.get("sequence_plus_stuck_angle_deg", TASK_PLUS_FINAL_PUSH_STUCK_ANGLE_DEG)),
                0.0,
                8.0,
            )
            TASK_SEQUENCE_START_FEEDFORWARD_ANGLE_DEG = self._clamp(
                float(data.get("sequence_start_ff_angle_deg", TASK_SEQUENCE_START_FEEDFORWARD_ANGLE_DEG)),
                0.0,
                4.0,
            )
            TASK_SEQUENCE_START_FEEDFORWARD_HOLD_MS = int(self._clamp(
                float(data.get("sequence_start_ff_hold_ms", TASK_SEQUENCE_START_FEEDFORWARD_HOLD_MS)),
                0.0,
                1000.0,
            ))
            TASK_SEQUENCE_START_FEEDFORWARD_FADE_MS = int(self._clamp(
                float(data.get("sequence_start_ff_fade_ms", TASK_SEQUENCE_START_FEEDFORWARD_FADE_MS)),
                1.0,
                2000.0,
            ))
            TASK_SEQUENCE_START_FEEDFORWARD_RELEASE_SPEED_CM_S = self._clamp(
                float(data.get("sequence_start_ff_release_speed_cm_s", TASK_SEQUENCE_START_FEEDFORWARD_RELEASE_SPEED_CM_S)),
                0.10,
                5.0,
            )
            TASK_SEQUENCE_START_FEEDFORWARD_RELEASE_PROGRESS_CM = self._clamp(
                float(data.get("sequence_start_ff_release_progress_cm", TASK_SEQUENCE_START_FEEDFORWARD_RELEASE_PROGRESS_CM)),
                0.05,
                2.0,
            )
            TASK_SEQUENCE_START_FEEDFORWARD_SLEW_DEG_S = self._clamp(
                float(data.get("sequence_start_ff_slew_deg_s", TASK_SEQUENCE_START_FEEDFORWARD_SLEW_DEG_S)),
                1.0,
                40.0,
            )
            TASK_MINUS_DRIVE_MAX_ANGLE_DEG = self._clamp(
                TASK_MINUS_DRIVE_MAX_ANGLE_DEG,
                0.2,
                5.4,
            )
            TASK_MINUS_DRIVE_ANGLE_SLEW_DEG_S = self._clamp(
                float(data.get("sequence_minus_slew_deg_s", TASK_MINUS_DRIVE_ANGLE_SLEW_DEG_S)),
                0.5,
                40.0,
            )
            TASK_MINUS_COAST_ANGLE_DEG = self._clamp(
                float(data.get("sequence_minus_coast_angle_deg", TASK_MINUS_COAST_ANGLE_DEG)),
                -2.0,
                0.0,
            )
            TASK_MINUS_APPROACH_BRAKE_ZONE_CM = self._clamp(
                float(data.get("sequence_minus_brake_zone_cm", TASK_MINUS_APPROACH_BRAKE_ZONE_CM)),
                0.0,
                10.0,
            )
            TASK_MINUS_APPROACH_BRAKE_MAX_ANGLE_DEG = self._clamp(
                float(data.get("sequence_minus_brake_angle_deg", TASK_MINUS_APPROACH_BRAKE_MAX_ANGLE_DEG)),
                0.0,
                4.0,
            )
            TASK_MINUS_TARGET_HOLD_ZONE_CM = self._clamp(
                float(data.get("sequence_minus_hold_zone_cm", TASK_MINUS_TARGET_HOLD_ZONE_CM)),
                0.1,
                5.0,
            )
            TASK_MINUS_TARGET_REST_BIAS_ANGLE_DEG = 0.0
            TASK_MINUS_TARGET_MIN_ANGLE_DEG = self._clamp(
                float(data.get("sequence_minus_min_angle_deg", TASK_MINUS_TARGET_MIN_ANGLE_DEG)),
                -5.0,
                0.0,
            )
            TASK_MINUS_STUCK_PUSH_ANGLE_DEG = self._clamp(
                TASK_MINUS_STUCK_PUSH_ANGLE_DEG,
                -5.0,
                0.0,
            )
            TASK_MINUS_TARGET_POSITION_GAIN = 0.0
            TASK_MINUS_TARGET_VELOCITY_BRAKE_GAIN = self._clamp(
                TASK_MINUS_TARGET_VELOCITY_BRAKE_GAIN,
                0.0,
                2.0,
            )
            print(
                "SEQUENCE PID loaded: Kp=%.3f Ki=%.3f Kd=%.3f static=%.3f deg"
                % (
                    self.position_gain,
                    self.integral_gain,
                    self.velocity_gain,
                    self.static_angle_deg,
                )
            )
            print(
                "HOLD PID loaded: Kp=%.3f Ki=%.3f Kd=%.3f static=%.3f deg"
                % (
                    self.hold_position_gain,
                    self.hold_integral_gain,
                    self.hold_velocity_gain,
                    self.hold_static_angle_deg,
                )
            )
        except BaseException:
            print(
                "SEQUENCE PID defaults: Kp=%.3f Ki=%.3f Kd=%.3f static=%.3f deg"
                % (
                    self.position_gain,
                    self.integral_gain,
                    self.velocity_gain,
                    self.static_angle_deg,
                )
            )
            print(
                "HOLD PID defaults: Kp=%.3f Ki=%.3f Kd=%.3f static=%.3f deg"
                % (
                    self.hold_position_gain,
                    self.hold_integral_gain,
                    self.hold_velocity_gain,
                    self.hold_static_angle_deg,
                )
            )

    def _save_tuning(self):
        try:
            data = {
                "version": PID_TUNING_VERSION,
                "sequence_position_gain_deg_per_cm": self.position_gain,
                "sequence_integral_gain_deg_per_cm_s": self.integral_gain,
                "sequence_velocity_gain_deg_s_per_cm": self.velocity_gain,
                "sequence_static_angle_deg": self.static_angle_deg,
                "hold_position_gain_deg_per_cm": self.hold_position_gain,
                "hold_integral_gain_deg_per_cm_s": self.hold_integral_gain,
                "hold_velocity_gain_deg_s_per_cm": self.hold_velocity_gain,
                "hold_static_angle_deg": self.hold_static_angle_deg,
                "hold_position_target_cm": TASK_HOLD_POSITION_TARGET_CM,
                "hold_max_return_speed_cm_s": TASK_HOLD_MAX_RETURN_SPEED_CM_S,
                "hold_approach_zone_cm": TASK_HOLD_APPROACH_ZONE_CM,
                "hold_approach_gain_cm_s_per_cm": TASK_HOLD_APPROACH_SPEED_GAIN_CM_S_PER_CM,
                "hold_overspeed_gain_deg_s_per_cm": TASK_HOLD_OVERSPEED_COMP_GAIN_DEG_S_PER_CM,
                "hold_hard_speed_limit_cm_s": TASK_HOLD_HARD_SPEED_LIMIT_CM_S,
                "hold_hard_brake_angle_deg": TASK_HOLD_HARD_BRAKE_ANGLE_DEG,
                "hold_positive_static_angle_deg": HOLD_POSITIVE_SIDE_STATIC_ANGLE_DEG,
                "hold_positive_max_angle_deg": HOLD_POSITIVE_SIDE_MAX_ANGLE_DEG,
                "hold_negative_static_angle_deg": HOLD_NEGATIVE_SIDE_STATIC_ANGLE_DEG,
                "hold_negative_max_angle_deg": HOLD_NEGATIVE_SIDE_MAX_ANGLE_DEG,
                "cart_start_ff_angle_deg": CART_START_FEEDFORWARD_ANGLE_DEG,
                "cart_start_ff_hold_ms": CART_START_FEEDFORWARD_HOLD_MS,
                "cart_start_ff_fade_ms": CART_START_FEEDFORWARD_FADE_MS,
                "sequence_plus_max_angle_deg": TASK_PLUS_DRIVE_MAX_ANGLE_DEG,
                "sequence_plus_slew_deg_s": TASK_PLUS_DRIVE_ANGLE_SLEW_DEG_S,
                "sequence_plus_push_angle_deg": TASK_PLUS_FINAL_PUSH_MIN_ANGLE_DEG,
                "sequence_plus_stuck_angle_deg": TASK_PLUS_FINAL_PUSH_STUCK_ANGLE_DEG,
                "sequence_start_ff_angle_deg": TASK_SEQUENCE_START_FEEDFORWARD_ANGLE_DEG,
                "sequence_start_ff_hold_ms": TASK_SEQUENCE_START_FEEDFORWARD_HOLD_MS,
                "sequence_start_ff_fade_ms": TASK_SEQUENCE_START_FEEDFORWARD_FADE_MS,
                "sequence_start_ff_release_speed_cm_s": TASK_SEQUENCE_START_FEEDFORWARD_RELEASE_SPEED_CM_S,
                "sequence_start_ff_release_progress_cm": TASK_SEQUENCE_START_FEEDFORWARD_RELEASE_PROGRESS_CM,
                "sequence_start_ff_slew_deg_s": TASK_SEQUENCE_START_FEEDFORWARD_SLEW_DEG_S,
                "sequence_minus_max_angle_deg": TASK_MINUS_DRIVE_MAX_ANGLE_DEG,
                "sequence_minus_slew_deg_s": TASK_MINUS_DRIVE_ANGLE_SLEW_DEG_S,
                "sequence_minus_coast_angle_deg": TASK_MINUS_COAST_ANGLE_DEG,
                "sequence_minus_brake_zone_cm": TASK_MINUS_APPROACH_BRAKE_ZONE_CM,
                "sequence_minus_brake_angle_deg": TASK_MINUS_APPROACH_BRAKE_MAX_ANGLE_DEG,
                "sequence_minus_hold_zone_cm": TASK_MINUS_TARGET_HOLD_ZONE_CM,
                "sequence_minus_bias_angle_deg": TASK_MINUS_TARGET_REST_BIAS_ANGLE_DEG,
                "sequence_minus_min_angle_deg": TASK_MINUS_TARGET_MIN_ANGLE_DEG,
                "sequence_minus_stuck_angle_deg": TASK_MINUS_STUCK_PUSH_ANGLE_DEG,
                "sequence_minus_position_gain": TASK_MINUS_TARGET_POSITION_GAIN,
                "sequence_minus_velocity_gain": TASK_MINUS_TARGET_VELOCITY_BRAKE_GAIN,
            }
            with open(PID_TUNING_FILE, "w") as file_obj:
                file_obj.write(ujson.dumps(data))
            self.tuning_dirty = False
            print("PID tuning saved:", PID_TUNING_FILE)
            return True
        except BaseException as error:
            print("PID tuning save error:", error)
            return False

    def maybe_save(self, now_ms, force=False):
        if not self.tuning_dirty:
            return
        if force:
            self._save_tuning()
            return
        if self.last_save_ms is None:
            self.last_save_ms = now_ms
            return
        if time.ticks_diff(now_ms, self.last_save_ms) >= PID_AUTO_TUNE_SAVE_INTERVAL_MS:
            if self._save_tuning():
                self.last_save_ms = now_ms

    def reset(self):
        self.integral_cm_s = 0.0
        self.output_angle_deg = 0.0
        self.previous_output_hard_saturated = False
        self.previous_error_cm = None
        self.stuck_since_ms = None
        self.static_error_since_ms = None
        self.static_comp_angle_deg = 0.0
        self.hold_speed_limit_cm_s = 0.0
        self.hold_target_velocity_cm_s = 0.0
        self.hold_overspeed_cm_s = 0.0
        self.hold_hard_brake_active = False
        self.profile_speed_limit_cm_s = 0.0
        self.profile_target_velocity_cm_s = 0.0
        self.profile_overspeed_cm_s = 0.0
        self.profile_brake_active = False
        self.profile_velocity_cm_s = 0.0
        self.profile_endpoint_cm = 0.0
        self.start_feedforward_start_ms = None
        self.sequence_start_ff_start_ms = None
        self.sequence_start_ff_start_position_cm = None
        self.sequence_start_ff_active = False
        self.minus_rebound_seen = False
        self.sequence_start_ff_angle_deg = 0.0
        self.settled_candidate_since_ms = None
        self.settled_candidate_target_cm = None
        self.settled_hold_active = False
        self.settled_hold_target_cm = None
        self.settled_hold_angle_deg = 0.0

    def trigger_start_feedforward(self, now_ms):
        self.start_feedforward_start_ms = now_ms

    def trigger_sequence_start_feedforward(self, position_cm, now_ms):
        if not TASK_SEQUENCE_START_FEEDFORWARD_ENABLED:
            return
        self.sequence_start_ff_start_ms = now_ms
        self.sequence_start_ff_start_position_cm = (
            None if position_cm is None else float(position_cm)
        )
        self.sequence_start_ff_active = True
        self.sequence_start_ff_angle_deg = (
            TASK_SEQUENCE_START_FEEDFORWARD_ANGLE_DEG
        )

    def _start_feedforward_angle(self, now_ms):
        if (
            not CART_START_FEEDFORWARD_ENABLED
            or self.start_feedforward_start_ms is None
        ):
            return 0.0
        elapsed_ms = time.ticks_diff(
            now_ms, self.start_feedforward_start_ms
        )
        if elapsed_ms < CART_START_FEEDFORWARD_HOLD_MS:
            return CART_START_FEEDFORWARD_ANGLE_DEG
        fade_elapsed_ms = elapsed_ms - CART_START_FEEDFORWARD_HOLD_MS
        if fade_elapsed_ms >= CART_START_FEEDFORWARD_FADE_MS:
            self.start_feedforward_start_ms = None
            return 0.0
        fade_ratio = 1.0 - (
            float(fade_elapsed_ms) / CART_START_FEEDFORWARD_FADE_MS
        )
        return CART_START_FEEDFORWARD_ANGLE_DEG * fade_ratio

    def _sequence_start_feedforward_angle(
        self,
        position_cm,
        velocity_cm_s,
        error_cm,
        now_ms,
    ):
        if (
            not TASK_SEQUENCE_START_FEEDFORWARD_ENABLED
            or not self.sequence_start_ff_active
            or self.sequence_start_ff_start_ms is None
        ):
            self.sequence_start_ff_angle_deg = 0.0
            return 0.0
        if self.sequence_start_ff_start_position_cm is None:
            self.sequence_start_ff_start_position_cm = float(position_cm)

        elapsed_ms = max(
            0,
            time.ticks_diff(now_ms, self.sequence_start_ff_start_ms),
        )
        progress_cm = (
            float(position_cm) - self.sequence_start_ff_start_position_cm
        )
        moving_confirmed = (
            float(velocity_cm_s)
            >= TASK_SEQUENCE_START_FEEDFORWARD_RELEASE_SPEED_CM_S
            or progress_cm
            >= TASK_SEQUENCE_START_FEEDFORWARD_RELEASE_PROGRESS_CM
        )
        wrong_direction = float(velocity_cm_s) < -0.30
        near_target = float(error_cm) <= TASK_PLUS_APPROACH_SPEED_ZONE_CM
        if wrong_direction or near_target:
            self.sequence_start_ff_active = False
            self.sequence_start_ff_start_ms = None
            self.sequence_start_ff_angle_deg = 0.0
            return 0.0

        fade_start_ms = TASK_SEQUENCE_START_FEEDFORWARD_HOLD_MS
        if moving_confirmed:
            fade_start_ms = 0
        fade_elapsed_ms = elapsed_ms - fade_start_ms
        if fade_elapsed_ms <= 0:
            angle_deg = TASK_SEQUENCE_START_FEEDFORWARD_ANGLE_DEG
        elif fade_elapsed_ms >= TASK_SEQUENCE_START_FEEDFORWARD_FADE_MS:
            self.sequence_start_ff_active = False
            self.sequence_start_ff_start_ms = None
            angle_deg = 0.0
        else:
            fade_ratio = 1.0 - (
                float(fade_elapsed_ms)
                / TASK_SEQUENCE_START_FEEDFORWARD_FADE_MS
            )
            angle_deg = (
                TASK_SEQUENCE_START_FEEDFORWARD_ANGLE_DEG * fade_ratio
            )
        self.sequence_start_ff_angle_deg = angle_deg
        return angle_deg

    def clear_integral(self):
        self.integral_cm_s = 0.0
        self.previous_output_hard_saturated = False
        self.previous_error_cm = None
        self.stuck_since_ms = None
        self.static_error_since_ms = None
        self.profile_velocity_cm_s = 0.0
        self.sequence_start_ff_start_ms = None
        self.sequence_start_ff_start_position_cm = None
        self.sequence_start_ff_active = False
        self.sequence_start_ff_angle_deg = 0.0
        self.settled_candidate_since_ms = None
        self.settled_candidate_target_cm = None
        self.settled_hold_active = False
        self.settled_hold_target_cm = None
        self.settled_hold_angle_deg = 0.0

    def _slew_profile_velocity(
        self,
        target_velocity_cm_s,
        dt_s,
        accel_cm_s2,
        decel_cm_s2,
    ):
        delta_cm_s = float(target_velocity_cm_s) - self.profile_velocity_cm_s
        if abs(delta_cm_s) < 0.0001:
            self.profile_velocity_cm_s = float(target_velocity_cm_s)
            return self.profile_velocity_cm_s
        reversing = (
            self.profile_velocity_cm_s * float(target_velocity_cm_s) < 0.0
        )
        rate_cm_s2 = (
            decel_cm_s2
            if reversing or abs(target_velocity_cm_s) < abs(self.profile_velocity_cm_s)
            else accel_cm_s2
        )
        maximum_step = max(0.0, rate_cm_s2 * dt_s)
        self.profile_velocity_cm_s += self._clamp(
            delta_cm_s,
            -maximum_step,
            maximum_step,
        )
        return self.profile_velocity_cm_s

    def slew_to_zero(self, dt_s):
        self.static_error_since_ms = None
        self.static_comp_angle_deg = 0.0
        maximum_step = BALANCE_ANGLE_SLEW_DEG_S * max(0.0, dt_s)
        if self.output_angle_deg > maximum_step:
            self.output_angle_deg -= maximum_step
        elif self.output_angle_deg < -maximum_step:
            self.output_angle_deg += maximum_step
        else:
            self.output_angle_deg = 0.0
        return self.output_angle_deg * BALANCE_PULSES_PER_DEG

    def _tune_if_needed(self, error_cm, velocity_cm_s, now_ms):
        if not PID_AUTO_TUNE_ENABLED:
            self.previous_error_cm = error_cm
            return

        can_tune = (
            self.last_tune_ms is None
            or time.ticks_diff(now_ms, self.last_tune_ms)
            >= PID_AUTO_TUNE_MIN_INTERVAL_MS
        )
        crossed_target_fast = (
            self.previous_error_cm is not None
            and error_cm * self.previous_error_cm < 0.0
            and abs(velocity_cm_s) >= PID_AUTO_TUNE_CROSSING_SPEED_CM_S
        )
        if can_tune and crossed_target_fast:
            # 高速穿过目标代表阻尼不足：轻微降低位置增益并增加速度阻尼。
            self.position_gain = self._clamp(
                self.position_gain * 0.97,
                PID_POSITION_GAIN_MIN,
                PID_POSITION_GAIN_MAX,
            )
            self.velocity_gain = self._clamp(
                self.velocity_gain * 1.06,
                PID_VELOCITY_GAIN_MIN,
                PID_VELOCITY_GAIN_MAX,
            )
            self.last_tune_ms = now_ms
            self.tuning_dirty = True
            self.stuck_since_ms = None
            print(
                "AUTO BALANCE damping: Kp=%.3f Kd=%.3f"
                % (self.position_gain, self.velocity_gain)
            )
        elif (
            abs(error_cm) >= PID_AUTO_TUNE_STUCK_ERROR_CM
            and abs(velocity_cm_s) <= PID_AUTO_TUNE_STUCK_SPEED_CM_S
        ):
            if self.stuck_since_ms is None:
                self.stuck_since_ms = now_ms
            elif (
                can_tune
                and time.ticks_diff(now_ms, self.stuck_since_ms)
                >= PID_AUTO_TUNE_STUCK_TIME_MS
            ):
                # 长时间有误差但不动，缓慢增加位置增益和机构破静摩擦角度。
                self.position_gain = self._clamp(
                    self.position_gain * 1.04,
                    PID_POSITION_GAIN_MIN,
                    PID_POSITION_GAIN_MAX,
                )
                self.static_angle_deg = self._clamp(
                    self.static_angle_deg + 0.02,
                    PID_STATIC_ANGLE_MIN,
                    PID_STATIC_ANGLE_MAX,
                )
                self.last_tune_ms = now_ms
                self.tuning_dirty = True
                self.stuck_since_ms = now_ms
                print(
                    "AUTO BALANCE drive: Kp=%.3f static=%.3f deg"
                    % (self.position_gain, self.static_angle_deg)
                )
        else:
            self.stuck_since_ms = None
        self.previous_error_cm = error_cm

    def step(
        self,
        target_cm,
        position_cm,
        velocity_cm_s,
        dt_s,
        now_ms,
        allow_auto_tune,
    ):
        if dt_s <= 0.0:
            return self.output_angle_deg * BALANCE_PULSES_PER_DEG

        error_cm = float(target_cm) - float(position_cm)
        velocity_cm_s = float(velocity_cm_s)
        self.profile_endpoint_cm = float(target_cm)
        self.profile_speed_limit_cm_s = 0.0
        self.profile_target_velocity_cm_s = 0.0
        self.profile_overspeed_cm_s = 0.0
        self.profile_brake_active = False
        sequence_task_active = (
            BALANCE_TASK_ENABLED
            and BALANCE_TASK_MODE == TASK_MODE_SEQUENCE
        )
        hold_task_active = (
            BALANCE_TASK_ENABLED
            and BALANCE_TASK_MODE in (
                TASK_MODE_HOLD_ZERO,
                TASK_MODE_HOLD_POSITION,
            )
        )
        return_to_start_active = (
            sequence_task_active
            and TASK_RETURN_ENABLED
            and abs(float(target_cm) - TASK_START_POSITION_CM) < 0.001
        )
        minus_move_active = (
            sequence_task_active
            and abs(float(target_cm) - TASK_MINUS_TARGET_CM) < 0.001
        )
        if not minus_move_active:
            self.minus_rebound_seen = False
        elif position_cm <= TASK_MINUS_TARGET_CM:
            self.minus_rebound_seen = True
        plus_move_active = (
            sequence_task_active
            and abs(float(target_cm) - TASK_PLUS_TARGET_CM) < 0.001
        )
        sequence_start_ff_angle_deg = 0.0
        if plus_move_active:
            sequence_start_ff_angle_deg = (
                self._sequence_start_feedforward_angle(
                    position_cm,
                    velocity_cm_s,
                    error_cm,
                    now_ms,
                )
            )
        minus_correction_active = (
            minus_move_active
            and abs(error_cm) <= TASK_MINUS_CORRECTION_ZONE_CM
        )
        if hold_task_active:
            position_gain = self.hold_position_gain
            integral_gain = self.hold_integral_gain
            velocity_gain = self.hold_velocity_gain
            static_angle_deg = self.hold_static_angle_deg
            self.active_profile = "HOLD"
        else:
            position_gain = self.position_gain
            integral_gain = self.integral_gain
            velocity_gain = self.velocity_gain
            static_angle_deg = self.static_angle_deg
            self.active_profile = "SEQUENCE"
        static_error_cm = BALANCE_STATIC_ERROR_CM
        static_apply_delay_ms = BALANCE_STATIC_APPLY_DELAY_MS
        static_slew_deg_s = BALANCE_STATIC_SLEW_DEG_S
        max_angle_deg = BALANCE_MAX_ANGLE_DEG
        angle_slew_deg_s = BALANCE_ANGLE_SLEW_DEG_S
        position_deadband_cm = BALANCE_POSITION_DEADBAND_CM
        speed_deadband_cm_s = BALANCE_SPEED_DEADBAND_CM_S
        if hold_task_active:
            position_deadband_cm = HOLD_POSITION_DEADBAND_CM
            speed_deadband_cm_s = HOLD_SPEED_DEADBAND_CM_S
        if return_to_start_active:
            position_gain = TASK_RETURN_POSITION_KP_DEG_PER_CM
            integral_gain = TASK_RETURN_INTEGRAL_KI_DEG_PER_CM_S
            velocity_gain = TASK_RETURN_VELOCITY_KD_DEG_S_PER_CM
            static_angle_deg = TASK_RETURN_STATIC_ANGLE_DEG
            static_apply_delay_ms = TASK_RETURN_STATIC_APPLY_DELAY_MS
            static_slew_deg_s = TASK_RETURN_STATIC_SLEW_DEG_S
            max_angle_deg = TASK_RETURN_MAX_ANGLE_DEG
            angle_slew_deg_s = TASK_RETURN_ANGLE_SLEW_DEG_S
            position_deadband_cm = TASK_RETURN_POSITION_DEADBAND_CM
            speed_deadband_cm_s = TASK_RETURN_SPEED_DEADBAND_CM_S
            if abs(error_cm) <= TASK_RETURN_BRAKE_ZONE_CM:
                position_gain = TASK_RETURN_BRAKE_POSITION_KP_DEG_PER_CM
                integral_gain = TASK_RETURN_BRAKE_INTEGRAL_KI_DEG_PER_CM_S
                velocity_gain = TASK_RETURN_BRAKE_VELOCITY_KD_DEG_S_PER_CM
                static_error_cm = TASK_RETURN_BRAKE_STATIC_ERROR_CM
                static_angle_deg = TASK_RETURN_BRAKE_STATIC_ANGLE_DEG
                static_apply_delay_ms = TASK_RETURN_BRAKE_STATIC_APPLY_DELAY_MS
                static_slew_deg_s = TASK_RETURN_BRAKE_STATIC_SLEW_DEG_S
                max_angle_deg = TASK_RETURN_BRAKE_MAX_ANGLE_DEG
                angle_slew_deg_s = TASK_RETURN_BRAKE_ANGLE_SLEW_DEG_S
        if plus_move_active:
            position_gain *= TASK_PLUS_DRIVE_POSITION_GAIN_SCALE
            integral_gain *= TASK_PLUS_DRIVE_INTEGRAL_GAIN_SCALE
            velocity_gain *= TASK_PLUS_DRIVE_VELOCITY_GAIN_SCALE
            static_angle_deg *= TASK_PLUS_DRIVE_STATIC_ANGLE_SCALE
            max_angle_deg = TASK_PLUS_DRIVE_MAX_ANGLE_DEG
            angle_slew_deg_s = max(
                angle_slew_deg_s,
                TASK_PLUS_DRIVE_ANGLE_SLEW_DEG_S,
            )
        if minus_move_active:
            position_gain *= TASK_MINUS_DRIVE_POSITION_GAIN_SCALE
            integral_gain *= TASK_MINUS_DRIVE_INTEGRAL_GAIN_SCALE
            velocity_gain *= TASK_MINUS_DRIVE_VELOCITY_GAIN_SCALE
            static_angle_deg *= TASK_MINUS_DRIVE_STATIC_ANGLE_SCALE
            max_angle_deg = TASK_MINUS_DRIVE_MAX_ANGLE_DEG
            angle_slew_deg_s = min(
                angle_slew_deg_s,
                TASK_MINUS_DRIVE_ANGLE_SLEW_DEG_S,
            )
        if minus_correction_active:
            position_gain *= TASK_MINUS_POSITION_GAIN_SCALE
            integral_gain *= TASK_MINUS_INTEGRAL_GAIN_SCALE
            static_angle_deg *= TASK_MINUS_STATIC_ANGLE_SCALE
            static_apply_delay_ms = TASK_MINUS_STATIC_APPLY_DELAY_MS
            static_slew_deg_s = TASK_MINUS_STATIC_SLEW_DEG_S
        if hold_task_active and error_cm < -position_deadband_cm:
            velocity_gain *= HOLD_POSITIVE_SIDE_VELOCITY_GAIN_SCALE
            static_angle_deg = max(
                static_angle_deg,
                HOLD_POSITIVE_SIDE_STATIC_ANGLE_DEG,
            )
            max_angle_deg = max(
                max_angle_deg,
                HOLD_POSITIVE_SIDE_MAX_ANGLE_DEG,
            )
        elif hold_task_active and error_cm > position_deadband_cm:
            static_angle_deg = max(
                static_angle_deg,
                HOLD_NEGATIVE_SIDE_STATIC_ANGLE_DEG,
            )
            max_angle_deg = max(
                max_angle_deg,
                HOLD_NEGATIVE_SIDE_MAX_ANGLE_DEG,
            )
        self.active_position_gain = position_gain
        self.active_integral_gain = integral_gain
        self.active_velocity_gain = velocity_gain
        self.active_static_angle_deg = static_angle_deg
        settle_candidate = (
            abs(error_cm) <= position_deadband_cm
            and abs(velocity_cm_s) <= speed_deadband_cm_s
            and self.start_feedforward_start_ms is None
        )

        same_settled_target = (
            self.settled_hold_target_cm is not None
            and abs(float(target_cm) - self.settled_hold_target_cm) < 0.001
        )
        if self.settled_hold_active:
            position_release_cm = max(
                BALANCE_SETTLED_MIN_POSITION_RELEASE_CM,
                position_deadband_cm
                * BALANCE_SETTLED_POSITION_RELEASE_SCALE,
            )
            speed_release_cm_s = max(
                BALANCE_SETTLED_MIN_SPEED_RELEASE_CM_S,
                speed_deadband_cm_s
                * BALANCE_SETTLED_SPEED_RELEASE_SCALE,
            )
            if (
                not same_settled_target
                or abs(error_cm) > position_release_cm
                or abs(velocity_cm_s) > speed_release_cm_s
            ):
                self.settled_hold_active = False
                self.settled_hold_target_cm = None
                self.settled_candidate_since_ms = None
                self.settled_candidate_target_cm = None
        if not self.settled_hold_active:
            same_candidate_target = (
                self.settled_candidate_target_cm is not None
                and abs(
                    float(target_cm) - self.settled_candidate_target_cm
                ) < 0.001
            )
            if not settle_candidate:
                self.settled_candidate_since_ms = None
                self.settled_candidate_target_cm = None
            elif (
                self.settled_candidate_since_ms is None
                or not same_candidate_target
            ):
                self.settled_candidate_since_ms = now_ms
                self.settled_candidate_target_cm = float(target_cm)
            elif (
                time.ticks_diff(now_ms, self.settled_candidate_since_ms)
                >= BALANCE_SETTLED_CONFIRM_MS
            ):
                self.settled_hold_active = True
                self.settled_hold_target_cm = float(target_cm)
                self.settled_hold_angle_deg = self.output_angle_deg
                self.settled_candidate_since_ms = None
                self.settled_candidate_target_cm = None
                self.integral_cm_s = 0.0
                self.static_error_since_ms = None
                self.static_comp_angle_deg = 0.0
                self.profile_velocity_cm_s = 0.0
        settled = self.settled_hold_active

        if allow_auto_tune and not return_to_start_active and not hold_task_active:
            self._tune_if_needed(error_cm, velocity_cm_s, now_ms)
        else:
            self.previous_error_cm = None
            self.stuck_since_ms = None

        if settled:
            # 锁存的是小球真正停稳时已经验证过的总倾角，不再强制回到机械零角。
            self.integral_cm_s = 0.0
            desired_angle_deg = self.settled_hold_angle_deg
            self.hold_speed_limit_cm_s = 0.0
            self.hold_target_velocity_cm_s = 0.0
            self.hold_overspeed_cm_s = 0.0
            self.hold_hard_brake_active = False
        else:
            if (
                abs(error_cm) <= BALANCE_INTEGRAL_ZONE_CM
                and (
                    not hold_task_active
                    or abs(velocity_cm_s) <= TASK_HOLD_INTEGRAL_MAX_SPEED_CM_S
                )
                and not self.previous_output_hard_saturated
            ):
                self.integral_cm_s += error_cm * dt_s
                self.integral_cm_s = self._clamp(
                    self.integral_cm_s,
                    -BALANCE_INTEGRAL_LIMIT_CM_S,
                    BALANCE_INTEGRAL_LIMIT_CM_S,
                )
            else:
                self.integral_cm_s *= 0.98

            if hold_task_active:
                # 持续平衡模式：剩余距离决定安全回调速度，速度误差决定驱动/制动角。
                # 离目标远时先提高回调速度；接近终点时按制动距离降低速度上限。
                distance_to_target_cm = abs(error_cm)
                effective_distance_cm = max(
                    0.0,
                    distance_to_target_cm - position_deadband_cm,
                )
                safe_return_speed_cm_s = min(
                    TASK_HOLD_MAX_RETURN_SPEED_CM_S,
                    self._safe_speed_for_distance(
                        effective_distance_cm,
                        TASK_HOLD_BRAKE_ACCEL_CM_S2,
                    ),
                )
                # In the final approach, use a linear speed envelope so the
                # ball reaches zero with near-zero velocity instead of
                # carrying the high-distance braking speed through the target.
                if distance_to_target_cm <= TASK_HOLD_APPROACH_ZONE_CM:
                    safe_return_speed_cm_s = min(
                        safe_return_speed_cm_s,
                        effective_distance_cm
                        * TASK_HOLD_APPROACH_SPEED_GAIN_CM_S_PER_CM,
                    )
                if error_cm > 0.0:
                    direction_to_target = 1.0
                elif error_cm < 0.0:
                    direction_to_target = -1.0
                else:
                    direction_to_target = 0.0
                raw_target_velocity_cm_s = (
                    direction_to_target * safe_return_speed_cm_s
                )
                target_velocity_cm_s = self._slew_profile_velocity(
                    raw_target_velocity_cm_s,
                    dt_s,
                    TASK_HOLD_PROFILE_ACCEL_CM_S2,
                    TASK_HOLD_PROFILE_DECEL_CM_S2,
                )
                desired_angle_deg = (
                    position_gain * error_cm
                    + integral_gain * self.integral_cm_s
                    + velocity_gain
                    * (target_velocity_cm_s - velocity_cm_s)
                )

                # 若朝目标的实际速度已经超过剩余距离允许的安全速度，
                # 叠加反向角度提前制动，避免高速穿过目标后无法拉回。
                speed_toward_target_cm_s = (
                    direction_to_target * velocity_cm_s
                )
                overspeed_cm_s = max(
                    0.0,
                    speed_toward_target_cm_s - safe_return_speed_cm_s,
                )
                if overspeed_cm_s > 0.0:
                    desired_angle_deg -= (
                        direction_to_target
                        * TASK_HOLD_OVERSPEED_COMP_GAIN_DEG_S_PER_CM
                        * overspeed_cm_s
                    )
                    self.integral_cm_s *= 0.8

                # 严重超速时优先按当前运动方向做最大反向制动，
                # 暂时压过位置项，直至速度回到可控范围。
                hard_brake_active = (
                    abs(velocity_cm_s) >= TASK_HOLD_HARD_SPEED_LIMIT_CM_S
                    or overspeed_cm_s >= TASK_HOLD_HARD_OVERSPEED_CM_S
                )
                if hard_brake_active:
                    desired_angle_deg = (
                        -TASK_HOLD_HARD_BRAKE_ANGLE_DEG
                        if velocity_cm_s > 0.0
                        else TASK_HOLD_HARD_BRAKE_ANGLE_DEG
                    )
                    # Hard braking must be allowed to exceed the normal HOLD
                    # drive limit; otherwise the reverse command is clipped
                    # below the mechanism's static-friction threshold.
                    max_angle_deg = max(
                        max_angle_deg,
                        TASK_HOLD_HARD_BRAKE_ANGLE_DEG,
                    )
                    self.integral_cm_s = 0.0
                elif (
                    direction_to_target != 0.0
                    and speed_toward_target_cm_s >= 0.0
                    and desired_angle_deg * direction_to_target > 0.0
                ):
                    # 球已经朝目标运动时，限制继续加速的倾角；
                    # 反向制动以及球背离目标时的拦截仍可使用完整角度范围。
                    desired_angle_deg = self._clamp(
                        desired_angle_deg,
                        -TASK_HOLD_MAX_ACCELERATING_ANGLE_DEG,
                        TASK_HOLD_MAX_ACCELERATING_ANGLE_DEG,
                    )

                self.hold_speed_limit_cm_s = safe_return_speed_cm_s
                self.hold_target_velocity_cm_s = target_velocity_cm_s
                self.hold_overspeed_cm_s = overspeed_cm_s
                self.hold_hard_brake_active = hard_brake_active
                self.profile_speed_limit_cm_s = safe_return_speed_cm_s
                self.profile_target_velocity_cm_s = target_velocity_cm_s
                self.profile_overspeed_cm_s = overspeed_cm_s
                self.profile_brake_active = hard_brake_active
            else:
                self.hold_speed_limit_cm_s = 0.0
                self.hold_target_velocity_cm_s = 0.0
                self.hold_overspeed_cm_s = 0.0
                self.hold_hard_brake_active = False
                # 动作任务沿用原状态反馈：位置回正，速度项提前制动。
                desired_angle_deg = (
                    position_gain * error_cm
                    + integral_gain * self.integral_cm_s
                    - velocity_gain * velocity_cm_s
                )
        # 破静摩擦补偿只能在“持续有偏差且球基本不动”时逐步加入。
        # 高速阶段直接加入固定角会造成目标附近突跳，反而放大超调。
        static_target_angle_deg = 0.0
        static_speed_limit_cm_s = BALANCE_STATIC_MAX_SPEED_CM_S
        if abs(self.static_comp_angle_deg) > 0.05:
            static_speed_limit_cm_s = max(
                static_speed_limit_cm_s,
                BALANCE_STATIC_RELEASE_SPEED_CM_S,
            )
        if (
            not settled
            and abs(error_cm) >= static_error_cm
            and abs(velocity_cm_s) < static_speed_limit_cm_s
        ):
            if self.static_error_since_ms is None:
                self.static_error_since_ms = now_ms
            elif (
                time.ticks_diff(now_ms, self.static_error_since_ms)
                >= static_apply_delay_ms
            ):
                if desired_angle_deg > 0.0 or (
                    desired_angle_deg == 0.0 and error_cm > 0.0
                ):
                    static_target_angle_deg = static_angle_deg
                elif desired_angle_deg < 0.0 or error_cm < 0.0:
                    static_target_angle_deg = -static_angle_deg
        else:
            self.static_error_since_ms = None

        effective_static_slew_deg_s = static_slew_deg_s
        if static_target_angle_deg == 0.0:
            effective_static_slew_deg_s = max(
                effective_static_slew_deg_s,
                BALANCE_STATIC_RELEASE_SLEW_DEG_S,
            )
        maximum_static_step = effective_static_slew_deg_s * dt_s
        self.static_comp_angle_deg += self._clamp(
            static_target_angle_deg - self.static_comp_angle_deg,
            -maximum_static_step,
            maximum_static_step,
        )
        desired_angle_deg += self.static_comp_angle_deg

        if (
            not settled
            and plus_move_active
            and TASK_PLUS_SWITCH_TOLERANCE_CM
            < error_cm
            <= TASK_PLUS_FINAL_PUSH_ZONE_CM
            and abs(velocity_cm_s) <= TASK_PLUS_FINAL_PUSH_MAX_SPEED_CM_S
        ):
            final_push_angle_deg = TASK_PLUS_FINAL_PUSH_MIN_ANGLE_DEG
            if (
                error_cm <= TASK_PLUS_FINAL_PUSH_STUCK_ZONE_CM
                and abs(velocity_cm_s)
                <= TASK_PLUS_FINAL_PUSH_STUCK_MAX_SPEED_CM_S
            ):
                final_push_angle_deg = TASK_PLUS_FINAL_PUSH_STUCK_ANGLE_DEG
            desired_angle_deg = max(
                desired_angle_deg,
                final_push_angle_deg,
            )
        if not settled and plus_move_active and error_cm > 0.0:
            effective_plus_distance_cm = max(
                0.0,
                error_cm - TASK_PLUS_SWITCH_TOLERANCE_CM,
            )
            plus_brake_zone_active = (
                error_cm <= TASK_PLUS_APPROACH_SPEED_ZONE_CM
            )
            plus_speed_limit_cm_s = TASK_PLUS_MAX_SPEED_CM_S
            if plus_brake_zone_active:
                plus_speed_limit_cm_s = min(
                    plus_speed_limit_cm_s,
                    self._safe_speed_for_distance(
                        effective_plus_distance_cm,
                        TASK_PLUS_BRAKE_ACCEL_CM_S2,
                    ),
                    effective_plus_distance_cm
                    * TASK_PLUS_APPROACH_SPEED_GAIN_CM_S_PER_CM,
                )
            plus_overspeed_cm_s = 0.0
            if plus_brake_zone_active:
                plus_overspeed_cm_s = max(
                    0.0,
                    velocity_cm_s - plus_speed_limit_cm_s,
                )
            plus_target_velocity_cm_s = self._slew_profile_velocity(
                plus_speed_limit_cm_s,
                dt_s,
                TASK_SEQUENCE_PROFILE_ACCEL_CM_S2,
                TASK_SEQUENCE_PROFILE_DECEL_CM_S2,
            )
            desired_angle_deg = (
                position_gain * error_cm
                + integral_gain * self.integral_cm_s
                + velocity_gain
                * (plus_target_velocity_cm_s - velocity_cm_s)
            )
            self.profile_speed_limit_cm_s = plus_speed_limit_cm_s
            self.profile_target_velocity_cm_s = plus_target_velocity_cm_s
            self.profile_overspeed_cm_s = plus_overspeed_cm_s
            self.profile_brake_active = plus_overspeed_cm_s > 0.0
            if (
                not plus_brake_zone_active
                and abs(velocity_cm_s) <= TASK_PLUS_CRUISE_MIN_SPEED_CM_S
            ):
                desired_angle_deg = max(
                    desired_angle_deg,
                    TASK_PLUS_CRUISE_MIN_ANGLE_DEG,
                )
            if plus_overspeed_cm_s > 0.0:
                desired_angle_deg = min(
                    desired_angle_deg,
                    -TASK_PLUS_OVERSPEED_BRAKE_GAIN_DEG_S_PER_CM
                    * plus_overspeed_cm_s,
                )
            elif sequence_start_ff_angle_deg > 0.0:
                desired_angle_deg = max(
                    desired_angle_deg,
                    sequence_start_ff_angle_deg,
                )
            # After the initial breakaway, do not keep accelerating toward
            # +5 cm with the full drive angle.  Reverse braking remains
            # unrestricted so a measured overspeed is still stopped quickly.
            if (
                sequence_start_ff_angle_deg > 0.0
                and abs(velocity_cm_s) <= TASK_PLUS_BREAKAWAY_MAX_SPEED_CM_S
            ):
                desired_angle_deg = max(
                    desired_angle_deg,
                    TASK_PLUS_BREAKAWAY_ANGLE_DEG,
                )
            elif error_cm <= TASK_PLUS_APPROACH_MAX_ANGLE_ZONE_CM:
                desired_angle_deg = min(
                    desired_angle_deg,
                    TASK_PLUS_APPROACH_MAX_ANGLE_DEG,
                )
        # Dedicated +5 -> -5 motion profile. Positive angle is active braking
        # while the ball is moving toward -5; cap it separately from drive.
        minus_target_hold_active = False
        minus_overshoot_active = False
        if minus_move_active and not settled:
            if error_cm >= TASK_MINUS_OVERSHOOT_TRIGGER_CM:
                minus_overshoot_active = True
                if velocity_cm_s < 0.0:
                    if error_cm >= TASK_MINUS_DEEP_OVERSHOOT_TRIGGER_CM:
                        desired_angle_deg = (
                            TASK_MINUS_DEEP_OVERSHOOT_CORRECTION_ANGLE_DEG
                        )
                    else:
                        desired_angle_deg = (
                            TASK_MINUS_OVERSHOOT_CORRECTION_MAX_ANGLE_DEG
                        )
                else:
                    desired_angle_deg = 0.0
            elif abs(error_cm) <= TASK_MINUS_TARGET_HOLD_ZONE_CM:
                minus_target_hold_active = True
                desired_angle_deg = self._clamp(
                    TASK_MINUS_TARGET_REST_BIAS_ANGLE_DEG
                    + TASK_MINUS_TARGET_POSITION_GAIN * error_cm
                    - (
                        TASK_MINUS_TARGET_VELOCITY_BRAKE_GAIN * velocity_cm_s
                        if TASK_MINUS_FINAL_HOLD_MIN_CM
                        <= position_cm
                        <= TASK_MINUS_FINAL_HOLD_MAX_CM
                        else 0.0
                    ),
                    TASK_MINUS_TARGET_MIN_ANGLE_DEG,
                    TASK_MINUS_TARGET_MAX_ANGLE_DEG,
                )
                self.static_comp_angle_deg = 0.0
                self.integral_cm_s = 0.0
            elif error_cm < 0.0:
                effective_minus_distance_cm = max(
                    0.0,
                    abs(error_cm) - TASK_MINUS_TARGET_HOLD_ZONE_CM,
                )
                minus_brake_zone_active = (
                    abs(error_cm) <= TASK_MINUS_APPROACH_SPEED_ZONE_CM
                )
                minus_speed_limit_cm_s = TASK_MINUS_MAX_SPEED_CM_S
                if minus_brake_zone_active:
                    minus_speed_limit_cm_s = min(
                        minus_speed_limit_cm_s,
                        self._safe_speed_for_distance(
                            effective_minus_distance_cm,
                            TASK_MINUS_BRAKE_ACCEL_CM_S2,
                        ),
                        effective_minus_distance_cm
                        * TASK_MINUS_APPROACH_SPEED_GAIN_CM_S_PER_CM,
                    )
                minus_speed_toward_target_cm_s = max(
                    0.0,
                    -velocity_cm_s,
                )
                minus_overspeed_cm_s = 0.0
                if minus_brake_zone_active:
                    minus_overspeed_cm_s = max(
                        0.0,
                        minus_speed_toward_target_cm_s
                        - minus_speed_limit_cm_s,
                    )
                minus_target_velocity_cm_s = self._slew_profile_velocity(
                    -minus_speed_limit_cm_s,
                    dt_s,
                    TASK_SEQUENCE_PROFILE_ACCEL_CM_S2,
                    TASK_SEQUENCE_PROFILE_DECEL_CM_S2,
                )
                desired_angle_deg = (
                    position_gain * error_cm
                    + integral_gain * self.integral_cm_s
                    + velocity_gain
                    * (minus_target_velocity_cm_s - velocity_cm_s)
                )
                self.profile_speed_limit_cm_s = minus_speed_limit_cm_s
                self.profile_target_velocity_cm_s = minus_target_velocity_cm_s
                self.profile_overspeed_cm_s = minus_overspeed_cm_s
                self.profile_brake_active = minus_overspeed_cm_s > 0.0
                if minus_overspeed_cm_s > 0.0:
                    desired_angle_deg = max(
                        desired_angle_deg,
                        TASK_MINUS_OVERSPEED_BRAKE_GAIN_DEG_S_PER_CM
                        * minus_overspeed_cm_s,
                    )
                if (
                    minus_brake_zone_active
                    and minus_overspeed_cm_s >= TASK_MINUS_HARD_OVERSPEED_CM_S
                ):
                    desired_angle_deg = max(
                        desired_angle_deg,
                        TASK_MINUS_HARD_BRAKE_ANGLE_DEG,
                    )
            if (
                error_cm < -TASK_MINUS_TARGET_HOLD_ZONE_CM
                and abs(velocity_cm_s) <= TASK_MINUS_STUCK_PUSH_MAX_SPEED_CM_S
            ):
                desired_angle_deg = min(
                    desired_angle_deg,
                    TASK_MINUS_STUCK_PUSH_ANGLE_DEG,
                )

        if hold_task_active:
            desired_angle_deg += self._start_feedforward_angle(now_ms)

        # On the approach side of -5 cm, do not maintain the normal
        # rightward velocity-braking angle. The dedicated -5.5 cm correction
        # remains active after the ball passes the target.
        if (
            minus_move_active
            and error_cm <= 0.0
            and abs(error_cm) <= TASK_MINUS_APPROACH_SPEED_ZONE_CM
        ):
            desired_angle_deg = min(desired_angle_deg, 0.0)

        # During the +5 -> -5 cruise, never apply a positive (rightward)
        # braking angle before the ball enters the near-target approach zone.
        # The near-target profile above is the only place it is allowed.
        if (
            minus_move_active
            and error_cm < -TASK_MINUS_APPROACH_SPEED_ZONE_CM
        ):
            desired_angle_deg = TASK_MINUS_CRUISE_MIN_ANGLE_DEG

        limited_angle_deg = self._clamp(
            desired_angle_deg,
            -max_angle_deg,
            max_angle_deg,
        )
        self.previous_output_hard_saturated = (
            abs(desired_angle_deg) > max_angle_deg
        )
        effective_angle_slew_deg_s = angle_slew_deg_s
        if hold_task_active and self.hold_hard_brake_active:
            effective_angle_slew_deg_s = max(
                effective_angle_slew_deg_s,
                TASK_HOLD_HARD_BRAKE_SLEW_DEG_S,
            )
        if minus_overshoot_active:
            effective_angle_slew_deg_s = TASK_MINUS_OVERSHOOT_ANGLE_SLEW_DEG_S
        elif minus_target_hold_active:
            effective_angle_slew_deg_s = TASK_MINUS_TARGET_HOLD_ANGLE_SLEW_DEG_S
        elif (
            minus_move_active
            and error_cm <= -9.5
            and velocity_cm_s > 0.0
            and limited_angle_deg < self.output_angle_deg
        ):
            effective_angle_slew_deg_s = max(
                effective_angle_slew_deg_s,
                TASK_PLUS_TO_MINUS_RELEASE_SLEW_DEG_S,
            )
        elif (
            minus_move_active
            and velocity_cm_s <= -BALANCE_BRAKE_MIN_SPEED_CM_S
            and limited_angle_deg > self.output_angle_deg
        ):
            effective_angle_slew_deg_s = max(
                effective_angle_slew_deg_s,
                TASK_MINUS_BRAKE_ANGLE_SLEW_DEG_S,
            )
        elif minus_move_active and self.profile_brake_active:
            effective_angle_slew_deg_s = max(
                effective_angle_slew_deg_s,
                30.0,
            )
        elif plus_move_active and sequence_start_ff_angle_deg > 0.0:
            effective_angle_slew_deg_s = max(
                effective_angle_slew_deg_s,
                TASK_SEQUENCE_START_FEEDFORWARD_SLEW_DEG_S,
            )
        elif (
            not return_to_start_active
            and abs(velocity_cm_s) >= BALANCE_BRAKE_MIN_SPEED_CM_S
            and limited_angle_deg * velocity_cm_s < 0.0
        ):
            brake_angle_slew_deg_s = BALANCE_BRAKE_ANGLE_SLEW_DEG_S
            if minus_move_active:
                brake_angle_slew_deg_s = TASK_MINUS_BRAKE_ANGLE_SLEW_DEG_S
            effective_angle_slew_deg_s = max(
                effective_angle_slew_deg_s,
                brake_angle_slew_deg_s,
            )
        maximum_step = effective_angle_slew_deg_s * dt_s
        output_delta = self._clamp(
            limited_angle_deg - self.output_angle_deg,
            -maximum_step,
            maximum_step,
        )
        self.output_angle_deg += output_delta
        if (
            minus_move_active
            and error_cm < -TASK_MINUS_APPROACH_SPEED_ZONE_CM
        ):
            # Hold a fixed cruise tilt until the -5 approach zone.  This
            # prevents the base PID from saturating the negative drive.
            self.output_angle_deg = TASK_MINUS_CRUISE_MIN_ANGLE_DEG
        if (
            minus_move_active
            and self.minus_rebound_seen
            and position_cm >= TASK_MINUS_REBOUND_BRAKE_START_CM
            and velocity_cm_s >= TASK_MINUS_REBOUND_BRAKE_MIN_SPEED_CM_S
        ):
            # Second damping stage for a rightward rebound after passing -5.
            self.output_angle_deg = TASK_MINUS_REBOUND_BRAKE_ANGLE_DEG
        if (
            minus_move_active
            and self.minus_rebound_seen
            and position_cm > (
                TASK_MINUS_TARGET_CM - TASK_MINUS_OVERSHOOT_TRIGGER_CM
            )
        ):
            # After the first pass through -5, permit rightward correction
            # only at or beyond the configured -5.5 cm threshold.
            self.output_angle_deg = min(self.output_angle_deg, 0.0)
        if (
            minus_move_active
            and self.minus_rebound_seen
            and not minus_overshoot_active
            and position_cm >= TASK_MINUS_FINAL_HOLD_MIN_CM
            and position_cm <= TASK_MINUS_FINAL_HOLD_MAX_CM
        ):
            # Return to the fixed hold angle smoothly after correction;
            # position and velocity PID remain disabled in this band.
            final_hold_target_angle_deg = (
                TASK_MINUS_FINAL_HOLD_ANGLE_DEG
                - TASK_MINUS_TARGET_VELOCITY_BRAKE_GAIN * velocity_cm_s
            )
            final_hold_step_deg = (
                TASK_MINUS_FINAL_HOLD_ANGLE_SLEW_DEG_S * dt_s
            )
            self.output_angle_deg += self._clamp(
                final_hold_target_angle_deg - self.output_angle_deg,
                -final_hold_step_deg,
                final_hold_step_deg,
            )
        return self.output_angle_deg * BALANCE_PULSES_PER_DEG


class BluetoothTuner:
    """UART1 文本调参接口；每条命令一行，绝不阻塞视觉主循环。"""

    def __init__(self, controller, motor_controller=None):
        fpioa = FPIOA()
        fpioa.set_function(BLUETOOTH_UART_TX_PIN, FPIOA.UART1_TXD)
        fpioa.set_function(BLUETOOTH_UART_RX_PIN, FPIOA.UART1_RXD)
        self.uart = UART(
            UART.UART1,
            baudrate=BLUETOOTH_UART_BAUDRATE,
            bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE,
        )
        self.controller = controller
        self.motor_controller = motor_controller
        self.rx_line = bytearray()
        self.tx_pending = bytearray()
        self.last_tx_ms = None
        self.command_count = 0
        self.error_count = 0
        print(
            "BLUETOOTH TUNER: GPIO%d TX, GPIO%d RX, %d baud"
            % (
                BLUETOOTH_UART_TX_PIN,
                BLUETOOTH_UART_RX_PIN,
                BLUETOOTH_UART_BAUDRATE,
            )
        )
        self._reply("OK READY")

    @staticmethod
    def _finite(value):
        return value == value and abs(value) < 1000000.0

    @staticmethod
    def _clamp(value, minimum, maximum):
        if value < minimum:
            return minimum
        if value > maximum:
            return maximum
        return value

    def _reply(self, message):
        try:
            payload = (str(message) + "\r\n").encode()
            if (
                len(self.tx_pending) + len(payload)
                > BLUETOOTH_UART_TX_BUFFER_LIMIT
            ):
                self.error_count += 1
                return
            self.tx_pending.extend(payload)
        except BaseException:
            # 蓝牙断开时不能让编码或缓冲异常打断控制循环。
            self.error_count += 1

    def _flush_tx(self):
        if self.uart is None or not self.tx_pending:
            return
        now_ms = time.ticks_ms()
        if (
            self.last_tx_ms is not None
            and time.ticks_diff(now_ms, self.last_tx_ms)
            < BLUETOOTH_UART_TX_INTERVAL_MS
        ):
            return
        chunk_size = min(
            len(self.tx_pending),
            BLUETOOTH_UART_TX_CHUNK_BYTES,
        )
        try:
            written = self.uart.write(bytes(self.tx_pending[:chunk_size]))
            if written is not None and int(written) > 0:
                self.tx_pending = self.tx_pending[int(written):]
            self.last_tx_ms = now_ms
        except BaseException:
            self.error_count += 1
            self.last_tx_ms = now_ms

    def _get_values(self):
        return (
            "HOLD pos_target=%+.2f kp=%.4f ki=%.4f kd=%.4f static=%.4f "
            "pos_static=%.4f pos_max=%.4f neg_static=%.4f neg_max=%.4f max_speed=%.3f zone=%.3f gain=%.3f "
            "over_gain=%.3f hard_speed=%.3f hard_brake=%.3f "
            "ff=%.3f ff_hold=%d ff_fade=%d"
            % (
                TASK_HOLD_POSITION_TARGET_CM,
                self.controller.hold_position_gain,
                self.controller.hold_integral_gain,
                self.controller.hold_velocity_gain,
                self.controller.hold_static_angle_deg,
                HOLD_POSITIVE_SIDE_STATIC_ANGLE_DEG,
                HOLD_POSITIVE_SIDE_MAX_ANGLE_DEG,
                HOLD_NEGATIVE_SIDE_STATIC_ANGLE_DEG,
                HOLD_NEGATIVE_SIDE_MAX_ANGLE_DEG,
                TASK_HOLD_MAX_RETURN_SPEED_CM_S,
                TASK_HOLD_APPROACH_ZONE_CM,
                TASK_HOLD_APPROACH_SPEED_GAIN_CM_S_PER_CM,
                TASK_HOLD_OVERSPEED_COMP_GAIN_DEG_S_PER_CM,
                TASK_HOLD_HARD_SPEED_LIMIT_CM_S,
                TASK_HOLD_HARD_BRAKE_ANGLE_DEG,
                CART_START_FEEDFORWARD_ANGLE_DEG,
                CART_START_FEEDFORWARD_HOLD_MS,
                CART_START_FEEDFORWARD_FADE_MS,
            )
        )

    def _get_sequence_values(self):
        return (
            "SEQ kp=%.4f ki=%.4f kd=%.4f static=%.4f "
            "plus_max=%.3f plus_slew=%.3f plus_push=%.3f plus_stuck=%.3f "
            "start_ff=%.3f start_hold=%d start_fade=%d start_slew=%.3f start_release_v=%.3f start_release_x=%.3f "
            "minus_max=%.3f minus_slew=%.3f minus_coast=%.3f minus_brake_zone=%.3f minus_brake=%.3f minus_hold_zone=%.3f minus_bias=%.3f minus_min=%.3f minus_stuck=%.3f "
            "minus_pos=%.3f minus_vel=%.3f"
            % (
                self.controller.position_gain,
                self.controller.integral_gain,
                self.controller.velocity_gain,
                self.controller.static_angle_deg,
                TASK_PLUS_DRIVE_MAX_ANGLE_DEG,
                TASK_PLUS_DRIVE_ANGLE_SLEW_DEG_S,
                TASK_PLUS_FINAL_PUSH_MIN_ANGLE_DEG,
                TASK_PLUS_FINAL_PUSH_STUCK_ANGLE_DEG,
                TASK_SEQUENCE_START_FEEDFORWARD_ANGLE_DEG,
                TASK_SEQUENCE_START_FEEDFORWARD_HOLD_MS,
                TASK_SEQUENCE_START_FEEDFORWARD_FADE_MS,
                TASK_SEQUENCE_START_FEEDFORWARD_SLEW_DEG_S,
                TASK_SEQUENCE_START_FEEDFORWARD_RELEASE_SPEED_CM_S,
                TASK_SEQUENCE_START_FEEDFORWARD_RELEASE_PROGRESS_CM,
                TASK_MINUS_DRIVE_MAX_ANGLE_DEG,
                TASK_MINUS_DRIVE_ANGLE_SLEW_DEG_S,
                TASK_MINUS_COAST_ANGLE_DEG,
                TASK_MINUS_APPROACH_BRAKE_ZONE_CM,
                TASK_MINUS_APPROACH_BRAKE_MAX_ANGLE_DEG,
                TASK_MINUS_TARGET_HOLD_ZONE_CM,
                TASK_MINUS_TARGET_REST_BIAS_ANGLE_DEG,
                TASK_MINUS_TARGET_MIN_ANGLE_DEG,
                TASK_MINUS_STUCK_PUSH_ANGLE_DEG,
                TASK_MINUS_TARGET_POSITION_GAIN,
                TASK_MINUS_TARGET_VELOCITY_BRAKE_GAIN,
            )
        )

    def _get_telemetry_values(self):
        if self.motor_controller is None:
            return "TEL motor=NONE"
        motor = self.motor_controller
        if motor.last_position_cm is None:
            position_text = "--"
        else:
            position_text = "%+.3f" % motor.last_position_cm
        if motor.last_velocity_cm_s is None:
            velocity_text = "--"
        else:
            velocity_text = "%+.3f" % motor.last_velocity_cm_s
        if motor.last_confidence is None:
            confidence_text = "--"
        else:
            confidence_text = "%.3f" % motor.last_confidence
        if motor.last_live_vision_ms is None:
            vision_age_ms = -1
        else:
            vision_age_ms = max(
                0,
                time.ticks_diff(time.ticks_ms(), motor.last_live_vision_ms),
            )
        task_elapsed_ms = motor.motion_task.elapsed_ms(time.ticks_ms())
        try:
            network_ip = network.WLAN(0).ifconfig()[0]
        except BaseException:
            network_ip = "--"
        return (
            "TEL ip=%s mode=%s task=%s t=%d budget=%d target=%+.2f pos=%s vel=%s live=%d "
            "conf=%s age=%d motor=%s angle=%+.3f pulse=%d "
            "vref=%+.3f vlim=%.3f over=%.3f brake=%d "
            "settled=%d hold_angle=%+.3f start_ff=%+.3f"
            % (
                network_ip,
                BALANCE_TASK_MODE,
                motor.motion_task.state,
                task_elapsed_ms,
                motor.motion_task.budget_status(),
                motor.motion_task.target_cm,
                position_text,
                velocity_text,
                1 if motor.last_measurement_live else 0,
                confidence_text,
                vision_age_ms,
                motor.state,
                motor.controller.output_angle_deg,
                motor.last_pulses,
                motor.controller.profile_target_velocity_cm_s,
                motor.controller.profile_speed_limit_cm_s,
                motor.controller.profile_overspeed_cm_s,
                1 if motor.controller.profile_brake_active else 0,
                1 if motor.controller.settled_hold_active else 0,
                motor.controller.settled_hold_angle_deg,
                motor.controller.sequence_start_ff_angle_deg,
            )
        )

    def _set_param(self, name, value):
        global TASK_HOLD_POSITION_TARGET_CM
        global TASK_HOLD_MAX_RETURN_SPEED_CM_S
        global TASK_HOLD_APPROACH_ZONE_CM
        global TASK_HOLD_APPROACH_SPEED_GAIN_CM_S_PER_CM
        global TASK_HOLD_OVERSPEED_COMP_GAIN_DEG_S_PER_CM
        global TASK_HOLD_HARD_SPEED_LIMIT_CM_S
        global TASK_HOLD_HARD_BRAKE_ANGLE_DEG
        global HOLD_POSITIVE_SIDE_STATIC_ANGLE_DEG
        global HOLD_POSITIVE_SIDE_MAX_ANGLE_DEG
        global HOLD_NEGATIVE_SIDE_STATIC_ANGLE_DEG
        global HOLD_NEGATIVE_SIDE_MAX_ANGLE_DEG
        global CART_START_FEEDFORWARD_ANGLE_DEG
        global CART_START_FEEDFORWARD_HOLD_MS
        global CART_START_FEEDFORWARD_FADE_MS
        global TASK_PLUS_DRIVE_MAX_ANGLE_DEG
        global TASK_PLUS_DRIVE_ANGLE_SLEW_DEG_S
        global TASK_PLUS_FINAL_PUSH_MIN_ANGLE_DEG
        global TASK_PLUS_FINAL_PUSH_STUCK_ANGLE_DEG
        global TASK_SEQUENCE_START_FEEDFORWARD_ANGLE_DEG
        global TASK_SEQUENCE_START_FEEDFORWARD_HOLD_MS
        global TASK_SEQUENCE_START_FEEDFORWARD_FADE_MS
        global TASK_SEQUENCE_START_FEEDFORWARD_RELEASE_SPEED_CM_S
        global TASK_SEQUENCE_START_FEEDFORWARD_RELEASE_PROGRESS_CM
        global TASK_SEQUENCE_START_FEEDFORWARD_SLEW_DEG_S
        global TASK_MINUS_DRIVE_MAX_ANGLE_DEG
        global TASK_MINUS_DRIVE_ANGLE_SLEW_DEG_S
        global TASK_MINUS_COAST_ANGLE_DEG
        global TASK_MINUS_APPROACH_BRAKE_ZONE_CM
        global TASK_MINUS_APPROACH_BRAKE_MAX_ANGLE_DEG
        global TASK_MINUS_TARGET_HOLD_ZONE_CM
        global TASK_MINUS_TARGET_REST_BIAS_ANGLE_DEG
        global TASK_MINUS_TARGET_MIN_ANGLE_DEG
        global TASK_MINUS_STUCK_PUSH_ANGLE_DEG
        global TASK_MINUS_TARGET_POSITION_GAIN
        global TASK_MINUS_TARGET_VELOCITY_BRAKE_GAIN

        aliases = {
            "target": "hold_target",
            "kp": "hold_kp",
            "ki": "hold_ki",
            "kd": "hold_kd",
            "static": "hold_static",
            "positive_static": "hold_positive_static",
            "positive_max": "hold_positive_max",
            "negative_static": "hold_negative_static",
            "negative_max": "hold_negative_max",
            "max_speed": "hold_max_speed",
            "approach_zone": "hold_approach_zone",
            "approach_gain": "hold_approach_gain",
            "overspeed_gain": "hold_overspeed_gain",
            "hard_speed": "hold_hard_speed",
            "hard_brake": "hold_hard_brake",
            "ff_angle": "cart_start_ff_angle",
            "ff_hold": "cart_start_ff_hold",
            "ff_fade": "cart_start_ff_fade",
            "start_ff": "seq_start_ff",
            "start_hold": "seq_start_hold",
            "start_fade": "seq_start_fade",
            "start_slew": "seq_start_slew",
            "start_release_v": "seq_start_release_v",
            "start_release_x": "seq_start_release_x",
        }
        name = aliases.get(name, name)
        if name == "hold_target":
            TASK_HOLD_POSITION_TARGET_CM = self._clamp(
                value,
                -(ROD_LENGTH_CM * 0.5 - ROD_END_MARGIN_CM),
                ROD_LENGTH_CM * 0.5 - ROD_END_MARGIN_CM,
            )
        elif name == "hold_kp":
            self.controller.hold_position_gain = self._clamp(
                value, PID_POSITION_GAIN_MIN, PID_POSITION_GAIN_MAX
            )
        elif name == "hold_ki":
            self.controller.hold_integral_gain = self._clamp(value, 0.0, 1.0)
        elif name == "hold_kd":
            self.controller.hold_velocity_gain = self._clamp(
                value, PID_VELOCITY_GAIN_MIN, PID_VELOCITY_GAIN_MAX
            )
        elif name == "hold_static":
            self.controller.hold_static_angle_deg = self._clamp(
                value, PID_STATIC_ANGLE_MIN, 2.0
            )
        elif name == "hold_positive_static":
            HOLD_POSITIVE_SIDE_STATIC_ANGLE_DEG = self._clamp(value, 0.0, 2.5)
        elif name == "hold_positive_max":
            HOLD_POSITIVE_SIDE_MAX_ANGLE_DEG = self._clamp(value, 0.2, 4.0)
        elif name == "hold_negative_static":
            HOLD_NEGATIVE_SIDE_STATIC_ANGLE_DEG = self._clamp(value, 0.0, 2.5)
        elif name == "hold_negative_max":
            HOLD_NEGATIVE_SIDE_MAX_ANGLE_DEG = self._clamp(value, 0.2, 4.0)
        elif name == "hold_max_speed":
            TASK_HOLD_MAX_RETURN_SPEED_CM_S = self._clamp(value, 0.2, 10.0)
        elif name == "hold_approach_zone":
            TASK_HOLD_APPROACH_ZONE_CM = self._clamp(value, 0.5, 20.0)
        elif name == "hold_approach_gain":
            TASK_HOLD_APPROACH_SPEED_GAIN_CM_S_PER_CM = self._clamp(value, 0.02, 3.0)
        elif name == "hold_overspeed_gain":
            TASK_HOLD_OVERSPEED_COMP_GAIN_DEG_S_PER_CM = self._clamp(value, 0.0, 4.0)
        elif name == "hold_hard_speed":
            TASK_HOLD_HARD_SPEED_LIMIT_CM_S = self._clamp(value, 0.5, 10.0)
        elif name == "hold_hard_brake":
            TASK_HOLD_HARD_BRAKE_ANGLE_DEG = self._clamp(value, 0.2, 4.0)
        elif name == "cart_start_ff_angle":
            CART_START_FEEDFORWARD_ANGLE_DEG = self._clamp(value, -4.0, 4.0)
        elif name == "cart_start_ff_hold":
            CART_START_FEEDFORWARD_HOLD_MS = int(self._clamp(value, 0.0, 5000.0))
        elif name == "cart_start_ff_fade":
            CART_START_FEEDFORWARD_FADE_MS = int(self._clamp(value, 0.0, 5000.0))
        elif name == "seq_kp":
            self.controller.position_gain = self._clamp(
                value, PID_POSITION_GAIN_MIN, PID_POSITION_GAIN_MAX
            )
        elif name == "seq_ki":
            self.controller.integral_gain = self._clamp(value, 0.0, 1.0)
        elif name == "seq_kd":
            self.controller.velocity_gain = self._clamp(
                value, PID_VELOCITY_GAIN_MIN, PID_VELOCITY_GAIN_MAX
            )
        elif name == "seq_static":
            self.controller.static_angle_deg = self._clamp(
                value, PID_STATIC_ANGLE_MIN, PID_STATIC_ANGLE_MAX
            )
        elif name == "plus_max":
            TASK_PLUS_DRIVE_MAX_ANGLE_DEG = self._clamp(value, 0.2, 8.0)
        elif name == "plus_slew":
            TASK_PLUS_DRIVE_ANGLE_SLEW_DEG_S = self._clamp(value, 0.5, 40.0)
        elif name == "plus_push":
            TASK_PLUS_FINAL_PUSH_MIN_ANGLE_DEG = self._clamp(value, 0.0, 8.0)
        elif name == "plus_stuck":
            TASK_PLUS_FINAL_PUSH_STUCK_ANGLE_DEG = self._clamp(value, 0.0, 8.0)
        elif name == "seq_start_ff":
            TASK_SEQUENCE_START_FEEDFORWARD_ANGLE_DEG = self._clamp(value, 0.0, 4.0)
        elif name == "seq_start_hold":
            TASK_SEQUENCE_START_FEEDFORWARD_HOLD_MS = int(self._clamp(value, 0.0, 1000.0))
        elif name == "seq_start_fade":
            TASK_SEQUENCE_START_FEEDFORWARD_FADE_MS = int(self._clamp(value, 1.0, 2000.0))
        elif name == "seq_start_slew":
            TASK_SEQUENCE_START_FEEDFORWARD_SLEW_DEG_S = self._clamp(value, 1.0, 40.0)
        elif name == "seq_start_release_v":
            TASK_SEQUENCE_START_FEEDFORWARD_RELEASE_SPEED_CM_S = self._clamp(value, 0.10, 5.0)
        elif name == "seq_start_release_x":
            TASK_SEQUENCE_START_FEEDFORWARD_RELEASE_PROGRESS_CM = self._clamp(value, 0.05, 2.0)
        elif name == "minus_max":
            TASK_MINUS_DRIVE_MAX_ANGLE_DEG = self._clamp(value, 0.2, 5.4)
        elif name == "minus_slew":
            TASK_MINUS_DRIVE_ANGLE_SLEW_DEG_S = self._clamp(value, 0.5, 40.0)
        elif name == "minus_coast":
            TASK_MINUS_COAST_ANGLE_DEG = self._clamp(value, -2.0, 0.0)
        elif name == "minus_brake_zone":
            TASK_MINUS_APPROACH_BRAKE_ZONE_CM = self._clamp(value, 0.0, 10.0)
        elif name == "minus_brake":
            TASK_MINUS_APPROACH_BRAKE_MAX_ANGLE_DEG = self._clamp(value, 0.0, 4.0)
        elif name == "minus_hold_zone":
            TASK_MINUS_TARGET_HOLD_ZONE_CM = self._clamp(value, 0.1, 5.0)
        elif name == "minus_bias":
            TASK_MINUS_TARGET_REST_BIAS_ANGLE_DEG = self._clamp(value, -5.0, 0.0)
        elif name == "minus_min":
            TASK_MINUS_TARGET_MIN_ANGLE_DEG = self._clamp(value, -5.0, 0.0)
        elif name == "minus_stuck":
            TASK_MINUS_STUCK_PUSH_ANGLE_DEG = self._clamp(value, -5.0, 0.0)
        elif name == "minus_pos":
            TASK_MINUS_TARGET_POSITION_GAIN = self._clamp(value, 0.0, 2.0)
        elif name == "minus_vel":
            TASK_MINUS_TARGET_VELOCITY_BRAKE_GAIN = self._clamp(value, 0.0, 2.0)
        else:
            return False
        self.controller.tuning_dirty = True
        return True

    def _handle_line(self, line):
        parts = line.strip().split()
        if not parts:
            return
        command = parts[0].upper()
        self.command_count += 1
        if command == "GET":
            if len(parts) == 2 and parts[1].upper() == "SEQ":
                self._reply("OK " + self._get_sequence_values())
            else:
                self._reply("OK " + self._get_values())
            return
        if command == "STATUS":
            values = self._get_values()
            if len(parts) == 2 and parts[1].upper() == "SEQ":
                values = self._get_sequence_values()
            if self.motor_controller is None:
                self._reply("OK MOTOR NONE " + values)
            else:
                self._reply(
                    "OK MOTOR=%s angle=%+.3f pulse=%d %s"
                    % (
                        self.motor_controller.state,
                        self.controller.output_angle_deg,
                        self.motor_controller.last_pulses,
                        values,
                    )
                )
            return
        if command in ("TEL", "TELEM"):
            self._reply("OK " + self._get_telemetry_values())
            return
        if command == "MODE" and len(parts) == 2:
            mode_name = parts[1].upper()
            mode_map = {
                "HOLD0": TASK_MODE_HOLD_ZERO,
                "HOLDPOS": TASK_MODE_HOLD_POSITION,
                "HOLD3": TASK_MODE_HOLD_POSITION,
                "SEQ": TASK_MODE_SEQUENCE,
                "CAL": TASK_MODE_CALIBRATION,
                "CALIBRATE": TASK_MODE_CALIBRATION,
            }
            requested_mode = mode_map.get(mode_name)
            if requested_mode is None or self.motor_controller is None:
                self._reply("ERR MODE")
                self.error_count += 1
                return
            self.motor_controller.set_task_mode(
                requested_mode,
                time.ticks_ms(),
            )
            self._reply("OK MODE %s" % mode_name)
            return
        if command == "TEACH" and len(parts) == 1:
            now_ms = time.ticks_ms()
            if self.motor_controller is None:
                self._reply("ERR MOTOR")
                self.error_count += 1
                return
            target_cm = self.motor_controller.teach_hold_position(now_ms)
            if target_cm is None:
                self._reply("ERR TEACH_NOT_STABLE")
                self.error_count += 1
                return
            self.motor_controller.set_task_mode(
                TASK_MODE_HOLD_POSITION,
                now_ms,
            )
            self._reply("OK TEACH %+.3f" % target_cm)
            return
        if command == "OPEN" and len(parts) == 2:
            now_ms = time.ticks_ms()
            if parts[1].upper() == "OFF":
                if self.motor_controller is None:
                    self._reply("ERR MOTOR")
                    self.error_count += 1
                else:
                    # Do not hand a moving ball directly back to PID.  OPEN OFF
                    # is an explicit safe stop; a MODE command is required to
                    # re-arm the vision controller.
                    self.motor_controller._stop_and_disarm(
                        "OPEN_OFF",
                        now_ms,
                    )
                    self._reply("OK OPEN OFF")
                return
            try:
                angle_deg = float(parts[1])
            except BaseException:
                self._reply("ERR VALUE")
                self.error_count += 1
                return
            if not self._finite(angle_deg):
                self._reply("ERR VALUE")
                self.error_count += 1
                return
            if (
                self.motor_controller is None
                or not self.motor_controller.set_open_loop_angle(angle_deg, now_ms)
            ):
                self._reply("ERR OPEN_NOT_READY")
                self.error_count += 1
            else:
                applied_angle_deg = self.motor_controller.open_loop_angle_deg
                self._reply("OK OPEN %+.3f" % applied_angle_deg)
            return
        if command == "STOP":
            if self.motor_controller is None:
                self._reply("ERR MOTOR")
                self.error_count += 1
            else:
                self.motor_controller._stop_and_disarm(
                    "REMOTE_STOP",
                    time.ticks_ms(),
                )
                self._reply("OK STOP")
            return
        if command == "HELP":
            self._reply(
                "OK GET [SEQ]|STATUS [SEQ]|TEL|TEACH|MODE HOLD0|HOLDPOS|SEQ|CAL|OPEN angle|OPEN OFF|STOP|"
                "SET target|kp|ki|kd|static|positive_static|positive_max|negative_static|negative_max|"
                "max_speed|approach_zone|approach_gain|overspeed_gain|hard_speed|hard_brake|"
                "ff_angle|ff_hold|ff_fade|seq_kp|seq_ki|seq_kd|seq_static|"
                "start_ff|start_hold|start_fade|start_slew|start_release_v|start_release_x|"
                "plus_max|plus_slew|plus_push|plus_stuck|minus_max|minus_slew|minus_coast|minus_brake_zone|minus_brake|minus_hold_zone|"
                "minus_bias|minus_min|minus_stuck|minus_pos|minus_vel value|SAVE"
            )
            return
        if command == "PING":
            self._reply("OK PONG")
            return
        if command == "SAVE":
            if self.controller._save_tuning():
                self._reply("OK SAVE")
            else:
                self._reply("ERR SAVE")
            return
        if command == "SET" and len(parts) == 3:
            try:
                value = float(parts[2])
            except BaseException:
                self._reply("ERR VALUE")
                self.error_count += 1
                return
            if not self._finite(value):
                self._reply("ERR VALUE")
                self.error_count += 1
                return
            if self._set_param(parts[1].lower(), value):
                self._reply("OK SET %s" % parts[1].lower())
            else:
                self._reply("ERR PARAM")
                self.error_count += 1
            return
        self._reply("ERR CMD")
        self.error_count += 1

    def poll(self):
        if self.uart is None:
            return
        self._flush_tx()
        try:
            available = self.uart.any()
            if not available:
                return
            data = self.uart.read(min(int(available), 128))
            if data is None:
                return
            for byte_value in data:
                if byte_value in (10, 13):
                    if self.rx_line:
                        try:
                            line = bytes(self.rx_line).decode().strip()
                            self._handle_line(line)
                        except BaseException:
                            self._reply("ERR ENCODING")
                            self.error_count += 1
                        self.rx_line = bytearray()
                elif byte_value in (8, 127):
                    if self.rx_line:
                        self.rx_line = self.rx_line[:-1]
                elif 32 <= byte_value < 127:
                    if len(self.rx_line) < BLUETOOTH_UART_MAX_LINE_LENGTH:
                        self.rx_line.append(byte_value)
                    else:
                        self.rx_line = bytearray()
                        self._reply("ERR LINE_TOO_LONG")
                        self.error_count += 1
        except BaseException:
            self.error_count += 1

    def deinit(self):
        if self.uart is not None:
            try:
                self.uart.deinit()
            except BaseException as error:
                print("Bluetooth tuner deinit error:", error)
            self.uart = None
        print("Bluetooth tuner stopped")


class LcdModeSelector:
    """ST7701 touch buttons for runtime balance-task mode selection."""

    def __init__(self):
        if TOUCH is None:
            raise RuntimeError("TOUCH is unavailable in this firmware")
        self.touch = TOUCH(0)
        self.last_poll_ms = None
        self.last_change_ms = None
        self.touch_down = False
        print("LCD MODE SELECTOR: touch buttons enabled")

    @staticmethod
    def _inside(x, y, button):
        _, _, bx, by, bw, bh = button
        return bx <= x < bx + bw and by <= y < by + bh

    def poll(self, motor_controller, now_ms):
        if self.touch is None or motor_controller is None:
            return
        if (
            self.last_poll_ms is not None
            and time.ticks_diff(now_ms, self.last_poll_ms)
            < LCD_MODE_TOUCH_POLL_MS
        ):
            return
        self.last_poll_ms = now_ms

        pressed = False
        display_x = None
        display_y = None
        try:
            points = self.touch.read(1)
            if points:
                point = points[0]
                if point.event == 2 or point.event == 3:
                    pressed = True
                    display_x = int(point.x)
                    display_y = int(point.y)
        except BaseException as error:
            print("LCD touch read error:", error)
            self.touch = None
            return

        debounce_ready = (
            self.last_change_ms is None
            or time.ticks_diff(now_ms, self.last_change_ms)
            >= LCD_MODE_TOUCH_DEBOUNCE_MS
        )
        if pressed and not self.touch_down and debounce_ready:
            for button in LCD_MODE_BUTTONS:
                if self._inside(display_x, display_y, button):
                    motor_controller.set_task_mode(button[0], now_ms)
                    self.last_change_ms = now_ms
                    break
        self.touch_down = pressed

    def draw(self, image_obj):
        for mode, label, x, y, width, height in LCD_MODE_BUTTONS:
            fill_color = (
                MODE_BUTTON_ACTIVE
                if mode == BALANCE_TASK_MODE
                else MODE_BUTTON_INACTIVE
            )
            image_obj.draw_rectangle(
                x,
                y,
                width,
                height,
                color=fill_color,
                thickness=1,
                fill=True,
            )
            image_obj.draw_rectangle(
                x,
                y,
                width,
                height,
                color=MODE_BUTTON_BORDER,
                thickness=2,
                fill=False,
            )
            image_obj.draw_string_advanced(
                x + 8,
                y + 10,
                18,
                label,
                color=WHITE,
            )

    def deinit(self):
        if self.touch is not None:
            try:
                self.touch.deinit()
            except BaseException:
                pass
            self.touch = None
        print("LCD mode selector stopped")


class BalanceMotionTask:
    """管理持续定点平衡或 0 -> +5 cm -> -5 cm 视觉运动任务。"""

    HOLD_ZERO = "HOLD_ZERO"
    HOLD_POSITION = "HOLD_POSITION"
    WAIT_START = "WAIT_START"
    MOVE_PLUS = "MOVE_PLUS"
    MOVE_MINUS = "MOVE_MINUS"
    COMPLETE = "COMPLETE"
    TIMEOUT = "TIMEOUT"

    def __init__(self):
        if BALANCE_TASK_MODE not in (
            TASK_MODE_HOLD_ZERO,
            TASK_MODE_HOLD_POSITION,
            TASK_MODE_SEQUENCE,
        ):
            raise ValueError("Invalid BALANCE_TASK_MODE: %s" % BALANCE_TASK_MODE)
        hold_position_limit_cm = ROD_LENGTH_CM * 0.5 - ROD_END_MARGIN_CM
        if (
            BALANCE_TASK_MODE == TASK_MODE_HOLD_POSITION
            and abs(TASK_HOLD_POSITION_TARGET_CM) > hold_position_limit_cm
        ):
            raise ValueError(
                "TASK_HOLD_POSITION_TARGET_CM must be within +/-%0.1f cm"
                % hold_position_limit_cm
            )
        self.reset()

    def reset(self):
        if BALANCE_TASK_ENABLED and BALANCE_TASK_MODE == TASK_MODE_HOLD_ZERO:
            self.state = self.HOLD_ZERO
            self.target_cm = TASK_HOLD_ZERO_TARGET_CM
        elif BALANCE_TASK_ENABLED and BALANCE_TASK_MODE == TASK_MODE_HOLD_POSITION:
            self.state = self.HOLD_POSITION
            self.target_cm = TASK_HOLD_POSITION_TARGET_CM
        else:
            self.state = self.WAIT_START
            self.target_cm = TASK_START_POSITION_CM
        self.start_ms = None
        self.start_settle_since_ms = None
        self.plus_settle_since_ms = None
        self.final_settle_since_ms = None
        self.complete_elapsed_ms = None

    def update(self, position_cm, velocity_cm_s, measurement_live, now_ms):
        global TASK_PLUS_TARGET_CM
        global TASK_MINUS_TARGET_CM
        if not BALANCE_TASK_ENABLED:
            self.target_cm = MOTOR_TARGET_CM
            return self.state

        if BALANCE_TASK_MODE == TASK_MODE_HOLD_ZERO:
            # HOLD_ZERO 永不自动完成或切换目标；每个控制周期都保持 0 cm 闭环。
            self.state = self.HOLD_ZERO
            self.target_cm = TASK_HOLD_ZERO_TARGET_CM
            return self.state

        if BALANCE_TASK_MODE == TASK_MODE_HOLD_POSITION:
            # HOLD_POSITION 与零点模式使用同一闭环，只把终点改为指定坐标。
            self.state = self.HOLD_POSITION
            self.target_cm = TASK_HOLD_POSITION_TARGET_CM
            return self.state

        if self.state == self.WAIT_START:
            self.target_cm = TASK_START_POSITION_CM
            if (
                measurement_live
                and position_cm is not None
                and abs(position_cm - TASK_START_POSITION_CM)
                <= TASK_START_POSITION_TOLERANCE_CM
                and velocity_cm_s is not None
                and abs(velocity_cm_s) <= TASK_START_MAX_SPEED_CM_S
            ):
                if self.start_settle_since_ms is None:
                    self.start_settle_since_ms = now_ms
                elif (
                    time.ticks_diff(now_ms, self.start_settle_since_ms)
                    >= TASK_START_SETTLE_MS
                ):
                    sequence_origin_cm = (
                        float(position_cm)
                        if TASK_SEQUENCE_RELATIVE_TO_START
                        else TASK_START_POSITION_CM
                    )
                    TASK_PLUS_TARGET_CM = (
                        sequence_origin_cm + TASK_PLUS_TARGET_OFFSET_CM
                    )
                    TASK_MINUS_TARGET_CM = (
                        sequence_origin_cm + TASK_MINUS_TARGET_OFFSET_CM
                    )
                    self.state = self.MOVE_PLUS
                    self.target_cm = TASK_PLUS_TARGET_CM
                    self.start_ms = now_ms
                    self.start_settle_since_ms = None
                    self.plus_settle_since_ms = None
                    print(
                        "TASK: origin=%+.3f cm, targets=%+.3f/%+.3f cm"
                        % (
                            sequence_origin_cm,
                            TASK_PLUS_TARGET_CM,
                            TASK_MINUS_TARGET_CM,
                        )
                    )
            else:
                self.start_settle_since_ms = None
            return self.state

        if (
            TASK_TIMEOUT_ENABLED
            and self.start_ms is not None
            and time.ticks_diff(now_ms, self.start_ms) >= TASK_DEADLINE_MS
        ):
            self.state = self.TIMEOUT
            self.target_cm = TASK_START_POSITION_CM
            print("TASK: deadline exceeded")
            return self.state

        if self.state == self.MOVE_PLUS:
            self.target_cm = TASK_PLUS_TARGET_CM
            if (
                measurement_live
                and position_cm is not None
                and abs(position_cm - TASK_PLUS_TARGET_CM)
                <= TASK_PLUS_SWITCH_TOLERANCE_CM
            ):
                if self.plus_settle_since_ms is None:
                    self.plus_settle_since_ms = now_ms
                elif (
                    time.ticks_diff(now_ms, self.plus_settle_since_ms)
                    >= TASK_PLUS_SETTLE_MS
                ):
                    self.state = self.MOVE_MINUS
                    self.target_cm = TASK_MINUS_TARGET_CM
                    self.plus_settle_since_ms = None
                    self.final_settle_since_ms = None
                    print(
                        "TASK: +5 cm settled; move to -5 cm; t=%d ms"
                        % time.ticks_diff(now_ms, self.start_ms)
                    )
            else:
                self.plus_settle_since_ms = None
            return self.state

        if self.state == self.MOVE_MINUS:
            self.target_cm = TASK_MINUS_TARGET_CM
            if (
                measurement_live
                and position_cm is not None
                and abs(position_cm - TASK_MINUS_TARGET_CM)
                <= TASK_POSITION_TOLERANCE_CM
                and abs(velocity_cm_s) <= TASK_FINAL_MAX_SPEED_CM_S
            ):
                if self.final_settle_since_ms is None:
                    self.final_settle_since_ms = now_ms
                elif (
                    time.ticks_diff(now_ms, self.final_settle_since_ms)
                    >= TASK_FINAL_SETTLE_MS
                ):
                    self.state = self.COMPLETE
                    self.complete_elapsed_ms = time.ticks_diff(
                        now_ms, self.start_ms
                    )
                    print(
                        "TASK: complete at -5 cm; t=%d ms"
                        % self.complete_elapsed_ms
                    )
            else:
                self.final_settle_since_ms = None
            return self.state

        if self.state == self.COMPLETE:
            self.target_cm = TASK_MINUS_TARGET_CM
        return self.state

    def status_text(self):
        if self.state == self.HOLD_ZERO:
            return "TASK HOLD 0"
        if self.state == self.HOLD_POSITION:
            return "TASK HOLD %+.1f" % self.target_cm
        if self.state == self.MOVE_PLUS:
            return "TASK +5"
        if self.state == self.MOVE_MINUS:
            return "TASK -5"
        if self.state == self.COMPLETE:
            return "TASK DONE"
        if self.state == self.TIMEOUT:
            return "TASK TIMEOUT"
        return "TASK WAIT"

    def elapsed_ms(self, now_ms):
        if self.complete_elapsed_ms is not None:
            return max(0, self.complete_elapsed_ms)
        if self.start_ms is None:
            return 0
        return max(0, time.ticks_diff(now_ms, self.start_ms))

    def budget_status(self):
        if self.complete_elapsed_ms is None:
            return -1
        return 1 if self.complete_elapsed_ms <= TASK_PERFORMANCE_BUDGET_MS else 0


class Emm42MotorController:
    """K230 UART2 直连 Emm42 的初始化、ACK、视觉闭环和丢球急停状态机。"""

    FUNCTION_CONFIG = 0xF1
    FUNCTION_ENABLE = 0xF3
    FUNCTION_POSITION = 0xFC
    FUNCTION_STOP = 0xFE
    RESULT_ACCEPTED = 0x02
    RESULT_COMPLETE = 0x9F

    def __init__(self):
        fpioa = FPIOA()
        fpioa.set_function(MOTOR_UART_TX_PIN, FPIOA.UART2_TXD)
        fpioa.set_function(MOTOR_UART_RX_PIN, FPIOA.UART2_RXD)
        self.uart = UART(
            UART.UART2,
            baudrate=MOTOR_UART_BAUDRATE,
            bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE,
        )
        self.controller = VisionPidController()
        self.motion_task = BalanceMotionTask()
        self.control_interval_ms = max(1, 1000 // MOTOR_CONTROL_HZ)
        self.last_control_ms = None
        self.next_control_ms = None
        self.control_overrun_active = False
        self.control_overrun_count = 0
        self.max_control_interval_ms = 0
        self.last_live_vision_ms = None
        self.last_position_cm = None
        self.last_velocity_cm_s = None
        self.last_measurement_live = False
        self.last_confidence = None
        self.last_disarm_ms = None
        self.consecutive_valid_frames = 0
        self.armed = False
        self.enabled = False
        self.ready = False
        self.cart_start_feedforward_used = False
        self.vision_safe_active = False
        self.open_loop_angle_deg = None
        self.remote_stop_latched = False
        self.calibration_start_ms = None
        self.calibration_samples = []
        self.state = "DISARMED"
        self.fault = None
        self.outstanding_function = None
        self.outstanding_since_ms = None
        self.command_tx_id = 0
        self.outstanding_tx_id = None
        self.consecutive_ack_misses = 0
        self.consecutive_write_failures = 0
        self.rx_window = bytearray()
        self.rx_accept_count = 0
        self.rx_complete_count = 0
        self.rx_orphan_count = 0
        self.rx_error_reply_count = 0
        self.sent_count = 0
        self.error_count = 0
        self.last_pulses = EMM42_CENTER_PULSE
        print(
            "DIRECT MOTOR: GPIO%d TX -> Emm42 RX, GPIO%d RX <- Emm42 TX, %d baud, center=%d pulse"
            % (
                MOTOR_UART_TX_PIN,
                MOTOR_UART_RX_PIN,
                MOTOR_UART_BAUDRATE,
                EMM42_CENTER_PULSE,
            )
        )
        print(
            "AUTO ARM: %d frames, |x|<=%.1f cm, |v|<=%.1f cm/s; lost %d ms -> stop"
            % (
                MOTOR_AUTO_ARM_VALID_FRAMES,
                MOTOR_AUTO_ARM_WINDOW_CM,
                MOTOR_AUTO_ARM_MAX_SPEED_CM_S,
                MOTOR_VISION_STOP_AGE_MS,
            )
        )
        print(
            "BALANCE TASK MODE: %s, target=%+.2f cm"
            % (BALANCE_TASK_MODE, self.motion_task.target_cm)
        )

    @staticmethod
    def _round_to_int(value):
        if value >= 0.0:
            return int(value + 0.5)
        return int(value - 0.5)

    @staticmethod
    def _clamp(value, minimum, maximum):
        if value < minimum:
            return minimum
        if value > maximum:
            return maximum
        return value

    @staticmethod
    def _build_enable():
        return bytes(
            (EMM42_ADDRESS, 0xF3, 0xAB, 0x01, 0x00, EMM42_FIXED_CHECKSUM)
        )

    @staticmethod
    def _build_config():
        return bytes(
            (
                EMM42_ADDRESS,
                0xF1,
                (EMM42_SPEED_RPM >> 8) & 0xFF,
                EMM42_SPEED_RPM & 0xFF,
                EMM42_ACCELERATION,
                0x01,
                0x00,
                EMM42_FIXED_CHECKSUM,
            )
        )

    @staticmethod
    def _build_position(pulses):
        encoded = int(pulses) & 0xFFFFFFFF
        return bytes(
            (
                EMM42_ADDRESS,
                0xFC,
                (encoded >> 24) & 0xFF,
                (encoded >> 16) & 0xFF,
                (encoded >> 8) & 0xFF,
                encoded & 0xFF,
                EMM42_FIXED_CHECKSUM,
            )
        )

    @staticmethod
    def _build_stop():
        return bytes(
            (EMM42_ADDRESS, 0xFE, 0x98, 0x00, EMM42_FIXED_CHECKSUM)
        )

    def _write_command(self, frame, function, now_ms):
        if self.uart is None or self.outstanding_function is not None:
            return False
        try:
            written = self.uart.write(frame)
            if written != len(frame):
                self.error_count += 1
                self.consecutive_write_failures += 1
                print(
                    "Motor UART short write: function=0x%02X wrote=%s expected=%d"
                    % (function, str(written), len(frame))
                )
                self._latch_fault("MOTOR_UART_SHORT_WRITE")
                return False
            self.consecutive_write_failures = 0
            self.outstanding_function = function
            self.outstanding_since_ms = now_ms
            self.command_tx_id += 1
            self.outstanding_tx_id = self.command_tx_id
            self.sent_count += 1
            return True
        except BaseException as error:
            self.error_count += 1
            self.consecutive_write_failures += 1
            print("Motor UART write error:", error)
            if self.consecutive_write_failures >= MOTOR_UART_WRITE_FAIL_LIMIT:
                self._latch_fault("MOTOR_UART_WRITE")
            return False

    def _write_stop_immediately(self):
        if self.uart is None:
            return
        frame = self._build_stop()
        for _ in range(MOTOR_STOP_WRITE_RETRIES):
            try:
                written = self.uart.write(frame)
                if written == len(frame):
                    self.sent_count += 1
                    break
                self.error_count += 1
                print(
                    "Motor stop short write: wrote=%s expected=%d"
                    % (str(written), len(frame))
                )
            except BaseException as error:
                self.error_count += 1
                print("Motor stop write error:", error)
        self.outstanding_function = None
        self.outstanding_since_ms = None
        self.outstanding_tx_id = None

    def _handle_reply(self, function, result):
        if result == self.RESULT_ACCEPTED:
            self.rx_accept_count += 1
        elif result == self.RESULT_COMPLETE:
            self.rx_complete_count += 1
        else:
            self.rx_error_reply_count += 1

        # 只接受当前事务的 0x02 接收确认；0x9F 是异步完成帧，不能释放新的 FC。
        if self.fault is not None or not self.armed:
            self.rx_orphan_count += 1
            return
        if (
            self.outstanding_function is None
            or function != self.outstanding_function
        ):
            self.rx_orphan_count += 1
            return

        if result not in (self.RESULT_ACCEPTED, self.RESULT_COMPLETE):
            if result in (0x12, 0x22):
                reason = "MOTOR_LIMIT_%02X" % result
            elif result == 0xE2:
                reason = "MOTOR_ERROR"
            elif result == 0xEE:
                reason = "MOTOR_PROTOCOL"
            else:
                reason = "MOTOR_REPLY_%02X" % result
            self._latch_fault(reason)
            return

        if result == self.RESULT_COMPLETE:
            return

        self.outstanding_function = None
        self.outstanding_since_ms = None
        self.outstanding_tx_id = None
        self.consecutive_ack_misses = 0

        if function == self.FUNCTION_ENABLE:
            self.enabled = True
            self.state = "CONFIGURING"
        elif function == self.FUNCTION_CONFIG:
            self.ready = True
            self.state = "RUN"
            if (
                CART_START_FEEDFORWARD_ENABLED
                and not self.cart_start_feedforward_used
            ):
                self.controller.trigger_start_feedforward(time.ticks_ms())
                self.cart_start_feedforward_used = True
                print(
                    "Cart start feedforward: %+.2f deg"
                    % CART_START_FEEDFORWARD_ANGLE_DEG
                )

    def _poll_replies(self):
        if self.uart is None:
            return
        try:
            available = self.uart.any()
            if not available:
                return
            data = self.uart.read(available)
            if not data:
                return
            for byte in data:
                self.rx_window.append(byte)
                while len(self.rx_window) >= 4:
                    # K230 MicroPython 的 bytearray 不支持 del；用切片重建来消费缓存。
                    if (
                        self.rx_window[0] == EMM42_ADDRESS
                        and self.rx_window[1]
                        in (
                            self.FUNCTION_CONFIG,
                            self.FUNCTION_ENABLE,
                            self.FUNCTION_POSITION,
                            self.FUNCTION_STOP,
                        )
                        and self.rx_window[3] == EMM42_FIXED_CHECKSUM
                    ):
                        function = self.rx_window[1]
                        result = self.rx_window[2]
                        self.rx_window = self.rx_window[4:]
                        self._handle_reply(function, result)
                    else:
                        self.rx_window = self.rx_window[1:]
        except BaseException as error:
            self.error_count += 1
            if self.error_count <= 3 or self.error_count % 100 == 0:
                print("Motor UART read error:", error)

    def _check_ack_timeout(self, now_ms):
        if self.outstanding_function is None or self.outstanding_since_ms is None:
            return
        if time.ticks_diff(now_ms, self.outstanding_since_ms) < MOTOR_ACK_TIMEOUT_MS:
            return
        timed_out_function = self.outstanding_function
        timed_out_tx_id = self.outstanding_tx_id
        self.outstanding_function = None
        self.outstanding_since_ms = None
        self.outstanding_tx_id = None
        self.consecutive_ack_misses += 1
        print(
            "Motor ACK timeout: function=0x%02X tx=%s miss=%d"
            % (timed_out_function, str(timed_out_tx_id), self.consecutive_ack_misses)
        )
        if timed_out_function == self.FUNCTION_POSITION:
            # 单次位置 ACK 丢包不立即打断任务；连续达到上限才停机，
            # 避免串口偶发丢帧把 0 -> +5 -> -5 任务截断在中途。
            if self.consecutive_ack_misses >= MOTOR_ACK_MISS_LIMIT:
                self._latch_fault("MOTOR_ACK_TIMEOUT")
            return
        if self.consecutive_ack_misses >= MOTOR_ACK_MISS_LIMIT:
            if timed_out_function in (self.FUNCTION_ENABLE, self.FUNCTION_CONFIG):
                # 电机尚未上电时不永久锁死；停止发送并按冷却时间等待下次自动探测。
                self._stop_and_disarm("MOTOR_OFFLINE", now_ms)
            else:
                self._latch_fault("MOTOR_ACK_TIMEOUT")

    def _latch_fault(self, reason):
        if self.fault is None:
            self.fault = reason
            print("MOTOR FAULT:", reason)
        self._write_stop_immediately()
        self.armed = False
        self.enabled = False
        self.ready = False
        self.vision_safe_active = False
        self.open_loop_angle_deg = None
        self.state = "FAULT"
        self.controller.reset()
        self.motion_task.reset()
        self.last_control_ms = None
        self.next_control_ms = None

    def _stop_and_disarm(self, reason, now_ms=None):
        self._write_stop_immediately()
        if reason in ("REMOTE_STOP", "OPEN_OFF", "CALIBRATION"):
            self.remote_stop_latched = True
        self.armed = False
        self.enabled = False
        self.ready = False
        self.vision_safe_active = False
        self.open_loop_angle_deg = None
        self.consecutive_valid_frames = 0
        self.state = reason
        self.last_disarm_ms = time.ticks_ms() if now_ms is None else now_ms
        self.controller.reset()
        self.motion_task.reset()
        self.last_control_ms = None
        self.next_control_ms = None
        print("Motor stopped:", reason)

    def _control_due(self, now_ms):
        if self.next_control_ms is None:
            self.next_control_ms = time.ticks_add(
                now_ms, self.control_interval_ms
            )
            return True
        lateness_ms = time.ticks_diff(now_ms, self.next_control_ms)
        if lateness_ms < 0:
            return False
        elapsed_periods = lateness_ms // self.control_interval_ms + 1
        self.next_control_ms = time.ticks_add(
            self.next_control_ms,
            elapsed_periods * self.control_interval_ms,
        )
        return True

    def _control_dt_s(self, now_ms):
        if self.last_control_ms is None:
            dt_ms = self.control_interval_ms
        else:
            dt_ms = time.ticks_diff(now_ms, self.last_control_ms)
        if dt_ms < 0:
            dt_ms = self.control_interval_ms
        if dt_ms > self.max_control_interval_ms:
            self.max_control_interval_ms = dt_ms
        self.control_overrun_active = dt_ms > MOTOR_CONTROL_OVERRUN_MS
        if self.control_overrun_active:
            self.control_overrun_count += 1
        dt_s = dt_ms / 1000.0
        if dt_s < 0.010:
            dt_s = 0.010
        elif dt_s > 0.100:
            dt_s = 0.100
        self.last_control_ms = now_ms
        return dt_s

    def _send_position(self, correction_pulse, now_ms):
        requested = EMM42_CENTER_PULSE + EMM42_ANGLE_TO_PULSE_SIGN * self._round_to_int(
            correction_pulse
        )
        pulses = int(self._clamp(requested, EMM42_MIN_PULSE, EMM42_MAX_PULSE))
        if self._write_command(
            self._build_position(pulses), self.FUNCTION_POSITION, now_ms
        ):
            self.last_pulses = pulses
            return True
        return False

    def set_open_loop_angle(self, angle_deg, now_ms):
        if self.fault is not None or not self.armed or not self.ready:
            return False
        self.open_loop_angle_deg = self._clamp(float(angle_deg), -3.0, 3.0)
        self.controller.clear_integral()
        self.last_control_ms = now_ms
        self.next_control_ms = now_ms
        self.control_overrun_active = False
        return True

    def clear_open_loop(self, now_ms):
        if self.open_loop_angle_deg is None:
            return False
        previous_output_angle_deg = self.controller.output_angle_deg
        self.open_loop_angle_deg = None
        self.controller.reset()
        self.controller.output_angle_deg = previous_output_angle_deg
        self.last_control_ms = now_ms
        self.next_control_ms = now_ms
        self.control_overrun_active = False
        return True

    def _update_calibration(
        self,
        live_valid,
        position_cm,
        now_ms,
    ):
        self.state = "CALIBRATING"
        if self.calibration_start_ms is None:
            self.calibration_start_ms = now_ms
            self.calibration_samples = []
        if live_valid and position_cm is not None:
            self.calibration_samples.append(float(position_cm))
            if len(self.calibration_samples) > 240:
                self.calibration_samples.pop(0)
        elapsed_ms = time.ticks_diff(now_ms, self.calibration_start_ms)
        if elapsed_ms < CALIBRATION_DURATION_MS:
            return
        if len(self.calibration_samples) < CALIBRATION_MIN_SAMPLES:
            self.calibration_start_ms = now_ms
            self.calibration_samples = []
            print("CALIBRATION: insufficient vision samples; restarting")
            return
        target_cm = median(self.calibration_samples)
        max_deviation_cm = max(
            abs(sample_cm - target_cm)
            for sample_cm in self.calibration_samples
        )
        if max_deviation_cm > CALIBRATION_MAX_DEVIATION_CM:
            self.calibration_start_ms = now_ms
            self.calibration_samples = []
            print(
                "CALIBRATION: unstable spread=%.3f cm; restarting"
                % max_deviation_cm
            )
            return
        global TASK_HOLD_POSITION_TARGET_CM
        TASK_HOLD_POSITION_TARGET_CM = float(target_cm)
        self.calibration_start_ms = None
        self.calibration_samples = []
        self.controller.clear_integral()
        self.controller.tuning_dirty = True
        print(
            "CALIBRATION: accepted target=%+.3f cm spread=%.3f cm"
            % (target_cm, max_deviation_cm)
        )
        self.set_task_mode(TASK_MODE_HOLD_POSITION, now_ms)

    def update(
        self,
        position_cm,
        velocity_cm_s,
        measurement_live,
        confidence,
        measurement_ms,
        now_ms,
    ):
        self.last_position_cm = position_cm
        self.last_velocity_cm_s = velocity_cm_s
        self.last_measurement_live = bool(measurement_live)
        self.last_confidence = confidence
        self._poll_replies()
        self._check_ack_timeout(now_ms)
        self.controller.maybe_save(now_ms)
        if self.fault is not None:
            return

        live_valid = (
            measurement_live
            and position_cm is not None
            and confidence is not None
            and confidence >= CONFIDENCE_THRESHOLD
            and measurement_ms is not None
        )
        if live_valid:
            # 使用 AI 输入帧时间戳，确保推理耗时被计入视觉年龄。
            self.last_live_vision_ms = measurement_ms
            self.consecutive_valid_frames += 1
        else:
            self.consecutive_valid_frames = 0

        if BALANCE_TASK_MODE == TASK_MODE_CALIBRATION:
            self._update_calibration(
                live_valid,
                position_cm,
                now_ms,
            )
            return

        if not self.armed:
            if self.remote_stop_latched:
                return
            cooldown_ready = (
                self.last_disarm_ms is None
                or time.ticks_diff(now_ms, self.last_disarm_ms)
                >= MOTOR_REARM_COOLDOWN_MS
            )
            if BALANCE_TASK_ENABLED and BALANCE_TASK_MODE == TASK_MODE_HOLD_ZERO:
                auto_arm_target_cm = TASK_HOLD_ZERO_TARGET_CM
                auto_arm_window_cm = TASK_HOLD_REARM_WINDOW_CM
            elif (
                BALANCE_TASK_ENABLED
                and BALANCE_TASK_MODE == TASK_MODE_HOLD_POSITION
            ):
                auto_arm_target_cm = TASK_HOLD_POSITION_TARGET_CM
                auto_arm_window_cm = TASK_HOLD_REARM_WINDOW_CM
            elif BALANCE_TASK_ENABLED:
                auto_arm_target_cm = TASK_START_POSITION_CM
                auto_arm_window_cm = (
                    TASK_RETURN_AUTO_ARM_WINDOW_CM
                    if TASK_RETURN_ENABLED
                    else MOTOR_AUTO_ARM_WINDOW_CM
                )
            else:
                auto_arm_target_cm = MOTOR_TARGET_CM
                auto_arm_window_cm = MOTOR_AUTO_ARM_WINDOW_CM
            if (
                live_valid
                and cooldown_ready
                and self.consecutive_valid_frames >= MOTOR_AUTO_ARM_VALID_FRAMES
                and abs(position_cm - auto_arm_target_cm) <= auto_arm_window_cm
                and abs(velocity_cm_s) <= MOTOR_AUTO_ARM_MAX_SPEED_CM_S
            ):
                self.armed = True
                self.enabled = False
                self.ready = False
                self.state = "ENABLING"
                self.controller.reset()
                self.motion_task.reset()
                self.last_control_ms = None
                self.next_control_ms = None
                self.consecutive_ack_misses = 0
                print("Motor auto-armed from stable vision")
                self._write_command(
                    self._build_enable(), self.FUNCTION_ENABLE, now_ms
                )
            return

        if self.last_live_vision_ms is None:
            self._stop_and_disarm("VISION_LOST", now_ms)
            return
        vision_age_ms = time.ticks_diff(now_ms, self.last_live_vision_ms)
        if vision_age_ms > MOTOR_VISION_STOP_AGE_MS:
            self._stop_and_disarm("VISION_LOST", now_ms)
            return

        if not self.enabled:
            self.state = "ENABLING"
            self._write_command(self._build_enable(), self.FUNCTION_ENABLE, now_ms)
            return
        if not self.ready:
            self.state = "CONFIGURING"
            self._write_command(self._build_config(), self.FUNCTION_CONFIG, now_ms)
            return

        if self.open_loop_angle_deg is not None:
            if not self._control_due(now_ms) or self.outstanding_function is not None:
                return
            dt_s = self._control_dt_s(now_ms)
            if (
                vision_age_ms <= MOTOR_VISION_SAFE_AGE_MS
                and position_cm is not None
            ):
                self.vision_safe_active = False
                self.state = "OPEN_LOOP"
                self.controller.output_angle_deg = self.open_loop_angle_deg
                correction_pulse = (
                    self.open_loop_angle_deg * BALANCE_PULSES_PER_DEG
                )
            else:
                self.open_loop_angle_deg = None
                self.controller.clear_integral()
                self.vision_safe_active = True
                self.state = "VISION_SAFE"
                correction_pulse = self.controller.slew_to_zero(dt_s)
            self._send_position(correction_pulse, now_ms)
            return

        previous_target_cm = self.motion_task.target_cm
        task_state = self.motion_task.update(
            position_cm,
            velocity_cm_s,
            live_valid,
            now_ms,
        )
        target_cm = self.motion_task.target_cm
        if target_cm != previous_target_cm:
            # 切换 0/+5/-5 时清积分和自动调参历史，但保留当前平滑角度输出。
            self.controller.clear_integral()
            if (
                BALANCE_TASK_MODE == TASK_MODE_SEQUENCE
                and task_state == self.motion_task.MOVE_PLUS
                and abs(target_cm - TASK_PLUS_TARGET_CM) < 0.001
            ):
                self.controller.trigger_sequence_start_feedforward(
                    position_cm,
                    now_ms,
                )
        if task_state == self.motion_task.TIMEOUT:
            self._latch_fault("TASK_TIMEOUT")
            return

        if not self._control_due(now_ms) or self.outstanding_function is not None:
            return

        dt_s = self._control_dt_s(now_ms)
        if (
            vision_age_ms <= MOTOR_VISION_SAFE_AGE_MS
            and position_cm is not None
            and not self.control_overrun_active
        ):
            self.vision_safe_active = False
            self.state = "RUN"
            correction_pulse = self.controller.step(
                target_cm,
                position_cm,
                velocity_cm_s,
                dt_s,
                now_ms,
                live_valid,
            )
        else:
            if not self.vision_safe_active:
                self.controller.clear_integral()
                self.vision_safe_active = True
                self.state = "VISION_SAFE"
            correction_pulse = self.controller.slew_to_zero(dt_s)
        self._send_position(correction_pulse, now_ms)

    def teach_hold_position(self, now_ms):
        global TASK_HOLD_POSITION_TARGET_CM
        if (
            self.fault is not None
            or not self.armed
            or not self.enabled
            or not self.ready
            or not self.last_measurement_live
            or self.last_position_cm is None
            or self.last_velocity_cm_s is None
            or self.last_confidence is None
            or self.last_confidence < CONFIDENCE_THRESHOLD
            or self.consecutive_valid_frames < MOTOR_AUTO_ARM_VALID_FRAMES
            or abs(self.last_velocity_cm_s) > 0.60
        ):
            return None
        TASK_HOLD_POSITION_TARGET_CM = float(self.last_position_cm)
        self.controller.clear_integral()
        self.controller.start_feedforward_start_ms = None
        self.controller.tuning_dirty = True
        self.motion_task.reset()
        self.last_control_ms = now_ms
        self.next_control_ms = now_ms
        self.control_overrun_active = False
        return TASK_HOLD_POSITION_TARGET_CM

    def set_task_mode(self, mode, now_ms):
        global BALANCE_TASK_MODE
        if mode not in (
            TASK_MODE_HOLD_ZERO,
            TASK_MODE_HOLD_POSITION,
            TASK_MODE_SEQUENCE,
            TASK_MODE_CALIBRATION,
        ):
            return False
        if mode == TASK_MODE_CALIBRATION:
            global TASK_HOLD_POSITION_TARGET_CM
            BALANCE_TASK_MODE = TASK_MODE_CALIBRATION
            self._stop_and_disarm("CALIBRATION", now_ms)
            self.motion_task.reset()
            self.calibration_start_ms = now_ms
            self.calibration_samples = []
            self.state = "CALIBRATING"
            print(
                "CALIBRATION: hold ball for %d ms"
                % CALIBRATION_DURATION_MS
            )
            return True
        resume_from_remote_stop = self.remote_stop_latched
        self.remote_stop_latched = False
        if mode == BALANCE_TASK_MODE:
            if resume_from_remote_stop:
                self.consecutive_valid_frames = 0
                self.last_disarm_ms = now_ms
                self.state = "DISARMED"
                print("Remote stop released by MODE %s" % mode)
                return True
            return False

        previous_output_angle_deg = self.controller.output_angle_deg
        self.open_loop_angle_deg = None
        BALANCE_TASK_MODE = mode
        self.motion_task.reset()
        self.controller.reset()
        # Preserve the last physical tilt and let the normal slew limiter move
        # into the new mode instead of commanding an immediate center jump.
        self.controller.output_angle_deg = previous_output_angle_deg
        self.last_control_ms = now_ms
        self.next_control_ms = now_ms
        self.control_overrun_active = False
        print(
            "LCD MODE: %s, target=%+.2f cm"
            % (BALANCE_TASK_MODE, self.motion_task.target_cm)
        )
        return True

    def status_text(self):
        if self.fault is not None:
            return "MOTOR FAULT %s" % self.fault
        if BALANCE_TASK_MODE == TASK_MODE_CALIBRATION:
            return "MOTOR CALIBRATING"
        return "MOTOR %s %s P%d A%+.2f" % (
            self.state,
            self.motion_task.status_text(),
            self.last_pulses,
            self.controller.output_angle_deg,
        )

    def deinit(self):
        self.controller.maybe_save(time.ticks_ms(), force=True)
        self._write_stop_immediately()
        if self.uart is not None:
            try:
                self.uart.deinit()
            except BaseException as error:
                print("Motor UART deinit error:", error)
            self.uart = None
        print("Direct motor control stopped")


class RodCalibration:
    """固定画面中心为零点的一维水平坐标系。"""

    # v2 不再使用 Y 方向参与位置换算，并把画面绝对中心固定为 0 cm。
    # v3 将三点标定位置改为 -5、0、+5 cm；强制旧标定文件失效。
    VERSION = 3

    def __init__(self, origin_x, origin_y, slope_x, slope_y):
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        # Coordinate convention: screen-left is +X, screen-right is -X.
        self.slope_x = -abs(float(slope_x))
        # 保留字段以兼容标定文件结构，但一维跟踪不再使用斜轴 slope_y。
        self.slope_y = 0.0
        if abs(self.slope_x) < 1.0:
            raise ValueError("Invalid calibration scale")
        self.pixels_per_cm = abs(self.slope_x)
        self.vertical_pixels_per_cm = self.pixels_per_cm * DISPLAY_Y_TO_X_SCALE

    def pixel_to_cm(self, pixel_x, pixel_y):
        # Y 仅用于 ROI 判定，不得影响发送给下位机的位置。
        return (float(pixel_x) - self.origin_x) / self.slope_x

    def cm_to_pixel(self, position_cm):
        return (
            self.origin_x + self.slope_x * position_cm,
            self.origin_y,
        )

    def perpendicular_distance_px(self, pixel_x, pixel_y):
        return abs(float(pixel_y) - self.origin_y)

    def pipe_outer_half_height_px(self):
        return self.vertical_pixels_per_cm * PIPE_OUTER_DIAMETER_CM * 0.5

    def pipe_inner_half_height_px(self):
        return self.vertical_pixels_per_cm * PIPE_INNER_DIAMETER_CM * 0.5

    def accepts(self, pixel_x, pixel_y):
        position_cm = self.pixel_to_cm(pixel_x, pixel_y)
        limit = ROD_LENGTH_CM * 0.5 + ROD_END_MARGIN_CM
        return (
            -limit <= position_cm <= limit
            and ball_center_inside_blue_roi(pixel_y, self)
        )

    def save(self, path=CALIBRATION_FILE):
        data = {
            "version": self.VERSION,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "slope_x": self.slope_x,
            "slope_y": self.slope_y,
        }
        with open(path, "w") as file_obj:
            file_obj.write(ujson.dumps(data))

    @classmethod
    def load(cls, path=CALIBRATION_FILE):
        try:
            with open(path, "r") as file_obj:
                data = ujson.loads(file_obj.read())
            if int(data.get("version", 0)) != cls.VERSION:
                print("Calibration version mismatch; recalibration required")
                return None
            calibration = cls(
                data["origin_x"],
                data["origin_y"],
                data["slope_x"],
                data["slope_y"],
            )
            print(
                "Calibration loaded: origin=(%.1f, %.1f), %.2f px/cm"
                % (
                    calibration.origin_x,
                    calibration.origin_y,
                    calibration.pixels_per_cm,
                )
            )
            return calibration
        except BaseException as error:
            print("No usable calibration file:", error)
            return None


def pipe_guide_vertical_bounds(calibration=None):
    """返回屏幕上可见的 (中心Y, 上蓝线Y, 下蓝线Y)。

    绘制和检测共用同一个几何函数，避免调线后 ROI 忘记同步。
    """
    center_y = int(SCREEN_CENTER_Y)
    if calibration is None:
        outer_half_height = PIPE_PREVIEW_HALF_HEIGHT_PX + PIPE_ROI_MARGIN_PX
    else:
        outer_half_height = (
            calibration.pipe_outer_half_height_px() + PIPE_ROI_MARGIN_PX
        )
    top_y = max(0, int(center_y - outer_half_height))
    bottom_y = min(DISPLAY_SIZE[1] - 1, int(center_y + outer_half_height))
    return center_y, top_y, bottom_y


def ball_center_inside_blue_roi(center_y, calibration=None):
    """只接受球心位于上下蓝线之间的候选。"""
    _, top_y, bottom_y = pipe_guide_vertical_bounds(calibration)
    extra_margin = max(0.0, BALL_DETECTION_EXTRA_ROI_MARGIN_PX)
    return top_y - extra_margin <= center_y <= bottom_y + extra_margin


def ball_box_crosses_green_line(box_y, box_height):
    """只有绿色中心线穿过检测框时，该候选才可能是钢球。"""
    box_top = float(box_y)
    box_bottom = box_top + float(box_height)
    tolerance = max(0.0, BALL_GREEN_LINE_TOLERANCE_PX)
    return (
        box_top - tolerance
        <= SCREEN_CENTER_Y
        <= box_bottom + tolerance
    )


def select_ball_detection(results, calibration=None, preferred_x=None):
    """在固定水平水管 ROI 内选择钢球，返回球心、置信度和检测框。"""
    if not results or len(results) < 3:
        return None

    boxes, class_ids, scores = results[0], results[1], results[2]
    if boxes is None or len(boxes) == 0:
        return None

    best = None
    best_rank = -1.0
    for index in range(len(boxes)):
        if int(class_ids[index]) != BALL_CLASS_ID:
            continue

        x, y, width, height = boxes[index]
        x = float(x)
        y = float(y)
        width = float(width)
        height = float(height)
        score = float(scores[index])
        if width <= 1.0 or height <= 1.0:
            continue

        # 显示画面横向可能经过缩放，因此只排除明显不是球的细长框。
        aspect_ratio = width / height
        if (
            aspect_ratio < BALL_ASPECT_RATIO_MIN
            or aspect_ratio > BALL_ASPECT_RATIO_MAX
        ):
            continue

        # 固定水管上的硬约束：绿线没有穿过该检测框，就一定不是钢球。
        if not ball_box_crosses_green_line(y, height):
            continue

        center_x = x + width * 0.5
        center_y = y + height * 0.5
        # 这是硬边界：蓝线外候选不进入排序、跟踪、距离计算或串口。
        if not ball_center_inside_blue_roi(center_y, calibration):
            continue
        if calibration is None:
            center_deviation = abs(center_y - SCREEN_CENTER_Y)
            outer_half_height = PIPE_PREVIEW_HALF_HEIGHT_PX
            inner_half_height = 1.0
        else:
            if not calibration.accepts(center_x, center_y):
                continue
            # 排序也以当前可见蓝线的中心为准，不受旧标定 origin_y 影响。
            center_deviation = abs(center_y - SCREEN_CENTER_Y)
            outer_half_height = calibration.pipe_outer_half_height_px()
            inner_half_height = calibration.pipe_inner_half_height_px()

        # 内径范围内不扣分；超出后轻微降低排序权重。置信度仍是主判据。
        outside_inner = max(0.0, center_deviation - inner_half_height)
        center_penalty = min(
            outside_inner / max(outer_half_height, 1.0), 1.0
        ) * 0.08
        proximity_penalty = 0.0
        if preferred_x is not None:
            distance_from_lock = abs(center_x - preferred_x)
            # 直接拒绝跨越大段画面的候选，宁可短时 PREDICT/LOST 也不跳假球。
            if distance_from_lock > BALL_TARGET_LOCK_GATE_PX:
                continue
            proximity_penalty = (
                distance_from_lock / BALL_TARGET_LOCK_GATE_PX
            ) * BALL_TARGET_PROXIMITY_PENALTY
        candidate_rank = score - center_penalty - proximity_penalty

        if candidate_rank > best_rank:
            best_rank = candidate_rank
            best = (center_x, center_y, score, x, y, width, height)

    return best


def draw_cross(image_obj, center_x, center_y, color=YELLOW, size=8):
    x = int(center_x)
    y = int(center_y)
    image_obj.draw_line(x - size, y, x + size, y, color=color, thickness=2)
    image_obj.draw_line(x, y - size, x, y + size, color=color, thickness=2)


def draw_locked_ball_box(image_obj, tracked_box):
    """绘制经延迟补偿后的轻量红框，不再绘制滞后的YOLO原始框和标签。"""
    if tracked_box is None:
        return

    center_x, center_y, width, height, _ = tracked_box
    margin = VISUAL_TRACK_BOX_MARGIN_PX
    left = max(0, int(center_x - width * 0.5 - margin))
    top = max(0, int(center_y - height * 0.5 - margin))
    right = min(
        DISPLAY_SIZE[0] - 1, int(center_x + width * 0.5 + margin)
    )
    bottom = min(
        DISPLAY_SIZE[1] - 1, int(center_y + height * 0.5 + margin)
    )
    if right > left and bottom > top:
        image_obj.draw_rectangle(
            left,
            top,
            right - left,
            bottom - top,
            color=RED,
            thickness=4,
        )


def draw_center_distance_line(image_obj, tracked_box):
    """从预测后的小球正中心连到画面正中心，线长随距离逐帧变化。"""
    if tracked_box is None:
        return

    ball_x = int(tracked_box[0])
    ball_y = int(tracked_box[1])
    center_x = int(SCREEN_CENTER_X)
    center_y = int(SCREEN_CENTER_Y)
    image_obj.draw_line(
        ball_x,
        ball_y,
        center_x,
        center_y,
        color=BLACK,
        thickness=CENTER_DISTANCE_LINE_THICKNESS,
    )


def keep_tracked_box_inside_blue_roi(tracked_box, calibration=None):
    """预测框也必须位于蓝区内，且绿线必须穿过框体。"""
    if tracked_box is None:
        return None
    center_y = tracked_box[1]
    box_height = tracked_box[3]
    box_top = center_y - box_height * 0.5
    if (
        ball_center_inside_blue_roi(center_y, calibration)
        and ball_box_crosses_green_line(box_top, box_height)
    ):
        return tracked_box
    return None


def draw_pipe_guides(image_obj, calibration=None):
    """画水管、中心零线以及中心左右约 5 cm 的边界竖线。"""
    center_x = int(SCREEN_CENTER_X)
    center_y, top_y, bottom_y = pipe_guide_vertical_bounds(calibration)

    if calibration is None:
        left_x = 0
        right_x = DISPLAY_SIZE[0] - 1
        boundary_negative_x = int(center_x - DISTANCE_REFERENCE_PX)
        boundary_positive_x = int(center_x + DISTANCE_REFERENCE_PX)
    else:
        endpoint_a = calibration.cm_to_pixel(-ROD_LENGTH_CM * 0.5)[0]
        endpoint_b = calibration.cm_to_pixel(ROD_LENGTH_CM * 0.5)[0]
        left_x = max(0, int(min(endpoint_a, endpoint_b)))
        right_x = min(DISPLAY_SIZE[0] - 1, int(max(endpoint_a, endpoint_b)))
        boundary_negative_x = int(
            calibration.cm_to_pixel(-DISTANCE_REFERENCE_CM)[0]
        )
        boundary_positive_x = int(
            calibration.cm_to_pixel(DISTANCE_REFERENCE_CM)[0]
        )

    # 青色：20 mm 外径的上下外壁；绿色：钢球水平运动中心轴。
    image_obj.draw_line(
        left_x, top_y, right_x, top_y, color=CYAN, thickness=2
    )
    image_obj.draw_line(
        left_x, center_y, right_x, center_y, color=GREEN, thickness=2
    )
    image_obj.draw_line(
        left_x, bottom_y, right_x, bottom_y, color=CYAN, thickness=2
    )
    # 黄色竖线的 X 永远是显示画面的正中心。只画水管上下边界之间（不贯穿全屏），
    # 减少每帧满屏竖线的渲染开销，避免红框经过竖线时卡顿。
    # 临时关闭竖线诊断：确认卡顿是否由竖线引起。
    # image_obj.draw_line(
    #     center_x,
    #     top_y,
    #     center_x,
    #     bottom_y,
    #     color=YELLOW,
    #     thickness=3,
    # )
    # 品红竖线：当前临时估算的 -5 cm 和 +5 cm 边界。不用红色，避免和红框重叠时视觉上拉成一长条。
    # 临时关闭竖线诊断：确认卡顿是否由竖线引起。
    # for boundary_x in (boundary_negative_x, boundary_positive_x):
    #     if 0 <= boundary_x < DISPLAY_SIZE[0]:
    #         image_obj.draw_line(
    #             boundary_x,
    #             top_y,
    #             boundary_x,
    #             bottom_y,
    #             color=MAGENTA,
    #             thickness=3,
    #     )


def fit_calibration(known_positions_cm, measured_points):
    """在固定屏幕中心零点的约束下，仅拟合水平方向 px/cm。"""
    denominator = sum(position * position for position in known_positions_cm)
    if denominator <= 0.0:
        raise RuntimeError("Calibration positions are invalid")
    slope_x = sum(
        known_positions_cm[i]
        * (measured_points[i][0] - SCREEN_CENTER_X)
        for i in range(len(known_positions_cm))
    ) / denominator
    endpoint_span_px = max(point[0] for point in measured_points) - min(
        point[0] for point in measured_points
    )
    if abs(slope_x) < 1.0:
        raise RuntimeError(
            "Calibration endpoint span is only %.1f px; move ball between %+.1f and %+.1f cm"
            % (
                endpoint_span_px,
                -DISTANCE_REFERENCE_CM,
                DISTANCE_REFERENCE_CM,
            )
        )
    return RodCalibration(
        SCREEN_CENTER_X, SCREEN_CENTER_Y, slope_x, 0.0
    )


def calibration_frame(pipeline, detector, message, mjpeg=None):
    """运行一帧标定画面，并返回当前最佳钢球检测。"""
    image_np = pipeline.get_frame()
    results = detector.run(image_np)
    # 不显示全帧原始框，蓝线外的候选在标定页也不会冒出来。
    pipeline.osd_img.clear()
    draw_pipe_guides(pipeline.osd_img)
    detection = select_ball_detection(results)
    if detection is not None:
        draw_cross(pipeline.osd_img, detection[0], detection[1])
        delta_x = detection[0] - SCREEN_CENTER_X
        delta_y = detection[1] - SCREEN_CENTER_Y
        center_distance_px = (delta_x * delta_x + delta_y * delta_y) ** 0.5
        center_distance_text = "DIST TO CENTER %.1f px  LIVE" % center_distance_px
    else:
        center_distance_text = "DIST TO CENTER --  LOST"
    pipeline.osd_img.draw_string_advanced(
        10, 10, 26, message, color=YELLOW
    )
    # 标定尚未完成、无法换算厘米时，也在左下角逐帧显示像素距离。
    pipeline.osd_img.draw_string_advanced(
        10,
        DISPLAY_SIZE[1] - 40,
        28,
        center_distance_text,
        color=YELLOW,
    )
    pipeline.show_image()
    if mjpeg is not None:
        mjpeg.send_if_due()
    return detection


def _capture_three_point_calibration(pipeline, detector, mjpeg=None):
    """依次采集 -5、0、+5 cm 三个位置，并将标定保存到 SD 卡。"""
    print("=" * 58)
    print("Three-point calibration is required.")
    print("Place the ball at each requested scale mark and keep it still.")
    print(
        "Positions in order: 0 cm, %+.1f cm, %+.1f cm"
        % (DISTANCE_REFERENCE_CM, -DISTANCE_REFERENCE_CM)
    )
    print(
        "IMPORTANT: physical 0 cm must overlap the screen-center vertical line x=%d"
        % int(SCREEN_CENTER_X)
    )
    print("=" * 58)

    measured_points = []
    for target_cm in CALIBRATION_POSITIONS_CM:
        print(
            "Place ball at %+.1f cm; timer starts only after it reaches the mark"
            % target_cm
        )

        settle_start = None
        while True:
            os.exitpoint()
            now_ms = time.ticks_ms()
            if settle_start is None:
                message = "CAL MOVE TO %+.1f cm" % target_cm
            else:
                elapsed = time.ticks_diff(now_ms, settle_start)
                remaining = (
                    CALIBRATION_SETTLE_MS - elapsed + 999
                ) // 1000
                message = "CAL HOLD %+.1f cm   %ds" % (
                    target_cm,
                    max(0, remaining),
                )
            detection = calibration_frame(
                pipeline,
                detector,
                message,
                mjpeg,
            )
            if (
                detection is not None
                and calibration_target_ready(target_cm, detection[0])
            ):
                if settle_start is None:
                    settle_start = now_ms
                    print(
                        "Ball reached %+.1f cm gate; hold still for %.1f s"
                        % (target_cm, CALIBRATION_SETTLE_MS / 1000.0)
                    )
                elif (
                    time.ticks_diff(now_ms, settle_start)
                    >= CALIBRATION_SETTLE_MS
                ):
                    break
            else:
                if settle_start is not None:
                    print("Ball left calibration gate; hold timer reset")
                settle_start = None

        sample_x = []
        sample_y = []
        attempts = 0
        max_attempts = CALIBRATION_SAMPLE_COUNT * 6
        while (
            len(sample_x) < CALIBRATION_SAMPLE_COUNT
            and attempts < max_attempts
        ):
            os.exitpoint()
            attempts += 1
            detection = calibration_frame(
                pipeline,
                detector,
                "SAMPLING %+.1f cm   %d/%d"
                % (target_cm, len(sample_x), CALIBRATION_SAMPLE_COUNT),
                mjpeg,
            )
            if (
                detection is not None
                and calibration_target_ready(target_cm, detection[0])
            ):
                sample_x.append(detection[0])
                sample_y.append(detection[1])

        if len(sample_x) < CALIBRATION_MIN_VALID_COUNT:
            raise RuntimeError(
                "Calibration failed at %+.1f cm: only %d valid detections"
                % (target_cm, len(sample_x))
            )

        point = (median(sample_x), median(sample_y))
        measured_points.append(point)
        print(
            "Captured %+.1f cm -> pixel (%.2f, %.2f), valid=%d/%d"
            % (
                target_cm,
                point[0],
                point[1],
                len(sample_x),
                attempts,
            )
        )
        gc.collect()

    calibration = fit_calibration(CALIBRATION_POSITIONS_CM, measured_points)
    residuals = []
    for index in range(len(CALIBRATION_POSITIONS_CM)):
        measured_cm = calibration.pixel_to_cm(
            measured_points[index][0], measured_points[index][1]
        )
        residuals.append(abs(measured_cm - CALIBRATION_POSITIONS_CM[index]))
        print(
            "Check target=%+.2f cm, measured=%+.3f cm, error=%.3f cm"
            % (
                CALIBRATION_POSITIONS_CM[index],
                measured_cm,
                abs(measured_cm - CALIBRATION_POSITIONS_CM[index]),
            )
        )

    maximum_error = max(residuals)
    print(
        "Calibration result: origin=(%.2f, %.2f), scale=%.3f px/cm, max error=%.3f cm"
        % (
            calibration.origin_x,
            calibration.origin_y,
            calibration.pixels_per_cm,
            maximum_error,
        )
    )
    if calibration.pixels_per_cm < 5.0:
        raise RuntimeError("Calibration scale is too small; move camera closer")
    zero_index = CALIBRATION_POSITIONS_CM.index(0.0)
    zero_offset_px = abs(measured_points[zero_index][0] - SCREEN_CENTER_X)
    if residuals[zero_index] > 0.5:
        raise RuntimeError(
            "Physical 0 cm is %.1f px away from screen center; align camera"
            % zero_offset_px
        )
    if maximum_error > 0.5:
        raise RuntimeError("Calibration error exceeds 0.5 cm; recalibrate")

    calibration.save()
    print("Calibration saved:", CALIBRATION_FILE)
    pipeline.osd_img.clear()
    draw_pipe_guides(pipeline.osd_img, calibration)
    pipeline.osd_img.draw_string_advanced(
        10, 10, 28, "CALIBRATION SAVED", color=YELLOW
    )
    pipeline.show_image()
    time.sleep_ms(1000)
    return calibration


def run_three_point_calibration(pipeline, detector, mjpeg=None):
    """标定失败时给出提示并自动重试，不让整个视觉/MJPEG程序退出。"""
    while True:
        try:
            return _capture_three_point_calibration(
                pipeline, detector, mjpeg
            )
        except (RuntimeError, ValueError) as error:
            print("Calibration invalid:", error)
            print(
                "Please move the ball to 0, %+.1f, %+.1f cm when prompted."
                % (DISTANCE_REFERENCE_CM, -DISTANCE_REFERENCE_CM)
            )
            print("At 0 cm, align the ball with the screen-center vertical line.")
            retry_start = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), retry_start) < 3000:
                os.exitpoint()
                calibration_frame(
                    pipeline,
                    detector,
                    "CAL FAILED - RETRY IN 3s",
                    mjpeg,
                )
            gc.collect()


def wifi_connect(ssid, password):
    """连接 2.4 GHz Wi-Fi，并返回 WLAN 对象。"""
    wlan = network.WLAN(0)

    # IDE 运行时网卡通常已经初始化；脱机冷启动需要主动复位并留出启动时间。
    try:
        wlan.active(False)
    except BaseException:
        pass
    time.sleep_ms(1000)
    wlan.active(True)
    time.sleep_ms(3000)

    for attempt in range(3):
        if wlan.isconnected():
            break

        print("Connecting WiFi:", ssid, "attempt", attempt + 1)
        try:
            wlan.disconnect()
        except BaseException:
            pass
        time.sleep_ms(500)
        wlan.connect(ssid, password)

        for wait_index in range(40):
            if wlan.isconnected():
                break
            if wait_index % 4 == 0:
                try:
                    print("WiFi status:", wlan.status())
                except BaseException:
                    print("WiFi waiting:", wait_index // 2, "s")
            time.sleep_ms(500)

        if not wlan.isconnected():
            # 某些固件冷启动后第一次连接会失败，重启网卡再试。
            try:
                wlan.active(False)
                time.sleep_ms(1000)
                wlan.active(True)
                time.sleep_ms(2000)
            except BaseException as error:
                print("WiFi reset failed:", error)

    if not wlan.isconnected():
        raise RuntimeError(
            "WiFi connection failed; check 2.4 GHz/WPA2 hotspot and password"
        )

    for _ in range(100):
        if wlan.ifconfig()[0] != "0.0.0.0":
            break
        time.sleep_ms(50)

    print("Network:", wlan.ifconfig())
    return wlan


def start_mjpeg_safely(mjpeg):
    """Start WBC/JPEG without taking down local vision on encoder failure."""
    if mjpeg is None:
        return False
    try:
        mjpeg.start()
        return True
    except Exception as error:
        print("MJPEG disabled because startup failed:")
        sys.print_exception(error)
        try:
            mjpeg.stop()
        except BaseException:
            pass
        return False


def _mjpeg_loop(mjpeg):
    """独立线程按 MJPEG_FPS 节拍发送图传，与 AI 主循环解耦。

    主循环推理约 12~13 fps；若在主循环里调 send_if_due，图传会被拖到
    同一帧率。WBC 抓的是 VO 合成帧，视频层按 sensor 原生帧率刷新，因此
    独立线程可以把图传稳定送到 MJPEG_FPS，仅 OSD 仍按 AI 帧率更新。
    encoder/client 只在本线程使用，避免与主循环竞争。
    """
    # 以 2 倍目标频率轮询，由 send_if_due 内部限频到 MJPEG_FPS。
    poll_ms = max(1, 1000 // (MJPEG_FPS * 2))
    while mjpeg.running:
        try:
            mjpeg.send_if_due()
        except BaseException as error:
            print("MJPEG thread error:", error)
        time.sleep_ms(poll_ms)
    print("MJPEG thread exited")


class WbcMjpegServer:
    """把 LCD/VO 最终合成画面硬编码为 JPEG，并通过 HTTP 连续发送。"""

    STREAM_HEADER = (
        b"HTTP/1.1 200 OK\r\n"
        b"Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        b"Cache-Control: no-store, no-cache, must-revalidate\r\n"
        b"Pragma: no-cache\r\n"
        b"Access-Control-Allow-Origin: *\r\n"
        b"Connection: close\r\n\r\n"
    )

    def __init__(self, local_ip, port=MJPEG_PORT):
        self.local_ip = local_ip
        self.port = port
        self.encoder = None
        self.server = None
        self.client = None
        self.client_address = None
        self.running = False
        self.frames_sent = 0
        self.clients_connected = 0
        self.client_frames_sent = 0
        self.last_send_ms = 0
        self.wbc_enabled = False
        self.capture_width = 0
        self.capture_height = 0
        self.last_accept_ms = 0

    @staticmethod
    def _errno(error):
        value = getattr(error, "errno", None)
        if value is None and getattr(error, "args", None):
            value = error.args[0]
        return value

    @staticmethod
    def _send_all(
        client, data, stall_timeout_ms=MJPEG_SEND_STALL_TIMEOUT_MS
    ):
        """非阻塞分块发送；超时就断开慢客户端，保证控制循环实时性。"""
        view = memoryview(data)
        offset = 0
        deadline = time.ticks_add(time.ticks_ms(), stall_timeout_ms)

        while offset < len(view):
            try:
                end = min(offset + MJPEG_SEND_CHUNK_BYTES, len(view))
                sent = client.send(view[offset:end])
                if sent:
                    offset += sent
                    deadline = time.ticks_add(time.ticks_ms(), stall_timeout_ms)
                    continue
            except OSError as error:
                if WbcMjpegServer._errno(error) not in (11, 110):
                    raise

            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                raise OSError(110)
            os.exitpoint()
            time.sleep_ms(1)

    def _close_client(self):
        if self.client is not None:
            try:
                self.client.close()
            except BaseException:
                pass
        self.client = None
        self.client_address = None
        self.client_frames_sent = 0
        # 客户端断开后只关闭编码器。WBC 一旦开启就保持到程序退出：
        # K230 反复 writeback(False/True) 会复用旧 OSD 合成缓冲，第二次
        # 打开网页时容易让程序绘制的线条出现残影。
        self._stop_encoder()

    def _start_encoder(self):
        """有客户端时才创建并启动 VENC JPEG 编码器；返回是否成功。"""
        if self.encoder is not None:
            return True
        try:
            self.encoder = Encoder()
            self.encoder.SetOutBufs(4, self.capture_width, self.capture_height)
            jpeg_payload_type = getattr(self.encoder, "PAYLOAD_TYPE_JPEG", None)
            if jpeg_payload_type is None:
                jpeg_payload_type = K_PT_JPEG
            attr = ChnAttrStr(
                jpeg_payload_type,
                0,
                self.capture_width,
                self.capture_height,
                src_frame_rate=MJPEG_FPS,
                dst_frame_rate=MJPEG_FPS,
                mjpeg_quality_factor=MJPEG_QUALITY,
            )
            self.encoder.Create(attr)
            self.encoder.Start()
            return True
        except BaseException as error:
            print("VENC JPEG encoder start failed:")
            sys.print_exception(error)
            self._stop_encoder()
            return False

    def _stop_encoder(self):
        """停止并销毁编码器；幂等。"""
        if self.encoder is None:
            return
        try:
            self.encoder.Stop()
        except BaseException:
            pass
        try:
            self.encoder.Destroy()
        except BaseException:
            pass
        self.encoder = None

    def _accept_client(self):
        if self.client is not None or self.server is None:
            return

        try:
            client, address = self.server.accept()
        except OSError as error:
            if self._errno(error) not in (11, 110):
                print("MJPEG accept error:", error)
            return

        client.setblocking(False)
        # 有客户端才开 WBC，无人观看时 WBC 关闭以保持高帧率。
        if not self.wbc_enabled:
            if not RawDisplay.writeback(True):
                print("Display.writeback(True) failed; closing client")
                try:
                    client.close()
                except BaseException:
                    pass
                return
            self.wbc_enabled = True
        if not self._start_encoder():
            print("Encoder start failed; closing client")
            try:
                client.close()
            except BaseException:
                pass
            self._close_client()
            return
        self.client = client
        self.client_address = address
        try:
            # URL 本身直接返回 MJPEG 流，浏览器和 VLC 都能打开。
            self._send_all(
                client,
                self.STREAM_HEADER,
                MJPEG_STARTUP_STALL_TIMEOUT_MS,
            )
        except OSError:
            self._close_client()
            return

        self.clients_connected += 1
        print("MJPEG client:", address)

    def start(self):
        if self.running:
            return
        if not RawDisplay.inited():
            raise RuntimeError("Display must be initialized before WBC/MJPEG")

        self.capture_width = align_up(RawDisplay.width(), 16)
        self.capture_height = RawDisplay.height()
        print("WBC MJPEG capture size:", self.capture_width, "x", self.capture_height)

        try:
            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            address = socket.getaddrinfo("0.0.0.0", self.port)[0][-1]
            self.server.bind(address)
            self.server.listen(1)
            self.server.setblocking(False)
            # 编码器和 WBC 都不在这里开，改由 _accept_client 在有客户端连上时才启动，
            # 避免无人观看时 VENC/WBC 把主循环从 55 FPS 拖到 ~8 FPS。
        except BaseException:
            if self.server is not None:
                self.server.close()
            self.server = None
            raise

        self.running = True
        now_ms = time.ticks_ms()
        self.last_send_ms = now_ms
        self.last_accept_ms = time.ticks_add(
            now_ms, -MJPEG_ACCEPT_POLL_MS
        )
        print("MJPEG URL:", self.get_url())
        print("lazy encoder+WBC: deferred to client connect, wbc_enabled=%s" % self.wbc_enabled)

    def get_url(self):
        return "http://%s:%d/" % (self.local_ip, self.port)

    def send_one_frame(self):
        """完成一次 WBC->JPEG->HTTP 发送；没有客户端时不编码。"""
        if not self.running or self.client is None:
            return False

        frame_info = None
        stream = None
        stream_acquired = False
        close_client = False
        try:
            frame_info = RawDisplay.writeback_dump(100)
            if not frame_info:
                return False

            # 脱机运行时 VENC 偶尔会暂时无响应；有限超时可避免只显示
            # 第一帧后主循环永久阻塞。
            if (
                self.encoder.SendFrame(
                    frame_info, timeout=MJPEG_ENCODER_TIMEOUT_MS
                )
                != 0
            ):
                # 超时后原地重建编码器，但保持 HTTP 客户端连接。
                print("MJPEG: VENC SendFrame timeout; restarting encoder")
                self._stop_encoder()
                if not self._start_encoder():
                    print("MJPEG: encoder restart failed")
                    close_client = True
                return False

            stream = StreamData()
            if (
                self.encoder.GetStream(
                    stream, timeout=MJPEG_ENCODER_TIMEOUT_MS
                )
                != 0
            ):
                # GetStream 超时后编码器内部状态可能已经不同步。
                # 保持 HTTP 客户端连接，原地重建编码器，下一帧继续发送。
                print("MJPEG: VENC GetStream timeout; restarting encoder")
                self._stop_encoder()
                if not self._start_encoder():
                    print("MJPEG: encoder restart failed")
                    close_client = True
                return False
            stream_acquired = True

            jpeg_size = 0
            for index in range(stream.pack_cnt):
                jpeg_size += stream.data_size[index]
            part_header = (
                "--frame\r\n"
                "Content-Type: image/jpeg\r\n"
                "Content-Length: %d\r\n\r\n"
            ) % jpeg_size
            if self.client_frames_sent < MJPEG_STARTUP_FRAME_COUNT:
                stall_timeout_ms = MJPEG_STARTUP_STALL_TIMEOUT_MS
            else:
                stall_timeout_ms = MJPEG_SEND_STALL_TIMEOUT_MS

            self._send_all(
                self.client, part_header.encode(), stall_timeout_ms
            )
            for index in range(stream.pack_cnt):
                jpeg_pack = uctypes.bytearray_at(
                    stream.data[index], stream.data_size[index]
                )
                self._send_all(self.client, jpeg_pack, stall_timeout_ms)
            self._send_all(self.client, b"\r\n", stall_timeout_ms)
            self.frames_sent += 1
            self.client_frames_sent += 1
            return True
        except OSError:
            print("MJPEG client disconnected")
            close_client = True
            return False
        except BaseException as error:
            print("MJPEG frame error:")
            sys.print_exception(error)
            close_client = True
            return False
        finally:
            if stream_acquired:
                try:
                    self.encoder.ReleaseStream(stream)
                except BaseException as error:
                    print("MJPEG ReleaseStream error:", error)
            if frame_info is not None:
                del frame_info
            if close_client:
                self._close_client()

    def send_if_due(self):
        """轮询客户端，并按 MJPEG_FPS 限频；供检测和标定流程共同调用。"""
        if not self.running:
            return False

        now_ms = time.ticks_ms()
        # K230 上 accept() 很重，每帧都调会把 FPS 从 55 拖到 ~8；限频轮询。
        if self.client is None and time.ticks_diff(now_ms, self.last_accept_ms) >= MJPEG_ACCEPT_POLL_MS:
            self._accept_client()
            self.last_accept_ms = now_ms
        if self.client is None:
            return False

        if time.ticks_diff(now_ms, self.last_send_ms) < 1000 // MJPEG_FPS:
            return False
        result = self.send_one_frame()
        self.last_send_ms = now_ms
        return result

    def stop(self):
        if self.encoder is None and self.server is None:
            return

        self.running = False
        self._close_client()

        if self.server is not None:
            try:
                self.server.close()
            except BaseException:
                pass

        # WBC 在客户端重连之间保持开启，只在整个服务停止时关闭。
        try:
            RawDisplay.writeback(False)
        except BaseException:
            pass
        self.wbc_enabled = False

        if self.encoder is not None:
            # 释放可能尚未取走的JPEG码流，再停止并销毁VENC通道。
            try:
                while True:
                    stream = StreamData()
                    if self.encoder.GetStream(stream, timeout=0) != 0:
                        break
                    self.encoder.ReleaseStream(stream)
            except BaseException:
                pass

            try:
                self.encoder.Stop()
                self.encoder.Destroy()
            except BaseException as error:
                print("MJPEG encoder stop error:", error)

        self.server = None
        self.encoder = None
        print("WBC MJPEG stopped")


def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    print("CONTROL BUILD:", CONTROL_BUILD_ID)

    wlan = None
    pipeline = None
    detector = None
    mjpeg = None
    calibration = None
    tracker = None
    visual_tracker = None
    motor_controller = None
    bluetooth_tuner = None
    lcd_mode_selector = None
    frame_count = 0
    mjpeg_started = False
    mjpeg_thread_started = False
    clock = time.clock()

    try:
        if ENABLE_MJPEG_STREAM:
            try:
                wlan = wifi_connect(WIFI_SSID, WIFI_PASSWORD)
                # 监听端口和 WBC 必须等首帧 LCD 画面就绪后再启动。
                if ENABLE_MJPEG_ENCODER:
                    mjpeg = WbcMjpegServer(wlan.ifconfig()[0])
                else:
                    print("WiFi only (encoder disabled for diagnosis)")
                    mjpeg = None
            except BaseException as error:
                print("WiFi/MJPEG disabled because initialization failed:")
                sys.print_exception(error)
                wlan = None
                mjpeg = None
        else:
            print("WiFi/MJPEG skipped for realtime tracking")

        # 只创建一套 Sensor/Display；模型继续使用第二个文件的原生解码。
        pipeline = NativeDisplayPipeline()
        pipeline.create()
        detector = NativeAnchorBaseDetector()
        print(
            "AI model: %s, input=%dx%d"
            % (
                detector.kmodel_path,
                detector.model_input_size[0],
                detector.model_input_size[1],
            )
        )

        if ENABLE_CALIBRATION and not FORCE_RECALIBRATE:
            calibration = RodCalibration.load()
        if ENABLE_CALIBRATION and calibration is None:
            # 先生成一帧 LCD 合成画面；按配置决定是否同步开启图传。
            calibration_frame(pipeline, detector, "CALIBRATION START")
            if mjpeg is not None:
                print("Starting MJPEG for calibration...")
                mjpeg_started = start_mjpeg_safely(mjpeg)
                if mjpeg_started:
                    print("Browser/VLC:", mjpeg.get_url())
                else:
                    mjpeg = None
            calibration = run_three_point_calibration(
                pipeline, detector, mjpeg
            )

        # 标定过程耗时较长，完成后重新建立 FPS 时钟。
        clock = time.clock()
        tracker = BallStateEstimator()
        visual_tracker = VisualBallTracker()

        # K230 UART2 直接接管 Emm42；不再向 MSPM0G3507 发送位置帧。
        if DIRECT_MOTOR_CONTROL_ENABLED:
            if calibration is None:
                print("Direct motor control disabled: centimeter calibration required")
            elif calibration.slope_x >= 0.0:
                print("Direct motor control disabled: calibration must be left=+x")
            else:
                try:
                    motor_controller = Emm42MotorController()
                except BaseException as error:
                    # 电机串口失败时保留视觉和图传，但绝不假装闭环仍在工作。
                    print("Direct motor control disabled because initialization failed:")
                    sys.print_exception(error)
                    motor_controller = None

        if motor_controller is not None:
            try:
                bluetooth_tuner = BluetoothTuner(
                    motor_controller.controller,
                    motor_controller,
                )
            except BaseException as error:
                print("Bluetooth tuner disabled because initialization failed:")
                sys.print_exception(error)
                bluetooth_tuner = None
            try:
                lcd_mode_selector = LcdModeSelector()
            except BaseException as error:
                print("LCD mode selector disabled because initialization failed:")
                sys.print_exception(error)
                lcd_mode_selector = None

        print("Steel-ball detection started")
        if calibration is None:
            direction_text = "left=+, right=-"
            print(
                "Temporary scale: %.1f px = %.1f cm, center x=%d (%s)"
                % (
                    DISTANCE_REFERENCE_PX,
                    DISTANCE_REFERENCE_CM,
                    int(SCREEN_CENTER_X),
                    direction_text,
                )
            )
        elif calibration.slope_x < 0.0:
            direction_text = "left=+, right=-"
        else:
            direction_text = "left=-, right=+"
        if calibration is not None:
            print(
                "1D coordinate: screen center x=%d is 0 cm (%s)"
                % (int(SCREEN_CENTER_X), direction_text)
            )
        print(
            "Pipe guide: OD=20.0 mm, ID=3.4 mm, center y=%d"
            % int(SCREEN_CENTER_Y)
        )
        if ENABLE_MJPEG_STREAM:
            print("Preparing first AI/display frame before MJPEG...")
        else:
            print("MJPEG disabled; realtime visual tracking has priority")

        while True:
            os.exitpoint()
            clock.tick()
            if bluetooth_tuner is not None:
                bluetooth_tuner.poll()
            if lcd_mode_selector is not None:
                lcd_mode_selector.poll(
                    motor_controller,
                    time.ticks_ms(),
                )

            try:
                frame = pipeline.get_frame()
                # 该时间戳属于AI输入帧；与绘制时刻之差就是需要补偿的处理延迟。
                ai_frame_ms = time.ticks_ms()
                results = detector.run(frame)
                frame = None
            except BaseException as error:
                print("Frame inference error:")
                sys.print_exception(error)
                # 即使推理失败，也必须继续轮询 ACK 和执行视觉超时急停。
                if motor_controller is not None:
                    motor_controller.update(
                        None,
                        0.0,
                        False,
                        None,
                        None,
                        time.ticks_ms(),
                    )
                gc.collect()
                continue

            # 不绘制原始框：它对应较早AI帧，会落后于CHN0视频层。
            pipeline.osd_img.clear()
            preferred_ball_x = visual_tracker.candidate_reference_x(ai_frame_ms)
            ball_detection = select_ball_detection(
                results, calibration, preferred_ball_x
            )
            ball_position_cm = None
            ball_offset_px = None
            control_position_cm = None
            confidence = None
            if ball_detection is not None:
                confidence = ball_detection[2]
                ball_offset_px = ball_detection[0] - SCREEN_CENTER_X
                if calibration is not None:
                    ball_position_cm = calibration.pixel_to_cm(
                        ball_detection[0], ball_detection[1]
                    )
                    control_position_cm = ball_position_cm
                else:
                    control_position_cm = (
                        -ball_offset_px / PROVISIONAL_PIXELS_PER_CM
                    )
            now_ms = time.ticks_ms()
            # 坐标属于 AI 输入帧，先在 ai_frame_ms 吸收测量，再预测到控制时刻 now_ms；
            # 避免把推理期间已经变旧的球位置误当成当前状态。
            tracker.observe(control_position_cm, ai_frame_ms)
            tracker.predict_to(now_ms)
            tracked_box = visual_tracker.update(
                ball_detection, ai_frame_ms, now_ms
            )
            tracked_box = keep_tracked_box_inside_blue_roi(
                tracked_box, calibration
            )
            draw_pipe_guides(pipeline.osd_img, calibration)
            draw_center_distance_line(pipeline.osd_img, tracked_box)
            # 红框中心改用 α-β 滤波位置（比原始检测平滑得多），牢牢锁住不乱动。
            # 检测球心每帧 ±5px 跳动经 α-β 平滑成连续曲线，红框不再左右抖。
            if tracked_box is not None and tracker.valid:
                if calibration is not None:
                    fx, fy = calibration.cm_to_pixel(tracker.position_cm)
                else:
                    fx = SCREEN_CENTER_X + tracker.position_cm * PROVISIONAL_PIXELS_PER_CM
                    fy = SCREEN_CENTER_Y
                tracked_box = (
                    fx, fy,
                    tracked_box[2], tracked_box[3], tracked_box[4],
                )
            draw_locked_ball_box(pipeline.osd_img, tracked_box)
            if lcd_mode_selector is not None:
                lcd_mode_selector.draw(pipeline.osd_img)

            # 控制优先于显示和图传：视觉状态估计、PID、Emm42 协议都在 K230 内完成。
            if motor_controller is not None:
                if tracker.valid and tracker.status != "LOST":
                    motor_position_cm = tracker.position_cm
                    motor_velocity_cm_s = tracker.velocity_cm_s
                else:
                    motor_position_cm = None
                    motor_velocity_cm_s = 0.0
                motor_controller.update(
                    motor_position_cm,
                    motor_velocity_cm_s,
                    tracker.accepted_this_frame,
                    confidence,
                    tracker.accepted_measurement_ms,
                    now_ms,
                )

            # 绿色十字表示滤波/预测后的钢球位置。
            if calibration is not None and tracker.valid:
                filtered_pixel = calibration.cm_to_pixel(tracker.position_cm)
                draw_cross(
                    pipeline.osd_img,
                    filtered_pixel[0],
                    filtered_pixel[1],
                    color=GREEN,
                    size=5,
                )

            if calibration is None and ball_offset_px is not None:
                raw_text = "RAW %+.1f px  C %.2f" % (
                    ball_offset_px,
                    confidence,
                )
            elif ball_position_cm is None:
                raw_text = "RAW --"
            else:
                raw_text = "RAW %+.2f cm  C %.2f" % (
                    ball_position_cm,
                    confidence,
                )

            # 速度由 K230 按真实帧间隔计算，标定模式和临时比例模式都可用；
            # 该值同时送入视觉 PID 的速度反馈项，不能只在标定模式显示。
            if tracker.valid:
                filtered_text = "POS %+.2f cm  VEL %+.2f cm/s" % (
                    tracker.position_cm,
                    tracker.velocity_cm_s,
                )
            else:
                filtered_text = "POS --  VEL --"

            # 临时模式直接显示有符号的水平像素偏移：左负、右正。
            if calibration is None and ball_offset_px is not None:
                approximate_position_cm = (
                    -ball_offset_px / PROVISIONAL_PIXELS_PER_CM
                )
                distance_text = "CENTER %+.2f cm  (%+.1f px)  LIVE" % (
                    approximate_position_cm,
                    ball_offset_px,
                )
            elif calibration is None:
                distance_text = "CENTER -- cm  (-- px)  LOST"
            # 标定模式优先使用本帧实测值，短暂漏检时才显示跟踪器预测值。
            else:
                if ball_position_cm is not None:
                    center_distance_cm = abs(ball_position_cm)
                    distance_pixel_x = ball_detection[0]
                    distance_source = "LIVE"
                elif tracker.valid:
                    center_distance_cm = abs(tracker.position_cm)
                    distance_pixel_x = filtered_pixel[0]
                    distance_source = "PRED"
                else:
                    center_distance_cm = None
                    distance_pixel_x = None
                    distance_source = "LOST"

                if center_distance_cm is not None:
                    if center_distance_cm < 0.05:
                        distance_direction = "CENTER"
                    elif distance_pixel_x < SCREEN_CENTER_X:
                        distance_direction = "LEFT"
                    else:
                        distance_direction = "RIGHT"
                    distance_text = "DIST TO CENTER %.2f cm  %s  %s" % (
                        center_distance_cm,
                        distance_direction,
                        distance_source,
                    )
                else:
                    distance_text = "DIST TO CENTER --  LOST"

            if calibration is None and ball_detection is not None:
                status_color = GREEN
                status_text = "APPROX MODE"
            elif calibration is None:
                status_color = RED
                status_text = "APPROX MODE  LOST"
            elif tracker.status == "TRACK" or tracker.status == "ACQUIRE":
                status_color = GREEN
                status_text = "%s  REJECT %d" % (
                    tracker.status,
                    tracker.rejected_count,
                )
            elif tracker.status == "PREDICT" or tracker.status == "REACQUIRE":
                status_color = YELLOW
                status_text = "%s  REJECT %d" % (
                    tracker.status,
                    tracker.rejected_count,
                )
            else:
                status_color = RED
                status_text = "%s  REJECT %d" % (
                    tracker.status,
                    tracker.rejected_count,
                )

            if SHOW_DEBUG_OVERLAY:
                pipeline.osd_img.draw_string_advanced(
                    10, 8, 24, raw_text, color=YELLOW
                )
                pipeline.osd_img.draw_string_advanced(
                    10, 38, 24, filtered_text, color=status_color
                )
                pipeline.osd_img.draw_string_advanced(
                    10,
                    68,
                    24,
                    status_text,
                    color=status_color,
                )
            if motor_controller is not None:
                if motor_controller.fault is not None:
                    motor_color = RED
                elif motor_controller.state == "RUN":
                    motor_color = GREEN
                else:
                    motor_color = YELLOW
                pipeline.osd_img.draw_string_advanced(
                    430,
                    8,
                    20,
                    motor_controller.status_text(),
                    color=motor_color,
                )
            pipeline.osd_img.draw_string_advanced(
                10, DISPLAY_SIZE[1] - 40, 28, distance_text, color=YELLOW
            )
            pipeline.show_image()

            frame_count += 1

            # 必须先让 LCD/VO 产生一帧完整合成画面，再启动 WBC。
            # 否则 writeback/encoder 可能在首帧前阻塞 AI 主循环。
            if ENABLE_MJPEG_STREAM and mjpeg is not None:
                if not mjpeg_started:
                    print("First AI/display frame ready; starting MJPEG...")
                    mjpeg_started = start_mjpeg_safely(mjpeg)
                    if mjpeg_started:
                        print("Browser/VLC:", mjpeg.get_url())
                    else:
                        mjpeg = None
                if mjpeg_started:
                    # 实测独立 _thread 跑图传会卡死主循环（K230 _thread 与 sensor/kpu 抢资源死锁），
                    # 改回主循环内同步发送。50 FPS 余量足够，同步带图传仍能稳过 20 FPS。
                    mjpeg.send_if_due()

            if frame_count % 30 == 0:
                if calibration is None:
                    if control_position_cm is None:
                        approximate_log = "--"
                    else:
                        approximate_log = "%+.3f" % control_position_cm
                    if tracker.valid:
                        print(
                            "Ball approximate=%s cm, velocity=%+.3f cm/s"
                            % (approximate_log, tracker.velocity_cm_s)
                        )
                    else:
                        print("Ball approximate=%s cm, velocity=--" % approximate_log)
                else:
                    if ball_position_cm is None:
                        raw_log = "--"
                    else:
                        raw_log = "%+.3f" % ball_position_cm
                    if tracker.valid:
                        filtered_log = "%+.3f" % tracker.position_cm
                        velocity_log = "%+.3f" % tracker.velocity_cm_s
                    else:
                        filtered_log = "--"
                        velocity_log = "--"
                    print(
                        "Ball raw=%s cm, filtered=%s cm, velocity=%s cm/s, status=%s, rejected=%d"
                        % (
                            raw_log,
                            filtered_log,
                            velocity_log,
                            tracker.status,
                            tracker.rejected_count,
                        )
                    )
                print("FPS:", clock.fps())
                print(
                    "Visual lock=%s, vx=%+.1f px/s, lead=%d ms"
                    % (
                        visual_tracker.status,
                        visual_tracker.velocity_x_px_s,
                        visual_tracker.prediction_lead_ms,
                    )
                )
                if mjpeg is not None:
                    print(
                        "MJPEG frames sent: %d, clients: %d, wbc=%s"
                        % (mjpeg.frames_sent, mjpeg.clients_connected, mjpeg.wbc_enabled)
                    )
                if motor_controller is not None:
                    if motor_controller.outstanding_function is None:
                        pending_text = "--"
                    else:
                        pending_text = "0x%02X#%s" % (
                            motor_controller.outstanding_function,
                            str(motor_controller.outstanding_tx_id),
                        )
                    print(
                        "Motor state=%s, pulse=%d, sent=%d, errors=%d, pending=%s, rx02=%d, rx9F=%d, orphan=%d"
                        % (
                            motor_controller.state,
                            motor_controller.last_pulses,
                            motor_controller.sent_count,
                            motor_controller.error_count,
                            pending_text,
                            motor_controller.rx_accept_count,
                            motor_controller.rx_complete_count,
                            motor_controller.rx_orphan_count,
                        )
                    )
                    print(
                        "%s PID state=%s target=%+.2f Kp=%.3f Ki=%.3f Kd=%.3f static=%.3f deg, angle=%+.3f deg"
                        % (
                            motor_controller.controller.active_profile,
                            motor_controller.motion_task.state,
                            motor_controller.motion_task.target_cm,
                            motor_controller.controller.active_position_gain,
                            motor_controller.controller.active_integral_gain,
                            motor_controller.controller.active_velocity_gain,
                            motor_controller.controller.active_static_angle_deg,
                            motor_controller.controller.output_angle_deg,
                        )
                    )
                    if motor_controller.motion_task.state in (
                        motor_controller.motion_task.MOVE_PLUS,
                        motor_controller.motion_task.MOVE_MINUS,
                    ):
                        print(
                            "SEQ vref=%+.2f limit=%.2f over=%.2f brake=%s"
                            % (
                                motor_controller.controller.profile_target_velocity_cm_s,
                                motor_controller.controller.profile_speed_limit_cm_s,
                                motor_controller.controller.profile_overspeed_cm_s,
                                str(motor_controller.controller.profile_brake_active),
                            )
                        )
                    if motor_controller.motion_task.state in (
                        motor_controller.motion_task.HOLD_ZERO,
                        motor_controller.motion_task.HOLD_POSITION,
                    ):
                        print(
                            "HOLD target=%+.2f cm, vref=%+.2f cm/s, safe=%.2f cm/s, overspeed=%.2f cm/s, hard=%s"
                            % (
                                motor_controller.motion_task.target_cm,
                                motor_controller.controller.hold_target_velocity_cm_s,
                                motor_controller.controller.hold_speed_limit_cm_s,
                                motor_controller.controller.hold_overspeed_cm_s,
                                str(motor_controller.controller.hold_hard_brake_active),
                            )
                        )
            if frame_count % 30 == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("User stopped")
    except BaseException as error:
        print("Program error:")
        sys.print_exception(error)
    finally:
        if lcd_mode_selector is not None:
            lcd_mode_selector.deinit()
        if bluetooth_tuner is not None:
            bluetooth_tuner.deinit()
        if motor_controller is not None:
            motor_controller.deinit()
        # 先停 WBC，再销毁 Display，顺序不能颠倒。
        if mjpeg is not None:
            mjpeg.stop()
        if detector is not None:
            detector.deinit()
        if pipeline is not None:
            pipeline.destroy()
        try:
            nn.shrink_memory_pool()
        except BaseException:
            pass
        gc.collect()
        print("Resources released")


if __name__ == "__main__":
    main()
