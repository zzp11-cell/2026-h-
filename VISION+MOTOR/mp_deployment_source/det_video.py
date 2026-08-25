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
    import _thread
except ImportError:
    _thread = None
import gc
import network
import os
import socket
import sys
import time
import uctypes
import ujson


# ======================== 配置区 ========================

WIFI_SSID = "ZP"
WIFI_PASSWORD = "zp201314520"

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
# 物理 LCD、UART 和 MJPEG 开关逻辑不受影响。需要在 IDE 看画面时再改回 True。
ENABLE_IDE_PREVIEW = False
# 当前阶段优先视觉控制实时性：关闭 WBC/JPEG/HTTP。以后需要图传再改 True。
ENABLE_MJPEG_STREAM = True
# 关闭左上角三行调试文字，只保留导轨、锁球框和左下角距离。
SHOW_DEBUG_OVERLAY = False

# 远距离小球置信度会明显降低。若误检增多可调回 0.25~0.35；
# 若仍有少量漏检可继续降到 0.15。
CONFIDENCE_THRESHOLD = 0.20
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
ENABLE_CALIBRATION = False

# ========= 临时像素/厘米换算：装置固定后只需修改下面两个宏 =========
# 例：实测球在 +5 cm 时距离画面中心 142 px，就保持 5.0 并把 160.0 改为 142.0。
DISTANCE_REFERENCE_CM = 5.0
DISTANCE_REFERENCE_PX = 148.0
PROVISIONAL_PIXELS_PER_CM = DISTANCE_REFERENCE_PX / DISTANCE_REFERENCE_CM
# ================================================================

CALIBRATION_POSITIONS_CM = (
    -DISTANCE_REFERENCE_CM,
    0.0,
    DISTANCE_REFERENCE_CM,
)
CALIBRATION_SETTLE_MS = 6000
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
BALL_DETECTION_EXTRA_ROI_MARGIN_PX = 0.0
# 真钢球的检测框必须穿过绿色中心线。5 px 用于包容运动时检测框的纵向抖动；
# 若还要更严可改成 0.0。
BALL_GREEN_LINE_TOLERANCE_PX = 5.0
# 尚未完成标定时用于显示水管边界的临时半高；与 8 px ROI 余量相加后，
# 蓝色上下线距当前 y=259 中心约 31 px，即 y=228 和 y=290。
PIPE_PREVIEW_HALF_HEIGHT_PX = 23.0
# 640x480 AI 画面显示到 800x480 OSD 后，X/Y 像素缩放比例不同。
DISPLAY_Y_TO_X_SCALE = (
    (DISPLAY_SIZE[1] / RGB888P_SIZE[1])
    / (DISPLAY_SIZE[0] / RGB888P_SIZE[0])
)
ROD_END_MARGIN_CM = 1.0

# 钢球位置状态估计参数。滤波使用真实帧间隔，不依赖固定 FPS。
FILTER_MEDIAN_WINDOW = 3
FILTER_ALPHA = 0.65
FILTER_BETA = 0.10
MAX_BALL_SPEED_CM_S = 60.0
OUTLIER_BASE_ALLOWANCE_CM = 1.0
PREDICTION_TIMEOUT_MS = 120
TRACK_LOST_TIMEOUT_MS = 200
REACQUIRE_RESET_MS = 350

# ========= LCD 实时锁球预测参数 =========
# CHN0 是实时视频层，AI 框来自较早的 CHN2 帧；按速度向前预测进行补偿。
VISUAL_TRACK_PREDICTION_ENABLED = True
# 实测 640 模型约 12~13 FPS，仅补偿推理时间仍会追在球后面。
# 这里额外补偿 CHN2 缓冲、CHN0/VO 扫描和 LCD 显示延迟；框落后就增大，超前就减小。
# 关闭 IDE 预览后显示链路延迟降低；60 ms 前馈偏少、80 ms 略偏多，
# 取 75 ms，使 12.5 FPS 时的总前馈约为 155 ms。
# 该参数只影响红框显示，不改变 UART 实测位置。
VISUAL_TRACK_EXTRA_LEAD_MS = 75
VISUAL_TRACK_MAX_LEAD_MS = 190
# 限制单次速度前馈的最大位移，防止异常速度把红框甩出很远。
VISUAL_TRACK_MAX_PREDICTION_OFFSET_PX = 120.0
# 低速时检测球心的 1~3 px 抖动会被长预测时间放大；低于此速度只跟实测中心。
VISUAL_TRACK_PREDICTION_MIN_SPEED_PX_S = 45.0
# 模型短暂漏检时只跨过约 2 帧，减少减速或反向后的盲目外推。
VISUAL_TRACK_HOLD_MS = 180
# 越大越紧跟速度变化，越小越平滑；滚球建议 0.80~0.95。
VISUAL_TRACK_VELOCITY_ALPHA = 0.88
VISUAL_TRACK_MAX_SPEED_PX_S = 2400.0
# 检测短暂丢失后保留速度预测锁定，避免使用过期球心排斥真实候选。
BALL_TARGET_LOCK_MEMORY_MS = 800
BALL_TARGET_LOCK_GATE_PX = 140.0
# 同一锁定范围内候选越靠近上一球心，排序权重越高。
BALL_TARGET_PROXIMITY_PENALTY = 0.30
# 检测框尺寸每帧采用新值的比例；调小可减少红框大小抖动。
VISUAL_TRACK_BOX_SIZE_ALPHA = 0.35
# 红框比检测框每侧扩大若干像素，降低快速运动时出框概率。
VISUAL_TRACK_BOX_MARGIN_PX = 7
# ========================================

# 第三步：将视觉状态发给下位机。庐山派 K230/K230D 的 UART2
# 使用 GPIO11(TX) 和 GPIO12(RX)，115200 8N1。
# 向 MSPM0 连续发送球位；MSPM0 端仍由机械标定安全锁决定是否允许电机运动。
UART_TELEMETRY_ENABLED = True
UART_TELEMETRY_BAUDRATE = 115200
UART_TELEMETRY_HZ = 20
UART_TELEMETRY_TX_PIN = 11
UART_TELEMETRY_RX_PIN = 12
# MSPM0 协议允许 [-130.0, +130.0] mm；超出范围的测量发送为无效帧。
UART_TELEMETRY_POSITION_LIMIT_CM = 13.0

# ========= MJPEG 图传参数：后续只需在这里调整 =========
MJPEG_PORT = 8080
# 由独立线程 _mjpeg_loop 按此频率发送；不受 AI 推理帧率（~12.5fps）限制。
MJPEG_FPS = 20
# JPEG 质量范围 1~99；越高越清晰、带宽越大。建议先用 50。
MJPEG_QUALITY = 50
# 非阻塞 TCP 分块发送，网络堵塞时尽快丢掉客户端，避免拖慢控制循环。
MJPEG_SEND_CHUNK_BYTES = 16384
# VLC 偶发会超过 100 ms 不读取数据；稳定阶段允许 300 ms 网络抖动。
MJPEG_SEND_STALL_TIMEOUT_MS = 300
# 新连接建立和解码器预热更慢，仅前几帧给予更长宽限。
MJPEG_STARTUP_STALL_TIMEOUT_MS = 1000
MJPEG_STARTUP_FRAME_COUNT = 5
# ========================================================

# image 绘图接口使用 RGB；单类别强制为红色。
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
CYAN = (0, 255, 255)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
BLACK = (0, 0, 0)

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


class BallStateEstimator:
    """钢球一维α-β状态估计器，输出位置、速度、有效性和跟踪状态。"""

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

    def _initialize(self, measurement_cm, now_ms, status="ACQUIRE"):
        self.raw_history = []
        filtered_measurement = self._append_raw(measurement_cm)
        self.position_cm = filtered_measurement
        self.velocity_cm_s = 0.0
        self.raw_cm = float(measurement_cm)
        self.median_cm = filtered_measurement
        self.valid = True
        self.status = status
        self.last_update_ms = now_ms
        self.last_measurement_ms = now_ms
        self.initialized = True
        return self

    def update(self, measurement_cm, now_ms):
        """传入本帧原始厘米坐标；未检测到钢球时传入 None。"""
        if not self.initialized:
            if measurement_cm is None:
                self.raw_cm = None
                self.valid = False
                self.status = "LOST"
                self.missed_count += 1
                return self
            return self._initialize(measurement_cm, now_ms)

        elapsed_ms = time.ticks_diff(now_ms, self.last_update_ms)
        if elapsed_ms < 1:
            elapsed_ms = 1
        elapsed_seconds = elapsed_ms / 1000.0

        # 先按匀速模型预测到当前时刻。
        predicted_position = self.position_cm + self.velocity_cm_s * elapsed_seconds
        rod_limit = ROD_LENGTH_CM * 0.5 + ROD_END_MARGIN_CM
        predicted_position = self._clamp(predicted_position, -rod_limit, rod_limit)
        self.position_cm = predicted_position
        self.last_update_ms = now_ms

        since_measurement_ms = time.ticks_diff(now_ms, self.last_measurement_ms)
        accepted_measurement = measurement_cm

        if measurement_cm is not None:
            measurement_cm = float(measurement_cm)
            self.raw_cm = measurement_cm

            # 长时间丢球后允许在任意合法位置重新捕获，避免旧预测阻碍恢复。
            if since_measurement_ms > REACQUIRE_RESET_MS:
                return self._initialize(measurement_cm, now_ms, "REACQUIRE")

            # 允许位移由基础误差和物理最大速度共同决定。
            allowance_cm = (
                OUTLIER_BASE_ALLOWANCE_CM
                + MAX_BALL_SPEED_CM_S * elapsed_seconds
            )
            if abs(measurement_cm - predicted_position) > allowance_cm:
                accepted_measurement = None
                self.rejected_count += 1
        else:
            self.raw_cm = None

        if accepted_measurement is not None:
            filtered_measurement = self._append_raw(accepted_measurement)
            residual = filtered_measurement - predicted_position
            self.position_cm = predicted_position + FILTER_ALPHA * residual
            velocity_correction = FILTER_BETA * residual / elapsed_seconds
            self.velocity_cm_s += velocity_correction
            self.velocity_cm_s = self._clamp(
                self.velocity_cm_s,
                -MAX_BALL_SPEED_CM_S,
                MAX_BALL_SPEED_CM_S,
            )
            self.median_cm = filtered_measurement
            self.last_measurement_ms = now_ms
            self.valid = True
            self.status = "TRACK"
            return self

        # 没有测量或测量被判为异常：短时使用速度预测，随后明确失效。
        self.missed_count += 1
        since_measurement_ms = time.ticks_diff(now_ms, self.last_measurement_ms)
        if since_measurement_ms <= PREDICTION_TIMEOUT_MS:
            self.valid = True
            self.status = "PREDICT"
        else:
            self.valid = False
            self.status = "LOST"
            if since_measurement_ms > TRACK_LOST_TIMEOUT_MS:
                self.velocity_cm_s = 0.0
        return self


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
            center_x = float(detection[0])
            center_y = float(detection[1])
            width = float(detection[5])
            height = float(detection[6])

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

                    # 反向时立即采用新速度，避免旧方向预测把框甩到球的另一侧。
                    if instant_velocity * self.velocity_x_px_s < 0.0:
                        self.velocity_x_px_s = instant_velocity
                    else:
                        self.velocity_x_px_s += (
                            VISUAL_TRACK_VELOCITY_ALPHA
                            * (instant_velocity - self.velocity_x_px_s)
                        )
                else:
                    self.velocity_x_px_s = 0.0
            else:
                self.velocity_x_px_s = 0.0

            if self.valid:
                # 框尺寸轻微平滑，不对中心位置做低通，避免额外相位滞后。
                self.box_width += VISUAL_TRACK_BOX_SIZE_ALPHA * (
                    width - self.box_width
                )
                self.box_height += VISUAL_TRACK_BOX_SIZE_ALPHA * (
                    height - self.box_height
                )
            else:
                self.box_width = width
                self.box_height = height

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


class BallUartTransmitter:
    """发送 MSPM0 需要的 A5 5A 固定 10 字节钢球位置帧。"""

    FRAME_SIZE = 10
    COMMAND_BALL_POSITION = 0x01
    FLAG_VALID = 0x01

    def __init__(self):
        fpioa = FPIOA()
        fpioa.set_function(UART_TELEMETRY_TX_PIN, FPIOA.UART2_TXD)
        fpioa.set_function(UART_TELEMETRY_RX_PIN, FPIOA.UART2_RXD)
        self.uart = UART(
            UART.UART2,
            baudrate=UART_TELEMETRY_BAUDRATE,
            bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE,
        )
        self.send_interval_ms = max(1, 1000 // UART_TELEMETRY_HZ)
        self.last_send_ms = None
        self.sequence = 0
        self.sent_count = 0
        self.error_count = 0
        print(
            "UART2 telemetry: GPIO%d TX, GPIO%d RX, %d baud, %d Hz"
            % (
                UART_TELEMETRY_TX_PIN,
                UART_TELEMETRY_RX_PIN,
                UART_TELEMETRY_BAUDRATE,
                UART_TELEMETRY_HZ,
            )
        )
        print("UART frame: A5 5A 01 SEQ POS_L POS_H QUALITY FLAGS 00 XOR")

    @staticmethod
    def _round_to_int(value):
        if value >= 0:
            return int(value + 0.5)
        return int(value - 0.5)

    @staticmethod
    def _xor_checksum(data, count):
        checksum = 0
        for index in range(count):
            checksum ^= data[index]
        return checksum

    def _build_frame(self, position_cm, confidence, measurement_valid):
        position_value = None if position_cm is None else float(position_cm)
        confidence_value = None if confidence is None else float(confidence)
        valid = (
            measurement_valid
            and position_value is not None
            and confidence_value is not None
            and position_value == position_value
            and 0.0 <= confidence_value
            and confidence_value <= 1.0
            and -UART_TELEMETRY_POSITION_LIMIT_CM <= position_value
            and position_value <= UART_TELEMETRY_POSITION_LIMIT_CM
        )

        if valid:
            # 1 cm = 100 个 0.1 mm，按 int16 小端格式发送。
            position_0p1mm = self._round_to_int(position_value * 100.0)
            encoded_position = position_0p1mm & 0xFFFF
            quality = self._round_to_int(confidence_value * 100.0)
            if quality < 0:
                quality = 0
            elif quality > 100:
                quality = 100
            flags = self.FLAG_VALID
        else:
            encoded_position = 0
            quality = 0
            flags = 0

        frame = bytearray(self.FRAME_SIZE)
        frame[0] = 0xA5
        frame[1] = 0x5A
        frame[2] = self.COMMAND_BALL_POSITION
        frame[3] = self.sequence
        frame[4] = encoded_position & 0xFF
        frame[5] = (encoded_position >> 8) & 0xFF
        frame[6] = quality
        frame[7] = flags
        frame[8] = 0
        frame[9] = self._xor_checksum(frame, self.FRAME_SIZE - 1)
        return frame

    def send_if_due(self, position_cm, confidence, measurement_valid, now_ms):
        if self.last_send_ms is not None:
            elapsed_ms = time.ticks_diff(now_ms, self.last_send_ms)
            if elapsed_ms < self.send_interval_ms:
                return False
        self.last_send_ms = now_ms

        try:
            # 构帧也放在保护区内，异常测量不能让视觉/LCD/MJPEG 主循环退出。
            frame = self._build_frame(position_cm, confidence, measurement_valid)
            self.uart.write(frame)
            self.sent_count += 1
            self.sequence = (self.sequence + 1) & 0xFF
            return True
        except BaseException as error:
            self.error_count += 1
            if self.error_count <= 3 or self.error_count % 100 == 0:
                print("UART send error:", error)
            return False

    def deinit(self):
        if self.uart is not None:
            try:
                self.uart.deinit()
            except BaseException as error:
                print("UART deinit error:", error)
            self.uart = None
        print("UART2 telemetry stopped")


class RodCalibration:
    """固定画面中心为零点的一维水平坐标系。"""

    # v2 不再使用 Y 方向参与位置换算，并把画面绝对中心固定为 0 cm。
    # v3 将三点标定位置改为 -5、0、+5 cm；强制旧标定文件失效。
    VERSION = 3

    def __init__(self, origin_x, origin_y, slope_x, slope_y):
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.slope_x = float(slope_x)
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
    # 黄色竖线的 X 永远是显示画面的正中心，并贯穿整个画面高度。
    image_obj.draw_line(
        center_x,
        0,
        center_x,
        DISPLAY_SIZE[1] - 1,
        color=YELLOW,
        thickness=3,
    )
    # 红色竖线：当前临时估算的 -5 cm 和 +5 cm 边界。
    for boundary_x in (boundary_negative_x, boundary_positive_x):
        if 0 <= boundary_x < DISPLAY_SIZE[0]:
            image_obj.draw_line(
                boundary_x,
                0,
                boundary_x,
                DISPLAY_SIZE[1] - 1,
                color=RED,
                thickness=3,
            )


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
    endpoint_span_px = abs(measured_points[-1][0] - measured_points[0][0])
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
        "Positions: %+.1f cm, 0 cm, %+.1f cm"
        % (-DISTANCE_REFERENCE_CM, DISTANCE_REFERENCE_CM)
    )
    print(
        "IMPORTANT: physical 0 cm must overlap the screen-center vertical line x=%d"
        % int(SCREEN_CENTER_X)
    )
    print("=" * 58)

    measured_points = []
    for target_cm in CALIBRATION_POSITIONS_CM:
        print("Place ball at %+.1f cm; sampling starts in %.1f s"
              % (target_cm, CALIBRATION_SETTLE_MS / 1000.0))

        settle_start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), settle_start) < CALIBRATION_SETTLE_MS:
            os.exitpoint()
            elapsed = time.ticks_diff(time.ticks_ms(), settle_start)
            remaining = (CALIBRATION_SETTLE_MS - elapsed + 999) // 1000
            calibration_frame(
                pipeline,
                detector,
                "CAL: ball at %+.1f cm   %ds" % (target_cm, remaining),
                mjpeg,
            )

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
            if detection is not None:
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
                "Please move the ball to %+.1f, 0, %+.1f cm when prompted."
                % (-DISTANCE_REFERENCE_CM, DISTANCE_REFERENCE_CM)
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
    wlan.active(True)
    time.sleep_ms(300)

    if not wlan.isconnected():
        print("Connecting WiFi:", ssid)
        wlan.connect(ssid, password)
        for _ in range(40):
            if wlan.isconnected():
                break
            time.sleep_ms(500)

    if not wlan.isconnected():
        raise RuntimeError("WiFi connection failed (only 2.4 GHz is supported)")

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

        width = align_up(RawDisplay.width(), 16)
        height = RawDisplay.height()
        print("WBC MJPEG capture size:", width, "x", height)

        encoder_created = False
        encoder_started = False
        writeback_enabled = False
        try:
            # 使用本固件已有的 VENC JPEG 通道；JPEG 每帧独立，即为 MJPEG。
            self.encoder = Encoder()
            self.encoder.SetOutBufs(4, width, height)
            jpeg_payload_type = getattr(
                self.encoder, "PAYLOAD_TYPE_JPEG", None
            )
            if jpeg_payload_type is None:
                jpeg_payload_type = K_PT_JPEG
            attr = ChnAttrStr(
                jpeg_payload_type,
                0,
                width,
                height,
                src_frame_rate=MJPEG_FPS,
                dst_frame_rate=MJPEG_FPS,
                mjpeg_quality_factor=MJPEG_QUALITY,
            )
            self.encoder.Create(attr)
            encoder_created = True
            self.encoder.Start()
            encoder_started = True

            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            address = socket.getaddrinfo("0.0.0.0", self.port)[0][-1]
            self.server.bind(address)
            self.server.listen(1)
            self.server.setblocking(False)

            # WBC 抓取 video layer + OSD layer，图传会带红框和距离文字。
            if not RawDisplay.writeback(True):
                raise RuntimeError("Display.writeback(True) failed")
            writeback_enabled = True
        except BaseException:
            if writeback_enabled:
                RawDisplay.writeback(False)
            if self.server is not None:
                self.server.close()
            if encoder_started:
                try:
                    self.encoder.Stop()
                except BaseException:
                    pass
            if encoder_created:
                try:
                    self.encoder.Destroy()
                except BaseException:
                    pass
            self.server = None
            self.encoder = None
            raise

        self.running = True
        self.last_send_ms = time.ticks_ms()
        print("MJPEG URL:", self.get_url())

    def get_url(self):
        return "http://%s:%d/" % (self.local_ip, self.port)

    def send_one_frame(self):
        """完成一次 WBC->JPEG->HTTP 发送；没有客户端时不编码。"""
        if not self.running or self.client is None:
            return False

        frame_info = None
        stream = None
        stream_acquired = False
        try:
            frame_info = RawDisplay.writeback_dump(100)
            if not frame_info:
                return False

            # timeout=-1 保证 WBC 帧一定被编码器消费，避免回写缓冲池耗尽。
            if self.encoder.SendFrame(frame_info, timeout=-1) != 0:
                raise RuntimeError("VENC JPEG SendFrame failed")

            stream = StreamData()
            if self.encoder.GetStream(stream, timeout=-1) != 0:
                raise RuntimeError("VENC JPEG GetStream failed")
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
            self._close_client()
            return False
        except BaseException as error:
            print("MJPEG frame error:")
            sys.print_exception(error)
            self._close_client()
            return False
        finally:
            if stream_acquired:
                try:
                    self.encoder.ReleaseStream(stream)
                except BaseException as error:
                    print("MJPEG ReleaseStream error:", error)
            if frame_info is not None:
                del frame_info

    def send_if_due(self):
        """轮询客户端，并按 MJPEG_FPS 限频；供检测和标定流程共同调用。"""
        if not self.running:
            return False

        self._accept_client()
        if self.client is None:
            return False

        now_ms = time.ticks_ms()
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

        try:
            if not RawDisplay.writeback(False):
                print("Display.writeback(False) failed")
        except BaseException:
            pass

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

    wlan = None
    pipeline = None
    detector = None
    mjpeg = None
    calibration = None
    tracker = None
    visual_tracker = None
    telemetry = None
    frame_count = 0
    mjpeg_started = False
    mjpeg_thread_started = False
    clock = time.clock()

    try:
        if ENABLE_MJPEG_STREAM:
            try:
                wlan = wifi_connect(WIFI_SSID, WIFI_PASSWORD)
                # 监听端口和 WBC 必须等首帧 LCD 画面就绪后再启动。
                mjpeg = WbcMjpegServer(wlan.ifconfig()[0])
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

        # 标定模式发送实测厘米；临时模式按配置区的 px/cm 比例发送近似厘米。
        if UART_TELEMETRY_ENABLED:
            try:
                telemetry = BallUartTransmitter()
            except BaseException as error:
                # 串口未初始化成功时保留视觉、LCD 和 MJPEG，避免整机退出。
                print("UART2 telemetry disabled because initialization failed:")
                sys.print_exception(error)
                telemetry = None

        print("Steel-ball detection started")
        if calibration is None:
            direction_text = "left=-, right=+"
            print(
                "Temporary scale: %.1f px = %.1f cm, center x=%d (%s)"
                % (
                    DISTANCE_REFERENCE_PX,
                    DISTANCE_REFERENCE_CM,
                    int(SCREEN_CENTER_X),
                    direction_text,
                )
            )
        elif calibration.slope_x > 0.0:
            direction_text = "left=-, right=+"
        else:
            direction_text = "left=+, right=-"
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

            try:
                frame = pipeline.get_frame()
                # 该时间戳属于AI输入帧；与绘制时刻之差就是需要补偿的处理延迟。
                ai_frame_ms = time.ticks_ms()
                results = detector.run(frame)
                frame = None
            except BaseException as error:
                print("Frame inference error:")
                sys.print_exception(error)
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
                        ball_offset_px / PROVISIONAL_PIXELS_PER_CM
                    )
            now_ms = time.ticks_ms()
            tracked_box = visual_tracker.update(
                ball_detection, ai_frame_ms, now_ms
            )
            tracked_box = keep_tracked_box_inside_blue_roi(
                tracked_box, calibration
            )
            draw_pipe_guides(pipeline.osd_img, calibration)
            draw_center_distance_line(pipeline.osd_img, tracked_box)
            draw_locked_ball_box(pipeline.osd_img, tracked_box)

            if calibration is not None:
                tracker.update(ball_position_cm, now_ms)

            # 控制数据优先于显示和图传发送，避免 VLC 网络延迟影响下位机。
            if telemetry is not None:
                # 控制端自行滤波和估速，只发送本帧实测位置；短时预测不冒充实测。
                telemetry.send_if_due(
                    control_position_cm,
                    confidence,
                    ball_detection is not None,
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

            if calibration is not None and tracker.valid:
                filtered_text = "POS %+.2f cm  VEL %+.2f cm/s" % (
                    tracker.position_cm,
                    tracker.velocity_cm_s,
                )
            else:
                filtered_text = "POS --  VEL --"

            # 临时模式直接显示有符号的水平像素偏移：左负、右正。
            if calibration is None and ball_offset_px is not None:
                approximate_position_cm = (
                    ball_offset_px / PROVISIONAL_PIXELS_PER_CM
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
                    if _thread is not None:
                        # 图传交给独立线程按 MJPEG_FPS 节拍发送，
                        # 不再随 AI 主循环（~12.5fps）一起发送。
                        if not mjpeg_thread_started:
                            mjpeg_thread_started = True
                            _thread.start_new_thread(_mjpeg_loop, (mjpeg,))
                    else:
                        # 无 _thread 时回退为主循环内同步发送。
                        mjpeg.send_if_due()

            if frame_count % 30 == 0:
                if calibration is None:
                    if control_position_cm is None:
                        approximate_log = "--"
                    else:
                        approximate_log = "%+.3f" % control_position_cm
                    print("Ball approximate=%s cm" % approximate_log)
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
                        "MJPEG frames sent: %d, clients: %d"
                        % (mjpeg.frames_sent, mjpeg.clients_connected)
                    )
                if telemetry is not None:
                    print(
                        "UART frames sent: %d, errors: %d"
                        % (telemetry.sent_count, telemetry.error_count)
                    )
            if frame_count % 30 == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("User stopped")
    except BaseException as error:
        print("Program error:")
        sys.print_exception(error)
    finally:
        if telemetry is not None:
            telemetry.deinit()
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
