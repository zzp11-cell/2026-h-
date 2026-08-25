# Diansai 2026

2026 年全国大学生电子设计竞赛（电赛）备赛工程，包含小车端 MSPM0G3507 固件和视觉/电机端 Python 部署代码。

## 目录结构

- `CAR/`：MSPM0G3507 小车端固件（Keil 工程）
  - `CAR/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`：主工程
  - `CAR/empty.syscfg`：SysConfig 外设配置
- `VISION+MOTOR/`：视觉识别与电机控制部署代码

## 使用前需要修改

1. Wi-Fi 配置：在 `VISION+MOTOR/mp_deployment_source/det_video.py` 中填写
   `WIFI_SSID` 和 `WIFI_PASSWORD`。
2. SysConfig 路径：本机安装的 TI SysConfig 不在默认位置时，修改
   `CAR/tools/keil/syscfg.bat` 中的 `SYSCFG_PATH`。
