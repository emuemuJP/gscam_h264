# gscam_h264

Jetson Orin 上で V4L2 カメラ映像を GStreamer HW H.264 エンコードし、ROS2 トピックへ publish する専用ノード。
CPU 負荷を最小に抑えつつ最大 6 台のカメラを同時に扱えるよう設計されている。

## 概要

- V4L2 デバイス (`/dev/video*`) からの映像取り込み
- NVIDIA Tegra HW アクセラレーションによる H.264 エンコード (`nvvidconv` + `nvv4l2h264enc`)
- `sensor_msgs/CompressedImage` として H.264 byte-stream (AnnexB) を AU 単位で publish
- GStreamer PTS ベースのタイムスタンプにより、カメラ間同期に利用可能な `header.stamp` を付与

## アーキテクチャ

```
v4l2src (UYVY)
  -> nvvidconv (HW色変換 -> NV12, NVMM)
    -> nvv4l2h264enc (HWエンコード, SPS/PPS挿入)
      -> h264parse (byte-stream, AU alignment)
        -> queue (max-buffers=1, drop=true)
          -> appsink -> ROS2 publish
```

### 3 フェーズ起動

Tegra ドライバの初期化競合を回避するため、以下の 3 段階で起動する。

1. **Phase 1 (Create & Pause)** -- GStreamer パイプラインを生成し PAUSED 状態へ。クロックキャリブレーションもここで実施。
2. **Phase 2 (Start Threads)** -- カメラごとのフレーム取得スレッドを起動（`pull_sample` でブロック待機）。
3. **Phase 3 (Playing)** -- 各パイプラインを順次 PLAYING へ遷移（500 ms 間隔でドライバ競合を緩和）。

### 実行モデル

同一バイナリで 2 つの運用形態をパラメータ切替で使い分けられる。

| 形態 | 特徴 |
|------|------|
| **1 プロセス / N 台** | DDS・GStreamer の固定費を 1 回に抑え CPU/メモリ最小 |
| **1 プロセス / 1 台 x N** | カメラごとに障害分離。クラッシュが他カメラに波及しない |

## 動作環境

- Jetson Orin (Tegra) / Ubuntu
- ROS2 Foxy
- GStreamer 1.0 (`gstreamer1.0`, `gstreamer-plugins-base1.0`)
- V4L2 対応カメラ（UYVY 出力）

## ビルド

```bash
cd ~/ros2_foxy_ws
colcon build --packages-select gscam_h264
source install/setup.bash
```

## パラメータ

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `camera_count` | 6 | 1 プロセスで扱うカメラ台数 |
| `device_base` | `/dev/video` | デバイスパスのベース（自動番号付与時） |
| `device_index` | 0 | 開始デバイス番号 |
| `device` | -- | 単一カメラ指定時のデバイスパス |
| `devices` | -- | CSV 形式のデバイスリスト (`id:path,...`) |
| `width` | 1920 | 映像幅 (px) |
| `height` | 1280 | 映像高さ (px) |
| `fps` | 30 | フレームレート |
| `bitrate` | 8000000 | H.264 ビットレート (bps) |
| `iframeinterval` | 30 | I フレーム間隔 |
| `io_mode` | 2 | V4L2 I/O モード (2 = MMAP) |
| `use_pts_stamp` | true | GStreamer PTS をタイムスタンプに使用 |
| `topic_prefix` | `/cam` | トピック名プレフィクス |

デフォルト値は `cfg/params.yaml` で一括管理。

## トピック

- **名前**: `<topic_prefix><cam_id>/h264` (例: `/cam0/h264` ... `/cam5/h264`)
- **型**: `sensor_msgs/msg/CompressedImage`
- **format**: `"h264"`
- **data**: H.264 byte-stream (AnnexB) の AU 単位データ
- **QoS**: SensorDataQoS 相当 (BestEffort / KeepLast / depth=1 / Volatile)

## 起動方法

### 6 台同時（1 プロセス）

```bash
ros2 launch gscam_h264 h264_cameras.launch.py
```

### 6 台同時（カメラごとにプロセス分離、2 秒間隔スタガード起動）

```bash
ros2 launch gscam_h264 h264_6cameras.launch.py
```

### マルチカメラ（プロセス分離、台数パラメータ指定）

```bash
ros2 launch gscam_h264 h264_cameras_multi.launch.py
```

### 単一カメラ（デバッグ・テスト用）

```bash
ros2 launch gscam_h264 single_camera.launch.py cam_id:=0 device:=/dev/video0
```

## ファイル構成

```
gscam_h264/
├── CMakeLists.txt
├── package.xml
├── README.md
├── cfg/
│   └── params.yaml                    # デフォルトパラメータ
├── include/gscam_h264/
│   └── h264_camera_publisher.hpp      # ノードヘッダ (Pimpl)
├── launch/
│   ├── h264_cameras.launch.py         # 1プロセス N台
│   ├── h264_cameras_multi.launch.py   # Nプロセス 各1台
│   ├── h264_6cameras.launch.py        # 6台スタガード起動
│   └── single_camera.launch.py        # 単一カメラ
└── src/
    ├── h264_camera_publisher.cpp       # ノード実装
    └── h264_camera_publisher_main.cpp  # エントリポイント
```

