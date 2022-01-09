# Customized dataset: epfl_parking

## Info

- FPS: ~3.0
- Number of frames: 433
- Duration: 00:02:24.34
- Resolution: 1280x960
- Location: Underground parking lot on EPFL campus
- Source: images subsampled from Redmi K30Pro 64MP (4624x3475@15.0 FPS) with [ZHIYUN SMOOTH 4](https://zhiyun.us/collections/new-gimbals/products/zhiyun-smooth-4) Gimbal, synced camera and IMU with [**VideoIMUCapture-Android**](https://github.com/hibetterheyj/VideoIMUCapture-Android) application
- Calibration: camera and cam-imu calibration with **[kalibr](https://github.com/ethz-asl/kalibr)**
- Benchmark: N/A

## Download

- `image.zip` can be downloaded from <https://drive.google.com/file/d/1dWRwkZaB_mY21HzAGXzcXKBwg_2cBn5c/view?usp=sharing> and extract as `path/to/epfl_parking/image/`

- Raw data including video & imu meta data can be downloaded from <https://drive.google.com/drive/folders/1E5GWKagbBWA4zzalkx8PdhqvEvf8fa46?usp=sharing>

- Modified calibration toolkit can be found [**here**](https://github.com/hibetterheyj/VideoIMUCapture-Android)

  ![app_screencast_calibration](./calibration.png)

## Folder structure

```plaintext
❯ tree lausanne_center_nav/ -L 2
├── calibration
│   ├── calibration.yaml
│   ├── camchain-imucam-kalibr.yaml
│   ├── report-cam-kalibr.pdf
│   ├── report-imucam-kalibr.pdf
│   └── K.txt
├── image
└── readme.md
```
