# Customized dataset: lausanne_center_nav

## Info

- FPS: 4.92 Hz
- Number of frames: 137
- Location: Lausanne center
- Source: images from RealSense camera
- Benchmark: VIO results from RealSense T265 camera

## Download

- `data.bag` can be downloaded from <https://drive.google.com/file/d/1Mvm5PLBks72eu9b9MgCeLPrbqaRTNRsF/view?usp=sharing> and put under `path/to/lausanne_center_nav/bag/`
- `image.zip` can be downloaded from <https://drive.google.com/file/d/1e1VCh19S4wyayX6s4J-8J6ad5YZx-B9a/view?usp=sharing> and extract as `path/to/lausanne_center_nav/image/`

## Folder structure

```plaintext
❯ tree lausanne_center_nav/ -L 2
├── K.txt
├── bag
│   └── data.yaml
├── cam_info.json
├── image
│   ├── 0000.png
│   ├── ...
│   └── 0136.png
├── image.zip
├── img_ts.csv
├── readme.md
└── scripts
    ├── bag2images.py
    └── gen_rosbag_yaml.sh
```

## To-do

- compute interpolated pose in corresponding timestamp of frames
