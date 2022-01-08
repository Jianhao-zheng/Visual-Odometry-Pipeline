#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   bag2images.py
@Date created  :   2022/01/07
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides workflow to extract images and camera_info from rosbag
"""
# =============================================================================

import os
import json
import argparse

import cv2
import numpy as np
import pandas as pd

import rosbag
from cv_bridge import CvBridge

# from sensor_msgs.msg import Image, CameraInfo

curr_dir_path = os.path.dirname(os.path.abspath(__file__))


def test_bgr2rgb():
    # https://stackoverflow.com/a/9641163
    srcBGR = cv2.imread("000000.png")
    destRGB = cv2.cvtColor(srcBGR, cv2.COLOR_BGR2RGB)
    # type(destRGB)
    # <class 'numpy.ndarray'>
    cv2.imwrite('testRGN.png', destRGB)


def ts_to_sec(ts):
    """convert ros timestamp into second"""
    return ts.secs + ts.nsecs / float(1e9)


def main():
    """Extract a folder of images from a rosbag."""
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument(
        "--bag_file",
        default="../bag/data.bag",
        help="Input ROS bag.",
    )
    # 52.4s 760 msgs 14.5FPS
    parser.add_argument(
        "--output_dir",
        default="../image",
        help="Output directory.",
    )
    parser.add_argument(
        "--image_topic",
        default="/camera_left/color/image_raw",
        help="Image topic.",
    )
    parser.add_argument(
        "--camera_info_topic",
        default="/camera_left/color/camera_info",
        help="Camera info topic.",
    )
    parser.add_argument(
        "--starting_frame",
        default=90,
        type=int,
    )
    parser.add_argument(
        "--ending_frame",
        default=500,
        type=int,
    )
    parser.add_argument(
        "--subsample",
        default=3,
        type=int,
    )
    parser.add_argument("--bgr2rgb", dest="bgr2rgb", action="store_true")
    parser.set_defaults(bgr2rgb=True)

    args = parser.parse_args()

    bagname = args.bag_file.split('/')[-1].split('.')[0]

    print(bagname)

    if args.output_dir is None:
        img_save_path = os.path.join(curr_dir_path, bagname)
    else:
        img_save_path = args.output_dir
    if not os.path.exists(img_save_path):
        os.makedirs(img_save_path)

    print(
        "Extract images from %s on topic %s into %s"
        % (args.bag_file, args.image_topic, img_save_path)
    )

    bag = rosbag.Bag(args.bag_file, "r")

    extract_image = True
    if extract_image:
        bridge = CvBridge()
        fr_cnt = 0
        data_cnt = 0
        print(fr_cnt, data_cnt)
        ts_list = []
        for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
            if fr_cnt < args.starting_frame:
                pass
            elif fr_cnt > args.ending_frame:
                break
            else:
                if fr_cnt % args.subsample == 0:
                    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    img_ts = ts_to_sec(t)

                    if args.bgr2rgb:
                        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

                    ts_list.append(img_ts)

                    # cv2.imwrite(os.path.join(img_save_path, "frame%06i.png" % count), cv_img)
                    img_path = os.path.join(img_save_path, "%04i.png" % data_cnt)
                    cv2.imwrite(img_path, cv_img)
                    print("Frame {}: {}".format(fr_cnt, img_path))
                    data_cnt += 1

            fr_cnt += 1

        ts_np = np.array(ts_list)
        raw_dict = {
            "timestamp": ts_np,
        }
        img_ts_path = os.path.join(img_save_path, '..', 'img_ts.csv')
        pd.DataFrame(raw_dict).to_csv(img_ts_path, index=False)

    extract_camera_info = True
    if extract_camera_info:
        fr_cnt = 0
        for topic, msg, t in bag.read_messages(topics=[args.camera_info_topic]):

            if fr_cnt == 0:
                height = msg.height
                width = msg.width
                distortion_model = msg.distortion_model
                # For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
                distortion_coeff = msg.D
                # (0.0, 0.0, 0.0, 0.0, 0.0)
                intrinsic_mat = msg.K
                # Projection/camera matrix
                #     [fx'  0  cx' Tx]
                # P = [ 0  fy' cy' Ty]
                #     [ 0   0   1   0]
                # projection_mat = msg.P

                # print(distortion_model, distortion_coeff)
                # print(projection_mat)
                # print(height, width)
                # print(intrinsic_mat)

                fr_cnt += 1
            else:
                break
        cam_info_dict = {
            'height': height,
            'width': width,
            'K': intrinsic_mat,
            'distortion_model': distortion_model,
            'D': distortion_coeff,
        }
        if extract_image:
            fps = data_cnt / (ts_np.max() - ts_np.min())
        cam_info_dict.update({'fps': fps})
        print("FPS of the generated data is {} Hz".format(fps))
        cam_info_path = os.path.join(img_save_path, '..', 'cam_info.json')
        with open(cam_info_path, 'w') as cam_info_file:
            json.dump(cam_info_dict, cam_info_file, indent=4, sort_keys=False)

    bag.close()


if __name__ == '__main__':
    main()
