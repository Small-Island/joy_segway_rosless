#!/usr/bin/python3
import rospy
from std_msgs.msg import String
import pyrealsense2 as rs
import numpy as np
import cv2

pub = rospy.Publisher('obstacle', String, queue_size=10)
rospy.init_node('objectDetection', anonymous=False)

config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipeline = rs.pipeline()
profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
clipping_distance_in_meters = 0.4
clipping_distance = clipping_distance_in_meters / depth_scale

align_to = rs.stream.color
align = rs.align(align_to)

threshold = (640*480*3)*0.95

try:
    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        detected = (depth_image < clipping_distance) | (depth_image <= 0)
        sum_detected = np.sum(detected)

        white_color = 255
        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), white_color, color_image)
        # 背景色となっているピクセル数をカウント
        # white_pic = np.sum(bg_removed == 255)
        # 背景色が一定値以下になった時に、「検出」を表示する
        # if(threshold > white_pic):
        #     print("検出 {}".format(white_pic))
        # else:
        #     print("{}".format(white_pic))
        if sum_detected > 640 * 480 * 0.15:
            pub.publish('obstacle')
            # print("検出 {}".format(sum_detected))
        else:
            # print("{}".format(sum_detected))
            pub.publish('clear')

        rospy.Rate(100).sleep()

        # images = np.hstack((bg_removed, color_image))
        # cv2.imshow('Frames', bg_removed)
        # if cv2.waitKey(1) & 0xff == 27:
        #     break

finally:
    # ストリーミング停止
    pipeline.stop()
    cv2.destroyAllWindows()
