#!/usr/bin/env python
import os
from sawyer_control.srv import image, video, get_kinectdata, imageResponse, videoResponse
import rospy
from sensor_msgs.msg import Image as Image_msg
import cv2
from cv2 import VideoWriter
from skimage.transform import downscale_local_mean
from cv_bridge import CvBridge, CvBridgeError
import copy
import thread
import numpy as np
import imutils
import pickle



class Latest_observation(object):
    def __init__(self):
        # color image:
        self.img_cv2 = None
        self.img_cropped = None
        self.img_msg = None

        # depth image:
        self.d_img_raw_npy = None  # 16 bit raw data
        self.d_img_cropped_npy = None
        self.d_img_cropped_8bit = None
        self.d_img_msg = None


class KinectRecorder(object):
    def __init__(self, image_shape=(32, 32, 3)):

        """
        Records joint data to a file at a specified rate.
        """

        self.image_shape = image_shape

        rospy.Subscriber("/kinect2/hd/image_color", Image_msg, self.store_latest_image)
        rospy.Subscriber("/kinect2/sd/image_depth_rect", Image_msg, self.store_latest_d_image)

        self.ltob = Latest_observation()
        self.ltob_aux1 = Latest_observation()

        self.bridge = CvBridge()
        self.video_writer = None
        self.video_writer_sm = None
        self.video_frames = []

        self.get_kinectdata_func = rospy.ServiceProxy('get_kinectdata', get_kinectdata)

        def spin_thread():
            rospy.spin()

        thread.start_new(spin_thread, ())

    def get_kinect_handler(self, req):
        img = np.asarray(self.ltob.img_cropped, dtype=np.float32)
        img = self.bridge.cv2_to_imgmsg(img)
        return get_kinectdataResponse(img)

    def store_latest_d_image(self, data):
        self.ltob.d_img_msg = data
        cv_image = self.bridge.imgmsg_to_cv2(data, '16UC1')

        self.ltob.d_img_raw_npy = np.asarray(cv_image, dtype=np.float32)
        img = cv2.resize(
            cv_image, (0, 0), fx=1 / 5.5, fy=1 / 5.5, interpolation=cv2.INTER_AREA)

        img = np.clip(img, 0, 1400)

        startcol = 7
        startrow = 0
        endcol = startcol + 64
        endrow = startrow + 64
        # crop image:
        img = img[startrow:endrow, startcol:endcol]

        self.ltob.d_img_cropped_npy = img
        img = img.astype(np.float32) / np.max(img) * 256
        img = img.astype(np.uint8)
        img = np.squeeze(img)
        self.ltob.d_img_cropped_8bit = img

    def store_latest_image(self, data):
        self.ltob.img_msg = data
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # (1920, 1080)
        self.ltob.img_cv2 = self.crop_highres(cv_image)

        # rospy.loginfo(str(self.video_writer))
        if self.video_writer is not None:
            video_frame = copy.deepcopy(cv_image[:900, 360:1560])
            video_frame = cv2.resize(
                video_frame, (800, 600), interpolation=cv2.INTER_AREA)
            # video_frame = downscale_local_mean(cv_image, (3, 3, 1)).astype(np.uint8)
            try:
                self.video_writer.write(video_frame)
                self.video_frames.append(self.ltob.img_cv2.astype(np.uint8))
                self.video_writer_sm.write(self.ltob.img_cv2.astype(np.uint8))
            except Exception as e:
                print(e)

        # cv2.putText(cv_image, f"Hello World!!! {self.video_writer}", (0, 0), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
        # cv2.imwrite('/tmp/jiiri.png', cv_image)

    def crop_highres(self, cv_image):
        # print("image shape:", cv_image.shape)
        crop_size = 1024
        original_image = cv_image.astype(np.uint8).copy()
        # cv_image = copy.deepcopy(cv_image[13:13+crop_size, 413:412+crop_size])

        cv_image = copy.deepcopy(cv_image[200:850, 600:1250])
        cv_image = cv2.resize(
            cv_image,
            (512, 512),
            interpolation=cv2.INTER_AREA)
        cv_image = downscale_local_mean(
            cv_image,
            (cv_image.shape[0] // self.image_shape[0],
             cv_image.shape[1] // self.image_shape[1],
             1))

        return cv_image

    def start_recording(self, video_path):
        self.video_writer = VideoWriter(video_path,
                                        0,
                                        # cv2.VideoWriter_fourcc(*'MPEG'),
                                        25,
                                        (800, 600),
                                        isColor=True)
        filepath, extension = os.path.splitext(video_path)
        small_video_path = filepath + '_sm' + extension
        self.video_writer_sm = VideoWriter(small_video_path,
                                           cv2.VideoWriter_fourcc(*'MPEG'),
                                           25,
                                           (32, 32),
                                           isColor=True)

        self.raw_frames_path = filepath + '_raw.pkl'
        self.video_frames = []

    def stop_recording(self):
        rospy.loginfo("Generating video files.")

        with open(self.raw_frames_path, 'wb') as f:
            pickle.dump(self.video_frames, f)
        self.video_frames = []

        video_writer = self.video_writer
        self.video_writer = None
        video_writer.release()
        del video_writer

        video_writer_sm = self.video_writer_sm
        self.video_writer_sm = None
        video_writer_sm.release()
        del video_writer_sm

        rospy.loginfo("Video files generated")


def get_observation(unused):
    img = kr.ltob.img_cv2
    # img = np.array(img)
    image = img.flatten().tolist()
    return imageResponse(image)

def video_callback(request):
    assert not (request.start_recording and request.stop_recording)
    assert request.start_recording or request.stop_recording

    if bool(request.start_recording):
        kr.start_recording(request.video_path)
    elif bool(request.stop_recording):
        kr.stop_recording()

    return videoResponse()

def image_server():
    s1 = rospy.Service('images', image, get_observation)
    s2 = rospy.Service('video', video, video_callback)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('image_server', anonymous=True)
    kr = KinectRecorder()
    image_server()
