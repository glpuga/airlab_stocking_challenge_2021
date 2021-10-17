#!/usr/bin/env python
import rospy

import cv2
import numpy as np

import random
import math

from tf.transformations import quaternion_from_euler

from datetime import datetime
from threading import Lock

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest
from ek_dataset_studio.srv import LightDirectionControl, LightDirectionControlRequest


from cv_bridge import CvBridge

MODEL_DATABASE_TEMPLATE = """\
<sdf version="1.4">
    <world name="default">
        <include>
            <uri>model://{}</uri>
        </include>
    </world>
</sdf>"""


class Photographer:

    def __init__(self):

        self._lock = Lock()

        self._model_names = [{
            "model_label": "tomato_can",
            "model_name": "standard_can_fit"}, ]

        self._max_distance_offset = 0.75
        self._min_distance_offset = -0.25

        self._samples = 3000

        self._model_id = "model_under_study"

        self._dataset_folder = rospy.get_param('~dataset_folder', '/tmp') + "/"

        self.camera_image_topic_ = "/studio_camera/image_raw"
        self.camera_info_topic_ = "/studio_camera/camera_info"

        self._model_spawner_model_service = "/gazebo/spawn_sdf_model"
        self._model_deleter_model_service = "/gazebo/delete_model"
        self._lightning_orientation_service = "/gazebo/set_sun_direction"

        self._camera_image = None
        self._camera_info = None

        self._bridge = CvBridge()

        self._capturing = False

        self._camera_image_sub = rospy.Subscriber(
            self.camera_image_topic_, Image, self._camera_image_callback)
        self._camera_info_sub = rospy.Subscriber(
            self.camera_info_topic_, CameraInfo, self._camera_info_callback)

        rospy.wait_for_service(self._model_spawner_model_service)
        rospy.wait_for_service(self._model_deleter_model_service)

        self._model_spawner_srv = rospy.ServiceProxy(
            self._model_spawner_model_service, SpawnModel)
        self._model_deleter_srv = rospy.ServiceProxy(
            self._model_deleter_model_service, DeleteModel)
        self._lightning_orientation_srv = rospy.ServiceProxy(
            self._lightning_orientation_service, LightDirectionControl)

    def run(self):
        seq = 0
        while seq < self._samples:
            label_index = random.choice(range(len(self._model_names)))
            model_data = self._model_names[label_index]
            model_name = model_data["model_name"]
            model_label = model_data["model_label"]
            img = self._capture_image(model_name)
            if img is not None:
                seq = seq + 1
                augmented_image, bounding_box = self._augment_and_bound_image(
                    img)
                filename = "{}-{}".format(model_name, seq)
                png_filename = filename + ".png"
                tag_filename = filename + ".txt"
                rospy.logwarn("Saving file {}".format(png_filename))
                self._save_image_file(png_filename, augmented_image)
                self._save_tag_file(tag_filename, label_index,
                                    bounding_box, augmented_image.shape[0:2])

    def _augment_and_bound_image(self, img):
        # find a mask
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(img_gray, 1, 128, 0)
        _, contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnt = contours[0]
        fg_mask = np.zeros(img_gray.shape, np.uint8)
        cv2.drawContours(fg_mask, [cnt], 0, 255, -1)
        bg_mask = np.zeros(img_gray.shape, np.uint8)
        np.bitwise_not(fg_mask, bg_mask)

        # extract the object and leave in a black field
        foreground = np.zeros(img.shape, np.uint8)
        cv2.bitwise_and(img, img, foreground, fg_mask)

        # create a background with colored noise
        colored_noise = np.uint8(np.random.random(img.shape) * 255)
        background = np.zeros(img.shape, np.uint8)
        cv2.bitwise_and(colored_noise, colored_noise, background, bg_mask)

        # assemble the image
        new_img = background + foreground
        # find the bounding box
        bounding_box = cv2.boundingRect(cnt)

        return new_img, bounding_box

    def _save_image_file(self, filename, image):
        filepath = self._dataset_folder + filename
        cv2.imwrite(filepath, image)

    def _save_tag_file(self, filename, model_index, bounding_box, img_size):
        filepath = self._dataset_folder + filename
        img_height, img_width = img_size
        x, y, w, h = (float(x) for x in bounding_box)
        with open(filepath, "w") as file_desc:
            # <object-class> <x_center> <y_center> <width> <height>
            line = "{} {} {} {} {}".format(
                model_index,
                (x + w/2.0) / img_width,
                (y + h/2.0) / img_height,
                w / img_width,
                h / img_height)
            file_desc.writelines(line + "\n")
            print("writing: " + line)

    def _camera_image_callback(self, msg):
        with self._lock:
            if (self._capturing and (not self._camera_info is None)):
                self._camera_image = msg
                self._capturing = False

    def _camera_info_callback(self, msg):
        with self._lock:
            self._camera_info = msg

    def _get_image(self):
        with self._lock:
            self._capturing = True
        while True:
            rospy.sleep(rospy.Duration(0.1))
            with self._lock:
                if rospy.is_shutdown():
                    return None
                if self._capturing == False:
                    img = self._bridge.imgmsg_to_cv2(
                        self._camera_image, desired_encoding='bgr8')
                    return img

    def _spawn_model(self, model_xml, distance_offset, orientation):
        msg = SpawnModelRequest()
        msg.model_name = self._model_id
        msg.model_xml = model_xml
        msg.robot_namespace = ""
        msg.initial_pose.position.x = 0
        msg.initial_pose.position.y = distance_offset
        msg.initial_pose.position.z = 0
        q = quaternion_from_euler(*orientation)
        msg.initial_pose.orientation.x = q[0]
        msg.initial_pose.orientation.y = q[1]
        msg.initial_pose.orientation.z = q[2]
        msg.initial_pose.orientation.w = q[3]
        msg.reference_frame = "world"
        self._model_spawner_srv(msg)

    def _remove_model(self):
        msg = DeleteModelRequest()
        msg.model_name = self._model_id
        return self._model_deleter_srv(msg)

    def _change_lightning(self, orientation):
        msg = LightDirectionControlRequest()
        q = quaternion_from_euler(*orientation)
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        self._lightning_orientation_srv(msg)

    def _create_xml_description(self, model_name):
        model_xml = MODEL_DATABASE_TEMPLATE.format(model_name)
        return model_xml

    def _capture_image(self, model_name):
        img = None
        def get_random_angle(): return 2.0 * math.pi * random.random()
        mroll, mpitch, myaw = get_random_angle(),  get_random_angle(),  get_random_angle()
        lroll, lpitch, lyaw = get_random_angle(),  get_random_angle(),  get_random_angle()
        distance_offset = random.random() * (self._max_distance_offset -
                                             self._min_distance_offset) + self._min_distance_offset
        model_xml = self._create_xml_description(model_name)
        # spawn the model in gazebo
        self._spawn_model(model_xml, distance_offset, (mroll, mpitch, myaw))
        # set the lightning
        self._change_lightning((lroll, lpitch, lyaw))
        # wait until the model gets spawned
        rospy.sleep(rospy.Duration(0.2))
        # capture an image
        img = self._get_image()
        # remove the model from gazebo
        resp = self._remove_model()
        return img if resp.success else None


def main():
    rospy.init_node('photographer_node', anonymous=True)
    try:
        node = Photographer()
        node.run()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
