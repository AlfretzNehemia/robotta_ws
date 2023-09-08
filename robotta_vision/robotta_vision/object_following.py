# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import cv2
import torch
import random

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

from ultralytics import YOLO
from ultralytics.tracker import BOTSORT, BYTETracker
from ultralytics.tracker.trackers.basetrack import BaseTrack
from ultralytics.yolo.utils import IterableSimpleNamespace, yaml_load
from ultralytics.yolo.utils.checks import check_requirements, check_yaml
from ultralytics.yolo.engine.results import Results

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import Detection2DArray
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

from robotta_vision.camera_util import *

class Yolov8Node(Node):

    def __init__(self) -> None:
        super().__init__("human_detection_node")

        # params
        self.declare_parameter("model", "yolov8m.pt")
        model = self.get_parameter(
            "model").get_parameter_value().string_value

        self.declare_parameter("tracker", "bytetrack.yaml")
        tracker = self.get_parameter(
            "tracker").get_parameter_value().string_value

        self.declare_parameter("device", "cuda:0")
        device = self.get_parameter(
            "device").get_parameter_value().string_value

        self.declare_parameter("threshold", 0.7)
        self.threshold = self.get_parameter(
            "threshold").get_parameter_value().double_value

        self.declare_parameter("enable", True)
        self.enable = self.get_parameter(
            "enable").get_parameter_value().bool_value

        self._class_to_color = {}
        self.cv_bridge = CvBridge()
        self.tracker = self.create_tracker(tracker)
        self.yolo = YOLO(model, 'gpu')
        self.yolo.fuse()
        self.yolo.to(device)

        # initial condition
        self.set_point_distance = 0.8
        self.set_point_x = 320.0
        self.linear_Kp = 50
        self.linear_Ki = 0.1
        self.linear_Kd = 0
        self.angular_Kp = 50
        self.angular_Ki = 0.1
        self.angular_Kd = 0
        self.targetpoint = 0
        self.integral = 0
        self.derivative = 0
        self.prevError = 0
        self.posObject_x = 0
        self.posObject_y = 0
        self.center_camera = 0
        # self.center_idx = 0
        # self.set_point = (320, 240)
        self.point_pose = (0,0,0)
        self.cols, self.rows = 0,0
        self.prev_dist = 0
        self.prev_angular = 0
        self.linear_PID = (0.75, 0.0, 2.0)
        self.angular_PID = (0.009, 0.0, 0.0)
        self.PID_init()
        # self.declare_param()
        print("init done")
        # self.angular_z = 0
        # self.linear_x = 0

        # topcis
        self.pub_position = self.create_publisher(Point, "/Current_point", 10)
        self.pub_cmdVel = self.create_publisher(Twist, '/robotta_drive_controller/cmd_vel_unstamped',10)
        self._pub = self.create_publisher(Detection2DArray, "detections", 10)
        self._dbg_pub = self.create_publisher(Image, "dbg_image", 10)
        self.rgb_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_cb,
            qos_profile_sensor_data
        )
        # self.rgb_sub = self.create_subscription(
        #     Image, "/camera/aligned_depth_to_color/image_raw", self.image_cb,
        #     qos_profile_sensor_data
        # )
        self.depth_sub = self.create_subscription(
            Image, "/camera/depth/image_rect_raw", self.depth_cb,
            qos_profile_sensor_data
        )

        # services
        self._srv = self.create_service(SetBool, "enable", self.enable_cb)

    # def declare_param(self):
    #     self.declare_parameter("linear_Kp",3.0)
    #     self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
    #     self.declare_parameter("linear_Ki",0.0)
    #     self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
    #     self.declare_parameter("linear_Kd",1.0)
    #     self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value
    #     self.declare_parameter("angular_Kp",0.5)
    #     self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
    #     self.declare_parameter("angular_Ki",0.0)
    #     self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
    #     self.declare_parameter("angular_Kd",2.0)
    #     self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value
    #     self.declare_parameter("scale",1000)
    #     self.scale = self.get_parameter('scale').get_parameter_value().integer_value
    #     self.declare_parameter("minDistance",1.0)
    #     self.minDistance = self.get_parameter('minDistance').get_parameter_value().double_value

    def get_param(self):
    #     self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
    #     self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
    #     self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value
    #     self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
    #     self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
    #     self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value
    #     self.scale = self.get_parameter('scale').get_parameter_value().integer_value
    #     self.minDistance = self.get_parameter('minDistance').get_parameter_value().double_value
        
        self.linear_PID = (self.linear_Kp, self.linear_Ki, self.linear_Kd)
        self.angular_PID = (self.angular_Kp, self.angular_Ki, self.angular_Kd)
    #     self.minDist = self.minDistance * 1000    

    def PID_init(self):
        # self.linear_pid = simplePID(self.linear_PID[0] / 1000.0, self.linear_PID[1] / 1000.0, self.linear_PID[2] / 1000.0)
        # self.angular_pid = simplePID(self.angular_PID[0] / 100.0, self.angular_PID[1] / 100.0, self.angular_PID[2] / 100.0)
        self.linear_pid = simplePID(self.linear_PID[0], self.linear_PID[1], self.linear_PID[2])
        self.angular_pid = simplePID(self.angular_PID[0], self.angular_PID[1], self.angular_PID[2])

    def create_tracker(self, tracker_yaml) -> BaseTrack:

        TRACKER_MAP = {"bytetrack": BYTETracker, "botsort": BOTSORT}
        check_requirements("lap")  # for linear_assignment

        tracker = check_yaml(tracker_yaml)
        cfg = IterableSimpleNamespace(**yaml_load(tracker))

        assert cfg.tracker_type in ["bytetrack", "botsort"], \
            f"Only support 'bytetrack' and 'botsort' for now, but got '{cfg.tracker_type}'"
        tracker = TRACKER_MAP[cfg.tracker_type](args=cfg, frame_rate=1)
        return tracker

    def enable_cb(self,
                  req: SetBool.Request,
                  res: SetBool.Response
                  ) -> SetBool.Response:
        self.enable = req.data
        res.success = True
        return res

    def image_cb(self, msg: Image) -> None:
        
        detection = 0

        if self.enable:

            # convert image + predict
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
            # cv_image = cv2.resize(cv_images, (500, 480))
            # cv2.crop()
            # cv_image = self.cv_bridge.cv2_to_imgmsg(cv_crop, "bgr8")
            colors_ = (255,255,255)
            min_x = 20
            max_x = 620
            min_y = 20
            max_y = 460
            cv_image_ = cv2.rectangle(cv_image, (min_x,min_y), (max_x,max_y), colors_, 2)

            results = self.yolo.predict(
                source=cv_image_,
                verbose=False,
                stream=False,
                conf=self.threshold,
                mode="track",
                classes = 0,
            )
            results: Results = results[0].cpu()

            # tracking
            det = results.boxes.numpy()

            if len(det) > 0:
                im0s = self.yolo.predictor.batch[2]
                im0s = im0s if isinstance(im0s, list) else [im0s]

                tracks = self.tracker.update(det, im0s[0])
                if len(tracks) > 0:
                    results.update(boxes=torch.as_tensor(tracks[:, :-1]))

            # create detections msg
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header

            for box_data in results.boxes:

                detection = Detection2D()

                # get label and score
                label = self.yolo.names[int(box_data.cls)]
                score = float(box_data.conf)

                if score > 0.90:


                    # get boxes values
                    box = box_data.xywh[0]
                    if (box[0] > min_x) and (box[0] < max_x) and (box[1] > min_y) and(box[1] < max_y):  # 200 box line (roi)
                        track_id = ""
                        if box_data.is_track:
                            track_id = str(int(box_data.id))
                        detection.tracking_id = track_id

                        detection.bbox.center.x = float(box[0])
                        detection.bbox.center.y = float(box[1])
                        detection.bbox.size_x = float(box[2])
                        detection.bbox.size_y = float(box[3])

                        position = Point()

                        self.posObject_x = detection.bbox.center.x
                        self.posObject_y = detection.bbox.center.y
                        position.x = detection.bbox.center.x
                        position.y = detection.bbox.center.y
                        # self.pub_position.publish(position)
                    
                        # get track id
                        # track_id = 
                        # if box_data.is_track:
                            
                        # if box_data.is_track:
                        #     track_id = str(int(box_data.id))
                        detection.tracking_id = track_id

                        # self.get_logger().info(f"track_id: {track_id}")
                        # if (int(track_id)) == 1:

                            # create hypothesis
                        hypothesis = ObjectHypothesisWithPose()
                        hypothesis.id = label
                        hypothesis.score = score
                        detection.results.append(hypothesis)

                        # draw boxes for debug
                        if label not in self._class_to_color:
                            r = random.randint(0, 255)
                            g = random.randint(0, 255)
                            box_data = random.randint(0, 255)
                            self._class_to_color[label] = (r, g, box_data)
                        color = self._class_to_color[label]

                        min_pt = (round(detection.bbox.center.x - detection.bbox.size_x / 2.0),
                                round(detection.bbox.center.y - detection.bbox.size_y / 2.0))
                        max_pt = (round(detection.bbox.center.x + detection.bbox.size_x / 2.0),
                                round(detection.bbox.center.y + detection.bbox.size_y / 2.0))
                        cv2.rectangle(cv_image_, min_pt, max_pt, color, 2)
                        
                        center_coordinates = (int(box[0]), int(box[1]))
                        radius = 10
                        colors = (0,0,255)
                        thickness = -1
                        cv2.circle(cv_image_, center_coordinates, radius, colors, thickness)

                        # pos_x = min_pt[0] + min_pt[1]
                        # pos_y = min_pt[]

                        label = "{} ({}) ({:.3f})".format(label, str(track_id), score)
                        # label = "{} ({:f}) ({:f})".format(label, int(box[0]), int(box[1]))
                        pos = (min_pt[0] + 5, min_pt[1] + 25)
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cv2.putText(cv_image_, label, pos, font,
                                    1, colors, 1, cv2.LINE_AA)
                    
                        # if ((posObject_x > 0) and (posObject_y > 0)) :
                        if (track_id == "1"):
                            dist_raw = (self.center_camera/10) # in cm
                            distance = (11.57 + 0.44 * dist_raw + 0.0128 * dist_raw * dist_raw)/100  # in meter
                            position.z = distance * 100
                            self.pub_position.publish(position)
                            print("distance: ", position.z)
                            print("angle x : ", self.posObject_x)
                            # print('object x pos: ', posObject_x)
                            self.execute(self.posObject_x, distance)

                        # append msg
                        detections_msg.detections.append(detection)

            # publish detections and dbg image
            self._pub.publish(detections_msg)
            self._dbg_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image_,
                                                            encoding=msg.encoding))
        # self.pub_position()

    def depth_cb(self, msg: Image): 
        try:
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")  # default is passthrough
            # depth_image_ = depth_image[int(self.posObject_y)][int(self.posObject_x)]
            # depth_array = np.array(depth_image_, dtype=np.float32)
            # center_idx = np.array(depth_array.shape)
            self.center_camera = depth_image[int(self.posObject_y)][int(self.posObject_x)]
            # center_idx = np.array(depth_array.shape) / 2
            # self.center_camera = 0.5   #depth_array[int(center_idx[0]), int(center_idx[1])]
            # self.center_camera = depth_array[int(self.posObject_x), int(self.posObject_y)]
            # print('center depth: ', center_camera/1000)
            # print('center depth:', depth_array[int(center_idx[0]), int(center_idx[1])])

            # print("Center_x: {}, Center_y: {}, distance: {}".format(self.posObject_x, self.posObject_y, self.center_camera))
        except CvBridgeError as e:
            print (e)

    def execute (self, point_x, point_y):
        self.get_param()
        # linear_x = 0
        # angular_z = 0
        # if (point_x < 630):
        #     angular_z = 80.0
        # if (point_x > 650):
        #     angular_z = -80.0
        # if (point_y < 350) :
        #     # angular_z = self.pid(self.minDist_y, point_y)
        #     linear_x = 50.0
        if abs(self.prev_dist - point_y) > 300:
            self.prev_dist = point_y
            return
        if abs(self.prev_angular - point_x) > 300:
            self.prev_angular = point_x
            return
        # if self.joy_active == True: return
        # linear_x = self.linear_pid.compute(point_y, self.minDist)
        linear_x = self.linear_pid.compute(point_y, self.set_point_distance)
        angular_z = self.angular_pid.compute(self.set_point_x, point_x)
        if abs(point_y - self.set_point_distance) < 0.05: linear_x = 0
        if abs(point_x - self.set_point_x) < 30: angular_z = 0
        twist = Twist()
        if angular_z > 3.14:
            angular_z = 3.14
        if angular_z < -3.14:
            angular_z = -3.14
        if linear_x > 0.5:
            linear_x = 0.5
        if linear_x < -0.5:
            linear_x = -0.5
        twist.angular.z = angular_z * 1.0
        twist.linear.x = linear_x * 0.0
        # print("twist.linear.x: ", twist.linear.x)
        # print("twist.angular.z: ", twist.angular.z)
        self.pub_cmdVel.publish(twist)
        # self.Robot_Run = True

    
    def reset(self):
        self.targetpoint = 0
        self.intergral = 0
        self.derivative = 0
        self.prevError = 0

def main():
    rclpy.init()
    node = Yolov8Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
