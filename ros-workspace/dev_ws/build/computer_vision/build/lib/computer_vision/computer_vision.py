import rclpy
from rclpy.node import Node
# from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float32
import cv2
import os
import pycuda.autoinit
from .yolo_with_plugins import TrtYOLO
from .visualization import BBoxVisualization
from .display import open_window, set_display, show_fps
# import pyzed.sl as sl
from .yolo_classes import get_cls_dict
import time
from .camera import add_camera_args, Camera
import shlex
import argparse
import subprocess

x = 0
offset = 0
threshold = 1000
output_list = []
list_number = 3
valid_output = False
DEBUG = False


# def print_camera_information(cam):
#     print("Resolution: {0}, {1}.".format(round(cam.get_camera_information(
#     ).camera_resolution.width, 2), cam.get_camera_information().camera_resolution.height))
#     print("Camera FPS: {0}.".format(cam.get_camera_information().camera_fps))
#     print("Firmware: {0}.".format(
#         cam.get_camera_information().camera_firmware_version))
#     print("Serial number: {0}.\n".format(
#         cam.get_camera_information().serial_number))

LEFT_WINDOW_NAME = "left window"
RIGHT_WINDOW_NAME = "right_window"


def stereo_triangulate(left_bounding_box, right_bounding_box):

    if len(left_bounding_box) == 0 or len(right_bounding_box) == 0:
        return 0

    # Image Resolution
    width = 1280
    height = 720

    fx = 699.51
    fy = 699.51

    base_line = 120  # in mm
    focal_length_pixels = 700  # in pixels
    # pixel_size = 0.004 # in mm
    # focal_length = focal_length_pixels * pixel_size
    # focal_length = ((fx+fy)/2) * pixel_size
    # left_x_center = (left_bounding_box[2]- left_bounding_box[0])/2
    # right_x_center = (right_bounding_box[2]- right_bounding_box[0])/2

    left_x_center = left_bounding_box[0]
    right_x_center = right_bounding_box[0]

    # left_y_center = (left_bounding_box[3]- left_bounding_box[1])/2
    left_y_center = left_bounding_box[1]
    # print(f"left center: ({left_x_center,left_y_center})")
    # right_y_center = (right_bounding_box[3]- right_bounding_box[1])/2

    # Frame to image coordinate

    x_origin = width/2
    y_origin = height/2

    x_position_left = (left_x_center - x_origin)
    y_position_left = (left_y_center - y_origin)
    x_position_right = (right_x_center - x_origin)

    disparity = abs(x_position_right - x_position_left)

    # Depth calculation
    z = (base_line * focal_length_pixels) / disparity

    x = (x_position_left * z) / fx
    y = (y_position_left * z) / fy
    y = -y

    return (z, x, y)


def parse_args(argString):
    """Parse input arguments."""
    desc = ('Capture and display live camera video, while doing '
            'real-time object detection with TensorRT optimized '
            'YOLO model on Jetson')
    parser = argparse.ArgumentParser(description=desc)
    parser = add_camera_args(parser)
    parser.add_argument(
        '-c', '--category_num', type=int, default=80,
        help='number of object categories [80]')
    parser.add_argument(
        '-t', '--conf_thresh', type=float, default=0.3,
        help='set the detection confidence threshold')
    parser.add_argument(
        '-m', '--model', type=str, required=True,
        help=('[yolov3-tiny|yolov3|yolov3-spp|yolov4-tiny|yolov4|'
              'yolov4-csp|yolov4x-mish|yolov4-p5]-[{dimension}], where '
              '{dimension} could be either a single number (e.g. '
              '288, 416, 608) or 2 numbers, WxH (e.g. 416x256)'))
    parser.add_argument(
        '-l', '--letter_box', action='store_true',
        help='inference with letterboxed image [False]')

    args = parser.parse_args(shlex.split(argString))
    return args


def loop_and_detect(self, cam, trt_yolo, conf_th, vis):
    """Continuously capture images from camera and do object detection.

    # Arguments
      cam: the camera instance (video source).
      trt_yolo: the TRT YOLO object detector instance.
      conf_th: confidence/score threshold for object detection.
      vis: for visualization.
    """
    global x
    global threshold
    global valid_output
    x_old = x

    fps = 0.0
    tic = time.time()
    # time.sleep(1)
    # while True:
    # if cv2.getWindowProperty(LEFT_WINDOW_NAME, 0) < 0 or \
    #     cv2.getWindowProperty(RIGHT_WINDOW_NAME, 0) < 0:
    #     break
    img_left, img_right = cam.read()
    if img_left is None or img_right is None:
        print("uh oh no images")
        self.get_logger().info('no images recorded')
        # break
        x = 0
        valid_output = False
        return 0

    boxes1, confs1, clss1 = trt_yolo.detect(img_left, conf_th)
    boxes2, confs2, clss2 = trt_yolo.detect(img_right, conf_th)

    if len(boxes1) == 0 or len(boxes2) == 0:
        print("No Objects detected")
        if DEBUG:
            self.get_logger().info('No Objects detected')
        # continue
        x = 0
        valid_output = False

        return 0

    # print(f"left bounding boxes (min x, min y, max x, max y): {boxes1[:1]}")
    # print(f"right bounding boxes (min x, min y, max x, max y): {boxes2[:1]}")
    # print(f"confidence left:{confs1[:1]}")
    # print(f"confidence right:{confs2[:1]}")

    z, x, y = stereo_triangulate(boxes1[0], boxes2[0])

    base_line = 120

    x = x + base_line

    if abs(x) > threshold:
        x = 0
        print("Object out of range")
        if DEBUG: 
            self.get_logger().info('Object out of range')
        valid_output = False
        return 0

    print(f"depth: {z} mm")
    print(f"x-coordinate: {x} mm")
    print(f"y-coordinate: {y} mm")
    # print(clss)
    # img_left = vis.draw_bboxes(img_left, boxes1, confs1, clss1)
    # img_right = vis.draw_bboxes(img_right, boxes, confs, clss)
    # img_left = show_fps(img_left, fps)
    # img_right= show_fps(img_right, fps)
    # cv2.imshow(LEFT_WINDOW_NAME, img_left)
    # cv2.imshow(RIGHT_WINDOW_NAME, img_right)
    toc = time.time()
    curr_fps = 1.0 / (toc - tic)
    # calculate an exponentially decaying average of fps number
    fps = curr_fps if fps == 0.0 else (fps*0.95 + curr_fps*0.05)
    tic = toc
    print(f"fps: {fps}")
    # key = cv2.waitKey(1)
    # if key == 27:  # ESC key: quit program
    #     break

    valid_output = True
    return x


# def main():
#     cls_dict = get_cls_dict(1)
#     vis = BBoxVisualization(cls_dict)
#     detector = "orange-ball-detector"
#     trt_yolo = TrtYOLO(detector,1)
#     confidence_threshold = 0.3
#     WINDOW_NAME = "Computer Vision Module"
#     fps = 0.0
#     tic = time.time()

class ComputerVisionPublisher(Node):

    def __init__(self):
        super().__init__('computer_vision')
        self.publisher_ = self.create_publisher(Float32, 'object_height', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription = self.create_subscription(
            Float32,
            'offset',
            self.offset_callback,
            10)
        self.subscription  # prevent unused variable warning

        print("starting up computer vision module")
        detector = "orange-ball-detector"
        confidence_threshold = 0.05
        # image = "/workspace/dev_ws/include/assets/orange-ball-test-1.jpeg" # test image
        argString = f"-m {detector} -c 1 -t {confidence_threshold} --zed"

        self.args = parse_args(argString)

        print(self.args)

        self.trt_yolo = TrtYOLO(
            self.args.model, self.args.category_num, self.args.letter_box)
        self.cam = Camera(self.args)
        if not self.cam.isOpened():
            raise SystemExit('ERROR: failed to open camera!')
        self.cls_dict = get_cls_dict(1)
        self.vis = BBoxVisualization(self.cls_dict)

        # trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box)

        # open_window(
        #     LEFT_WINDOW_NAME, 'Left camera',
        #     cam.img_width, cam.img_height)

        # open_window(
        #     RIGHT_WINDOW_NAME, 'right camera',
        #     cam.img_width, cam.img_height)

    def timer_callback(self):
        global offset
        global threshold
        global output_list
        global list_number
        global valid_output

        

        msg = Float32()

        height = loop_and_detect(self,
            self.cam, self.trt_yolo, self.args.conf_thresh, vis=self.vis)

        if DEBUG:
            self.get_logger().info('Valid output: "%r"' % valid_output)

        # heightAndOffset = height+offset

        if valid_output is False:
            return

        output_list.append(height)
        output = 0

        if len(output_list) > list_number:
            output_list.pop(0)
            sum = 0
            for n in output_list:
                sum = sum + n
            output = (sum/len(output_list))
        else:           
            return

        try:
            output = output + offset
            msg.data = output

        except:
            self.get_logger().info('Unable to publish vision message')
            return

        if valid_output:
            self.publisher_.publish(msg)
            if DEBUG:
                self.get_logger().info('Publishing: "%d"' % output)

    def offset_callback(self, msg):
        # self.get_logger().info('offset %d' % msg.data)
        global offset
        offset_requested = msg.data
        try:
            if abs(offset_requested) > 500:
                offset = 0
            else: 
                offset = offset_requested
        except:
            return


def main(args=None):

    # Kill processes using webcam
    subprocess.call(
        "/workspace/dev_ws/include/kill_proccesses_using_webcam.sh")

    rclpy.init(args=args)

    computer_vision_publisher = ComputerVisionPublisher()

    rclpy.spin(computer_vision_publisher)

    computer_vision_publisher.cam.release()
    cv2.destroyAllWindows()

    computer_vision_publisher.destroy_node()
    rclpy.shutdown()

    # # zed.close()
    # # print("\nFINISH")


if __name__ == '__main__':
    main()
