import rclpy
from rclpy.node import Node
from std_msgs.msg import String
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


def loop_and_detect(cam, trt_yolo, conf_th, vis):
    """Continuously capture images from camera and do object detection.

    # Arguments
      cam: the camera instance (video source).
      trt_yolo: the TRT YOLO object detector instance.
      conf_th: confidence/score threshold for object detection.
      vis: for visualization.
    """
    full_scrn = False
    fps = 0.0
    tic = time.time()
    while True:
        if cv2.getWindowProperty(LEFT_WINDOW_NAME, 0) < 0 or \
            cv2.getWindowProperty(RIGHT_WINDOW_NAME, 0) < 0:
            break
        img_left, img_right = cam.read()
        if img_left is None or img_right is None:
            break
        boxes, confs, clss = trt_yolo.detect(img_left, conf_th)
        boxes, confs, clss = trt_yolo.detect(img_right, conf_th)
        print(boxes)
        print(confs)
        print(clss)
        img_left = vis.draw_bboxes(img_left, boxes, confs, clss)
        img_right = vis.draw_bboxes(img_right, boxes, confs, clss)
        img_left = show_fps(img_left, fps)
        img_right= show_fps(img_right, fps)
        cv2.imshow(LEFT_WINDOW_NAME, img_left)
        cv2.imshow(RIGHT_WINDOW_NAME, img_right)
        toc = time.time()
        curr_fps = 1.0 / (toc - tic)
        # calculate an exponentially decaying average of fps number
        fps = curr_fps if fps == 0.0 else (fps*0.95 + curr_fps*0.05)
        tic = toc
        key = cv2.waitKey(1)
        if key == 27:  # ESC key: quit program
            break


# def main():
#     cls_dict = get_cls_dict(1)
#     vis = BBoxVisualization(cls_dict)
#     detector = "orange-ball-detector"
#     trt_yolo = TrtYOLO(detector,1)
#     confidence_threshold = 0.3
#     WINDOW_NAME = "Computer Vision Module"
#     fps = 0.0
#     tic = time.time()


def main():

    # Kill processes using webcam
    subprocess.call("/workspace/dev_ws/include/kill_proccesses_using_webcam.sh")
    
    detector = "orange-ball-detector"
    confidence_threshold = 0.3
    # image = "/workspace/dev_ws/include/assets/orange-ball-test-1.jpeg" # test image
    argString = f"-m {detector} -c 1 -t {confidence_threshold} --zed"

    args = parse_args(argString)
    print(args)

    trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box)
    print(args)

    cam = Camera(args)
    if not cam.isOpened():
        raise SystemExit('ERROR: failed to open camera!')

    cls_dict = get_cls_dict(1)
    vis = BBoxVisualization(cls_dict)
    # trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box)

    

    open_window(
        LEFT_WINDOW_NAME, 'Left camera',
        cam.img_width, cam.img_height)
    
    open_window(
        RIGHT_WINDOW_NAME, 'right camera',
        cam.img_width, cam.img_height)

    loop_and_detect(cam, trt_yolo, args.conf_thresh, vis=vis)

    cam.release()
    cv2.destroyAllWindows()

    # # zed.close()
    # # print("\nFINISH")


if __name__ == '__main__':
    main()
