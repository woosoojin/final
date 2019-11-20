#!/usr/bin/python
import os
import time
import atexit
import cv2
import math
import numpy as np
import sys
import params
import argparse
import rospy

from PIL import Image
from PIL import ImageDraw
import local_common as cm

import input_kbd

from geometry_msgs.msg import Twist
from std_msgs.msg import String

##########################################################
# import deeppicar's sensor/actuator modules
##########################################################
camera   = __import__(params.camera)
#actuator = __import__(params.actuator)

##########################################################
# global variable initialization
##########################################################
use_dnn = False
use_thread = True
view_video = False
fpv_video = False

cfg_cam_res = (320, 240)
cfg_cam_fps = 30
cfg_throttle = 50 # 50% power.

NCPU = 2

frame_id = 0
angle = 0.0
btn   = ord('k') # center
period = 0.05 # sec (=50ms)

##########################################################
# local functions
##########################################################
def deg2rad(deg):
    return deg * math.pi / 180.0
def rad2deg(rad):
    return 180.0 * rad / math.pi

def g_tick():
    t = time.time()
    count = 0
    while True:
        count += 1
        yield max(t + count*period - time.time(),0)

def turn_off():
    #actuator.stop()
    camera.stop()

    keyfile.close()
    keyfile_btn.close()
    vidfile.release()

# functions
WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    return vel

def line_tracer(mail):
    pub = rospy.Publisher('line_tracer', Twist, queue_size=10) # to turtlebot3
    
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0

    if mail == "forward":
        rospy.loginfo("forward")
        target_linear_vel = 0.03
        target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)

        twist = Twist()

        twist.linear.x = target_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = target_angular_vel

               
        pub.publish(twist)

    elif mail == "right":
        target_linear_vel   = 0.02
        target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)

        twist = Twist()

        twist.linear.x = target_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = target_angular_vel

        pub.publish(twist)
    
    elif mial == "left":
        target_linear_vel   = 0.02
        target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                
        twist = Twist()

        twist.linear.x = target_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = target_angular_vel

        pub.publish(twist)

def line_detect():
    rospy.init_node('line_tracing', anonymous=True)
    rospy.Subscriber("line_tracing", String, line_tracer)

##########################################################
# program begins
##########################################################
parser = argparse.ArgumentParser(description='DeepPicar main')
parser.add_argument("-d", "--dnn", help="Enable DNN", action="store_true")
parser.add_argument("-t", "--throttle", help="throttle percent. [0-100]%", type=int)
parser.add_argument("-n", "--ncpu", help="number of cores to use.", type=int)
parser.add_argument("-f", "--fpvvideo", help="Take FPV video of DNN driving", action="store_true")
args = parser.parse_args()

if args.dnn:
    print ("DNN is on")
    use_dnn = True
if args.throttle:
    cfg_throttle = args.throttle
    print ("throttle = %d pct" % (args.throttle))
if args.ncpu > 0:
    NCPU = args.ncpu
if args.fpvvideo:
    fpv_video = True

# create files for data recording
keyfile = open('out-key.csv', 'w+')
keyfile_btn = open('out-key-btn.csv', 'w+')
keyfile.write("ts_micro,frame,wheel\n")
keyfile_btn.write("ts_micro,frame,btn,speed\n")
rec_start_time = 0
try:
    fourcc = cv2.cv.CV_FOURCC(*'XVID')
except AttributeError as e:
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
vidfile = cv2.VideoWriter('out-video.avi', fourcc,
                          cfg_cam_fps, cfg_cam_res)

# initlaize deeppicar modules
#actuator.init(cfg_throttle)
camera.init(res=cfg_cam_res, fps=cfg_cam_fps, threading=use_thread)
atexit.register(turn_off)

# initilize dnn model
if use_dnn == True:
    print ("Load TF")
    import tensorflow as tf
    model = __import__(params.model)
    import local_common as cm
    import preprocess

    print ("Load Model")
    config = tf.ConfigProto(intra_op_parallelism_threads=NCPU,
                            inter_op_parallelism_threads=NCPU, \
                            allow_soft_placement=True,
                            device_count = {'CPU': 1})

    sess = tf.InteractiveSession(config=config)
    saver = tf.train.Saver()
    model_load_path = cm.jn(params.save_dir, params.model_load_file)
    saver.restore(sess, model_load_path)
    print ("Done..")

# null_frame = np.zeros((cfg_cam_res[0],cfg_cam_res[1],3), np.uint8)
# cv2.imshow('frame', null_frame)

g = g_tick()
start_ts = time.time()

frame_arr = []
angle_arr = []

# enter main loop
while True:
    if use_thread:
        time.sleep(next(g))
    frame = camera.read_frame()
    ts = time.time()

    # read a frame

    if view_video == True:
        cv2.imshow('frame', frame)
        ch = cv2.waitKey(1) & 0xFF
    else:
        ch = ord(input_kbd.read_single_keypress())

    if ch == ord('j'):
        #actuator.left()
        print ("left")
        angle = deg2rad(-30)
        btn   = ord('j')
    elif ch == ord('k'):
        #actuator.center()
        print ("center")
        angle = deg2rad(0)
        btn   = ord('k')
    elif ch == ord('l'):
        #actuator.right()
        print ("right")
        angle = deg2rad(30)
        btn   = ord('l')
    elif ch == ord('a'):
        #actuator.ffw()
        print ("accel")
    elif ch == ord('s'):
        #actuator.stop()
        print ("stop")
        btn   = ch
    elif ch == ord('z'):
        #actuator.rew()
        print ("reverse")
    elif ch == ord('q'):
        break
    elif ch == ord('r'):
        print ("toggle record mode")
        if rec_start_time == 0:
            rec_start_time = ts
        else:
            rec_start_time = 0
    elif ch == ord('t'):
        print ("toggle video mode")
        if view_video == False:
            view_video = True
        else:
            view_video = False
    elif ch == ord('d'):
        print ("toggle DNN mode")
        if use_dnn == False:
            use_dnn = True
        else:
            use_dnn = False

    if use_dnn == True:
        #actuator.ffw()
        # 1. machine input
        frame = camera.read_frame()
        img = preprocess.preprocess(frame)
        #ts_ = time.time()
        angle = model.y.eval(feed_dict={model.x: [img]})[0][0]
        middle_time = time.time()
        car_angle = 0

        # controll mode
        degree = rad2deg(angle)
        message = "nothing"
        if degree < 15 and degree > -15:
            #actuator.center()
            car_angle = 0
            btn = ord('k')
            message = "forward"
            rospy.loginfo("forward")
            line_detect(message)
        elif degree >= 15:
            #actuator.right()
            car_angle = 30
            btn = ord('l')
            message = "right"
            line_detect(message)
        elif degree <= -15:
            #actuator.left()
            car_angle = -30
            btn = ord('j')
            message = "left"
            line_detect(message)

#   print "%.3f: took %.2fd ms" %(ts - start_ts, float(dur*1000))

    if rec_start_time > 0 and not frame is None:
        # increase frame_id
        frame_id += 1

        # write input (angle)
        str = "{},{},{}\n".format(int(ts*1000), frame_id, angle)
        keyfile.write(str)

        # write input (button: left, center, stop, speed)
        str = "{},{},{},{}\n".format(int(ts*1000), frame_id, btn, cfg_throttle)
        keyfile_btn.write(str)

        if use_dnn and fpv_video:
            textColor = (255,255,255)
            bgColor = (0,0,0)
            newImage = Image.new('RGBA', (100, 20), bgColor)
            drawer = ImageDraw.Draw(newImage)
            drawer.text((0, 0), "Frame #{}".format(frame_id), fill=textColor)
            drawer.text((0, 10), "Angle:{}".format(car_angle), fill=textColor)
            newImage = cv2.cvtColor(np.array(newImage), cv2.COLOR_BGR2RGBA)
            frame = cm.overlay_image(frame,
                                     newImage,
                                     x_offset = 0, y_offset = 0)
        # write video stream
        vidfile.write(frame)
        if frame_id >= 1000:
            print ("recorded 1000 frames")
            break
        print ("%.3f %d %.3f %d %d(ms)" %
           (ts, frame_id, angle, btn, int((time.time() - ts)*1000)))

print ("Finish..")


turn_off()
