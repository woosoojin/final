#!/usr/bin/env python 
from __future__ import division

import tensorflow as tf
import params
model = __import__(params.model)
import cv2
import subprocess as sp
import itertools
import sys
import os
import preprocess
import time
import math
import numpy as np
import local_common as cm



def deg2rad(deg):
        return deg * math.pi / 180.0
def rad2deg(rad):
        return 180.0 * rad / math.pi

#Get and set the number of cores to be used by TensorFlow
NCPU=int(sys.argv[1])
config = tf.ConfigProto(intra_op_parallelism_threads=NCPU, inter_op_parallelism_threads=NCPU, \
                        allow_soft_placement=True, device_count = {'CPU': 1})

#The max number of frames to be processed, and the number of frames already processed
NFRAMES = 1000
curFrame = 0

#Load the model
sess = tf.InteractiveSession(config=config)
saver = tf.train.Saver()
model_load_path = cm.jn(params.save_dir, params.model_load_file)
saver.restore(sess, model_load_path)

#List the epochs to be used for testing
epoch_ids = sorted(list(set(itertools.chain(*params.epochs.values()))))
epoch_ids = [1,1]

#Create lists for tracking operation timings
cap_time_list = []
prep_time_list = []
pred_time_list = []
tot_time_list = []

image_time_list = []
dnn_time_list = []
motor_time_list = []

#Initialize the desired number of bandwidth co-runners with the given access type
numCR = int(sys.argv[2]) + 1
if numCR > 1:
    numCR2 = numCR - 1
    
    access = sys.argv[3]
    accessCap = access.capitalize()

    folderName = "+{}{}CR".format(numCR2, accessCap)
    os.system('mkdir datafiles/{}CR/{}'.format(accessCap, folderName))

    for i in range(1,numCR):
        os.system('bandwidth -a {} -m 16384 -t 10000 -c {} &'.format(access,i))

#Process all epochs
dead_count = 0
for epoch_id in epoch_ids:
    print '---------- processing video for epoch {} ----------'.format(epoch_id)

    #Get the number of frames in the epoch
    vid_path = cm.jn(params.data_dir, 'out-video-{}.avi'.format(epoch_id))
    assert os.path.isfile(vid_path)
    frame_count = cm.frame_count(vid_path)
    a = time.time()
    cap = cv2.VideoCapture(vid_path)
    b = time.time()
    print (b-a)*1000

    machine_steering = []

    #Process the current epoch while recording the operation execution times
    print 'performing inference...'
    time_start = time.time()
    for frame_id in xrange(frame_count):
        if curFrame < NFRAMES:
            cam_start = time.time()
            ret, img = cap.read()
            assert ret

            #prep_start = time.time()
            img = preprocess.preprocess(img)

            pred_start = time.time()
            rad = model.y.eval(feed_dict={model.x: [img]})[0][0]
            middle_time = time.time()
            deg = rad2deg(rad)
            pred_end   = time.time()

            #cam_time  = (prep_start - cam_start)*1000
            #prep_time = (pred_start - prep_start)*1000
            #pred_time = (pred_end - pred_start)*1000
            tot_time  = (pred_end - cam_start)*1000

            image_time = (pred_start - cam_start)*1000
            dnn_time = (middle_time - pred_start)*1000
            motor_time = (pred_end - middle_time)*1000

            if frame_id > 1:
                 if tot_time > 50:
                     dead_count = dead_count + 1

           # print  'took: {:0.2f} ms'.format(tot_time)
           # print  'image processing: %.2f | dnn: %.2f | motor: %.2f' % (image_time, dnn_time, motor_time)
        
            if frame_id > 0:
                image_time_list.append(image_time)
                dnn_time_list.append(dnn_time)
                motor_time_list.append(motor_time)
                tot_time_list.append(tot_time)
                #machine_steering.append(deg)
                curFrame += 1
        
    cap.release()

    fps = frame_count / (time.time() - time_start)
    
    print 'completed inference, total frames: {}, average fps: {} Hz'.format(frame_count, round(fps, 1))
    
#Interrupt all bandwidth co-runners
if numCR > 1:
        os.system('killall -SIGINT bandwidth')

#Calculate and display statistics of the total inferencing times
#print "count:", len(tot_time_list)
print "deadline miss:", dead_count
print "======== IMAGE TIME ========"
#print "deadline miss pct:",dead_count / len(tot_time_list) * 100, "%"
print "mean:", np.mean(image_time_list)
print "max:", np.max(image_time_list)
print "min:", np.min(image_time_list)
#print "99.999pct:", np.percentile(tot_time_list, 99.999)
#print "99.99pct:", np.percentile(tot_time_list, 99.99)
#print "99.9pct:", np.percentile(tot_time_list, 99.9)
#print "99pct:", np.percentile(tot_time_list, 99)
#print "median:", np.median(tot_time_list)
print "stdev:", np.std(image_time_list)

print "======== DNN TIME ========"
#SET = $(seq 0 100)
print "mean:", np.mean(dnn_time_list)
print "max:", np.max(dnn_time_list)
print "min:", np.min(dnn_time_list)
print "stdev:", np.std(dnn_time_list)

print "======== MOTOR TIME ========"
print "mean:", np.mean(motor_time_list)
print "max:", np.max(motor_time_list)
print "min:", np.min(motor_time_list)
print "stdev:", np.std(motor_time_list)

print "========= TOTAL TIME ========"
print "mean:", np.mean(tot_time_list)
print "max:", np.max(tot_time_list)
print "min:", np.min(tot_time_list)
print "stdev:", np.std(tot_time_list)
