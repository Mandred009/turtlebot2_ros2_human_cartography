# Script to perform human pose estimation using tflite model based on MOVENET

#! /usr/bin/env python3

import tensorflow as tf
import numpy as np
import cv2

def get_pose(frame):
    # Loading the TFLITE model
    interpreter = tf.lite.Interpreter(model_path='/home/mandred/models/human_pose.tflite')
    interpreter.allocate_tensors()
    img=frame.copy()
    img = tf.image.resize_with_pad(np.expand_dims(img, axis=0), 192,192) # Model takes 192X192 image as input
    input_image = tf.cast(img, dtype=tf.float32)
    
    # Setup input and output 
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    
    # Make predictions 
    confidence_threshold=0.4
    interpreter.set_tensor(input_details[0]['index'], np.array(input_image))
    interpreter.invoke()
    keypoints_with_scores = interpreter.get_tensor(output_details[0]['index'])
    y, x, c = frame.shape

    shaped = np.squeeze(np.multiply(keypoints_with_scores, [y,x,1]))

    lx=[]
    ly=[]
    for kp in shaped: # going through the different predicted joints
        ky, kx, kp_conf = kp
        if kp_conf > confidence_threshold:
            lx.append(kx)
            ly.append(ky)
            cv2.circle(frame, (int(kx), int(ky)), 6, (255,255,0), -1) 
    return frame,lx[0:4],ly[0:4]

## Uncomment to test on your webcam
# cap = cv2.VideoCapture(0)
# while cap.isOpened():
#         ret, frame = cap.read()
        
#         framee=get_pose(frame)
#         cv2.imshow('MoveNet Lightning', framee)
        
#         if cv2.waitKey(10) & 0xFF==ord('q'):
#             break
            
# cap.release()
# cv2.destroyAllWindows()