import asyncio
import base64
import json
import time
import os
from io import BytesIO
from multiprocessing import Process, Queue, Manager

import cv2
import numpy as np
import websockets
from PIL import Image

from Gray_lane_cal import cal_steering
from traffic_sign_detection import *

# Initalize traffic sign classifier
file_path = os.path.dirname(os.path.realpath(__file__))
traffic_sign_model = cv2.dnn.readNetFromONNX(
    fr"{file_path}\traffic_sign_classifier_lenet_v3.onnx")

# Global queue to save current image
# We need to run the sign classification model in a separate process
# Use this queue as an intermediate place to exchange images
g_image_queue = Queue(maxsize=5)

# Function to run sign classification model continuously
# We will start a new process for this
def process_traffic_sign_loop(g_image_queue, pre_signs, current_signs, processed):
    while True:
        if g_image_queue.empty():
            time.sleep(0.1)
            continue
        image = g_image_queue.get()

        # Prepare visualization image
        draw = image.copy()

        # Detect traffic signs
        # Store the sign used to compare and get the loss sign
        pre_signs[:] = []
        pre_signs.extend(current_signs)
        current_signs[:] = []
        current_signs.extend(list(detect_traffic_signs(image, traffic_sign_model, None)))
        loss_sign = None 

        # Compare and get the lossing sign
        if len(pre_signs) > 0:
            if len(current_signs) == 0:
                loss_sign = pre_signs[0][0]
            else:
                if pre_signs[0][0] not in current_signs[0]:
                    loss_sign = pre_signs[0][0]
                else:
                    loss_sign = None
        
        # Assignt loss_sign for processed, which is used to check the turn is do or don't
        if len(processed[0]) == 0 and loss_sign:
            processed[:] = []
            processed.extend([loss_sign, time.time()]) # get the time when sign loss

        # Show the result to a window
        cv2.imshow("Traffic signs", draw)
        cv2.waitKey(1)

async def process_image(websocket, path, processed):
    async for message in websocket:
        # Get image from simulation
        data = json.loads(message)
        speed = float(data['speed'])
        image = Image.open(BytesIO(base64.b64decode(data["image"])))
        image = np.asarray(image)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # Prepare visualization image
        draw = image.copy()

        # Is processed command is request
        sign_ = None
        if len(processed[0]) > 0:
            sign_ = processed[0]
        
        # Cal and get throttle and steering angle
        throttle, steering_angle, done = cal_steering(image, turn_range = 0.85, sign=sign_, draw=draw)

        # The turn have done of wait time is out, remove processed sign
        if done is True or time.time() - processed[1] > 5:
            processed[:] = ['',0]        

        # Update image to g_image_queue - used to run sign detection
        if not g_image_queue.full():
            g_image_queue.put(image)

        # Show the result to a window
        cv2.imshow("Result", draw)
        cv2.waitKey(1)

        # Send back throttle and steering angle
        message = json.dumps(
            {"throttle": throttle, "steering": steering_angle})
        await websocket.send(message)

async def main(processed):
    async with websockets.serve(lambda wbsoc, path:process_image(wbsoc, path, processed), "0.0.0.0", 4567, ping_interval=None):
        await asyncio.Future()  # run forever

if __name__ == '__main__':
    # The sharing variables
    man = Manager()
    pre_signs = man.list([]) 
    current_signs = man.list([])
    processed = man.list(['',0])

    # Multiprocessing start
    p = Process(target=process_traffic_sign_loop, 
                args=(g_image_queue, pre_signs, current_signs, processed))
    p.start()
    asyncio.run(main(processed))