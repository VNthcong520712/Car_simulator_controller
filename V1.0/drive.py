import asyncio
import base64
import json
import time
from io import BytesIO
from multiprocessing import Process, Queue, Manager

import cv2
import numpy as np
import websockets
from PIL import Image

from Gray_lane_cal import cal_steering
from traffic_sign_detection import *

# Initalize traffic sign classifier
traffic_sign_model = cv2.dnn.readNetFromONNX(
    r"D:\Documents\Learning\Programming\AI\via\via_from_git\hello-via\p2_traffic_sign_detection\traffic_sign_classifier_lenet_v3.onnx")

# Global queue to save current image
# We need to run the sign classification model in a separate process
# Use this queue as an intermediate place to exchange images
g_image_queue = Queue(maxsize=5)

# Function to run sign classification model continuously
# We will start a new process for this
def process_traffic_sign_loop(g_image_queue, pre_signs, current_signs, processed, public_throttle, public_steering):
    while True:
        if g_image_queue.empty():
            time.sleep(0.1)
            continue
        image = g_image_queue.get()

        # Prepare visualization image
        draw = image.copy()
        x_d = image.copy()
        # Detect traffic signs
        pre_signs[:] = []
        pre_signs.extend(current_signs)
        current_signs[:] = []
        current_signs.extend(list(detect_traffic_signs(image, traffic_sign_model, None)))
        loss_sign = None
        # public_throttle.value, public_steering.value = 0,0

        if len(pre_signs) > 0:
            if len(current_signs) == 0:
                loss_sign = pre_signs[0][0]
            else:
                if pre_signs[0][0] not in current_signs[0]:
                    loss_sign = pre_signs[0][0]
                else:
                    loss_sign = None
        
        if len(processed[0]) == 0 and loss_sign:
            processed[:] = []
            processed.extend([loss_sign, time.time()])

        # Show the result to a window
        cv2.imshow("Traffic signs", draw)
        cv2.waitKey(1)


async def process_image(websocket, path, public_throttle, public_steering, processed):
    async for message in websocket:
        # Get image from simulation
        data = json.loads(message)
        speed = float(data['speed'])
        image = Image.open(BytesIO(base64.b64decode(data["image"])))
        image = np.asarray(image)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # Prepare visualization image
        draw = image.copy()

        # Send back throttle and steering angle
        sign_ = None
        if len(processed[0]) > 0:
            sign_ = processed[0]

        throttle, steering_angle, done = cal_steering(image, turn_range = 0.85, sign=sign_, draw=draw)

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


async def main(public_throttle, public_steering, processed):
    async with websockets.serve(lambda wbsoc, pat:process_image(wbsoc, pat, public_throttle, public_steering, processed), "0.0.0.0", 4567, ping_interval=None):
        await asyncio.Future()  # run forever

if __name__ == '__main__':
    man = Manager()
    pre_signs = man.list([]) 
    current_signs = man.list([])
    processed = man.list(['',0])
    public_throttle = man.Value(float, 0.0)
    public_steering = man.Value(float, 0.0)

    p = Process(target=process_traffic_sign_loop, 
                args=(g_image_queue, pre_signs, current_signs, processed, public_throttle, public_steering))
    p.start()
    asyncio.run(main(public_throttle, public_steering, processed))