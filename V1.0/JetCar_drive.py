import cv2
import numpy as np
import time
from multiprocessing import Process, Queue, Manager
from jetcam.csi_camera import CSICamera
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import pygame
from Gray_lane_cal import cal_steering

# Set up I2C and PCA9685
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 60

# Set up pygame 
pygame.init()
pygame.joystick.init()

# Define channels and pulse settings for steering and throttle
STEERING_CHANNEL = 0
THROTTLE_CHANNEL = 1
LEFT_PULSE = 290
RIGHT_PULSE = 490
MAX_PULSE = 500
ZERO_PULSE = 370
MIN_PULSE = 220

# Initialize shared resources
frame_queue = Queue(maxsize=4)
manager = Manager()

set_car = manager.list([0, 0])
js_sig = manager.list([[], []])

emergency_flag = manager.Value(bool, False)

# Functions for car movement
def set_pwm(channel, pulse):
    pca.channels[channel].duty_cycle = int(pulse * 65535 / 4096)

def move_car(steering_angle, throttle_speed):
    steering_pulse = int(ZERO_PULSE + (RIGHT_PULSE - LEFT_PULSE) * steering_angle / 2)
    throttle_pulse = int(ZERO_PULSE + (MAX_PULSE - MIN_PULSE) * throttle_speed / 2)
    set_pwm(STEERING_CHANNEL, steering_pulse)
    set_pwm(THROTTLE_CHANNEL, throttle_pulse)

def apply(set_car):
    """Continuously apply steering and throttle values."""
    while True:
        move_car(*set_car)

def stop_car():
    """Stop the car."""
    set_pwm(THROTTLE_CHANNEL, ZERO_PULSE)
    set_pwm(STEERING_CHANNEL, ZERO_PULSE)
    print("Stopped the car")

# Camera frame capturing
def capture_frames(frame_queue):
    camera = CSICamera(width=680, height=480, capture_fps=24)
    while True:
        frame = camera.read()
        _, jpeg_frame = cv2.imencode('.jpg', frame)
        if not frame_queue.full():
            frame_queue.put(jpeg_frame.tobytes())
        else:
            frame_queue.get()
            frame_queue.put(jpeg_frame.tobytes())
        time.sleep(0.008)
    camera.cap.release()

def get_frame():
    """Retrieve the latest frame from the queue."""
    if not frame_queue.empty():
        return frame_queue.get()
    return None

# Emergency handling
def emergency():
    emergency_flag.value = not emergency_flag.value
    set_car[:] = [0, 0]

# Process management
def stop_process():
    """Stop all running processes."""
    capture_process.terminate()
    capture_process.join()
    send_process.terminate()
    send_process.join()
    process_control.terminate()
    process_control.join()
    stop_car()
    print("Stopped the processes...")

def start_process():
    """Start the car's processes."""
    global capture_process, send_process, process_control

    # Start the camera process
    capture_process = Process(target=capture_frames, args=(frame_queue,))
    
    # Start the car control process
    send_process = Process(target=apply, args=(set_car,))
    
    # Start the processing and control process
    process_control = Process(target=process_and_control)

    capture_process.start()
    send_process.start()
    process_control.start()

    time.sleep(2)
    print("System has been started...")


def process_and_control():
    """ Process images from the camera and control the car directly """
    start_time = time.time()
    try:
        while True:
            frame = get_frame()
            if frame is None:
                continue
            
            try:
                steering_angle, throttle, _ = cal_steering(frame, turn_range=0.85, speed=0.5)
            except Exception as e:
                print(f"Error in processing frame: {e}")
                continue
            
            control_signal = {
                "steering": steering_angle,
                "throttle": throttle
            }
            send(control_signal)
            if time.time() - start_time > 360:
                break

            #cv2.imshow("Processed Frame", frame)  
            
    except:
        stop_process()

def send(signal: dict):
    """
    Send control signals to the car.
    steering: float
    throttle: float
    """
    if not emergency_flag.value:
        set_car[:] = [signal['steering'], signal['throttle']]
