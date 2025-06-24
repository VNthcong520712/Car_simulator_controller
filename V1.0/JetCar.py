from jetcam.csi_camera import CSICamera
import multiprocessing as mp
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import cv2
import time
import pygame

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
frame_queue = mp.Queue(maxsize=4)
manager = mp.Manager()

set_car = manager.list([0, 0])
js_sig = manager.list([[],[]])

emergency_flag = manager.Value(bool, False)

error_capture = manager.list([])

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

# Pygame handling
def js():
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    axes = joystick.get_numaxes()
    buttons = joystick.get_numbuttons()

    while True: 
        pygame.event.pump()
        axes_sig = [0]*axes
        button_sig = [0]*buttons

        for i in range(buttons): 
            if i <= axes: 
                axes_sig[i] = joystick.get_axis(i)
            button_sig[i] = joystick.get_button(i)
        if button_sig[15] == 1: # start button in gamepad
            emergency()
        js_sig[:] = [axes_sig, button_sig]
        time.sleep(0.01)

# Control functions
def send(signal:dict):
    """Send control signals to the car.  
    steering: float  
    throttle: float"""
    if not emergency_flag.value:
        set_car[:] = [signal['steering'], signal['throttle']]

# Process management
def stop_process():
    """Stop all running processes."""
    capture_process.terminate()
    capture_process.join()
    send_process.terminate()
    send_process.join()
    joy_process.terminate()
    joy_process.join()
    stop_car()
    print("Stopped the processes...")

# Emergency handling
def emergency():
    emergency_flag.value = not emergency_flag.value
    set_car[:] = [0,0]
    
# Driving mode
def mode(mode:int):
    '''Mode 1: auto  
    Mode 2: gamepad'''
    if mode == 2:
        sig = {}
        sig['throttle'] = js_sig[1][0]
        sig['steering'] = js_sig[1][1]
        send(sig)



def start_process():
    """Start the car's processes."""
    global capture_process, send_process, joy_process

    capture_process = mp.Process(target=capture_frames, args=(frame_queue,))
    send_process = mp.Process(target=apply, args=(set_car,))
    joy_process = mp.Process(target=js)

    capture_process.start()
    send_process.start()

    if pygame.joystick.get_count() == 0: 
        error_capture.append(error("NoJoystickConnected"))
        pygame.quit()
    else: 
        joy_process.start()

    time.sleep(2)

    print("System has been started...")

def error(error_message):
    print(error_message)
    return error_message