
import cv2
import numpy as np
import matplotlib.pyplot as plt
from IPython.display import clear_output, display
import ipywidgets as widgets
from ipykernel.kernelapp import IPKernelApp
from Transbot_Lib import Transbot
import time
 
bot = Transbot()
bot.set_pwm_servo(1, 90)   # pan
bot.set_pwm_servo(2, 60)   # tilt
 
ARM_LOW_POSITION = (89, 201, 37)
 
kernel = IPKernelApp.instance().kernel
 
def pump_events(n=1):
    for _ in range(n):
        kernel.do_one_iteration()
 
def car_motion(line, angular):
    if abs(line) >= 5 or abs(angular) >= 10:
        print(f"driving: line={line/100.0} angular={angular/100.0}")
        bot.set_car_motion(line / 100.0, angular / 100.0)
    else:
        bot.set_car_motion(0, 0)
 
def arm_servo_all(s7, s8, s9):
    bot.set_uart_servo_angle_array(s7, s8, s9)
 
def arm_to_low():
    arm_servo_all(*ARM_LOW_POSITION)
 
def safe_stop(bot, retries=3):
    for _ in range(retries):
        try:
            bot.set_car_motion(0, 0)
            time.sleep(0.05)
        except Exception as e:
            print(f"stop attempt failed: {e}")
    try:
        if hasattr(bot, 'ser') and bot.ser and bot.ser.is_open:
            bot.ser.close()
            print("serial connection closed")
    except Exception as e:
        print(f"serial close failed: {e}")
 
def manual_drive_and_drop():
    print("Manual drive mode - click a direction to start moving, click again to stop")
 
    drive_state = {"direction": None}
    drop_complete = [False]
 
    def on_forward_toggle(change):
        if change["name"] == "value":
            drive_state["direction"] = "forward" if change["new"] else None
            print("moving: forward" if change["new"] else "stopped")
 
    def on_backward_toggle(change):
        if change["name"] == "value":
            drive_state["direction"] = "backward" if change["new"] else None
            print("moving: backward" if change["new"] else "stopped")
 
    def on_left_toggle(change):
        if change["name"] == "value":
            drive_state["direction"] = "left" if change["new"] else None
            print("moving: left" if change["new"] else "stopped")
 
    def on_right_toggle(change):
        if change["name"] == "value":
            drive_state["direction"] = "right" if change["new"] else None
            print("moving: right" if change["new"] else "stopped")
 
    def on_stop(b):
        drive_state["direction"] = None
        btn_forward.value = False
        btn_backward.value = False
        btn_left.value = False
        btn_right.value = False
        print("stopped")
 
    def on_drop(b):
        drive_state["direction"] = None
        car_motion(0, 0)
        print("dropping off")
        arm_servo_all(110, 190, 150)
        time.sleep(1)
        bot.set_uart_servo_angle(9, 37)
        time.sleep(1)
        print("returning to home position")
        arm_to_low()
        drop_complete[0] = True
 
    btn_forward = widgets.ToggleButton(description="Forward")
    btn_backward = widgets.ToggleButton(description="Backward")
    btn_left = widgets.ToggleButton(description="Left")
    btn_right = widgets.ToggleButton(description="Right")
    btn_stop = widgets.Button(description="Stop", button_style="warning")
    btn_drop = widgets.Button(description="Drop", button_style="danger")
    btn_forward.observe(on_forward_toggle)
    btn_backward.observe(on_backward_toggle)
    btn_left.observe(on_left_toggle)
    btn_right.observe(on_right_toggle)
    btn_stop.on_click(on_stop)
    btn_drop.on_click(on_drop)
 
    top_row = widgets.HBox([btn_left, btn_forward, btn_right])
    img_output = widgets.Output()
    display(top_row, btn_backward, widgets.HBox([btn_stop, btn_drop]), img_output)

    last_send = 0
    frame_counter = 0
    while not drop_complete[0]:
        pump_events(5)
        now = time.time()
        if now - last_send >= 0.15:
            d = drive_state["direction"]
            if d == "forward":
                car_motion(15, 0)
            elif d == "backward":
                car_motion(-15, 0)
            elif d == "left":
                car_motion(0, 30)
            elif d == "right":
                car_motion(0, -30)
            else:
                car_motion(0, 0)
            last_send = now

        ret, frame = cap.read()
        if ret:
            frame_counter += 1
            if frame_counter % 4 == 0:  # slower refresh so it's actually viewable
                with img_output:
                    clear_output(wait=True)
                    plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                    plt.axis('off')
                    plt.show()
                    plt.close()
        time.sleep(0.02)

def pickup_sequence():
    bot.set_car_motion(0, 0)
    time.sleep(0.2)
 
    print("moving to grab position")
    arm_servo_all(110, 190, 37)
    time.sleep(1)
 
    print("closing gripper")
    arm_servo_all(110, 190, 130)
    time.sleep(1)
 
    print("lifting arm")
    arm_servo_all(160, 190, 150)
    time.sleep(1)
 
    print("lifting higher")
    arm_servo_all(160, 140, 160)
    time.sleep(1)
 
    manual_drive_and_drop()
 
 
arm_to_low()
time.sleep(1)
 
net = cv2.dnn.readNetFromDarknet('/root/temp/models/yolov4-tiny.cfg', '/root/temp/models/yolov4-tiny.weights')
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
 
with open('/root/temp/models/coco.names', 'r') as f:
    classes = [line.strip() for line in f.readlines()]
 
conf_threshold = 0.20
target_classes = ["vase", "bottle", "pottedplant"]
 
layer_names = net.getLayerNames()
unconnected = net.getUnconnectedOutLayers()
if hasattr(unconnected[0], '__len__'):
    output_layers = [layer_names[i[0] - 1] for i in unconnected]
else:
    output_layers = [layer_names[i - 1] for i in unconnected]
 
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
 
pan_angle = 90
tilt_angle = 60
dead_zone = 40
state = "centering"
initial_area = None
has_backed_up = False
approach_start_time = None
min_approach_duration = 1.5  # seconds
# tuning constants
LOST_FRAMES_REQUIRED = 5         # consecutive missed frames before treating as "lost"
RENDER_EVERY = 3                 # only draw every 3rd frame to speed up the loop
CLOSE_RANGE_AREA_PCT = 0.047      # absolute frame coverage at which pivoting-in-place risks swinging arm past object
BACKUP_DURATION = 1.1            # seconds to back up before re-centering at close range
TURN_PULSE = 0.3                 # seconds per centering turn burst (bumped up for carpet friction)
BLIND_CREEP_DURATION = 0.9       # seconds of blind forward creep once detection is lost - tune to hit ~3 inches
search_lost_since = None
SEARCH_GRACE_PERIOD = 1.0     # seconds of no detection before starting to search
SEARCH_TURN_SPEED = 14        # angular speed while searching (continuous, not pulsed)
 
frame_count = 0
consecutive_lost = 0
lost_since = None
 
 
try:
    while True:
        ret, frame = cap.read()
        print(f"cap.read() ret={ret}")
        if not ret:
            continue
 
        h, w = frame.shape[:2]
        frame_area = w * h
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
        net.setInput(blob)
        outputs = net.forward(output_layers)
 
        best_box = None
        best_conf = 0
        best_class_id = None
 
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if classes[class_id] in target_classes and confidence > conf_threshold:
                    if confidence > best_conf:
                        cx, cy, bw, bh = detection[0]*w, detection[1]*h, detection[2]*w, detection[3]*h
                        x = int(cx - bw/2)
                        y = int(cy - bh/2)
                        best_box = (x, y, int(bw), int(bh))
                        best_conf = confidence
                        best_class_id = class_id
        if best_box:
            consecutive_lost = 0
            lost_since = None
            search_lost_since = None
            if state == "searching":
                car_motion(0, 0)
                state = "centering"
                print("found target - re-centering")
 
            x, y, bw, bh = best_box
            box_area = bw * bh
            area_pct = box_area / frame_area
            box_center_x = x + bw / 2
            box_center_y = y + bh / 2
            frame_center_x = w / 2
            frame_center_y = h / 2
            offset_x = box_center_x - frame_center_x
            offset_y = box_center_y - frame_center_y
 
            cv2.rectangle(frame, (x, y), (x+bw, y+bh), (0, 255, 0), 2)
 
            if state == "centering":
                print(f"centering: offset_x={offset_x:.0f} dead_zone={dead_zone} area_pct={area_pct:.2%}")
                if abs(offset_x) > dead_zone:
                    if area_pct >= CLOSE_RANGE_AREA_PCT and not has_backed_up:
                        print(f"still close and off-center (area_pct={area_pct:.2%}) - backing up before pivot")
                        car_motion(-15, 0)
                        time.sleep(BACKUP_DURATION)
                        car_motion(0, 0)
                        has_backed_up = True
                    turn_speed = max(25, min(35, abs(offset_x) / 5))
                    angular = turn_speed if offset_x < 0 else -turn_speed
                    print(f"centering: angular={angular}")
                    car_motion(0, angular)
                    time.sleep(TURN_PULSE)
                    car_motion(0, 0)
                else:
                    car_motion(0, 0)
                    has_backed_up = False
                    state = "approaching"
                    if initial_area is None:
                        initial_area = box_area
                    approach_start_time = time.time()
                    print("centered, approaching")
 
            elif state == "approaching":
                print(f"approach: area={box_area:.0f} area_pct={area_pct:.2%}")
                if abs(offset_x) > dead_zone:
                    if area_pct >= CLOSE_RANGE_AREA_PCT and not has_backed_up:
                        print(f"drifted off-center at close range (area_pct={area_pct:.2%}) - backing up before recenter")
                        car_motion(-15, 0)
                        time.sleep(BACKUP_DURATION)
                        car_motion(0, 0)
                        has_backed_up = True
                    state = "centering"
                    print("lost center, re-centering")
                else:
                    speed = 10 if area_pct < 1.8 else 4
                    car_motion(speed, 0)
 
 
            print(f"state={state} class={classes[best_class_id]} conf={best_conf:.2f} offset_x={offset_x:.0f} offset_y={offset_y:.0f} area={box_area:.0f}")
 
        else:
            consecutive_lost += 1
            if state == "approaching":
                if lost_since is None:
                    lost_since = time.time()
                    car_motion(0, 0)
                    time.sleep(0.15)
                    print("lost sight while approaching - creeping forward blind")
                elapsed = time.time() - lost_since
                if elapsed < BLIND_CREEP_DURATION:
                    car_motion(8, 0)
                else:
                    car_motion(0, 0)
                    if consecutive_lost >= LOST_FRAMES_REQUIRED:
                        print(f"[TRIGGER] arm-blocked fallback: elapsed={elapsed:.2f}s consecutive_lost={consecutive_lost}")
                        pickup_sequence()
                        state = "done"
            elif state == "centering":
                if search_lost_since is None:
                    search_lost_since = time.time()
                elapsed = time.time() - search_lost_since
                if elapsed > SEARCH_GRACE_PERIOD:
                    state = "searching"
                    print("no detection for a while - starting search")
                else:
                    car_motion(0, 0)
            elif state == "searching":
                print("searching: rotating to find target")
                car_motion(0, SEARCH_TURN_SPEED)
            elif state != "done":
                car_motion(0, 0)
                state = "centering"
                print("no detection")
 
        frame_count += 1
        if frame_count % RENDER_EVERY == 0:
            clear_output(wait=True)
            plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            plt.axis('off')
            plt.show()
            plt.show()
        if state == "done":
            print("task complete")
            break
 
finally:
    safe_stop(bot)
    bot.set_pwm_servo(1, 90)
    bot.set_pwm_servo(2, 60)
    arm_to_low()
    cap.release()
