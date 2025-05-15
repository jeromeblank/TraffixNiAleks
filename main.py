import cv2
import numpy as np
import time
import serial

# Serial communication setup for EW lanes on COM4 and NS lanes on COM6
arduino_ew = serial.Serial('COM4', 9600)  # For East-West lanes
arduino_ns = serial.Serial('COM5', 9600)  # For North-South lanes

# Load video captures
caps = [
    cv2.VideoCapture("videos/traffic1.mp4"),  # NS lane 1
    cv2.VideoCapture("videos/traffic2.mp4"),  # EW lane 1
    cv2.VideoCapture("videos/traffic1.mp4"),  # NS lane 2
    cv2.VideoCapture("videos/traffic2.mp4")   # EW lane 2
]

# Background subtractors
bg_subtractors = [
    cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=25, detectShadows=True)
    for _ in range(4)
]

# Track paused frames and vehicle counts
paused_frames = [None] * 4
last_vehicle_count = [0] * 4
width, height = 384, 216

# Signal state tracking
signal_state = "NS_GREEN"
last_green_time = time.time()
yellow_start_time = None
in_yellow_phase = False
stop_start_time = time.time()
yellow_duration = 10
go_duration = 30

# Store red-light vehicle counts and comparison timing
red_vehicle_counts = {'NS': 0, 'EW': 0}
comparison_triggered = False
comparison_time = None

def detect_vehicles(frame, subtractor, index):
    fg_mask = subtractor.apply(frame)
    _, mask = cv2.threshold(fg_mask, 220, 255, cv2.THRESH_BINARY)
    kernel = np.ones((2, 2), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    vehicle_count = 0
    for cnt in contours:
        if cv2.contourArea(cnt) > 50:
            vehicle_count += 1
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    edges = cv2.Canny(mask, 40, 120)
    mask_visual = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

    if vehicle_count > last_vehicle_count[index]:
        last_vehicle_count[index] = vehicle_count

    return frame, mask_visual, vehicle_count

def overlay_info(frame, signal, count, timer_text):
    if signal == "GREEN":
        color, text = (0, 255, 0), "GO"
    elif signal == "YELLOW":
        color, text = (0, 255, 255), "READY"
    else:
        color, text = (0, 0, 255), "STOP"

    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
    cv2.putText(frame, f"Vehicles: {count}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f"Timer: {timer_text}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return frame

while True:
    color_frames, mask_frames, counts = [], [], []
    now = time.time()

    ns_total, ew_total = 0, 0

    # Iterate over the cameras for both NS and EW lanes
    for i, cap in enumerate(caps):
        if i in [0, 2]:  # North-South lanes
            group = "NS"
            group_is_green = (signal_state == "NS_GREEN")
            group_is_yellow = (in_yellow_phase and signal_state != "NS_GREEN" and signal_state == "EW_GREEN")
        else:  # East-West lanes
            group = "EW"
            group_is_green = (signal_state == "EW_GREEN")
            group_is_yellow = (in_yellow_phase and signal_state != "EW_GREEN" and signal_state == "NS_GREEN")

        play_video = group_is_green
        frame = None

        if play_video:
            ret, frame = cap.read()
            if not ret or cap.get(cv2.CAP_PROP_POS_FRAMES) >= cap.get(cv2.CAP_PROP_FRAME_COUNT):
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = cap.read()
            paused_frames[i] = frame.copy() if ret else paused_frames[i]
        else:
            frame = paused_frames[i] if paused_frames[i] is not None else np.zeros((height, width, 3), np.uint8)

        frame = cv2.resize(frame, (width, height))
        frame, mask, count = detect_vehicles(frame, bg_subtractors[i], i)

        if group == "NS":
            ns_total += count
        else:
            ew_total += count

        if in_yellow_phase:
        # During yellow phase, both lanes are "READY" (yellow)
            status = "YELLOW"
        else:
            if group_is_green:
                status = "GREEN"
            else:
                status = "RED"


        # Timer logic
        if in_yellow_phase:
            remaining_time = max(yellow_duration - int(now - yellow_start_time), 0)
        else:
            remaining_time = max(go_duration - int(now - stop_start_time), 0)

        timer_text = f"{remaining_time}s"
        frame = overlay_info(frame, status, count, timer_text)

        color_frames.append(frame)
        mask_frames.append(mask)
        counts.append(count)

        # Send data to Arduino
        # Send signals to Arduino based on global signal state and yellow phase
        if in_yellow_phase:
            # Both lanes yellow
            arduino_ns.write('Y'.encode())
            arduino_ew.write('Y'.encode())
        else:
            if signal_state == "NS_GREEN":
                arduino_ns.write('G'.encode())
                arduino_ew.write('R'.encode())
            elif signal_state == "EW_GREEN":
                arduino_ns.write('R'.encode())
                arduino_ew.write('G'.encode())


    # Determine which is red and which is green
    if signal_state == "NS_GREEN":
        red_group = "EW"
        red_count = ew_total
        green_count = ns_total
    elif signal_state == "EW_GREEN":
        red_group = "NS"
        red_count = ns_total
        green_count = ew_total
    else:
        red_group = None
        red_count = 0
        green_count = 0

    # Save red group count when signal turns red
    if not in_yellow_phase and now - stop_start_time < 1:  # just switched
        red_vehicle_counts[red_group] = red_count  # Save the vehicle count of the red lane just before it goes red
        comparison_time = now + 15  # Schedule comparison in 15 seconds
        comparison_triggered = False

    # Trigger early switch if condition met
    if (
        not in_yellow_phase
        and comparison_time
        and now >= comparison_time
        and not comparison_triggered
    ):
        if red_vehicle_counts[red_group] > green_count:
            print(f"Early switch triggered: {red_group} has more vehicles.")
            print(f"Red Lane ({red_group}) Vehicle Count: {red_vehicle_counts[red_group]}")
            print(f"Green Lane ({'EW' if red_group == 'NS' else 'NS'}) Vehicle Count: {green_count}")
            in_yellow_phase = True
            yellow_start_time = now
            comparison_triggered = True

    # Signal logic
    # Yellow light timing and transition logic

    if in_yellow_phase:
        # Yellow phase ongoing for both lanes
        if time.time() - yellow_start_time >= yellow_duration:
            # After yellow, switch signals
            if signal_state == "NS_GREEN":
                signal_state = "EW_GREEN"
                print("Yellow ended: NS lanes STOP | EW lanes GO")
            else:
                signal_state = "NS_GREEN"
                print("Yellow ended: EW lanes STOP | NS lanes GO")
            
            # Reset timers and yellow flag
            stop_start_time = time.time()
            yellow_start_time = None
            in_yellow_phase = False

    else:
        # If green phase duration elapsed, start yellow phase for both lanes
        if time.time() - stop_start_time >= go_duration:
            in_yellow_phase = True
            yellow_start_time = time.time()
            print("Green phase ended: Both lanes READY (Yellow)")


    row1 = np.hstack((color_frames[0], mask_frames[0]))
    row2 = np.hstack((color_frames[1], mask_frames[1]))
    row3 = np.hstack((color_frames[2], mask_frames[2]))
    row4 = np.hstack((color_frames[3], mask_frames[3]))
    combined = np.vstack((row1, row2, row3, row4))

    cv2.imshow("Traffic Management Simulation", combined)

    if cv2.waitKey(30) & 0xFF == 27:
        break


for cap in caps:
    cap.release()
cv2.destroyAllWindows()
