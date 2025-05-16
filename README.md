# 🚦 Traffix - Smart Traffic Management Simulation 🚦

*Traffix* is an advanced traffic management simulation system combining *real-time vehicle detection* with *Arduino-controlled traffic lights*. Using OpenCV for video processing and Arduino hardware signaling, it dynamically manages traffic lights based on live vehicle counts from multiple lanes.

---

## 🌟 Features
🚗 Real-time vehicle detection via background subtraction and contour analysis
🛣️ Supports multiple lanes: North-South & East-West via video feeds
s
* 🔴🟡🟢 Dynamic traffic light control with green, yellow, and red phases
⏩ Early green light switching triggered by vehicle queue comparisons for optimized flow
📊 Visual overlays showing traffic signal states, vehicle counts, and countdown timers on video
🔌 Serial communication with Arduino boards to operate physical traffic lights
s
⏯️ Video playback pauses/resumes to simulate stopped/moving traffic realistically


---

## 📂 Folder Structure
'''
Traffix/
│
├── Arduino/
│   ├── Arduino Hardware.png         # Diagram of the Arduino setup
│   └── Traffix/
│       └── Traffix.ino              # Arduino sketch for traffic light control
│
├── videos/
│   ├── traffic1.mp4                 # Video feed for North-South lanes
│   └── traffic2.mp4                 # Video feed for East-West lanes
│
├── main.py                         # Sample Python script for vehicle detection demo
├── video.py                        # Main traffic management simulation script
├── scripts.py                      # Script launcher for concurrent processes
├── README.md                       # This file
└── requirements.txt                # Python dependencies
'''
---

## ⚙️ System Requirements

### Hardware

2 Arduino boards (for East-West and North-South traffic lights)
USB cables to connect Arduinos to PC
Traffic light hardware (LEDs or equivalent for physical simulation)
Optional: 7-segment display for timer simulation


### Software

Python 3.x
Arduino IDE (to upload the Arduino sketch)


### Python Dependencies

Install the following libraries via:

pip install -r requirements.txt

Contents of *requirements.txt*:

numpy==2.2.3
opencv-python==4.11.0.86
pyserial==3.5

---

## 🚀 Setup and Usage

1. *Arduino Setup*

   * Connect two Arduino boards to your PC on ports *COM4* and *COM5* (adjust as needed).
   * Upload the Traffix.ino sketch from the Arduino folder to each Arduino.
   * Connect your traffic light LEDs to the respective Arduino pins as per the sketch diagram.

2. *Video Setup*

   * Place your lane videos (traffic1.mp4, traffic2.mp4) inside the videos/ folder.
   * You can replace or add your own video files to simulate different traffic lanes.

3. *Running the Simulation*

   Use the launcher script scripts.py to run traffic detection and simulation concurrently:

   
   python scripts.py
   

   Alternatively, run the main simulation script directly:

   
   python video.py
   

4. *Controls*

   * A window titled *Traffic Management Simulation* will display video streams with overlays.
   * Vehicle counts, signal states, and timers will update in real time.
   * Press ESC key to exit the simulation.

---

## 🧩 How It Works

The system captures frames from four video feeds (2 North-South, 2 East-West lanes).

* Vehicle detection uses *MOG2 background subtraction* and contour filtering to count vehicles.
Traffic signals cycle through green, yellow, and red phases with configurable durations.
When a lane is green, video plays normally; when red, the video pauses (simulates stopped cars).
The system compares vehicle queues in red and green lanes to trigger early green light switching for efficiency.
Serial commands are sent to Arduino boards to control the physical traffic lights.
Visual overlays indicate current signal states, vehicle counts, and countdown timers on each video frame.


---

## 🔧 Customization

* Change video sources by replacing files in the videos/ folder or modifying paths in video.py.
Update serial COM port names in the script to match your Arduino connections.

* Tune vehicle detection parameters (contour area threshold, background subtractor settings) in video.py.
* Adjust signal timing by editing go_duration and yellow_duration variables in video.py.

---

## 📜 Code Highlights

### main.py — Sample vehicle detection snippet:

import cv2
import numpy as np

cap = cv2.VideoCapture("videos/traffic2.mp4")
object_detector = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=50, detectShadows=True)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    height, width, _ = frame.shape
    roi = frame[550:height, 100:int(width * 0.88)]

    mask = object_detector.apply(roi)
    _, mask = cv2.threshold(mask, 244, 255, cv2.THRESH_BINARY)
    kernel = np.ones((3,3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        if cv2.contourArea(cnt) > 500:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(roi, (x, y), (x + w, y + h), (0, 255, 0), 2)

    frame_resized = cv2.resize(frame, (1024, 576))
    roi_resized = cv2.resize(roi, (512, 288))
    mask_resized = cv2.resize(mask, (512, 288))

    cv2.imshow("Frame", frame_resized)
    cv2.imshow("ROI", roi_resized)
    cv2.imshow("Mask", mask_resized)

    if cv2.waitKey(30) == 27:
        break

cap.release()
cv2.destroyAllWindows()

### video.py — Main traffic simulation logic

Processes 4 video feeds.
Detects vehicles with background subtraction.
Controls signal phases with timing and early green switch logic.
Displays vehicle counts, signal status, and timers.
Simulates pause/play of videos according to signal.


---

## ⚙️ Arduino Sketch (Excerpt from Traffix.ino)

const int redLED = 13;
const int yellowLED = 12;
const int greenLED = 11;

// Segment pins for 7-seg display (A-G)
const int segmentPins[] = {10, 9, 8, 7, 6, 5, 4};

void setup() {
  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  for (int i = 0; i < 7; i++) {
    pinMode(segmentPins[i], OUTPUT);
  }
  Serial.begin(9600);
}

void loop() {
  // Code to read commands from serial and control LEDs & 7-seg display
  // ...
}

---

## 🙋‍♂️ Support & Contribution

Feel free to open issues or submit pull requests for improvements!

---

## 📄 License

MIT License

---

*Enjoy managing traffic smarter and safer with Traffix!* 🚦🚗🛑

---
