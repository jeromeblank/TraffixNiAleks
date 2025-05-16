# Traffix

Traffix is a traffic management simulation system that uses video processing and Arduino-based hardware signaling to simulate and control traffic lights based on real-time vehicle detection. This project uses OpenCV for vehicle detection on video feeds representing traffic lanes and communicates with Arduino microcontrollers to control physical traffic lights.

## Features

- Real-time vehicle detection using background subtraction and contour analysis.
- Supports multiple traffic lanes (North-South and East-West) via video feeds.
- Dynamic traffic light control with green, yellow, and red phases.
- Early green light switching triggered by vehicle queue comparison.
- Visual overlay of traffic signal states, vehicle counts, and timers on video frames.
- Serial communication with Arduino boards to control physical traffic light signals.
- Simulation with video playback pausing/resuming according to signal states.

## Folder Structure
\\
Traffix/
│
├── Arduino/
│ ├── Arduino Hardware.png # Diagram of the Arduino setup
│ └── Traffix/
│ └── Traffix.ino # Arduino sketch for traffic light control
│
├── videos/
│ ├── traffic1.mp4 # Video for North-South lanes
│ └── traffic2.mp4 # Video for East-West lanes
│
├── main.py # Main Python script for traffic simulation
├── README.md # This file
└── requirements.txt # Python dependencies
\\


## Requirements

- Python 3.x
- OpenCV (`opencv-python`)
- NumPy
- PySerial

Install Python dependencies via:

```bash
pip install -r requirements.txt
Setup and Usage
Connect two Arduino boards to your computer on COM4 and COM5 ports (adjust COM ports as needed in main.py):

One Arduino controls East-West lanes (COM4).

One Arduino controls North-South lanes (COM5).

Load the Arduino sketch Traffix.ino onto your Arduino boards.

Ensure the videos folder contains the traffic lane videos traffic1.mp4 and traffic2.mp4.

Run the simulation:

bash
Copy
Edit
python main.py
A window titled Traffic Management Simulation will open showing the processed video streams with overlays of signal status, vehicle counts, and timers.

Press Esc key to exit the simulation.

How It Works
The script captures frames from four video streams representing two North-South and two East-West lanes.

Vehicle detection uses background subtraction (MOG2) and contour detection to count vehicles.

Traffic signals cycle through green, yellow, and red phases with configurable durations (go_duration and yellow_duration).

When a lane’s signal is green, the video plays normally; when red, the video pauses to simulate stopped traffic.

The system compares vehicle counts on red and green lanes to trigger early green light switching to optimize traffic flow.

Signal commands are sent via serial communication to Arduino boards to control real-world traffic lights accordingly.

Customization
Adjust video sources by replacing files in videos/ or modifying the paths in main.py.

Modify serial port names (COM4, COM5) as per your setup.

Tune detection parameters such as contour area threshold and background subtractor settings.

Change traffic signal timing by editing go_duration and yellow_duration variables in the script.
