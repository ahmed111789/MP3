## Ros 2 Mini projet 3 

### Dependencies

sudo apt install python3-rosdep python3-rosinstall-generator python3-vcstool python3-colcon-common-extensions -y

sudo apt install ros-humble-rclpy ros-humble-std-msgs ros-humble-sensor-msgs -y

sudo apt install ros-humble-cv-bridge -y

sudo apt install ros-humble-usb-cam -y


pip install --upgrade pip

pip install opencv-python==4.8.1.78

pip install ultralytics==8.0.138

pip install torch==2.2.0+cpu torchvision==0.17.1+cpu

pip install numpy==1.26.0


#### Note: 

- If ultralytics displays a problem its probably from the installation of pytorch

- Example launch file launches usb_camera(publishes default camera view) then launches intruder_detection.py with the inputs from the alert.yaml file

- Details about the inputs are written in the yaml file  itself

