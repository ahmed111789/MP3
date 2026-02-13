## Ros 2 Mini projet 3 

### Dependencies

pip3 install ultralytics opencv-python
sudo apt install ros-humble-usb-cam
sudo apt install ros-humble-cv-bridge

#### Note: 

- If ultralytics displays a problem its probably from the installation of pytorch

- Example launch file launches usb_camera(publishes default camera view) then launches intruder_detection.py with the inputs from the alert.yaml file

- Details about the inputs are written in the yaml file  itself

