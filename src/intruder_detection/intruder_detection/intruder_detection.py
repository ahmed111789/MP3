import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os
from datetime import datetime
from ultralytics import YOLO
# broadcasting stuff dont touch :)
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
alert_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class HumanDetector(Node):
    def __init__(self):
        super().__init__('human_detector')

        # Parameters
        self.declare_parameter('debug_mode', True)
        self.declare_parameter('alert_frequency', 5.0)
        self.declare_parameter('saved_face_topic', 'None')
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('save_face_frequency', 0.2) # About 10 seconds, set to zero to disallow saving faces
        self.declare_parameter('save_face_path', '/tmp/detected_faces')

        self.debug_mode = self.get_parameter(
            'debug_mode').get_parameter_value().bool_value

        self.alert_frequency = self.get_parameter(
            'alert_frequency').get_parameter_value().double_value
        
        self.save_face_frequency = self.get_parameter(
            'save_face_frequency').get_parameter_value().double_value

        self.saved_face_topic = self.get_parameter(
            'saved_face_topic').get_parameter_value().string_value

        self.camera_topic = self.get_parameter(
            'camera_topic').get_parameter_value().string_value
        
        self.face_dir = self.get_parameter(
            'save_face_path').get_parameter_value().string_value

        # ROS
        self.bridge = CvBridge()
        self.alert_pub = self.create_publisher(String, '/alert_detected', alert_qos)
        if self.saved_face_topic != 'None' and self.alert_frequency > 0.0:
            self.alert_face = self.create_publisher(Image,self.saved_face_topic,alert_qos)
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,   # change if needed
            self.image_callback,
            10
        )
        # YOLOv8 (person detector)
        self.model = YOLO("yolov8n.pt")
        self.conf_threshold = 0.6

        # Face detector (OpenCV Haar)
        self.face_cascade = cv2.CascadeClassifier(
            "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
        )

        # Alert control
        self.alert_cooldown = 1.0  # seconds
        self.last_alert_time = 0.0

        # Save faces here
        
        os.makedirs(self.face_dir, exist_ok=True)
        self.last_save_face_time = 0.0

        self.get_logger().info("🚨 YOLO Human + Face Detector started")

    def image_callback(self, msg):
        
        # ROS img -> opencv
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') 
        
        # Resize for performance
        frame = cv2.resize(frame, (320, 240))

        # YOLO 
        results = self.model(frame, conf=self.conf_threshold, verbose=False)

        person_detected = False
        now = time.time()

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])

                # YOLO class 0 = person
                if cls == 0:
                    person_detected = True

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])

                    # Draw person box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(
                        frame,
                        f"Person {conf:.2f}",
                        (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 255),
                        2
                    )

                    # Crop person region
                    person_roi = frame
                    if person_roi.size == 0:
                        continue

                    gray = cv2.cvtColor(person_roi, cv2.COLOR_BGR2GRAY)

                    # Face detection inside person
                    faces = self.face_cascade.detectMultiScale(
                        gray,
                        scaleFactor=1.1,
                        minNeighbors=5,
                        minSize=(40, 40)
                    )
                    
                    # Save face only once per alert
                    if self.save_face_frequency != 0.0 and (now - self.last_save_face_time) > (1/self.save_face_frequency):
                        for number,(fx, fy, fw, fh) in enumerate(faces):
                            face_img = frame[fy:fy+fh, fx:fx+fw]
                            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                            filename = f"{self.face_dir}/face_{timestamp}_n{number}.jpg"
                            
                            if self.saved_face_topic != 'None':
                                self.alert_face.publish(self.bridge.cv2_to_imgmsg(face_img))
                            
                            self.last_save_face_time = now
                            cv2.imwrite(filename, face_img)
                            self.get_logger().warn(f"📸 Face saved: {filename}")

                    for (fx, fy, fw, fh) in faces:
                        
                        # Draw face box
                        cv2.rectangle(
                            person_roi,
                            (fx, fy),
                            (fx+fw, fy+fh),
                            (255, 0, 0),
                            2
                        )
                        

                    

        # Publish alert (with cooldown)
        if person_detected and (now - self.last_alert_time) > (1/self.alert_frequency):
            msg = String()
            msg.data = "human_detected"
            self.alert_pub.publish(msg)

            self.last_alert_time = now
            self.get_logger().warn("🚨 ALERT: Human detected")

        if self.debug_mode:
            # Debug view
            cv2.imshow("Security Robot - YOLO Detector", frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = HumanDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
