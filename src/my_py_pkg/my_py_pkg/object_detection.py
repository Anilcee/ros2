import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D

from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        self.bridge = CvBridge()
        self.model = YOLO("models/yolov8m.pt")
        self.detection_publisher = self.create_publisher(Detection2DArray, 'detections', 10)
        self.image_publisher = self.create_publisher(Image, 'annotated_image', 10)
        self.subscriber_ = self.create_subscription(Image,  '/image_raw', self.image_callback, 10)

        self.target_classes = ['person', 'car', 'bicycle', 'truck']

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, verbose=False)[0]

        detection_msg = Detection2DArray()
        detection_msg.header = msg.header
        
        # Çizim için frame kopyası
        annotated_frame = frame.copy()

        for box in results.boxes:
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]

            if cls_name not in self.target_classes:
                continue

            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(float, box.xyxy[0])
            w, h = x2 - x1, y2 - y1

            det = Detection2D()
            det.header = msg.header
            det.bbox = BoundingBox2D()
            
            det.bbox.center.position.x = x1 + w / 2
            det.bbox.center.position.y = y1 + h / 2
            det.bbox.center.theta = 0.0
            det.bbox.size_x = w
            det.bbox.size_y = h

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(cls_id)
            hyp.hypothesis.score = conf

            det.results.append(hyp)
            detection_msg.detections.append(det)
            
            cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(annotated_frame, f"{cls_name}: {conf:.2f}", 
                       (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        self.detection_publisher.publish(detection_msg)
        
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        annotated_msg.header = msg.header
        self.image_publisher.publish(annotated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()