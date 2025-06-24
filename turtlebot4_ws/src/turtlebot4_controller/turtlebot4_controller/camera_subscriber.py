import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class TurtleBotYOLOSubscriber(Node):
    def __init__(self):
        super().__init__('turtlebot_yolo_subscriber')
        
        # ROS2 setup
        self.subscription = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',  # Change this to match your camera topic
            self.image_callback,
            10)
        self.bridge = CvBridge()
        
        # YOLO setup
        self.MODEL_NAME = 'yolov10n.pt'
        self.MODEL_DIR = 'models'
        self.yolo_model = self.ensure_model_downloaded()
        
        self.get_logger().info('TurtleBot4 YOLO Subscriber initialized')
        self.get_logger().info(f'Subscribed to topic: /oakd/rgb/preview/image_raw')
        self.get_logger().info('Press Ctrl+C to stop')

    def ensure_model_downloaded(self):
        """
        Ensures the YOLO model is downloaded.
        """
        model_path = os.path.join(self.MODEL_DIR, self.MODEL_NAME)

        # Create the models directory if it doesn't exist
        if not os.path.exists(self.MODEL_DIR):
            os.makedirs(self.MODEL_DIR)
            self.get_logger().info(f"Created directory: {self.MODEL_DIR}")

        if not os.path.exists(model_path):
            self.get_logger().info(f"Model '{self.MODEL_NAME}' not found. Downloading...")
            try:
                model = YOLO(self.MODEL_NAME)
                self.get_logger().info(f"Model '{self.MODEL_NAME}' downloaded successfully")
            except Exception as e:
                self.get_logger().error(f"Error downloading model: {e}")
                exit()
        else:
            self.get_logger().info(f"Model '{self.MODEL_NAME}' already exists. Loading...")

        return YOLO(self.MODEL_NAME)

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run YOLO detection on the frame
            results = self.yolo_model.predict(source=cv_image, conf=0.25, show=False, save=False)
            
            # Process and display results
            annotated_frame = cv_image.copy()
            
            for r in results:
                if r.boxes is not None:
                    # Draw bounding boxes and labels
                    boxes = r.boxes.xyxy.cpu().numpy()  # Get bounding boxes
                    confidences = r.boxes.conf.cpu().numpy()  # Get confidence scores
                    class_ids = r.boxes.cls.cpu().numpy().astype(int)  # Get class IDs
                    
                    for box, conf, class_id in zip(boxes, confidences, class_ids):
                        x1, y1, x2, y2 = map(int, box)
                        class_name = self.yolo_model.names[class_id]
                        
                        # Draw bounding box
                        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        
                        # Draw label with confidence
                        label = f"{class_name}: {conf:.2f}"
                        label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                        cv2.rectangle(annotated_frame, (x1, y1 - label_size[1] - 10), 
                                    (x1 + label_size[0], y1), (0, 255, 0), -1)
                        cv2.putText(annotated_frame, label, (x1, y1 - 5), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    
                    # Log detection info
                    self.get_logger().info(f"Detected {len(boxes)} objects in frame")
            
            # Display the annotated frame
            cv2.imshow("TurtleBot4 Camera - YOLO Detection", annotated_frame)
            
            # Break on 'q' key press
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("Stopping detection...")
                rclpy.shutdown()
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def detect_in_image(image_path, model):
    """
    Performs object detection on a static image.
    """
    if not os.path.exists(image_path):
        print(f"Error: Image not found at '{image_path}'")
        return

    print(f"\n--- Running detection on image: {image_path} ---")
    try:
        results = model.predict(source=image_path, save=True, show=True)
        
        for r in results:
            if r.boxes is not None:
                print(f"Detected {len(r.boxes)} objects.")
                for box in r.boxes:
                    class_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    class_name = model.names[class_id]
                    print(f"  - Class: {class_name}, Confidence: {conf:.2f}")

        print("Detection complete. Results saved in 'runs/detect' directory.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except Exception as e:
        print(f"An error occurred during image detection: {e}")

def main(args=None):
    print("\n--- TurtleBot4 YOLO Detection ---")
    
    # Initialize YOLO model
    MODEL_NAME = 'yolov10n.pt'
    MODEL_DIR = 'models'
    
    # Create models directory if needed
    if not os.path.exists(MODEL_DIR):
        os.makedirs(MODEL_DIR)
    
    yolo_model = YOLO(MODEL_NAME)
    
    while True:
        choice = input(
            "\nChoose an option:\n"
            "1. Detect objects in an image\n"
            "2. Live detection from TurtleBot4 camera\n"
            "3. Exit\n"
            "Enter your choice (1/2/3): "
        )

        if choice == '1':
            image_path = input("Enter the path to the image file: ")
            detect_in_image(image_path, yolo_model)
            
        elif choice == '2':
            print("Starting live detection from TurtleBot4...")
            print("Make sure your TurtleBot4 is running and publishing camera data.")
            print("Press Ctrl+C to stop live detection.")
            
            rclpy.init(args=args)
            try:
                camera_subscriber = TurtleBotYOLOSubscriber()
                camera_subscriber.yolo_model = yolo_model  # Use the same model instance
                rclpy.spin(camera_subscriber)
            except KeyboardInterrupt:
                print("\nStopping live detection...")
            finally:
                if 'camera_subscriber' in locals():
                    camera_subscriber.destroy_node()
                rclpy.shutdown()
                cv2.destroyAllWindows()
                
        elif choice == '3':
            print("Exiting program. Goodbye!")
            break
        else:
            print("Invalid choice. Please enter 1, 2, or 3.")

if __name__ == '__main__':
    main()
