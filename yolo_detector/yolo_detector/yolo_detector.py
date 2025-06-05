import time

import cv2
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool

model_path = "/home/team 46/ros2_ws/model"


class YoloRunner(Node):
    def __init__(self):
        super().__init__("yolo_detect")

        self.mat = None

        self.sub_arrived = self.create_subscription(Bool, "/arrived_at_Object", 100)
        self.sub = self.create_subscription(
            Image, "image_raw", self.image_callback, 100
        )
        # Publishes number of trash detected
        self.pub_num = self.create_publisher(int, "trash_num", 10)
        # Publishes each type of trash
        self.pub_type = self.create_publisher(String, "trash_type", 10)

        self.model = YOLO(model_path, task="detect")
        self.labels = self.model.names
        self.avg_frame_rate = 0
        self.frame_rate_buffer = []
        self.fps_avg_len = 200

    def image_callback(self, msg):
        bbox_colors = [
            (164, 120, 87),
            (68, 148, 228),
            (93, 97, 209),
            (178, 182, 133),
            (88, 159, 106),
            (96, 202, 231),
            (159, 124, 168),
            (169, 162, 241),
            (98, 118, 150),
            (172, 176, 184),
        ]

        sz = (msg.height, msg.width)

        if msg.step * msg.height != len(msg.data):
            print("bad step/height/data size")
            return

        if msg.encoding == "rgb8":
            dirty = (
                self.mat is None
                or msg.width != self.mat.shape[1]
                or msg.height != self.mat.shape[0]
                or len(self.mat.shape) < 2
                or self.mat.shape[2] != 3
            )
            if dirty:
                self.mat = np.zeros([msg.height, msg.width, 3], dtype=np.uint8)
            self.mat[:, :, 2] = np.array(msg.data[0::3]).reshape(sz)
            self.mat[:, :, 1] = np.array(msg.data[1::3]).reshape(sz)
            self.mat[:, :, 0] = np.array(msg.data[2::3]).reshape(sz)
        elif msg.encoding == "mono8":
            self.mat = np.array(msg.data).reshape(sz)
        else:
            print("unsupported encoding {}".format(msg.encoding))
            return

        if self.mat is None:
            return

        t_start = time.perf_counter()

        frame = self.mat

        results = self.model(frame, verbose=False)

        # Extract results
        detections = results[0].boxes

        # Initialize variable for basic object counting example
        object_count = 0

        # Go through each detection and get bbox coords, confidence, and class
        for i in range(len(detections)):
            # Get bounding box coordinates
            # Ultralytics returns results in Tensor format, which have to be converted to a regular Python array
            xyxy_tensor = detections[
                i
            ].xyxy.cpu()  # Detections in Tensor format in CPU memory
            xyxy = xyxy_tensor.numpy().squeeze()  # Convert tensors to Numpy array
            xmin, ymin, xmax, ymax = xyxy.astype(
                int
            )  # Extract individual coordinates and convert to int

            # Get bounding box class ID and name
            classidx = int(detections[i].cls.item())
            classname = self.labels[classidx]
            self.pub_type.publish(classname)
            # Get bounding box confidence
            conf = detections[i].conf.item()

            # Draw box if confidence threshold is high enough
            if conf > 0.5:
                color = bbox_colors[classidx % 10]
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)

                label = f"{classname}: {int(conf * 100)}%"
                labelSize, baseLine = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
                )  # Get font size
                label_ymin = max(
                    ymin, labelSize[1] + 10
                )  # Make sure not to draw label too close to top of window
                cv2.rectangle(
                    frame,
                    (xmin, label_ymin - labelSize[1] - 10),
                    (xmin + labelSize[0], label_ymin + baseLine - 10),
                    color,
                    cv2.FILLED,
                )  # Draw white box to put label text in
                cv2.putText(
                    frame,
                    label,
                    (xmin, label_ymin - 7),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 0),
                    1,
                )  # Draw label text

                # Basic example: count the number of objects in the image
                object_count = object_count + 1

        cv2.putText(
            frame,
            f"FPS: {self.avg_frame_rate:0.2f}",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )  # Draw framerate
        self.pub_num.publish(object_count)
        # Display detection results
        cv2.putText(
            frame,
            f"Number of objects: {object_count}",
            (10, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )  # Draw total number of detected objects
        cv2.imshow("YOLO detection results", frame)  # Display image

        key = cv2.waitKey(5)

        if key == ord("q") or key == ord("Q"):  # Press 'q' to quit
            return
        elif key == ord("s") or key == ord("S"):  # Press 's' to pause inference
            cv2.waitKey()
        elif key == ord("p") or key == ord(
            "P"
        ):  # Press 'p' to save a picture of results on this frame
            cv2.imwrite("capture.png", frame)

        t_stop = time.perf_counter()
        frame_rate_calc = float(1 / (t_stop - t_start))

        if len(self.frame_rate_buffer) >= self.fps_avg_len:
            self.frame_rate_buffer.append(frame_rate_calc)
        else:
            self.frame_rate_buffer.append(frame_rate_calc)

        # Calculate average FPS for past frames
        self.avg_frame_rate = np.mean(self.frame_rate_buffer)


def main(args=None):
    rclpy.init(args=args)

    yolo_runner = YoloRunner()

    try:
        rclpy.spin(yolo_runner)
    except KeyboardInterrupt:
        pass

    yolo_runner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
