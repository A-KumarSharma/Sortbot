import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

TOP, C = -0.01, 0.04                  # table top height, cube size
XR, YR = (0.34, 0.68), (-0.32, 0.30)  # workspace limits (m)
HSV = {'red': [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))],
       'green': [((40, 100, 50), (85, 255, 255))],
       'blue': [((100, 120, 50), (130, 255, 255))]}


def rot(x, y, z, w):
    """Quaternion -> 3x3 rotation matrix."""
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Detector(Node):
    def __init__(self):
        super().__init__('detector')
        self.K, self.tracks, self.bridge, self.n = None, [], CvBridge(), 0
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(
            CameraInfo, 'camera/camera_info',
            lambda m: setattr(self, 'K', np.array(m.k).reshape(3, 3)), 1)
        self.create_subscription(Image, 'camera/image_raw', self.on_image,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(Detection3DArray, 'detections', 10)

    def on_image(self, m):
        if self.K is None:
            return
        try:
            T = self.tf.lookup_transform(
                'panda_link0', m.header.frame_id, rclpy.time.Time()).transform
        except Exception:
            return
        R = rot(T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w)
        t = np.array([T.translation.x, T.translation.y, T.translation.z])
        K = self.K

        def ground(u, v, z):
            d = R @ np.array([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], 1.0])
            return t + d * (z - t[2]) / d[2]

        hsv = cv2.cvtColor(cv2.GaussianBlur(self.bridge.imgmsg_to_cv2(m, 'bgr8'), (5, 5), 0),
                           cv2.COLOR_BGR2HSV)
        now, seen = self.get_clock().now().nanoseconds * 1e-9, []

        for color, ranges in HSV.items():
            mask = sum(cv2.inRange(hsv, np.array(a), np.array(b)) for a, b in ranges)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
                (_, _), (pw, ph), _ = cv2.minAreaRect(c)
                if not 150 < pw * ph < 4000 or max(pw, ph) > 1.8 * min(pw, ph):
                    continue
                # The blob contains the top AND the visible side faces, so measure it in world
                # space on the cube's mid-height plane (not the top plane) to avoid a biased centre.
                pts = np.array([ground(u, v, TOP + C / 2)[:2] for u, v in c.reshape(-1, 2)], np.float32)
                rect = cv2.minAreaRect(pts)
                (cx, cy), (w, h), _ = rect
                if not (0.03 < min(w, h) and max(w, h) < 0.07):
                    continue
                if not (XR[0] < cx < XR[1] and YR[0] < cy < YR[1]):
                    continue
                box = cv2.boxPoints(rect)
                phi = (math.atan2(box[1][1] - box[0][1], box[1][0] - box[0][0])
                       + math.pi / 4) % (math.pi / 2) - math.pi / 4
                seen.append((color, np.array([cx, cy, TOP + C / 2]), np.exp(4j * phi)))

        for color, pos, z4 in seen:
            tr = next((k for k in self.tracks
                       if k['c'] == color and np.linalg.norm(k['p'][:2] - pos[:2]) < 0.04), None)
            if tr is None:
                self.n += 1
                self.tracks.append({'c': color, 'p': pos, 'z': z4, 'n': 1, 't': now, 'id': self.n})
            else:
                tr['p'], tr['z'], tr['n'], tr['t'] = (
                    0.6 * tr['p'] + 0.4 * pos, 0.6 * tr['z'] + 0.4 * z4, tr['n'] + 1, now)

        self.tracks = [k for k in self.tracks if now - k['t'] < 1.0]

        out = Detection3DArray()
        out.header.stamp, out.header.frame_id = m.header.stamp, 'panda_link0'
        for k in self.tracks:
            if k['n'] < 5:
                continue
            d, h, phi = Detection3D(), ObjectHypothesisWithPose(), np.angle(k['z']) / 4
            d.id, d.header = f"{k['c']}_{k['id']}", out.header
            h.hypothesis.class_id = k['c']
            (h.pose.pose.position.x, h.pose.pose.position.y,
             h.pose.pose.position.z) = map(float, k['p'])
            h.pose.pose.orientation.z = float(math.sin(phi / 2))
            h.pose.pose.orientation.w = float(math.cos(phi / 2))
            d.results.append(h)
            out.detections.append(d)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
