"""Image inputs. No ROS publishers, robot mode changes, or HAL commands."""
from collections import deque
import threading
import time

import cv2
import numpy as np

from .geometry import CHAIN, interpolate_joints, marker_points, project, table_corners, transform


def aruco_dictionary(name):
    if not hasattr(cv2, 'aruco') or not hasattr(cv2.aruco, name) or not name.startswith('DICT_'):
        raise ValueError('ArUco dictionary unavailable; install opencv-contrib-python-headless')
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def marker_image(cfg, pixels=600):
    dictionary = aruco_dictionary(cfg['marker']['dictionary'])
    marker_id = cfg['marker']['id']
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, pixels)
    return cv2.aruco.drawMarker(dictionary, marker_id, pixels)


def detect_marker(frame, cfg):
    dictionary = aruco_dictionary(cfg['marker']['dictionary'])
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(gray)
    else:
        params = cv2.aruco.DetectorParameters_create()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None:
        raise ValueError('Configured tabletop marker not visible')
    matches = [c.reshape(4, 2) for c, i in zip(corners, ids.flatten()) if i == cfg['marker']['id']]
    if len(matches) != 1:
        raise ValueError('Configured marker absent or duplicated')
    return marker_points(cfg), matches[0]


class Frame:
    def __init__(self, image, stamp=None, clock='unavailable', positions=None, joint_status='unavailable'):
        self.image = image
        self.capture_ns = stamp
        self.clock = clock
        self.received_ns = time.monotonic_ns()
        self.positions = positions
        self.joint_status = joint_status


class OpenCVSource:
    def __init__(self, source, device=None, image=None, cfg=None):
        self.still = source == 'image'
        self.last = 0
        self.done = False
        if self.still:
            self.image = cv2.imread(image)
            if self.image is None:
                raise ValueError('Cannot read image: ' + str(image))
            self.cap = None
        else:
            if device is None:
                raise ValueError('--device /dev/videoN is required; select ONE physical lens')
            dev = int(device) if device.isdecimal() else device
            self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                raise ValueError('Cannot open selected V4L2 device: ' + device)
            w, h = cfg['image_size']
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def read(self):
        if self.still:
            if self.done:
                time.sleep(.05)
                return None
            self.done = True
            return Frame(self.image, clock='offline_image_no_capture_timestamp')
        ok, image = self.cap.read()
        if not ok:
            raise ValueError('Selected camera stopped producing frames')
        # OpenCV does not expose a reliable V4L2 exposure timestamp here.
        return Frame(image, clock='v4l2_receipt_only_no_exposure_timestamp')

    def close(self):
        if self.cap:
            self.cap.release()


class DemoSource:
    def __init__(self, model, cfg):
        self.cfg = cfg
        self.q = {n: 0. for n in CHAIN}
        self.q['head_pitch_joint'] = .2
        self.truth = transform((-2.2, .12, .45))
        self.t_camera_table = np.linalg.inv(self.truth @ model.optical_pose(self.q, cfg))

    def read(self):
        time.sleep(.1)
        cfg = self.cfg
        image = np.full((960, 1280, 3), (65, 65, 65), np.uint8)
        pixels = project(table_corners(cfg), self.t_camera_table, cfg).astype(np.int32)
        cv2.fillConvexPoly(image, pixels, (140, 75, 20))
        cv2.polylines(image, [pixels], True, (240, 240, 240), 4)
        # A quiet white margin around the black square, kept coplanar.
        marker = marker_image(cfg, 400)
        src = np.array([[0, 0], [399, 0], [399, 399], [0, 399]], np.float32)
        dst = project(marker_points(cfg), self.t_camera_table, cfg).astype(np.float32)
        center = dst.mean(axis=0)
        cv2.fillConvexPoly(image, (center+(dst-center)*1.3).astype(np.int32), (255, 255, 255))
        homography = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(marker, homography, (1280, 960), borderValue=255)
        mask = np.zeros((960, 1280), np.uint8)
        cv2.fillConvexPoly(mask, dst.astype(np.int32), 255)
        image[mask > 0] = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)[mask > 0]
        cv2.putText(image, 'SYNTHETIC - NOT ROBOT CAMERA', (25, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 220, 255), 2)
        return Frame(image, time.monotonic_ns(), 'demo_monotonic', self.q, 'synthetic')

    def close(self):
        pass


def decode_raw(msg):
    """Decode common raw Image encodings, respecting row stride. No cv_bridge ABI dependency."""
    channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}
    c = channels.get(msg.encoding)
    if c is None:
        raise ValueError('Unsupported Image encoding: ' + msg.encoding)
    if msg.step < msg.width*c or len(msg.data) < msg.height*msg.step:
        raise ValueError('Truncated raw ROS Image')
    data = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width*c]
    image = data.reshape(msg.height, msg.width, c).copy()
    if c == 1:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if c == 4:
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR if msg.encoding == 'rgba8' else cv2.COLOR_BGRA2BGR)
    return image


def normalize_joints(names, values):
    positions, seen = {}, set()
    for index, name in enumerate(names):
        name = 'head_pitch_joint' if name == 'ultra_head_pitch_joint' else name
        if name not in CHAIN:
            continue
        if name in seen or index >= len(values):
            positions[name] = float('nan')
        else:
            positions[name] = float(values[index])
        seen.add(name)
    return positions


class ROSSource:
    def __init__(self, args):
        import rclpy
        from sensor_msgs.msg import Image, CompressedImage, JointState
        from rclpy.qos import qos_profile_sensor_data
        if not args.image_topic:
            raise ValueError('--image-topic must select one physical face camera topic')
        if args.joint_topic and not args.ros_clock_confirmed:
            raise ValueError('--ros-clock-confirmed required: image/joint headers must share a progressing clock')
        self.rclpy = rclpy
        rclpy.init(args=[])
        self.node = rclpy.create_node('a3u_face_table_readonly')
        self.lock = threading.Lock()
        self.frames, self.joints = deque(maxlen=10), deque(maxlen=1000)
        self.error = None
        self.last_image_stamp = None
        self.last_joint_stamp = None
        self.use_joints = bool(args.joint_topic)
        def image_callback(msg):
            try:
                if args.compressed:
                    image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                    if image is None:
                        raise ValueError('Cannot decode CompressedImage')
                else:
                    image = decode_raw(msg)
                stamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
                with self.lock:
                    if stamp <= 0 or (self.last_image_stamp is not None and stamp <= self.last_image_stamp):
                        self.frames.clear()
                        self.error = 'Invalid, repeated or backwards image header timestamp'
                    else:
                        self.frames.append(Frame(image, stamp, 'ros_header_common_clock_required'))
                        self.error = None
                    self.last_image_stamp = stamp
            except Exception as exc:
                with self.lock:
                    self.error = str(exc)
        self.node.create_subscription(CompressedImage if args.compressed else Image,
            args.image_topic, image_callback, qos_profile_sensor_data)
        if args.joint_topic:
            def joint_callback(msg):
                stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
                positions = normalize_joints(msg.name, msg.position)
                with self.lock:
                    if stamp <= 0 or (self.last_joint_stamp is not None and stamp <= self.last_joint_stamp):
                        self.joints.clear()
                    else:
                        self.joints.append((stamp, positions))
                    self.last_joint_stamp = stamp
            self.node.create_subscription(JointState, args.joint_topic, joint_callback, qos_profile_sensor_data)
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()

    def read(self):
        with self.lock:
            if self.error:
                raise ValueError(self.error)
            # Wait up to ~120 ms for the upper joint bracket; image is never relabelled as new.
            if self.frames and (not self.use_joints or time.monotonic_ns()-self.frames[0].received_ns > 120_000_000):
                frame = self.frames.popleft()
                if self.use_joints:
                    try:
                        frame.positions = interpolate_joints(list(self.joints), frame.capture_ns)
                        frame.joint_status = 'header_interpolated_not_hardware_sync_verified'
                    except ValueError as exc:
                        frame.joint_status = str(exc)
                return frame
        time.sleep(.01)
        return None

    def close(self):
        self.rclpy.shutdown()
        self.thread.join(timeout=2)
        self.node.destroy_node()
