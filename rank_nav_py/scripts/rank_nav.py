#!/usr/bin/env python3
from __future__ import annotations

import torch

import os
import sys
import threading
import queue
import time
import traceback
import faulthandler
faulthandler.enable()
from pathlib import Path
from typing import Optional

# Add VLM_Rank_Nav to sys.path
# Assuming it is installed in the same directory as this script under 'VLM_Rank_Nav' folder
current_dir = os.path.dirname(os.path.abspath(__file__))
vlm_path = os.path.join(current_dir, 'VLM_Rank_Nav')
if vlm_path not in sys.path:
    sys.path.append(vlm_path)

import cv2
import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge

# Import from VLM_Rank_Nav modules
try:
    from data_cls import TrackerConfig, FrameDetections, PersonObservation
    from msg_utils import camera_model_from_info, TfLookup, transform_to_matrix
    from personal_space_estimator import PersonalSpaceEstimator
    from personal_space_publisher import RankNavPublisher
except ImportError as e:
    print(f"Error importing VLM_Rank_Nav modules: {e}")
    print(f"sys.path: {sys.path}")
    # Fallback/Exit?
    # sys.exit(1)

from tf2_ros import TransformListener, Buffer, TransformException
try:
    from people_msgs.msg import People
except ImportError:
    People = None

# Default path relative to this script after install
# structure:
# lib/rank_nav_py/
#   rank_nav.py
#   VLM_Rank_Nav/
#     distillation/
#       checkpoints/
DEFAULT_SCORING_REL = Path("VLM_Rank_Nav") / "distillation" / "checkpoints" / "ranking_distillation_model.pth"


class TfLookupWrapper:
    """Wrapper to adapt ROS 2 tf2_ros Buffer to the TfLookup interface."""
    
    def __init__(self, tf_buffer: Buffer, node):
        self.tf_buffer = tf_buffer
        self.node = node
        self._static_transforms = {}
        self._available_frames = set()
        
    def add_static_transform(self, transform_msg: TransformStamped):
        """Add a static transform to our cache."""
        parent = transform_msg.header.frame_id
        child = transform_msg.child_frame_id
        matrix = transform_to_matrix(transform_msg.transform)
        
        self._static_transforms[(parent, child)] = matrix
        self._static_transforms[(child, parent)] = np.linalg.inv(matrix)
        self._available_frames.add(parent)
        self._available_frames.add(child)
    
    def lookup(self, timestamp_ns: int, source_frame: str, target_frame: str) -> Optional[np.ndarray]:
        """
        Look up transform from source_frame to target_frame at the given timestamp.
        
        Returns:
            4x4 transformation matrix or None if lookup fails
        """
        if not source_frame or not target_frame:
            return None
        if source_frame == target_frame:
            return np.eye(4, dtype=np.float64)
        
        # First try static transforms
        if (source_frame, target_frame) in self._static_transforms:
            return self._static_transforms[(source_frame, target_frame)]
        
        # Clean frame names (remove leading slash if present)
        source_clean = source_frame.lstrip('/')
        target_clean = target_frame.lstrip('/')
        
        # Try various frame name formats
        frame_variants = [
            (source_frame, target_frame),
            (source_clean, target_clean),
            (f"/{source_clean}", f"/{target_clean}"),
        ]
        
        for src_var, tgt_var in frame_variants:
            if (src_var, tgt_var) in self._static_transforms:
                return self._static_transforms[(src_var, tgt_var)]
        
        if source_clean != source_frame or target_clean != target_frame:
            # Try with cleaned names recursively
            result = self.lookup(timestamp_ns, source_clean, target_clean)
            if result is not None:
                return result
        
        try:
            from rclpy.time import Time
            
            # Try current time first for odom -> map transforms
            current_time = self.node.get_clock().now()
            timeout = rclpy.duration.Duration(seconds=0.2)
            
            # For map/odom transforms, try current time (they're usually published continuously)
            if 'map' in [source_frame, target_frame] or 'odom' in [source_frame, target_frame]:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        current_time,
                        timeout=timeout
                    )
                    return self._transform_to_matrix(transform.transform)
                except TransformException as e:
                    # Try with Time(0) for latest available
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            target_frame,
                            source_frame,
                            Time(seconds=0),
                            timeout=timeout
                        )
                        return self._transform_to_matrix(transform.transform)
                    except TransformException:
                        pass
            
            # Try with the requested timestamp
            ros_time = Time(nanoseconds=timestamp_ns)
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                ros_time,
                timeout=timeout
            )
            
            return self._transform_to_matrix(transform.transform)
            
        except (TransformException, Exception) as e:
            # Only log if it's not a common expected failure
            if 'map' not in [source_frame, target_frame]:
                self.node.get_logger().debug(f'TF lookup failed {source_frame} -> {target_frame}: {type(e).__name__}')
            return None

    def get_available_frames(self) -> set:
        """Get list of available frames."""
        return self._available_frames.copy()
    
    def _transform_to_matrix(self, transform) -> np.ndarray:
        """Convert ROS transform to 4x4 matrix."""
        trans = transform.translation
        rot = transform.rotation
        
        # Use msg_utils transform_to_matrix function
        from types import SimpleNamespace
        transform_ns = SimpleNamespace(
            translation=SimpleNamespace(x=trans.x, y=trans.y, z=trans.z),
            rotation=SimpleNamespace(x=rot.x, y=rot.y, z=rot.z, w=rot.w)
        )
        return transform_to_matrix(transform_ns)


class CabotPersonalSpaceNode(Node):
    """
    Real-time personal space estimation node.
    """

    def __init__(self):
        super().__init__('cabot_personal_space_node')
        
        self.get_logger().info("Initializing CaBot Personal Space Node...")
        
        # Declare parameters
        default_scoring_path = Path(os.path.dirname(os.path.abspath(__file__))) / DEFAULT_SCORING_REL
        self.declare_parameter('scoring_model', str(default_scoring_path))
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('min_depth', 0.3)
        self.declare_parameter('max_depth', 30.0)
        self.declare_parameter('camera_serial', '')
        self.declare_parameter('people_topic', '/people')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_rank_nav_image', False)

        # Get parameters
        self.scoring_model = self.get_parameter('scoring_model').value
        self.device = self.get_parameter('device').value
        self.get_logger().info(f"Using device: {self.device}")
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.camera_serial = self.get_parameter('camera_serial').value
        people_topic = self.get_parameter('people_topic').value
        # odom_topic is unused in main logic but kept for consistency
        odom_topic = self.get_parameter('odom_topic').value
        self.publish_rank_nav_image = self.get_parameter('publish_rank_nav_image').value

        if not self.camera_serial:
            self.camera_serial = None
            
        # CvBridge for converting ROS Image to OpenCV
        self.bridge = CvBridge()
        
        # Data storage
        self.latest_image = None
        self.latest_odom = None
        self.latest_timestamp_ns = None
        self.camera_info = None
        self.camera_model = None
        self.estimator = None
        
        # Thread-safe queues for async processing
        self.data_queue = queue.Queue(maxsize=3)  # Keep only recent frames
        self.result_queue = queue.Queue(maxsize=5)
        
        # Threading control
        self.running = True
        self.lock = threading.Lock()
        
        self.latest_people = None

        if People is None:
            self.get_logger().error("people_msgs not found. Cannot use cabot-people.")
            # Should probably exit here or raise exception
        else:
            self.people_sub = self.create_subscription(
                People,
                people_topic,
                self.people_callback,
                10
            )
            self.get_logger().info(f"Subscribing to {people_topic}")
        
        # TF lookup using ROS 2 tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_lookup = TfLookupWrapper(self.tf_buffer, self)
        
        # Subscribe to TF topics for static and dynamic transforms
        tf_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            tf_qos
        )
        
        tf_static_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.tf_static_sub = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.tf_static_callback,
            tf_static_qos
        )
        
        # Publishers
        self.personal_space_pub = RankNavPublisher()
        
        if self.publish_rank_nav_image:
            self.rank_nav_image_pub = self.create_publisher(Image, '/rank_nav_image', 10)
        
        # Subscribe to camera info first
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
            )
        )
        
        # Subscribe to topics (using raw Image instead of CompressedImage)
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        
        self.get_logger().info(f"Subscribed to:")
        self.get_logger().info(f"  - Image: {image_topic} (raw)")
        self.get_logger().info(f"  - Camera Info: {camera_info_topic}")
        self.get_logger().info(f"  - /tf (dynamic transforms)")
        self.get_logger().info(f"  - /tf_static (static transforms)")
        
        # Start worker threads
        self.tracking_thread = threading.Thread(target=self.tracking_worker, daemon=True)
        self.scoring_thread = threading.Thread(target=self.scoring_worker, daemon=True)
        
        self.tracking_thread.start()
        self.scoring_thread.start()
        
        self.get_logger().info("Worker threads started")

    def people_callback(self, msg):
        """Callback for People from cabot-people."""
        self.latest_people = msg
        
    def tf_callback(self, msg: TFMessage):
        """Handle dynamic TF messages."""
        # tf_buffer automatically handles these through tf_listener
        pass

    def tf_static_callback(self, msg: TFMessage):
        """Handle static TF messages."""
        if not hasattr(self, '_tf_static_initialized'):
            self._tf_static_initialized = True
            self.get_logger().info(f"Received {len(msg.transforms)} static transforms")
        
        for transform in msg.transforms:
            self.tf_lookup.add_static_transform(transform)

    def camera_info_callback(self, msg: CameraInfo):
        """Initialize camera model from camera info."""
        if self.camera_model is None:
            self.camera_model = camera_model_from_info(msg)
            self.get_logger().info(f"Camera initialized: {self.camera_model.frame_id}")
            
            # Initialize estimator once we have camera model
            if self.estimator is None:
                self.initialize_estimator()

    def initialize_estimator(self):
        """Initialize the PersonalSpaceEstimator."""
        if self.camera_model is None:
            self.get_logger().warn("Cannot initialize estimator without camera model")
            return
        
        # Wait a bit for TF data to accumulate
        time.sleep(1.0)
        
        # Infer lidar frame dynamically
        lidar_frame = self._infer_lidar_frame()
        if not lidar_frame:
            if "lidar_link" in self.tf_lookup._available_frames:
                lidar_frame = "lidar_link"
            else:
                lidar_frame = "velodyne"  # Temporary default
        
        # Check which frames are actually available
        available_frames = self.tf_lookup.get_available_frames()
        
        # Try to find odom frame (may have different names)
        odom_frame = None
        for frame in available_frames:
            if frame in ["odom", "odom_combined", "base_footprint"]:
                odom_frame = frame
                break
        
        # If no odom frame in TF, we'll use odometry message directly
        if not odom_frame:
            odom_frame = "base_link"
        
        # Try to find map frame
        map_frame = None
        for frame in ["map", "map_global", "map_carto1_1"]:
            if frame in available_frames:
                map_frame = frame
                break
        
        if not map_frame:
            self.get_logger().warn("Map frame not found in TF")
        
        # Create config for coordinate transforms in process_tracked_boxes
        self.tracker_config = TrackerConfig(
            confidence=0.0, # unused
            min_depth=self.min_depth,
            max_depth=self.max_depth,
            point_step=1, # unused
            camera_frame=self.camera_model.frame_id,
            lidar_frame=lidar_frame,
            odom_frame=odom_frame,
            robot_frame="base_link",
            map_frame=map_frame,
        )
        
        # Ensure scoring model path is absolute
        scoring_model_path = os.path.abspath(self.scoring_model)
        self.get_logger().info(f"Loading scoring model from: {scoring_model_path}")
        
        ranknav_alpha = float(os.environ.get('RANKNAV_ALPHA', 0.20))
        self.get_logger().info(f"Using RANKNAV_ALPHA: {ranknav_alpha}")

        try:
            self.estimator = PersonalSpaceEstimator(
                scoring_model_path=scoring_model_path,
                device=self.device,
                alpha=ranknav_alpha,
            )
            self.get_logger().info(f"Estimator initialized (lidar:{lidar_frame}, odom:{odom_frame}, map:{map_frame})")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize PersonalSpaceEstimator: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def image_callback(self, msg: Image):
        if self.estimator is None:
            return
        """Store latest image and trigger processing (using raw Image)."""
        timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        
        # Convert ROS Image to OpenCV image
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if image is None:
                return
            # Force copy to ensure we own the memory and it's not a view into the ROS message buffer
            # which might be freed after this callback returns.
            image = image.copy()
            import numpy as np
            if not image.flags['C_CONTIGUOUS']:
                 image = np.ascontiguousarray(image)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        self.estimator.store_data(image, 'image', timestamp_ns)

    def process_people(self):
        msg = self.latest_people
        self.latest_people = None # Consume
        if msg is None: return
        
        # Time alignment
        timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        
        # Finding closest image
        image_timestamps = list(self.estimator.images.keys())
        if not image_timestamps:
            return
            
        closest_ts = min(image_timestamps, key=lambda x: abs(x - timestamp_ns))
        # Use a tolerance for synchronization (e.g. 500ms)
        if abs(closest_ts - timestamp_ns) > 0.5 * 1e9:
             pass
        
        image = self.estimator.images[closest_ts]
        pil_image = PILImage.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        
        people = {}
        
        odom_frame = self.tracker_config.odom_frame
        map_frame = self.tracker_config.map_frame
        people_frame = msg.header.frame_id
        camera_frame = self.camera_model.frame_id
        
        # Transforms
        T_camera_people = self.tf_lookup.lookup(timestamp_ns, people_frame, camera_frame)
        T_odom_people   = self.tf_lookup.lookup(timestamp_ns, people_frame, odom_frame)
        
        if T_camera_people is None:
             self.get_logger().debug(f"Missing TF {people_frame} -> {camera_frame}")
             return

        for person in msg.people:
            # Check for camera_id filtering
            if self.camera_serial:
                camera_id_tag_found = False
                camera_id_match = False
                if person.tagnames and person.tags and len(person.tagnames) == len(person.tags):
                    for tag_name, tag_value in zip(person.tagnames, person.tags):
                        if tag_name == "camera_id":
                            camera_id_tag_found = True
                            if tag_value == self.camera_serial:
                                camera_id_match = True
                            break
                
                # If the person has a 'camera_id' tag, filtering rule applies strictly.
                if camera_id_tag_found and not camera_id_match:
                    continue

            try:
                tid = int(person.name)
            except ValueError:
                tid = abs(hash(person.name)) % 100000

            # Point in People Frame
            p_people = np.array([person.position.x, person.position.y, person.position.z, 1.0])
            
            # Point in Camera Frame
            p_cam = T_camera_people @ p_people
            
            # Check if in front of camera
            if p_cam[2] <= 0.1: # Behind camera or too close
                continue
                
            # Project 3D -> 2D
            fx = self.camera_model.matrix[0, 0]
            fy = self.camera_model.matrix[1, 1]
            cx = self.camera_model.matrix[0, 2]
            cy = self.camera_model.matrix[1, 2]
            
            # u = fx * X/Z + cx
            # v = fy * Y/Z + cy
            u_c = fx * (p_cam[0] / p_cam[2]) + cx
            v_c = fy * (p_cam[1] / p_cam[2]) + cy
            
            # Dimensions
            H_real = 1.75
            W_real = 0.6
            h_px = fy * H_real / p_cam[2]
            w_px = fx * W_real / p_cam[2]
            
            x1 = u_c - w_px/2
            y1 = v_c - h_px/2
            x2 = u_c + w_px/2
            y2 = v_c + h_px/2
            
            w_img = self.camera_model.width
            h_img = self.camera_model.height
            
            # Simple check if "roughly" in view
            if x2 < 0 or x1 > w_img or y2 < 0 or y1 > h_img:
                continue

            centroid_px = (u_c, v_c)
            centroid_norm = (u_c/w_img, v_c/h_img)
            bbox_center_px = centroid_px
            bbox_xyxy = (float(x1), float(y1), float(x2), float(y2))
            
            depth = float(p_cam[2])
            camera_point = [float(p_cam[0]), float(p_cam[1]), float(p_cam[2])]
            
            # Odom point
            odom_point = None
            if T_odom_people is not None:
                 p_odom = T_odom_people @ p_people
                 odom_point = p_odom[:3].tolist()
            
            # Map point
            map_point = None
            T_map_people = self.tf_lookup.lookup(timestamp_ns, people_frame, map_frame)
            if T_map_people is not None:
                p_map = T_map_people @ p_people
                map_point = p_map[:3].tolist()
            elif odom_point is None and map_frame == odom_frame:
                 pass
                 
            person_obs = PersonObservation(
                track_id=int(tid),
                centroid_norm=centroid_norm,
                centroid_px=centroid_px,
                bbox_center_px=bbox_center_px,
                bbox_xyxy=bbox_xyxy,
                depth_m=depth,
                camera_point_m=camera_point,
                odom_point_m=odom_point,
                map_point_m=map_point
            )
            people[int(tid)] = person_obs
            
        detections = FrameDetections(
            timestamp_ns=timestamp_ns, 
            relative_time_s=timestamp_ns/1e9,
            pil_image=pil_image,
            bgr_image=image,
            track_centers={tid: p.centroid_px for tid, p in people.items()},
            people=people,
            lidar_points_odom=None, 
            robot_pose_odom=None, 
            odom_to_map=None,
            robot_pose_map=None
        )
        
        self.estimator.update_detections(detections)

        if self.publish_rank_nav_image:
            annotated_img = image.copy()
            for tid, person in people.items():
                x1, y1, x2, y2 = person.bbox_xyxy
                pt1 = (int(x1), int(y1))
                pt2 = (int(x2), int(y2))
                cv2.rectangle(annotated_img, pt1, pt2, (0, 255, 0), 2)
                cv2.putText(annotated_img, str(tid), (pt1[0], pt1[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            try:
                msg_img = self.bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8")
                from rclpy.time import Time
                msg_img.header.stamp = Time(nanoseconds=timestamp_ns).to_msg()
                msg_img.header.frame_id = self.camera_model.frame_id
                self.rank_nav_image_pub.publish(msg_img)
            except Exception as e:
                self.get_logger().error(f"Failed to publish rank_nav_image: {e}")

    def tracking_worker(self):
        self.get_logger().info("Tracking worker started")
        
        # Wait for TF to be available (only check once at startup)
        if not hasattr(self, '_tf_checked'):
            time.sleep(1.0)  # Wait for TF to accumulate
            self._tf_checked = True
        
        while self.running:
            if self.estimator is None:
                time.sleep(0.1)
                continue
            
            if self.latest_people is None:
                time.sleep(0.01)
                continue
            try:
                self.process_people()
            except Exception as e:
                self.get_logger().error(f"Error processing people: {e}")
                self.get_logger().error(traceback.format_exc())
                time.sleep(0.1)

    def scoring_worker(self):
        """Worker thread for scoring and personal space calculation."""
        
        while self.running:
            if self.estimator is None:
                time.sleep(0.1)
                continue
            try:
                frames_for_scoring = self.estimator.get_frame_histories_for_scoring()
                if len(frames_for_scoring) == 0:
                    time.sleep(0.05)
                    continue
                
                # Additional None checks
                latest_frame = frames_for_scoring[-1]
                if latest_frame is None or latest_frame.people is None or len(latest_frame.people) == 0:
                    time.sleep(0.05)
                    continue
                
                self.get_logger().info("Starting scoring inference...")
                score_map = self.estimator.get_importance_score(frames_for_scoring)
                self.get_logger().info("Scoring inference finished")
                if not score_map:
                    time.sleep(0.05)
                    continue
                
                results = []
                for tid, person in latest_frame.people.items():
                    if person is None or tid not in score_map:
                        continue
                    
                    # Skip if we don't have map coordinates
                    if person.map_point_m is None:
                        continue
                    
                    person.ranking_score = score_map[tid]
                    velocity, heading = self.estimator.get_velocity_and_radius(tid)
                    sigma_h = self.estimator.get_personal_space_params(person.ranking_score, velocity)
                    
                    if heading is None:
                        sigma_r = sigma_h / 2.0
                        sigma_h = sigma_r
                        sigma_s = sigma_r
                    else:
                        sigma_s = sigma_h * 2.0 / 3.0
                        sigma_r = sigma_h / 2.0

                    # directly publish topic
                    self.personal_space_pub.publish_msg(
                        person_id=int(tid),
                        x=float(person.map_point_m[0]),
                        y=float(person.map_point_m[1]),
                        yaw=float(heading) if heading is not None else -10.0,
                        sigma_h=float(sigma_h),
                        sigma_s=float(sigma_s),
                        sigma_r=float(sigma_r),
                        score=float(person.ranking_score),
                    )
                
                if results:
                    try:
                        self.result_queue.put_nowait(results)
                    except queue.Full:
                        pass  # Silently drop if queue is full
                        
            except Exception as e:
                self.get_logger().error(f"Scoring error: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())

    def _infer_lidar_frame(self) -> Optional[str]:
        """Try to infer the LiDAR frame name from available transforms."""
        preferred_frames = [
            'lidar_link', 'velodyne', 'velodyne_link', 'velodyne_base_link',
            'laser', 'lidar', 'base_laser', 'laser_link', 'velodyne_base'
        ]
        
        # Check available frames from tf_lookup (prefer non-local frames)
        for frame in preferred_frames:
            if frame in self.tf_lookup._available_frames:
                return frame
        
        # Look for frames with lidar/velodyne/laser in the name (skip local/ prefix)
        non_local_frames = [f for f in self.tf_lookup._available_frames if not f.startswith("local/")]
        for frame in non_local_frames:
            frame_lower = frame.lower()
            if any(keyword in frame_lower for keyword in ["lidar", "velodyne", "laser"]):
                return frame
        
        return None

    def shutdown(self):
        """Clean shutdown."""
        self.get_logger().info("Shutting down...")
        self.running = False
        
        # Wait for threads to finish
        if self.tracking_thread.is_alive():
            self.tracking_thread.join(timeout=2.0)
        if self.scoring_thread.is_alive():
            self.scoring_thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    
    node = CabotPersonalSpaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
