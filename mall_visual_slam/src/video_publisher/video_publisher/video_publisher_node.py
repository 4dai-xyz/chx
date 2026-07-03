import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher_node')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        # 兼容性: 保留 /camera/image_raw_full topic (内容与 /camera/image_raw 相同,
        # 早期版本为 logo_detector 提供高分辨率分支; 现在 SLAM 也吃 1600x1200, 两者相同)
        self.publisher_full = self.create_publisher(
            Image, '/camera/image_raw_full', 10)
        self.publisher_mask = self.create_publisher(
            Image, '/yolo/dynamic_mask', 10)

        self.declare_parameter('publish_every_n_frames', 1)
        self.declare_parameter('filter_enabled', False)
        self.declare_parameter('start_offset_seconds', 0.0)
        self.declare_parameter('mask_video_path', '')
        # 输出分辨率: 默认 0 = 用源视频分辨率 (1600x1200)
        # KannalaBrandt8.yaml 已按 1600x1200 标定
        # 如果需要降采样省 CPU, 设 640x480 (但要相应修改相机内参 yaml)
        self.declare_parameter('output_width', 0)
        self.declare_parameter('output_height', 0)
        self.declare_parameter('publish_overlay_mask', False)
        self.declare_parameter('mask_include_yellow', True)
        self.declare_parameter('mask_include_green', False)
        self.declare_parameter('mask_dilation_kernel', 7)
        self.declare_parameter('debug_mask_save_every_n', 100)
        self.publish_every_n = self.get_parameter('publish_every_n_frames').value
        self.filter_enabled = self.get_parameter('filter_enabled').value
        start_offset = self.get_parameter('start_offset_seconds').value
        self.mask_video_path = self.get_parameter('mask_video_path').value
        self.output_width = int(self.get_parameter('output_width').value)
        self.output_height = int(self.get_parameter('output_height').value)
        self.publish_overlay_mask = bool(
            self.get_parameter('publish_overlay_mask').value)
        self.mask_include_yellow = bool(
            self.get_parameter('mask_include_yellow').value)
        self.mask_include_green = bool(
            self.get_parameter('mask_include_green').value)
        self.mask_dilation_kernel = int(
            self.get_parameter('mask_dilation_kernel').value)
        self.debug_mask_save_every_n = int(
            self.get_parameter('debug_mask_save_every_n').value)
        self.frame_count = 0

        self.declare_parameter('video_path', '')
        video_path = self.get_parameter('video_path').value
        if not video_path:
            home_dir = os.path.expanduser('~')
            video_path = os.path.join(home_dir, 'ros2_orbslam3', 'resources', 'input_video.mp4_bev.mp4')

        if not os.path.exists(video_path):
            self.get_logger().error(f"找不到视频文件: {video_path}")
            return

        self.cap = cv2.VideoCapture(video_path)
        self.mask_cap = None
        if self.publish_overlay_mask and self.mask_video_path:
            if os.path.exists(self.mask_video_path):
                self.mask_cap = cv2.VideoCapture(self.mask_video_path)
            else:
                self.get_logger().warn(f"找不到 mask 视频文件: {self.mask_video_path}")
        self.bridge = CvBridge()

        # 读源视频实际分辨率, 决定输出分辨率
        self.src_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.src_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if self.output_width <= 0 or self.output_height <= 0:
            # 0 表示保持原分辨率
            self.output_width = self.src_w
            self.output_height = self.src_h
            self.do_resize = False
        else:
            self.do_resize = (self.output_width != self.src_w
                              or self.output_height != self.src_h)

        # 跳转到指定时间偏移
        self.video_fps = self.cap.get(cv2.CAP_PROP_FPS)
        if self.video_fps <= 0:
            self.video_fps = 30.0
        self.start_frame_offset = 0
        if start_offset > 0:
            self.start_frame_offset = int(start_offset * self.video_fps)
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, self.start_frame_offset)
            if self.mask_cap is not None:
                self.mask_cap.set(cv2.CAP_PROP_POS_FRAMES, self.start_frame_offset)
            self.get_logger().info(f'跳转到 {start_offset:.1f}s (frame {self.start_frame_offset})')

        self.timer = self.create_timer(1.0 / self.video_fps, self.timer_callback)

        # 帧筛选器状态
        self.last_published_frame = None   # 上一帧已发布的灰度图
        self.last_published_kp = None      # 上一帧的 ORB 关键点
        self.last_published_des = None     # 上一帧的 ORB 描述子
        self.skipped_count = 0             # 连续跳过的帧数
        self.max_skip = 30                 # 最多连续跳过30帧后强制发布
        self.orb = cv2.ORB_create(nfeatures=1000)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        self.get_logger().info(
            f"视频发布节点已启动 (源={self.src_w}x{self.src_h} "
            f"-> 输出={self.output_width}x{self.output_height}"
            f"{', 重采样' if self.do_resize else ', 不缩放'})")
        self.get_logger().info(
            f"彩色遮挡转 mask: {'开启' if self.publish_overlay_mask else '关闭'}")
        if self.mask_cap is not None:
            self.get_logger().info(
                f"mask 来源: {self.mask_video_path}"
                f" (yellow={'on' if self.mask_include_yellow else 'off'}, "
                f"green={'on' if self.mask_include_green else 'off'})")

    def _preprocess(self, frame_bgr):
        """转灰度 (供 ORB 帧筛选用); 缩放到输出分辨率"""
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        if self.do_resize:
            gray = cv2.resize(gray, (self.output_width, self.output_height))
        return gray

    def _resize_color(self, frame_bgr):
        if self.do_resize:
            return cv2.resize(frame_bgr, (self.output_width, self.output_height))
        return frame_bgr

    def _extract_overlay_mask(self, frame_bgr):
        """从黄色/绿色覆盖区域生成二值 mask，255 表示需要屏蔽。"""
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        # OpenCV HSV: H 范围是 0-179。这里只抓人工涂上的高饱和黄/绿区域。
        masks = []
        if self.mask_include_yellow:
            masks.append(cv2.inRange(
                hsv,
                np.array([15, 70, 70], dtype=np.uint8),
                np.array([42, 255, 255], dtype=np.uint8),
            ))
        if self.mask_include_green:
            masks.append(cv2.inRange(
                hsv,
                np.array([40, 60, 60], dtype=np.uint8),
                np.array([95, 255, 255], dtype=np.uint8),
            ))

        if not masks:
            return np.zeros(hsv.shape[:2], dtype=np.uint8)

        mask = masks[0]
        for extra in masks[1:]:
            mask = cv2.bitwise_or(mask, extra)

        if self.mask_dilation_kernel > 1:
            k = self.mask_dilation_kernel
            if k % 2 == 0:
                k += 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.dilate(mask, kernel)

        return mask

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info('视频播放完毕，节点退出')
            rclpy.shutdown()
            return

        mask_frame = None
        if self.mask_cap is not None:
            ret_mask, mask_frame = self.mask_cap.read()
            if not ret_mask:
                self.get_logger().warn('mask 视频播放完毕或读取失败，后续将不再发布动态 mask')
                self.mask_cap.release()
                self.mask_cap = None

        if not ret:
            return

        self.frame_count += 1

        # Step 1: 预处理当前帧
        current_gray = self._preprocess(frame)

        # Step 2: 决定是否发布
        should_publish = False
        reason = ""

        if not self.filter_enabled:
            # 简单跳帧：每N帧发布一次（保证SLAM有足够基线）
            if self.frame_count % self.publish_every_n != 0:
                return
            should_publish = True
            reason = f"unfiltered (every {self.publish_every_n} frames)"
        elif self.last_published_frame is None:
            # 第一帧总是发布
            should_publish = True
            reason = "first frame"
        else:
            self.skipped_count += 1

            # 提取当前帧的ORB特征
            kp, des = self.orb.detectAndCompute(current_gray, None)

            if des is not None and self.last_published_des is not None and len(des) > 10:
                matches = self.bf.match(self.last_published_des, des)
                good_count = len([m for m in matches if m.distance < 50])

                if good_count >= 150:
                    # 帧太相似，跳过（相机未移动或场景无变化）
                    pass
                else:
                    # 帧有足够差异，发布
                    should_publish = True
                    reason = f"scene changed ({good_count} matches)"
                    self.skipped_count = 0

                if self.skipped_count >= self.max_skip:
                    should_publish = True
                    reason = f"forced (skipped {self.skipped_count})"
                    self.skipped_count = 0
            elif self.skipped_count >= self.max_skip:
                should_publish = True
                reason = f"forced no features (skipped {self.skipped_count})"

        # Step 3: 发布
        if should_publish:
            self.skipped_count = 0
            self.last_published_frame = current_gray.copy()
            kp, des = self.orb.detectAndCompute(current_gray, None)
            self.last_published_kp = kp
            self.last_published_des = des

            # 发布彩色版本给下游
            color_out = self._resize_color(frame)
            # 时间戳: 帧计数 / fps (整段视频时间)
            video_ts = (self.start_frame_offset + self.frame_count - 1) \
                / self.video_fps
            sec = int(video_ts)
            nanosec = int((video_ts - sec) * 1e9)

            # 编码一次, 同时发到两个 topic (内容相同)
            msg = self.bridge.cv2_to_imgmsg(color_out, "bgr8")
            msg.header.stamp.sec = sec
            msg.header.stamp.nanosec = nanosec
            msg.header.frame_id = "camera"
            self.publisher_.publish(msg)

            if self.publish_overlay_mask:
                mask_source = self._resize_color(mask_frame) if mask_frame is not None else color_out
                mask_out = self._extract_overlay_mask(mask_source)
                mask_msg = self.bridge.cv2_to_imgmsg(mask_out, "mono8")
                mask_msg.header = msg.header
                self.publisher_mask.publish(mask_msg)
                if (self.debug_mask_save_every_n > 0
                        and self.frame_count % self.debug_mask_save_every_n == 0):
                    os.makedirs('/home/ros/ros2_orbslam3/output', exist_ok=True)
                    cv2.imwrite('/home/ros/ros2_orbslam3/output/debug_overlay_mask.jpg',
                                mask_out)
            # 兼容旧版的 _full topic
            if self.do_resize:
                # 如果用户硬把 SLAM 输出降到 640x480, 仍提供原分辨率给 logo_detector
                msg_full = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                msg_full.header.stamp.sec = sec
                msg_full.header.stamp.nanosec = nanosec
                msg_full.header.frame_id = "camera"
                self.publisher_full.publish(msg_full)
            else:
                # 与主 topic 相同, 直接复用
                self.publisher_full.publish(msg)

            if self.frame_count % 30 == 0:
                self.get_logger().info(f"Frame {self.frame_count}: published ({reason})")


def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
