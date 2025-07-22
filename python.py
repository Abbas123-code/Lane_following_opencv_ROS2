import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class LineFollower(Node):
    
    def __init__(self):
        super().__init__('line_follower')

        self.image_callback_group = MutuallyExclusiveCallbackGroup()
        self.scan_callback_group = MutuallyExclusiveCallbackGroup()

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            qos_profile,
            callback_group=self.image_callback_group
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile,
            callback_group=self.scan_callback_group
        )

        self.br = CvBridge()
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.dist = None
        self.state = "follow_line"

        # Velocity gains
        self.linear_velocity_gain = 0.1
        self.slow_linear_velocity_gain = 0.05
        self.angular_velocity_gain = 0.04
        self.fast_angular_velocity_gain = 0.04
        
        

    def listener_callback(self, data):
          

        self.get_logger().info('Receiving video frame')
        
        # Convert ROS image to OpenCV image
        img = self.br.imgmsg_to_cv2(data, 'bgr8')

        frame = cv2.resize(img, (320, 240))
        h, w = frame.shape[:2]
        roi = frame[int(h*0.4):, :]
        cv2.imshow("HSV Image", roi)
        # Convert BGR to HSVs
        imgHSV = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        

        # Yellow line mask
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        mask_yellow = cv2.inRange(imgHSV, lower_yellow, upper_yellow)

        # White line mask
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])
        mask_white = cv2.inRange(imgHSV, lower_white, upper_white)
        
        

        roi_yellow = mask_yellow  # Use full image
        roi_white = mask_white
        combined_mask = cv2.bitwise_or(mask_white, mask_yellow)
        cv2.imshow("combined", combined_mask)
        cv2.waitKey(1)
        # Find centroids
        # Yellow mask centroid (bottom row)
        ys_y, xs_y = np.where(roi_yellow > 0)
        if len(ys_y) > 0:
            max_y_yellow = np.max(ys_y)
            bottom_x_yellow = xs_y[ys_y == max_y_yellow]
            cx_y = int(np.mean(bottom_x_yellow))
            self.get_logger().info(f'cx_y={cx_y}')
            
        else:
            cx_y = None

        # White mask centroid (bottom row)
        ys_w, xs_w = np.where(roi_white > 0)
        if len(ys_w) > 0:
            max_y_white = np.max(ys_w)
            bottom_x_white = xs_w[ys_w == max_y_white]
            cx_w = int(np.mean(bottom_x_white))
            self.get_logger().info(f'cx_y={cx_w}')
        else:
            cx_w = None

        
        # self.state="follow_wall"
        if cx_y is not None and cx_w is not None:

              self.state="follow_line"
        elif cx_y is not None and cx_w is None:

              self.state="turn" #if any one line is not detected
              
        elif cx_y is None and cx_w is not None:

              self.state="turn"

        else:
            self.get_logger().info('didnt find either lines')

      
        if self.state=="follow_line":
            twist = Twist()
            if cx_y is None or cx_w is None:
                self.state="turn"
            elif cx_y is not None and cx_w is not None:

                ys_y, xs_y = np.where(roi_yellow > 0)
                if len(ys_y) > 0:
                    max_y_yellow = np.max(ys_y)
                    bottom_x_yellow = xs_y[ys_y == max_y_yellow]
                    cx_y = int(np.mean(bottom_x_yellow))
                    self.get_logger().info(f'cx_y={cx_y}')
                else:
                    cx_y = None

            # White mask centroid (bottom row)
                ys_w, xs_w = np.where(roi_white > 0)
                if len(ys_w) > 0:
                    max_y_white = np.max(ys_w)
                    bottom_x_white = xs_w[ys_w == max_y_white]
                    cx_w = int(np.mean(bottom_x_white))
                    self.get_logger().info(f'cx_y={cx_w}')
                else:
                    cx_w = None
                self.get_logger().info(f"State: {self.state}")
                distance_between_lines = abs(cx_y - cx_w)
                self.dist=distance_between_lines/2
                lane_center = (cx_y + cx_w) // 2


                img_center = roi.shape[1] // 2
                error = img_center - lane_center #find the lane center
                
                
                self.get_logger().info(f"error:{error}")
                # Create Twist message
                twist = Twist()
                twist.linear.x = self.linear_velocity_gain
                #follow lane center by adjusting the error
                if error<0.1:
                    error = img_center - lane_center
                    self.get_logger().info(f"error <0.1:{error}")
                    self.get_logger().info("Forward turniung right")
                    nerror=abs(error)
                    correction=self.angular_velocity_gain * -nerror
                    self.get_logger().info(f"correction:{correction}")
                    twist.angular.z = self.angular_velocity_gain * -nerror
                if error>0.1:
                    error = img_center - lane_center
                    self.get_logger().info(f"error>0.1:{error}")
                    self.get_logger().info("Forward turniung left")
                    n2error=abs(error)
                    correction2=self.fast_angular_velocity_gain * n2error
                    self.get_logger().info(f"correction2:{correction2}")
                    twist.angular.z = self.fast_angular_velocity_gain * n2error
                self.publisher_.publish(twist)
            elif cx_y is None or cx_w is None:
                self.state="turn"

        # if it sees only one line
        elif self.state=="turn":
            ys_y, xs_y = np.where(roi_yellow > 0)
            if len(ys_y) > 0:
                max_y_yellow = np.max(ys_y)
                bottom_x_yellow = xs_y[ys_y == max_y_yellow]
                cx_y = int(np.mean(bottom_x_yellow))
                self.get_logger().info(f'cx_y={cx_y}')
            else:
                cx_y = None

            # White mask centroid (bottom row)
            ys_w, xs_w = np.where(roi_white > 0)
            if len(ys_w) > 0:
                max_y_white = np.max(ys_w)
                bottom_x_white = xs_w[ys_w == max_y_white]
                cx_w = int(np.mean(bottom_x_white))
                self.get_logger().info(f'cx_y={cx_w}')
            else:
                cx_w = None

            self.get_logger().info(f"State: {self.state}")
            twist = Twist()
            img_center = roi.shape[1] // 2
            #if yellow is visible
            if cx_y is not None:
                    twist.linear.x = self.slow_linear_velocity_gain
                    self.get_logger().info(' turn alliginig to the yellow_left')
                    
                    act_dist=abs(cx_y-img_center)
                    error = abs(act_dist-self.dist)
                    # error = act_dist - self.dist
                    if error>0.1:
                        self.get_logger().info("turn YELLOW turn turniung left")
                        twist.angular.z = self.fast_angular_velocity_gain * error
                    elif error<0.1:
                        self.get_logger().info(" turn YELLOW turn turniung right")
                        twist.angular.z = self.fast_angular_velocity_gain * -error

                    self.publisher_.publish(twist)
            #if white is visible
            elif cx_w is not None:
                    self.get_logger().info(' turn alliginig to the white_right')
                    twist.linear.x = self.slow_linear_velocity_gain
                    act_dist=abs(cx_w-img_center)
                    error = abs(act_dist - self.dist)
                    if error>0.1:
                        self.get_logger().info("turn white turn turniung left")
                        twist.angular.z = self.fast_angular_velocity_gain * -error
                    elif error<0.1:
                        self.get_logger().info(" turn white turn turniung right")
                        twist.angular.z = self.fast_angular_velocity_gain * error
                    self.publisher_.publish(twist)
            elif cx_y is not None and cx_w is not None:
                          

                    self.state="follow_wall"

        else:
            # Stop if no lines are detected
            self.get_logger().info("One or both lines not detected")
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        self.publisher_.publish(twist)
    
    def scan_callback(self, msg):
        front = msg.ranges[0]
        twist = Twist()
        if front<0.3:
            
            self.get_logger().info("obsacle ahead")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        self.publisher_.publish(twist)
            


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LineFollower()
        
        # Use MultiThreadedExecutor to handle callbacks concurrently
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
            
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
