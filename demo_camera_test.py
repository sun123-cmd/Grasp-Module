import os
import argparse
import numpy as np
import open3d as o3d
from graspnetAPI import GraspGroup
# from tracker import AnyGraspTracker
from gsnet import AnyGrasp
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, JointState

from tf.transformations import euler_from_quaternion, quaternion_matrix, quaternion_from_matrix

import sys
import moveit_commander
import geometry_msgs.msg
import numpy as np
from scipy.spatial.transform import Rotation as R

dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, 'log/checkpoint_detection.tar')

parser = argparse.ArgumentParser()
parser.add_argument('--checkpoint_path', required=False, default=filename, help='Model checkpoint path')
parser.add_argument('--filter', type=str, default='oneeuro', help='Filter to smooth grasp parameters(rotation, width, depth). [oneeuro/kalman/none]')
parser.add_argument('--debug', action='store_true', help='Enable visualization')
parser.add_argument('--max_gripper_width', type=float, default=0.085, help='Maximum gripper width (<=0.1m)')
parser.add_argument('--gripper_height', type=float, default=0.03, help='Gripper height')
parser.add_argument('--top_down_grasp', type=bool, default=True, help='Output top-down grasps')
parser.add_argument('--method', type=String, default="detection", help='Method to get grasping positions')
cfgs = parser.parse_args()

class RealSense2Camera:
    def __init__(self):
        rospy.init_node('Anygrasp', anonymous=False)
        
        # self.cv_bridge = CvBridge()
        self.current_image, self.current_depth = None, None
        self.current_point_cloud = None

        # # Camera--simulation
        self.frame_id = "camera_color_optical_frame"
        self.height, self.width = None, None        
        
        # self.color_info_topic = "/camera/color/camera_info"
        # self.depth_info_topic = "/camera/depth/color/points"
        # self.intrinsics_topic = "/camera/color/camera_info"
        
        # self.color_topic = "/camera/color/image_raw"
        # self.depth_topic = "/camera/depth/image_raw"
        
        # #self.camera_topic = "/vrpn_client_node/franka_base16/pose"
        # self.camera_topic = "/vrpn_client_node/cam_grasp/pose_transform"
        # self.color_info_topic = "/g_d435/rgb/camera_info"
        # self.depth_info_topic = "/g_d435/depth/camera_info"
        # self.intrinsics_topic = "/g_d435/rgb/camera_info"
        
        # self.color_topic = "/g_d435/rgb/g_image_raw"
        # self.depth_topic = "/g_d435/depth/g_image_raw"

        self.color_info_topic = "/l_camera/color/camera_info"
        self.depth_info_topic = "/l_camera/aligned_depth_to_color/camera_info"
        self.intrinsics_topic = "/l_camera/color/camera_info"
        
        self.color_topic = "/l_camera/color/image_raw"
        self.depth_topic = "/l_camera/aligned_depth_to_color/image_raw"
        
        #self.camera_topic = "/vrpn_client_node/franka_base16/pose"
        self.camera_topic = "/vrpn_client_node/cam_grasp/pose_transform"

        # From rostopic /camera/aligned_depth_to_color/camera_info K:
        #TODO get this from rostopic
        self.camera_intrinsics = None
        self.camera_pose = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.scale = None

        self.color_sensor_state = {'active': False, 'ready': False}
        self.depth_sensor_state = {'active': False, 'ready': False}

        # Subscribers
        self.image_sub = rospy.Subscriber(
            self.color_topic, Image, self.callback_receive_color_image, queue_size=1)
        self.depth_sub = rospy.Subscriber(
            self.depth_topic, Image, self.callback_receive_depth_image, queue_size=1)
        self.intrinsics_sub = rospy.Subscriber(
            self.intrinsics_topic, CameraInfo, self.callback_intrinsics, queue_size=1)
        self.camera_sub = rospy.Subscriber(
            self.camera_topic, PoseStamped, self.callback_camera_pose)
        
        # Publishers
        # self.grasp_pose_pub = rospy.Publisher('/grasp_pose', PoseStamped, queue_size=1)
        self.pose_pub = rospy.Publisher('/detect_grasps/pose_grasps', Pose, queue_size=1)
        
        self.rate = rospy.Rate(1)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        

    def _active_sensor(self):
        self.color_sensor_state['active'] = True
        self.depth_sensor_state['active'] = True
        
    
    def callback_camera_pose(self, data):# Extract quaternion from PoseStamped message
        self.camera_pose = data.pose
        
        # quaternion = (
        #     data.pose.orientation.x,
        #     data.pose.orientation.y,
        #     data.pose.orientation.z,
        #     data.pose.orientation.w
        # )

        # # Convert quaternion to Euler angles in degrees
        # euler_angles = euler_from_quaternion(quaternion)
        # roll, pitch, yaw = euler_angles
        # self.roll, self.pitch, self.yaw = roll * 180 / 3.14159, pitch * 180 / 3.14159, yaw * 180 / 3.14159

    def callback_intrinsics(self, data):
        self.intrinsics = data
        
        self.height, self.width = self.intrinsics.height, self.intrinsics.width
        self.camera_intrinsics = np.array(self.intrinsics.K).reshape(3, 3)
        self.fx = self.camera_intrinsics[0, 0]
        self.fy = self.camera_intrinsics[1, 1]
        self.cx = self.camera_intrinsics[0, 2]
        self.cy = self.camera_intrinsics[1, 2]
        self.scale = 1/0.001
        self.intrinsics_sub.unregister()

    def callback_receive_color_image(self, image):
        if not self.color_sensor_state['active']:
            return

        # Get BGR image from data
        self.current_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)

        self.color_sensor_state['active'] = False
        self.color_sensor_state['ready'] = True

    def callback_receive_depth_image(self, depth):
        """ Callback. Get raw depth from data (Unit: mm). """

        if not self.depth_sensor_state['active']:
            return
        """
            Reference here:
                current_depth = self.cv_bridge.imgmsg_to_cv2(depth, "passthrough")
        """

        # Way 1: works
        if depth.encoding == '16UC1':
            channel = 1
            dtype = np.dtype('uint16')
            dtype = dtype.newbyteorder('>' if depth.is_bigendian else '<')

        # NOTE! not sure
        elif depth.encoding == '32FC1':
            channel = 1
            dtype = np.dtype('float32')
            dtype = dtype.newbyteorder('>' if depth.is_bigendian else '<')

        current_depth = np.frombuffer(depth.data, dtype=dtype).reshape(
            depth.height, depth.width, channel)

        # Way 2: works
        # if depth.encoding == '16UC1':
        #     depth.encoding = "mono16"
        #     current_depth = self.cv_bridge.imgmsg_to_cv2(depth, "mono16")

        # elif depth.encoding == '32FC1':
        #     depth.encoding = "mono16"
        #     current_depth = self.cv_bridge.imgmsg_to_cv2(depth, "mono16")


        # Way 3: works
        # current_depth = self.cv_bridge.imgmsg_to_cv2(
        #     depth, desired_encoding="passthrough")

        # Convert unit from millimeter into meter
        # current_depth = current_depth.astype(float) / 1000.
        current_depth = current_depth.astype(float)

        self.current_depth = current_depth.squeeze(axis=2) \
                             if len(current_depth.shape) >= 3 else current_depth

        self.depth_sensor_state['active'] = False
        self.depth_sensor_state['ready'] = True

    def get_rgbd_images(self):
        self._active_sensor()
        i = 0
        while True:
            if (self.color_sensor_state['ready'] and
                self.depth_sensor_state['ready']):
                color_image = self.current_image
                depth_image = self.current_depth

                self.color_sensor_state['ready'] = False
                self.depth_sensor_state['ready'] = False
                return color_image, depth_image

            rospy.sleep(0.1)
            i += 1
            print(i, end='\r')
            if i >= 50:
                print("No image")
                exit()
                
    def point_from_depth_image(self, depth, organized=True):
        """ Generate points using depth image only.\\
            Args:
                depth    (np.float): Depth image with the shape (H,W)
                organized    (bool): True for keeping the cloud in image shape (H,W,3)
            Returns:
                cloud: (np.float): points with shape (H,W,3) or (H*W,3) / point cloud
        """

        assert(depth.shape[0] == self.height and depth.shape[1] == self.width)
        xmap = np.arange(self.width)
        ymap = np.arange(self.height)
        xmap, ymap = np.meshgrid(xmap, ymap)
        points_z = depth/1000
        points_x = (xmap - self.cx) * points_z / self.fx
        points_y = (ymap - self.cy) * points_z / self.fy
        cloud = np.stack([points_x, points_y, points_z], axis=-1)
        if not organized:
            cloud = cloud.reshape([-1, 3])
        return cloud

def create_point_cloud_from_depth_image(depth, camera, organized=True):
    # TODO: delete this and directly use PC from ROS topic of the camera
    assert(depth.shape[0] == camera.height and depth.shape[1] == camera.width)
    xmap = np.arange(camera.width)
    ymap = np.arange(camera.height)
    xmap, ymap = np.meshgrid(xmap, ymap)
    points_z = depth / camera.scale
    points_z = depth 
    points_x = (xmap - camera.cx) * points_z / camera.fx
    points_y = (ymap - camera.cy) * points_z / camera.fy
    points = np.stack([points_x, points_y, points_z], axis=-1)
    if not organized:
        points = points.reshape([-1, 3])
    return points

def get_data(camera):
    # load image
    colors, depths = camera.get_rgbd_images()
    colors = colors / 255

    # get point cloud
    points = create_point_cloud_from_depth_image(depths, camera)
    mask = (points[:,:,2] > 0) & (points[:,:,2] < 1.5)
    points = points[mask]
    colors = colors[mask]

    return points, colors

def pose_stamped_to_matrix(pose_stamped_msg):
    # Extract position and orientation from the PoseStamped message
    position = pose_stamped_msg.position
    orientation = pose_stamped_msg.orientation

    # Convert quaternion to a 3x3 rotation matrix
    rotation_matrix = quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])

    # Create a 4x4 transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix[:3, :3]
    transformation_matrix[:3, 3] = [position.x, position.y, position.z]

    return transformation_matrix

def matrix_to_pose_stamped(transform_matrix):
    # Extract rotation matrix and translation vector from the transformation matrix
    rotation_matrix = transform_matrix[:3, :3]
    translation_vector = transform_matrix[:3, 3]

    # Convert the rotation matrix to quaternion
    quaternion = quaternion_from_matrix(transform_matrix)

    # Create a PoseStamped message
    pose_stamped_msg = PoseStamped()
    pose_stamped_msg.pose.position.x = translation_vector[0]
    pose_stamped_msg.pose.position.y = translation_vector[1]
    pose_stamped_msg.pose.position.z = translation_vector[2]
    pose_stamped_msg.pose.orientation.x = quaternion[0]
    pose_stamped_msg.pose.orientation.y = quaternion[1]
    pose_stamped_msg.pose.orientation.z = quaternion[2]
    pose_stamped_msg.pose.orientation.w = quaternion[3]

    return pose_stamped_msg


def q_list_to_joint_state(q):
    q_ros = JointState()
    n = q.size
    
    
    # Set the header information (optional)
    q_ros.header.stamp = rospy.Time.now()
    q_ros.header.frame_id = 'base_link'  # Replace with your desired frame_id

    # Set the joint names
    q_ros.name = [f'joint{i}' for i in range(1, n + 1)]  # Replace with your joint names

    # Set the joint positions
    q_ros.position = q  # Replace with your desired joint positions

    # Set the joint velocities (zero velocities)
    q_ros.velocity =  [0.0] * n

    # Set the joint efforts (optional, set to zero if not applicable)
    q_ros.effort =  [0.0] * n
    
    return q_ros

def points_in_lims(lims, points):
    x_min, x_max, y_min, y_max, z_min, z_max = lims
    
    # Create boolean masks for each dimension
    mask_x = (points[:, 0] >= x_min) & (points[:, 0] <= x_max)
    mask_y = (points[:, 1] >= y_min) & (points[:, 1] <= y_max)
    mask_z = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)

    # Combine the masks using logical AND to find points inside all limits
    mask_all = mask_x & mask_y & mask_z

    # Get the indices of points that satisfy the conditions
    indices_of_points_inside_limits = np.where(mask_all)[0]
    return indices_of_points_inside_limits


def demo():
    # intialization
    # TODO remove tracking once sure detection is the good method
    # NOTE "--checkpoint_path" must be given accordingly in json file
    if cfgs.method.data == "tracking":
        anygrasp_tracker = AnyGraspTracker(cfgs)
        anygrasp_tracker.load_net()
    elif cfgs.method.data == "detection":
        anygrasp = AnyGrasp(cfgs)
        anygrasp.load_net()
    
    camera = RealSense2Camera()
    grasp_ids = [0]
    while camera.scale == None:
        print("Waiting for camera intrinsics", end='\r')
        camera.rate.sleep()
    print("Intrinsics parameters acquired")
    
    
      
    if cfgs.debug:
        vis = o3d.visualization.Visualizer()
        vis.create_window(height=camera.height, width=camera.width)
    
    # NOTE GPU memory depends on size of lims
    xmin, xmax = -0.30, 0.30
    ymin, ymax = -0.30, 0.30
    zmin, zmax = 0.1, 2.8
    lims = [xmin, xmax, ymin, ymax, zmin, zmax]
    
   
    POINTSLIMS = False
    
    
    for i in range(1000000):
        input("Enter next loop...")
        # get prediction
        points, colors = get_data(camera)
        
        if cfgs.method.data == "tracking":
            target_gg, curr_gg, target_grasp_ids, corres_preds = anygrasp_tracker.update(points, colors, grasp_ids)
            if i == 0:
                # select grasps on objects to track for the 1st frame
                grasp_mask_x = ((curr_gg.translations[:,0]>-0.18) & (curr_gg.translations[:,0]<0.18))
                grasp_mask_y = ((curr_gg.translations[:,1]>-0.12) & (curr_gg.translations[:,1]<0.12))
                grasp_mask_z = ((curr_gg.translations[:,2]>0.35) & (curr_gg.translations[:,2]<0.55))
                grasp_ids = np.where(grasp_mask_x & grasp_mask_y & grasp_mask_z)[0][:30:6]
                target_gg = curr_gg[grasp_ids]
            else:
                grasp_ids = target_grasp_ids
            #print(i, target_grasp_ids)
            
            best_index = np.argmax(target_gg.scores)
            best_depth = target_gg.depths[best_index]
            best_height = target_gg.heights[best_index]
            best_width = target_gg.widths[best_index]
            best_trans = target_gg.translations[best_index]
            best_rot = target_gg.rotation_matrices[best_index]
            
            print(best_trans)
            
            # visualization
            if cfgs.debug:
                trans_mat = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
                cloud = o3d.geometry.PointCloud()
                cloud.points = o3d.utility.Vector3dVector(points)
                cloud.colors = o3d.utility.Vector3dVector(colors)
                cloud.transform(trans_mat)
                grippers = target_gg.to_open3d_geometry_list()
                for gripper in grippers:
                    gripper.transform(trans_mat)
                vis.add_geometry(cloud)
                for gripper in grippers:
                    vis.add_geometry(gripper)
                vis.poll_events()
                vis.remove_geometry(cloud)
                for gripper in grippers:
                    vis.remove_geometry(gripper)
        


        elif cfgs.method.data == "detection":
            points = np.float32(points)
            colors = np.float32(colors)
            if (POINTSLIMS):
                index_in_lims = points_in_lims(lims, points)
                points = points[index_in_lims,:]
                colors = colors[index_in_lims,:]
            
            print('points_size = ',points.size) # 1280*720=921600
            print('colors_size = ',colors.size)
            if points.size == 0 or colors.size==0:
                print('No points in the space!')
                camera.rate.sleep()
                continue
            try:
                gg, cloud = anygrasp.get_grasp(points, colors, lims)
            except:
                print('Problem inside NN')
                camera.rate.sleep()
                continue
            # gg, cloud = anygrasp.get_grasp(points, colors, lims)

            if len(gg) == 0:
                print('No Grasp detected after collision detection!')
                camera.rate.sleep()
                continue

            gg = gg.nms().sort_by_score()
            target_gg = gg[0:1]
            
            best_index = np.argmax(target_gg.scores)
            best_depth = target_gg.depths[best_index]
            best_height = target_gg.heights[best_index]
            best_width = target_gg.widths[best_index]
            best_trans = target_gg.translations[best_index]
            best_rot = target_gg.rotation_matrices[best_index]            
            # print("best_index is :", best_index)
            # print("best_depth is:", best_depth)
            # print("best_height is:",best_height)
            # print("best_width is:", best_width)
            # print("best_trans is :", best_trans)
            # print("best_rot is:", best_rot)
            

            #Transformation matrix camera-anygrap pose，from current camera to any 
            T_c_a = np.eye(4)
            T_c_a[:3, :3] = best_rot
            T_c_a[:3, 3] = best_trans
            
           
            T_r_a = T_c_a
            
            
            # Y rotation to correct for Anygrasp frame -> franka robot frame
            rotation_matrix_y = np.array([
                [0, 0, 1],
                [0, 1, 0],
                [-1, 0, 0]
            ])
            rotation_matrix_z = np.array([
                [0, 1, 0],
                [-1, 0, 0],
                [0, 0, 1]
            ])
            matrix_orientation =  np.eye(4)
            matrix_orientation[:3, :3] = rotation_matrix_y #@ rotation_matrix_z
            T_r_a = T_r_a @ matrix_orientation
            
            
            # # Transformation to get the EE gripper at correct pose
            offset_grasp = cfgs.gripper_height
            T_r_a[:3, 3] = T_r_a[:3, 3] + offset_grasp * T_r_a[:3, 2]      
            
            
            # # Pose topic creation
            pose_grasp = matrix_to_pose_stamped(T_r_a)
            print(pose_grasp.pose)
           
                            
            if cfgs.debug:
                #TODO plot only points in the workspace limits
                trans_mat = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
                cloud = o3d.geometry.PointCloud()
                cloud.points = o3d.utility.Vector3dVector(points)
                cloud.colors = o3d.utility.Vector3dVector(colors)
                cloud.transform(trans_mat)
                grippers = target_gg.to_open3d_geometry_list()
                for gripper in grippers:
                    gripper.transform(trans_mat)
                vis.add_geometry(cloud)
                count = 0
                for gripper in grippers:
                    vis.add_geometry(gripper)
                    count = count + 1
                vis.poll_events()


                print("grippers size: " + str(count))

                vis.remove_geometry(cloud)
                for gripper in grippers:
                    vis.remove_geometry(gripper)
            
                camera.pose_pub.publish(pose_grasp.pose)
      
        camera.rate.sleep()

if __name__ == "__main__":
    demo()
