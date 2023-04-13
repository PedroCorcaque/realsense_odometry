#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
import ros_numpy
import pyrealsense2

class ImageListener():
    """ The ImageListener is used to get the z distance from depth camera """
    def __init__(self, topic):
        self.topic = topic
        self.sub = rospy.Subscriber(topic, Image, self.image_depth_callback)
        self.sub_info = rospy.Subscriber('/camera/depth/camera_info', \
                                         CameraInfo, \
                                         self.image_depth_info_callback)
        self.intrinsics = None

    def image_depth_callback(self, image: Image) -> None:
        """ A callback for image depth camera """
        np_image = ros_numpy.numpify(image)
        pixel = (np_image.shape[1]//2, np_image.shape[0]//2)
        if self.intrinsics:
            depth = np_image[pixel[1], pixel[0]]
            result = pyrealsense2.rs2_deproject_pixel_to_point(self.intrinsics,\
                                                                [pixel[0], pixel[1]],\
                                                                depth)
            print(f"result {result}")

    def image_depth_info_callback(self, camera_info: CameraInfo) -> None:
        """ A callback for camera info depth """
        if self.intrinsics:
            return
        self.intrinsics = pyrealsense2.intrinsics()
        self.intrinsics.width = camera_info.width
        self.intrinsics.height = camera_info.height
        self.intrinsics.ppx = camera_info.K[2]
        self.intrinsics.ppy = camera_info.K[5]
        self.intrinsics.fx = camera_info.K[0]
        self.intrinsics.fy = camera_info.K[4]
        if camera_info.distortion_model == 'plumb_bob':
            self.intrinsics.model = pyrealsense2.distortion.brown_conrady
        elif camera_info.distortion_model == 'equidistant':
            self.intrinsics.model = pyrealsense2.distortion.kannala_brandt4
        self.intrinsics.coeffs = list(camera_info.D)

class PointCloudListener():
    """ The PointCloudListener is used to get the z distance from point cloud """
    def __init__(self, topic):
        self.topic = topic
        self.sub = rospy.Subscriber(self.topic, PointCloud2, self.point_cloud_callback)

    def point_cloud_callback(self, points: PointCloud2) -> None:
        """ A callback for point cloud subscriber """
        np_points = ros_numpy.numpify(points)
        pixel = (np_points.shape[0]//2, np_points.shape[1]//2)
        _, _, z_point, _ = np_points[pixel[0], pixel[1]]
        print(z_point)

def main(option="points"):
    """ The main function that will be used to choose the method """
    if option == "points":
        topic = "/camera/depth/color/points"
        _ = PointCloudListener(topic)
    else:
        topic = '/camera/depth/image_rect_raw'
        _ = ImageListener(topic)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node("odometry_node")
    main()
