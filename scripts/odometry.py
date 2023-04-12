#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
import ros_numpy
import pyrealsense2

class ImageListener():

    def __init__(self, topic):
        self.topic = topic
        self.sub = rospy.Subscriber(topic, Image, self.imageDepthCallback)
        self.sub_info = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.imageDepthInfoCallback)
        self.intrinsics = None

    def imageDepthCallback(self, image):
        try:
            np_image = ros_numpy.numpify(image)
            pixel = (np_image.shape[1]//2, np_image.shape[0]//2)
            if self.intrinsics:
                depth = np_image[pixel[1], pixel[0]]
                result = pyrealsense2.rs2_deproject_pixel_to_point(self.intrinsics, [pixel[0], pixel[1]], depth)
                print(f"result {result}")
            
        except Exception as e:
            print(f"Error in imageDepthCallback: {str(e)}")

    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = pyrealsense2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = pyrealsense2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = pyrealsense2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except Exception as e:
            print(f"Error in imageDepthInfoCallback: {str(e)}")
            return

class PointCloudListener():

    def __init__(self, topic):
        self.topic = topic
        self.sub = rospy.Subscriber(self.topic, PointCloud2, self.pointCloudCallback)

    def pointCloudCallback(self, points):
        try:
            np_points = ros_numpy.numpify(points)
            pixel = (np_points.shape[0]//2, np_points.shape[1]//2)
            x_point, y_point, z_point, _ = np_points[pixel[0], pixel[1]]
            print(z_point)
        except Exception as e:
            print(f"Error in pointCloudCallback: {str(e)}")

def main(foo="points"):
    if foo == "points":
        topic = "/camera/depth/color/points"
        listener = PointCloudListener(topic)
    else:
        topic = '/camera/depth/image_rect_raw'
        listener = ImageListener(topic)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node("odometry_node")
    main()