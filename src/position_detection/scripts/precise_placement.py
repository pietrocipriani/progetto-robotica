import open3d;
import rospy
import rospkg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from ctypes import *
from block_detector.srv import DetectBlocks
import os

def convertCloudFromRosToOpen3d(ros_cloud: PointCloud2):
    convert_rgbUint32_to_tuple = lambda rgb_uint32: (
        (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
    )
    convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
        int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
    )
    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    open3d_cloud = open3d.geometry.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

        # combine
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud






class PrecisePlacement:
    def __init__(self, point_cloud_srv):
        self.bridge = CvBridge()
        self.point_cloud = None
        rospy.Subscriber(point_cloud_srv, PointCloud2, self.callback_cloud)

        #load meshes
        rospack = rospkg.RosPack()
        model_path = os.path.join(rospack.get_path("position_detection"), "blocks")
        self.meshes={}

        mesh = open3d.io.read_triangle_mesh("model://brick_1x1_H/mesh.stl")
        print(mesh)

    def image_callback(self, data: Image):
        data = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        cv2.imshow("test", data)
        cv2.waitKey(1)

    def callback_cloud(self, data: PointCloud2):
        self.point_cloud = convertCloudFromRosToOpen3d(data)
        #print(self.point_cloud)

        # visualizzation
        open3d.visualization.draw_geometries([data],
            zoom=0.3412,
            front=[0.4257, -0.2125, -0.8795],
            lookat=[2.6172, 2.0475, 1.532],
            up=[-0.0694, -0.9768, 0.2024])

#cv_bridge = CvBridge()

    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)



def main():
    print(open3d.__version__)
    rospy.init_node("position_detection")
    rospy.loginfo("position_detection init")
    detect_blocks_srv = rospy.ServiceProxy("detect_blocks", DetectBlocks)
    cloud_srv="/ur5/zed_node/point_cloud/cloud_registered"
    precise =PrecisePlacement(cloud_srv, )
    rospy.loginfo("registered")
    #rospy.Subscriber("/ur5/zed_node/left/image_rect_color", Image, callback_image)
    
    #proc = PrecisePlacement()
    #rospy.Service("detect_blocks", DetectBlocks, proc.callback)
    try:
        while True:
            rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass