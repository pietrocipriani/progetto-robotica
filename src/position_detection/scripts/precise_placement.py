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
import math

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

camera_transform = np.array([[ 0, -0.5,  0.866,  -0.4],
                            [ -1,  0,  0,  0.59],
                            [ 0, -0.866, -0.5, 1.4],
                            [0, 0, 0, 1]])

inv_camera_transform = np.linalg.inv(camera_transform)
print(camera_transform*inv_camera_transform)
#given x, y coordinates (in respect to optical center), it generates 2 points
def generate_points(x, y):
    return [[0.1*math.tan(0.88)/960*x, 0.1*math.tan(0.88)/960*y, 0.1],
     [2*math.tan(0.88)/960*x, 2*math.tan(0.88)/960*y, 2]
     ]
    

def generate_intersection_mesh(x_min, y_min, x_max, y_max):
    print(generate_points(x_min, y_min))
    points = np.array([generate_points(x_min, y_min)])
    points = np.append(points, [generate_points(x_min, y_max)])
    points = np.append(points, [generate_points(x_max, y_min)])
    points = np.append(points, [generate_points(x_max, y_max)])
    points = np.reshape(points, (8, 3))
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(points)
    mesh, _ = pcd.compute_convex_hull()
    mesh.compute_vertex_normals()
    print(mesh)
    open3d.io.write_triangle_mesh("trapezio.stl", mesh)
    #open3d.io.write_point_cloud("trapezio.pcd", pcd)

generate_intersection_mesh(-960, -540, 960, 540)


#bbox = bbox.transform(inv_camera_transform)

class PrecisePlacement:
    def __init__(self, point_cloud_srv):
        self.bridge = CvBridge()
        self.point_cloud = None
        rospy.Subscriber(point_cloud_srv, PointCloud2, self.callback_cloud)

        #load meshes
        rospack = rospkg.RosPack()
        model_path = os.path.join(rospack.get_path("controller"), "models")#/mesh.stl")  
        self.meshes={
            "brick_1x1_H": None,
            "brick_2x1_T": None,
            "brick_2x1_L": None,
            "brick_2x1_H": None,
            "brick_2x1_U": None,
            "brick_2x2_H": None,
            "brick_2x2_U": None,
            "brick_3x1_H": None,
            "brick_3x1_U": None,
            "brick_4x1_H": None,
            "brick_4x1_L": None,
            }
        for key in self.meshes.keys():
            cur_model_path = os.path.join(model_path, os.path.join(key, "mesh.stl"))
            self.meshes[key] = open3d.io.read_triangle_mesh(cur_model_path)
            print(self.meshes[key])
        
        

    def image_callback(self, data: Image):
        data = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        cv2.imshow("test", data)
        cv2.waitKey(1)

    def callback_cloud(self, data: PointCloud2):
        converted = convertCloudFromRosToOpen3d(data)
        bbox = open3d.geometry.AxisAlignedBoundingBox(min_bound=(0, 0.14, 0.86), max_bound=(1., 0.8, 0.97))

        bbox = bbox.get_oriented_bounding_box()
        bbox = bbox.rotate(inv_camera_transform[0:3, 0:3], [0, 0, 0 ])
        bbox = bbox.translate(inv_camera_transform[0:3, 3].transpose())
        
        mesh = open3d.geometry.TriangleMesh.create_from_oriented_bounding_box(bbox)
        mesh.compute_vertex_normals()
        open3d.io.write_triangle_mesh("bbox.stl", mesh)
        #print(self.point_cloud.get_axis_aligned_bounding_box())

        self.point_cloud=converted.crop(bbox)
        self.point_cloud = self.point_cloud.transform(camera_transform)
        print(self.point_cloud)
        open3d.io.write_point_cloud("test.pcd", self.point_cloud)
        # visualizzation
        open3d.visualization.draw_geometries([self.point_cloud],
            zoom=0.3,
            front=[0.0, -0.0, 1.0],
            lookat=[0.0, 0.0, 1.0],
            up=[-0.0, -1.0, 0.0])

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