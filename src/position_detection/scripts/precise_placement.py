import open3d;
import rospy
import rospkg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import Point
from std_msgs.msg import Header
#from sensor_msgs.msg import 
import cv2
from cv_bridge import CvBridge
import numpy as np
from ctypes import *
from block_detector.srv import DetectBlocks
import os
import math
import threading
import time
from open3d.pipelines import registration as treg
from position_detection.msg import BlockPosition, BlockPositions 
import copy

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
        #print("Converting an empty cloud")
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
#print(camera_transform*inv_camera_transform)
#given x, y coordinates (in respect to optical center), it generates 2 points
def generate_points(x, y, y_min):
    dist=0.53/(0.00109*y_min+0.5)
    #print(dist, y_min)
    return [[0.1*math.tan(0.88)/960*x, 0.1*math.tan(0.88)/960*y, 0.1],
     [dist*math.tan(0.88)/960*x, dist*math.tan(0.88)/960*y, dist]]
    

def generate_intersection_mesh(x_min, x_max, y_min, y_max):
    x_min-=960
    x_max-=960
    y_min-=540
    y_max-=540
    ##print(generate_points(x_min, y_min))
    points = np.array([generate_points(x_min, y_min, y_min)])
    points = np.append(points, [generate_points(x_min, y_max, y_min)])
    points = np.append(points, [generate_points(x_max, y_min, y_min)])
    points = np.append(points, [generate_points(x_max, y_max, y_min)])
    points = np.reshape(points, (8, 3))
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(points)
    mesh = pcd.get_oriented_bounding_box()
    #mesh, _ = pcd.compute_convex_hull()
    #mesh.compute_vertex_normals()
    ##print(mesh)
    return mesh

def approx_coordinate(x, y):
    x-=960
    y-=540
    

class PrecisePlacement:
    def update_visualizer(self):
        while True:
            ##print("run")
            self.vis.poll_events()
            self.vis.update_renderer()
            time.sleep(0.020)
    def point_cloud_processer(self):
        while True:
            if self.raw_point_cloud is None:
                time.sleep(0.1)
            else:
                self.process_latest_point_cloud()
            ##print("run")
            #self.vis.poll_events()
            #self.vis.update_renderer()
            #time.sleep(0.020)

    def __init__(self, point_cloud_srv, use_visualizer=True):
        self.msg_seq=0
        self.point_cloud = None
        self.use_visualizer = use_visualizer
        self.raw_point_cloud = None

        if use_visualizer:
            self.vis=open3d.visualization.Visualizer()
            self.vis.create_window()
            v_control = self.vis.get_view_control()
            v_control.set_zoom(0.3)
            v_control.set_front([-1, -0.0, 0.])
            v_control.set_lookat([0.4, 0.5, 0.86])
            v_control.set_up([-0.0, 0.0, 1.0])
            #self.vis.reset_view_point( reset_bounding_box=True)
            bbox = generate_intersection_mesh(0, 100, 0, 100)
            bbox = open3d.geometry.TriangleMesh.create_from_oriented_bounding_box(bbox)
            bbox.paint_uniform_color([1, 0.706, 0])
            bbox.transform(camera_transform)
            self.vis.add_geometry(bbox)

            self.thread_visualizer = threading.Thread(target=self.update_visualizer, name="process_visualization")
            self.thread_visualizer.start()
        
        self.thread_point_cloud_processer = threading.Thread(target=self.point_cloud_processer, name="process_visualization")
        self.thread_point_cloud_processer.start()

        self.bridge = CvBridge()
        
        rospy.Subscriber(point_cloud_srv, PointCloud2, self.callback_cloud)
        rospy.Subscriber("/ur5/zed_node/left/image_rect_color", Image, self.image_callback)
        self.detect_blocks_srv = rospy.ServiceProxy("detect_blocks", DetectBlocks)
        self.pub = rospy.Publisher("block_positions", BlockPositions, queue_size=1)
        #load meshes
        rospack = rospkg.RosPack()
        model_path = os.path.join(rospack.get_path("controller"), "models")#/mesh.stl")  
        self.meshes={
            "1x1_H": None,
            "2x1_T": None,
            "2x1_L": None,
            "2x1_H": None,
            "2x1_U": None,
            "2x2_H": None,
            "2x2_U": None,
            "3x1_H": None,
            "3x1_U": None,
            "4x1_H": None,
            "4x1_L": None,
            }
        for key in self.meshes.keys():
            cur_model_path = os.path.join(model_path, os.path.join("brick_" + key, "mesh.stl"))
            self.meshes[key] = open3d.io.read_triangle_mesh(cur_model_path)
            self.meshes[key].compute_vertex_normals()

            ##print(self.meshes[key])
        
        
    #called every time an image is ready
    def image_callback(self, data: Image):
        # if I don't have a converted point cloud it's useless

        if self.point_cloud is None:
            return
        decoded_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        decoded_image = decoded_image[396:912, 676:1544, :]
        resp = self.detect_blocks_srv(self.bridge.cv2_to_imgmsg(decoded_image, encoding="bgr8"))
        to_send=[]
        for box in resp.boxes:
            mesh=generate_intersection_mesh(676+box.x1, 676+box.x2, 396+box.y1, 396+box.y2)
            mesh.rotate(camera_transform[0:3, 0:3], [0, 0, 0])
            mesh = mesh.translate(camera_transform[0:3, 3].transpose())
            
            cropped = self.point_cloud.crop(mesh)
            if len(cropped.points)< 100:
                continue
            cropped.paint_uniform_color(np.random.rand(3))
            self.vis.add_geometry(cropped, reset_bounding_box=False)
            center = cropped.get_center()
            transform = np.eye(4)
            transform[0:3, 3]=center


            source= self.meshes[box.label].sample_points_uniformly(number_of_points=2000)
            target=cropped
            threshold = 0.01

            start = time.time()
            target.estimate_normals()
            result = open3d.pipelines.registration.registration_icp(
                source, target, threshold, transform,
                open3d.pipelines.registration.TransformationEstimationPointToPoint(),
                open3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
            transform = copy.deepcopy(result.transformation)

            
            def zeroed(vec):
                vec[2]=0
                mod=1/(vec[1]*vec[1]+vec[0]*vec[0])
                return vec*mod

            transform[3][3]=1
            transform[1, 0:3] = zeroed(transform[1, 0:3])
            transform[0, 0:3] = zeroed(transform[0, 0:3])
            transform[0:3, 1] = zeroed(transform[0:3, 1])
            transform[0:3, 0] = zeroed(transform[0:3, 0])
            source.transform(transform)
            if result.fitness>0.6:
                self.vis.add_geometry(source, reset_bounding_box=False)
                print(result.fitness)
            #print(transform)
            point = Point(x= transform[0][3], y=transform[1][3], z=transform[2][3])
            to_add = BlockPosition(block_type=box.label,
                                   confidence=result.fitness*box.confidence,
                                   point=point,
                                   angle=math.acos(max(min(transform[0][0], 1), -1)))
            to_send.append(to_add)
            #print(to_add)
        header = Header(seq=self.msg_seq, frame_id = "base link", stamp=rospy.Time.now())
        to_send = BlockPositions(header=header, blocks=to_send)
        #print(to_send)
        self.pub.publish(to_send)
        #cv2.imshow("test", decoded_image)
        #cv2.waitKey(1)

    def callback_cloud(self, data: PointCloud2):
        ##print("point_cloud")
        self.raw_point_cloud=data

    def process_latest_point_cloud(self):
        converted = convertCloudFromRosToOpen3d(self.raw_point_cloud)
        bbox = open3d.geometry.AxisAlignedBoundingBox(min_bound=(0, 0.16, 0.87), max_bound=(1., 0.8, 0.97))

        bbox = bbox.get_oriented_bounding_box()
        bbox = bbox.rotate(inv_camera_transform[0:3, 0:3], [0, 0, 0 ])
        bbox = bbox.translate(inv_camera_transform[0:3, 3].transpose())
        
        mesh = open3d.geometry.TriangleMesh.create_from_oriented_bounding_box(bbox)
        mesh.compute_vertex_normals()
        open3d.io.write_triangle_mesh("bbox.stl", mesh)
        ###print(self.point_cloud.get_axis_aligned_bounding_box())
        #is_none = self.point_cloud is None
        point_cloud=converted.crop(bbox)
        self.point_cloud = point_cloud.transform(camera_transform)
        ##print(self.point_cloud)

        # visualizzation
        if self.use_visualizer:
            self.vis.clear_geometries()
            self.vis.add_geometry(self.point_cloud, reset_bounding_box=False)

#cv_bridge = CvBridge()

    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)





def main():
    ##print(open3d.__version__)
    rospy.init_node("position_detection")
    rospy.loginfo("position_detection init")

    cloud_srv="/ur5/zed_node/point_cloud/cloud_registered"
    precise =PrecisePlacement(cloud_srv)
    rospy.loginfo("Ready")
    
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