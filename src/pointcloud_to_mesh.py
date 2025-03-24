#!/usr/bin/env python3
import rospy
import numpy as np
import struct
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import open3d as o3d
import os

from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Point, Pose, Quaternion

o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)

class PointCloudToMesh:
    def __init__(self):
        rospy.init_node("pointcloud_outlier_removal", anonymous=True)

        self.read_mesh = rospy.get_param("~read_mesh", True) 

        self.obstacle_subscriber = rospy.Subscriber(
            "/vlm_seg/point_cloud", 
            PointCloud2, 
            self.point_cloud_subscriber
        )

        self.save_mesh_path = "/ros_ws/src/cartesio_collision_avoidance/meshes/obstacle.stl"

        if not os.path.exists(os.path.dirname(self.save_mesh_path)):
            os.makedirs(os.path.dirname(self.save_mesh_path))
            
        self.alpha_used = 0.037
        self.eps = 0.1
        self.min_points = 10 

        self.ransac_distance_threshold = 0.01
        self.ransac_n = 10
        self.num_iterations = 10 

        self.planning_scene_pub = rospy.Publisher(
            "/cartesian/collision_avoidance/monitored_planning_scene",
            PlanningScene, 
            queue_size=1
        )

        if self.read_mesh:
            rospy.loginfo("Reading existing mesh from file.")
            self.read_and_publish_mesh()

    def read_and_publish_mesh(self):
        """
        Reads the mesh from disk and publishes it to the planning scene.
        """
        if not os.path.isfile(self.save_mesh_path):
            rospy.logwarn(f"Mesh file {self.save_mesh_path} does not exist.")
            return
        
        o3d_mesh = o3d.io.read_triangle_mesh(self.save_mesh_path)
        
        if len(o3d_mesh.triangles) == 0 or len(o3d_mesh.vertices) == 0:
            rospy.logwarn("Loaded mesh is empty.")
            return
        
        rospy.loginfo("Publishing mesh read from disk.")
        self.publish_obstacle_mesh(o3d_mesh)

    def publish_obstacle_mesh(self, o3d_mesh):
        ros_mesh = self.triangle_mesh_to_shape_msgs(o3d_mesh)

        collision_object = CollisionObject()
        collision_object.id = "obstacle_from_pcd"
        collision_object.header.frame_id = "ci/world"  

        
    
        collision_object.meshes = [ros_mesh]
        collision_object.operation = CollisionObject.ADD

        planning_scene_msg = PlanningScene()
        planning_scene_msg.is_diff = True
        planning_scene_msg.world.collision_objects.append(collision_object)

        self.planning_scene_pub.publish(planning_scene_msg)
        rospy.loginfo("Obstacle mesh published to the PlanningScene.")

    def obstacle_to_mesh(self, pcd):
        tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(pcd)
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
            pcd, self.alpha_used, tetra_mesh, pt_map
        )
        mesh.compute_vertex_normals()
        o3d.visualization.draw_geometries([mesh])

    def triangle_mesh_to_shape_msgs(self, o3d_mesh):
        ros_mesh = Mesh()

        vertices = np.asarray(o3d_mesh.vertices)
        triangles = np.asarray(o3d_mesh.triangles)

        for v in vertices:
            p = Point()
            p.x, p.y, p.z = v[0], v[1], v[2]
            ros_mesh.vertices.append(p)

        for t in triangles:
            tri = MeshTriangle()
            tri.vertex_indices[0] = t[0]
            tri.vertex_indices[1] = t[1]
            tri.vertex_indices[2] = t[2]
            ros_mesh.triangles.append(tri)

        return ros_mesh

    def remove_clusters(self, raw_pcd):
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.array(raw_pcd)))
        pcd.estimate_normals()

        labels = np.array(
            pcd.cluster_dbscan(
                eps=self.eps, 
                min_points=self.min_points, 
                print_progress=False
            )
        )

        if len(labels) == 0 or np.all(labels == -1):
            rospy.logwarn("No clusters found.")
            return None

        valid_labels = labels[labels >= 0]
        if len(valid_labels) == 0:
            rospy.logwarn("No valid clusters found.")
            return None
        
        largest_cluster_label = np.bincount(valid_labels).argmax()
        largest_cluster_indices = np.where(labels == largest_cluster_label)[0]
        pcd_largest_cluster = pcd.select_by_index(largest_cluster_indices)

        return pcd_largest_cluster

    def plane_to_mesh(self, pcd):
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=self.ransac_distance_threshold,
            ransac_n=self.ransac_n,
            num_iterations=self.num_iterations
        )
        [a, b, c, d] = plane_model

        pcd_plane = pcd.select_by_index(inliers)

        hull_mesh, _ = pcd_plane.compute_convex_hull()
        hull_mesh.compute_vertex_normals()

        return pcd_plane, hull_mesh
    
    def save_mesh(self, mesh):
        o3d.io.write_triangle_mesh(self.save_mesh_path, mesh)
        rospy.loginfo(f"Mesh saved at {self.save_mesh_path}")

    def reduce_density(self, pcd):
        pcd = pcd.voxel_down_sample(voxel_size=0.05)
        return pcd

    def point_cloud_subscriber(self, msg):
        """
        This callback is only meaningful if we're NOT in read_mesh mode.
        Because if read_mesh == True, we won't build the mesh from the PCD.
        """
        if self.read_mesh:
            return

        rospy.loginfo("Received point cloud.")
        raw_data = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not raw_data:
            rospy.logwarn("No points in the point cloud.")
            return
        
        obstacle = self.remove_clusters(raw_data)
        if obstacle is None:
            return

        obstacle = self.reduce_density(obstacle)
        _, mesh = self.plane_to_mesh(obstacle)

        # Optionally save the mesh if you want to keep it.
        # self.save_mesh(mesh)

        # Publish the newly created mesh
        self.publish_obstacle_mesh(mesh)


if __name__ == "__main__":
    PointCloudToMesh()
    rospy.spin()
