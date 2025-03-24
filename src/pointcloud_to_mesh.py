#!/usr/bin/env python3
import rospy
import numpy as np
import struct
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import open3d as o3d
import os

o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)

class PointCloudToMesh:
    def __init__(self):
        rospy.init_node("pointcloud_outlier_removal", anonymous=True)

        self.obstacle_subscriber = rospy.Subscriber("/vlm_seg/point_cloud", PointCloud2, self.point_cloud_subscriber)

        self.save_mesh_path = "/ros_ws/src/cartesio_collision_avoidance/meshes/obstacle.stl"

        if not os.path.exists(os.path.dirname('/ros_ws/src/cartesio_collision_avoidance/meshes/')):
            os.makedirs(os.path.dirname('/ros_ws/src/cartesio_collision_avoidance/meshes/'))
            
        self.alpha_used = 0.037
        self.eps = 0.1
        self.min_points = 10 
 
        self.ransac_distance_threshold = 0.01
        self.ransac_n = 10
        self.num_iterations = 10 

    def obstacle_to_mesh(self, pcd):
        tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(pcd)
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
            pcd, self.alpha_used, tetra_mesh, pt_map
        )
        mesh.compute_vertex_normals()
        o3d.visualization.draw_geometries([mesh])

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

        #o3d.visualization.draw_geometries([hull_mesh])

        return pcd_plane, hull_mesh
    
    def save_mesh(self, mesh):
        o3d.io.write_triangle_mesh(self.save_mesh_path, mesh)
        rospy.loginfo(f"Mesh saved at {self.save_mesh_path}")

    def reduce_density(self, pcd):
        pcd = pcd.voxel_down_sample(voxel_size=0.05)
        return pcd

    def point_cloud_subscriber(self, msg):
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
        self.save_mesh(mesh)


if __name__ == "__main__":
    PointCloudToMesh()
    rospy.spin()
