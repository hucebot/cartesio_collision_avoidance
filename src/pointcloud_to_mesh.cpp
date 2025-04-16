#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/ApplyPlanningSceneRequest.h>
#include <moveit_msgs/ApplyPlanningSceneResponse.h>
#include <geometry_msgs/Point.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/IO.h>
#include <open3d/io/TriangleMeshIO.h>
#include <fstream>
#include <unordered_map>
#include <string>
#include <vector>

class PointCloudToMesh
{
public:
    PointCloudToMesh(ros::NodeHandle& nh)
    {
        ros::NodeHandle pnh("~");
        pnh.param<bool>("read_mesh", read_mesh_, false);
        cloud_sub_ = nh.subscribe("/vlm_seg/point_cloud", 1, &PointCloudToMesh::pointCloudCallback, this);
        save_mesh_path_ = "/ros_ws/src/cartesio_collision_avoidance/meshes/obstacle.stl";
        alpha_used_ = 0.037;
        eps_ = 0.1;
        min_points_ = 10;
        ransac_distance_threshold_ = 0.01;
        ransac_n_ = 10;
        num_iterations_ = 10;
        std::string srv_name = "/cartesian/collision_avoidance/apply_planning_scene";
        ros::service::waitForService(srv_name);
        apply_planning_scene_client_ = nh.serviceClient<moveit_msgs::ApplyPlanningScene>(srv_name);
        if (read_mesh_) readAndPublishMesh();
    }

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        if (read_mesh_) return;
        open3d::geometry::PointCloud cloud = rosMsgToOpen3d(*msg);
        if (cloud.points_.empty()) return;
        auto largest_cluster = removeLargestCluster(cloud);
        if (!largest_cluster || largest_cluster->points_.empty()) return;
        largest_cluster = largest_cluster->VoxelDownSample(0.05);
        auto mesh = planeToMesh(*largest_cluster);
        if (!mesh) return;
        publishObstacleMesh(*mesh);
        open3d::io::WriteTriangleMesh(save_mesh_path_, *mesh, true);
    }

    open3d::geometry::PointCloud rosMsgToOpen3d(const sensor_msgs::PointCloud2& ros_cloud)
    {
        open3d::geometry::PointCloud cloud;
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(ros_cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(ros_cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(ros_cloud, "z");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) continue;
            cloud.points_.emplace_back(
                Eigen::Vector3d(static_cast<double>(*iter_x),
                                static_cast<double>(*iter_y),
                                static_cast<double>(*iter_z)));
        }
        return cloud;
    }

    std::shared_ptr<open3d::geometry::PointCloud> removeLargestCluster(const open3d::geometry::PointCloud& input_pcd)
    {
        auto pcd_ptr = std::make_shared<open3d::geometry::PointCloud>(input_pcd);
        pcd_ptr->EstimateNormals();
        std::vector<int> labels = pcd_ptr->ClusterDBSCAN(eps_, min_points_, false);
        if (labels.empty()) return nullptr;
        std::unordered_map<int, int> label_counts;
        for (auto lbl : labels) if (lbl >= 0) label_counts[lbl]++;
        if (label_counts.empty()) return nullptr;
        int largest_label = -1;
        int max_count = 0;
        for (auto& kv : label_counts) if (kv.second > max_count) { largest_label = kv.first; max_count = kv.second; }
        std::vector<size_t> indices;
        indices.reserve(max_count);
        for (size_t i = 0; i < labels.size(); ++i) if (labels[i] == largest_label) indices.push_back(i);
        return pcd_ptr->SelectByIndex(indices);
    }

    std::shared_ptr<open3d::geometry::TriangleMesh> planeToMesh(const open3d::geometry::PointCloud& pcd)
    {
        if (pcd.points_.empty()) return nullptr;
        auto pcd_ptr = std::make_shared<open3d::geometry::PointCloud>(pcd);
        Eigen::Vector4d plane_model;
        std::vector<size_t> inliers;
        std::tie(plane_model, inliers) = pcd_ptr->SegmentPlane(ransac_distance_threshold_, ransac_n_, num_iterations_);
        if (inliers.empty()) return nullptr;
        auto plane_pcd = pcd_ptr->SelectByIndex(inliers);
        std::shared_ptr<open3d::geometry::TriangleMesh> hull_mesh;
        std::vector<size_t> hull_indices;
        std::tie(hull_mesh, hull_indices) = plane_pcd->ComputeConvexHull();
        if (!hull_mesh) return nullptr;
        hull_mesh->ComputeVertexNormals();
        return hull_mesh;
    }

    shape_msgs::Mesh triangleMeshToShapeMsg(const open3d::geometry::TriangleMesh& o3d_mesh)
    {
        shape_msgs::Mesh ros_mesh;
        ros_mesh.vertices.resize(o3d_mesh.vertices_.size());
        ros_mesh.triangles.resize(o3d_mesh.triangles_.size());
        for (size_t i = 0; i < o3d_mesh.vertices_.size(); ++i)
        {
            geometry_msgs::Point p;
            p.x = o3d_mesh.vertices_;
            p.y = o3d_mesh.vertices_;
            p.z = o3d_mesh.vertices_;
            ros_mesh.vertices[i] = p;
        }
        for (size_t i = 0; i < o3d_mesh.triangles_.size(); ++i)
        {
            shape_msgs::MeshTriangle tri;
            tri.vertex_indices);
            tri.vertex_indices);
            tri.vertex_indices);
            ros_mesh.triangles[i] = tri;
        }
        return ros_mesh;
    }

    void publishObstacleMesh(const open3d::geometry::TriangleMesh& o3d_mesh)
    {
        moveit_msgs::CollisionObject co;
        co.id = "obstacle_from_pcd";
        co.header.frame_id = "world";
        co.operation = moveit_msgs::CollisionObject::ADD;
        co.meshes.push_back(triangleMeshToShapeMsg(o3d_mesh));
        moveit_msgs::ApplyPlanningScene srv;
        srv.request.scene.is_diff = true;
        srv.request.scene.world.collision_objects.push_back(co);
        apply_planning_scene_client_.call(srv);
    }

    void readAndPublishMesh()
    {
        std::ifstream f(save_mesh_path_.c_str());
        if (!f.good()) return;
        auto o3d_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
        if (!open3d::io::ReadTriangleMesh(save_mesh_path_, *o3d_mesh)) return;
        if (o3d_mesh->triangles_.empty() || o3d_mesh->vertices_.empty()) return;
        publishObstacleMesh(*o3d_mesh);
    }

    ros::Subscriber cloud_sub_;
    ros::ServiceClient apply_planning_scene_client_;
    bool read_mesh_;
    std::string save_mesh_path_;
    double alpha_used_, eps_, ransac_distance_threshold_;
    int min_points_, ransac_n_, num_iterations_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_outlier_removal_cpp");
    ros::NodeHandle nh;
    PointCloudToMesh node(nh);
    ros::spin();
    return 0;
}
