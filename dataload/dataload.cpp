#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/console/parse.h>
#include <fstream>
#include <pcl/registration/correspondence_estimation.h>

int main(int argc, char** argv) {
    // 检查命令行参数数量
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <source_ply_file> <target_ply_file>" << std::endl;
        return -1;
    }

    // 读取源点云和目标点云的文件路径
    std::string source_ply = argv[1];
    std::string target_ply = argv[2];

    // 读取点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(source_ply, *cloud_source) == -1) {
        PCL_ERROR("Couldn't read file %s \n", source_ply.c_str());
        return (-1);
    }
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(target_ply, *cloud_target) == -1) {
        PCL_ERROR("Couldn't read file %s \n", target_ply.c_str());
        return (-1);
    }

    // 估计法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_source);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_source_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.03);
    ne.compute(*cloud_source_normals);

    // 提取FPFH特征描述符
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud_source);
    fpfh.setInputNormals(cloud_source_normals);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_source_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfh.setRadiusSearch(0.05);
    fpfh.compute(*cloud_source_fpfh);

    // 匹配特征描述符
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> ce;
    ce.setInputSource(cloud_source); // 使用 cloud_source 而不是 source_ply
    ce.setInputTarget(cloud_target); // 使用 cloud_target 而不是 target_ply
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    ce.determineReciprocalCorrespondences(*correspondences);

    // 保存对应关系文件
    std::ofstream file;
    file.open ("correspondences.txt");
    for (const auto& correspondence : *correspondences) {
        file << correspondence.index_query << " " << correspondence.index_match << std::endl;
    }
    file.close();

    // 使用ICP估计变换矩阵
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    if (icp.hasConverged()) {
        std::cout << "ICP has converged, score is " << icp.getFitnessScore() << std::endl;
        std::cout << "ICP transformation: \n" << icp.getFinalTransformation() << std::endl;

        // 保存变换矩阵到文件
        Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();
        std::ofstream matrix_file;
        matrix_file.open ("transformation_matrix.txt");
        for (int i = 0; i < transformation_matrix.rows(); ++i) {
            for (int j = 0; j < transformation_matrix.cols(); ++j) {
                matrix_file << transformation_matrix(i, j) << " ";
            }
            matrix_file << std::endl;
        }
        matrix_file.close();
    } else {
        std::cout << "ICP did not converge." << std::endl;
    }

    return 0;
}
