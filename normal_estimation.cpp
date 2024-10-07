#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h> //法线估计类头文件
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

int main()
{
    //打开点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../table_scene_lms400.pcd",*cloud);

    //创建法线估计向量
    pcl::NormalEstimation <pcl::PointXYZ,pcl::Normal> ne;
    ne.setInputCloud(cloud);
    //创建一个空的Kdtree，将他传递给法线估计向量
    //即如给出的输入数据集，Kdtree将被建立
    pcl::search::KdTree <pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);//存储输出的数据
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    //查询点周围3厘米内邻近的所有元素
    ne.setRadiusSearch(0.03);
    //计算特征值
    ne.compute(*cloud_normals);
    //储存特征值为点云
    pcl::PCDWriter writer;
    writer.write<pcl::Normal>("../table_cloud_normals.pcd",*cloud_normals,false);//保存文件
    //可视化
    pcl::visualization::PCLVisualizer viewer ("PCL Viewer");
    viewer.setBackgroundColor(0.0,0.0,0.0);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal> (cloud,cloud_normals);//将估计的点云法线添加到画面中

    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}