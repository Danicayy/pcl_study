//不仅限于对单一坐标轴的过滤，其实主要就是再一次进行目标坐标轴过滤即可，通过重复使用直通滤波就可以进行三维区间的滤波
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  //生成并填充点云
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)   //填充数据
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before filtering: " << std::endl;   //打印
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;

  // 设置滤波器对象
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);            //设置输入点云
  pass.setFilterFieldName ("z");         //设置过滤时所需要点云类型的Z字段
  pass.setFilterLimits (0.0, 1.0);        //设置在过滤字段的范围
  //pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内 是否保存滤波的限制范围内的点云，默认为false，保存限制范围内点云，true时候是相反。
  pass.filter (*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered

  std::cerr << "Cloud after filtering: " << std::endl;   //打印
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " " 
                        << cloud_filtered->points[i].y << " " 
                        << cloud_filtered->points[i].z << std::endl;

  return (0);
}