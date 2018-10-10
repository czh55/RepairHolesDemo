//https://blog.csdn.net/aileennut/article/details/74857595
#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/boundary.h>
#include <pcl/common/transforms.h>  //allows us to use pcl::transformPointCloud function
#include <pcl/io/ply_io.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


int main() {
	PointCloud::Ptr cloud_src_o(new PointCloud);//原点云
	pcl::io::loadPLYFile("bunny.ply", *cloud_src_o);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZ>);
	//计算法线
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);

	tree->setInputCloud(cloud_src_o);
	normEst.setInputCloud(cloud_src_o);
	normEst.setSearchMethod(tree);
	normEst.setKSearch(20);
	normEst.compute(*normals);
	//判断边缘点
	pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
	tree2->setInputCloud(cloud_src_o);
	boundEst.setInputCloud(cloud_src_o);
	boundEst.setInputNormals(normals);
	boundEst.setSearchMethod(tree2);
	boundEst.setKSearch(20);
	boundEst.setAngleThreshold(M_PI / 2);
	boundEst.compute(boundaries);
	//提取边缘点重组点云
	cloud_b->width = cloud_src_o->points.size();
	cloud_b->height = 1;
	cloud_b->points.resize(cloud_b->width*cloud_b->height);
	int j = 0;
	for (int i = 0; i < cloud_src_o->points.size(); i++)
	{
		if (boundaries.points[i].boundary_point != 0)
		{
			cloud_b->points[j].x = cloud_src_o->points[i].x;
			cloud_b->points[j].y = cloud_src_o->points[i].y;
			cloud_b->points[j].z = cloud_src_o->points[i].z;
			j++;
		}
		continue;
	}
	cloud_b->width = j;
	cloud_b->points.resize(cloud_b->width*cloud_b->height);

	// Create a PCLVisualizer object
	pcl::visualization::PCLVisualizer viewer("registration Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> srcb_h(cloud_b, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(cloud_src_o, 0, 0, 255);
	viewer.addPointCloud(cloud_src_o, src_h, "src cloud");//蓝
	viewer.addPointCloud(cloud_b, srcb_h, "srcb cloud");//红
	
	//viewer.addCoordinateSystem(1.0);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}