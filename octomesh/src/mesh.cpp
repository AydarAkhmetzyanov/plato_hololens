#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/vtk_io.h>
#include <pcl/common/io.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/filters/filter.h>
#include <unistd.h>

ros::Publisher pub;

double NormalSearchRadius=0; //2
float ReductionFactor = 0.5f;
double TriangulationSearchRadius=1.3;
double Mu=16; //20


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  ROS_INFO("Message received");

  ////Convert cloud for pcl
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;

  pcl_conversions::toPCL(*input, cloud_blob);

  pcl::fromPCLPointCloud2(cloud_blob, *cloud);



  if(NormalSearchRadius>0){
      ////SMOOTHING or normal filtering
      // Create a KD-Tree
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree3 (new pcl::search::KdTree<pcl::PointXYZ>);
      // Output has the PointNormal type in order to store the normals calculated by MLS
      pcl::PointCloud<pcl::PointNormal> mls_points;
      // Init object (second point type is for the normals, even if unused)
      pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
      mls.setComputeNormals (true);
      // Set parameters
      mls.setInputCloud (cloud);
      mls.setPolynomialOrder (5);
      mls.setSearchMethod (tree3);
      mls.setSearchRadius (NormalSearchRadius);
      // Reconstruct
      mls.process (mls_points);


      ////remove nan points and normals
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(mls_points, mls_points, indices);
      pcl::removeNaNNormalsFromPointCloud(mls_points, mls_points, indices);
      //pcl::io::savePCDFile ("/home/aydar/bun0-mls.pcd", mls_points);

      ////To mesh generator
      copyPointCloud(mls_points, *cloud);
  }
  


  ////Mesh generator
  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (40);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals


  ////mesh decimilator
  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (TriangulationSearchRadius);

  // Set typical values for the parameters
  gp3.setMu (Mu);
  gp3.setMaximumNearestNeighbors (110);
  gp3.setMaximumSurfaceAngle((M_PI+0.2)/4); // 45 degrees
  gp3.setMinimumAngle((M_PI-0.02)/18); // 10 degrees
  gp3.setMaximumAngle(2*(M_PI+0.1)/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  //std::vector<int> parts = gp3.getPartIDs();
  //std::vector<int> states = gp3.getPointStates();

  

  pcl::io::saveVTKFile("/tmp/mesh.vtk", triangles);

  //boost::shared_ptr<pcl::PolygonMesh> original_mesh = triangles; 

  //pcl::PolygonMesh::Ptr decimated_mesh(new pcl::PolygonMesh()); 
  boost::shared_ptr<pcl::PolygonMesh> decimated_mesh = boost::make_shared<pcl::PolygonMesh> (); 

  //pcl::PolygonMesh out;
  pcl::MeshQuadricDecimationVTK mesh_decimator; 
  //pcl::PolygonMesh::Ptr inptr = &triangles;
  //pcl::PolygonMesh::ConstPtr inp_ptr(&triangles); 
  boost::shared_ptr<pcl::PolygonMesh> original_mesh(&triangles);

  mesh_decimator.setInputMesh(original_mesh);
  mesh_decimator.setTargetReductionFactor(ReductionFactor);
  mesh_decimator.process(*decimated_mesh);

  pcl::io::saveVTKFile("/tmp/mesh_decimal.vtk", *decimated_mesh);

  //ROS_INFO("File saved 2");
  //sensor_msgs::PointCloud2 output;

  //output = *input;

  //pub.publish (output);

  pcl_msgs::PolygonMesh messageout;
  pcl_conversions::fromPCL(*decimated_mesh, messageout);
  pub.publish (messageout);

  ROS_INFO("mesh sent, sleeping 3 sec and exiting");
  usleep(1000000);
  exit(0);
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "octolens");
  ros::NodeHandle nh;

  if(nh.getParam("normalsearchradius", NormalSearchRadius)){
    ROS_INFO("loaded");
  }

  nh.getParam("ReductionFactor", ReductionFactor);
  nh.getParam("TriangulationSearchRadius", TriangulationSearchRadius);
  nh.getParam("Mu", Mu);

  ros::Subscriber sub = nh.subscribe ("/voxel_grid/output", 1, cloud_cb);

  pub = nh.advertise<pcl_msgs::PolygonMesh> ("/octolens/mesh", 1);
  

  ros::spin ();
}