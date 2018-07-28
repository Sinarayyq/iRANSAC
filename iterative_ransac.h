//Sinara YANG


#ifndef ITERATIVE_H
#define ITERATIVE_H


#include <iostream>
#include <math.h>
#include <fstream>
#include <vector>


#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>


#include <pcl/common/geometry.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/ros/conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>


#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry> 

#include <opennurbs.h> 

#include <io.h> 
#include "../server3/Geometry.h"

//openmesh structure
//old
/*struct MyTraits : OpenMesh::DefaultTraits
{
	// Let Point and Normal be a vector of doubles
	typedef OpenMesh::Vec3d Point;
	typedef OpenMesh::Vec3d Normal;

	// Already defined in OpenMesh::DefaultTraits
	// HalfedgeAttributes( OpenMesh::Attributes::PrevHalfedge );

	// Uncomment next line to disable attribute PrevHalfedge
	// HalfedgeAttributes( OpenMesh::Attributes::None );
	//
	// or
	//
	// HalfedgeAttributes( 0 );
};
typedef OpenMesh::PolyMesh_ArrayKernelT<MyTraits>  OpenMesh_Mesh;*/
//new
/*struct MyTraits : OpenMesh::DefaultTraits	//Set Point and Normal to be double precision.
{
	typedef OpenMesh::Vec3d Point;
	typedef OpenMesh::Vec3d Normal;
	VertexTraits
	{
		int patch;	//flag on Vertices. Initialized as 0. i for the i-th patch for the iterative RANSAC process.
	};

};
typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits>  FMesh;	//Use customized traits.
*/

//Output structure
class PatchType
{
public:
	bool plane;      //0 = not a plane, 1 = is a plane. If all three exist, then always Plane.
	bool cylinder;	//Cone is preferred over cylinder. If both exist, then always select Cone.
	bool cone;

	bool plane2;	//This set is for the second run after the patches are formed. The second run only run once on each patch, excluding boundary points.
	bool cylinder2;	//The second run result can be totally different. E.g., in the first run the result is C+C/N (Then eventually output as CN coz N is preferred). Then in the second run, it may become C/N + C, which outputs as NC. This is because the number of sample points has been changed, which affects the inlier criteria.
	bool cone2;

	//double perror;
	//double cyerror;
	//double coerror;
	//int bestfit; //1 for plane, 2 for cylinder, 3 for cone.
	ON_3dPoint BoundaryLines[4];	//boundary generatrix lines (PCA's end points on the start and end point of each patch). [p0_start, p0_end, p1_start, p1_end]
	ON_3dPointArray EndBoundaryPoints;	//If there is next adjacent patch, this list include the points in the boundary region, INCLUDING points from the next patch.
	ON_3dPointArray StartBoundaryPoints;

	pcl::ModelCoefficients::Ptr coefficients_plane;	//4 parameters - a,b,c,d: where a,b,c are the coordinates of plane's normal; d is the fourth Hessian component of the plane's equation.
	pcl::ModelCoefficients::Ptr coefficients_cylinder;	//(patchtype_temp.coefficients_cylinder)->values to get the coefficient. 7 parameters: 0-2: x,y,z of a point located on the cylinder axis; 3-5: x,y,z of the cylinder's axis direction; 6 - cylinder's radius. 
	pcl::ModelCoefficients::Ptr coefficients_cone;	//

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane;	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder;	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cone;

	//For the second run
	pcl::PointCloud<pcl::Normal>::Ptr normal_plane;
	pcl::PointCloud<pcl::Normal>::Ptr normal_cylinder;
	pcl::PointCloud<pcl::Normal>::Ptr normal_cone;

	pcl::ModelCoefficients::Ptr coefficients_plane2;
	pcl::ModelCoefficients::Ptr coefficients_cylinder2;	
	pcl::ModelCoefficients::Ptr coefficients_cone2;	

	PatchType();
	~PatchType() {};
};







//================================dijkstra=====================================

int DijkstraAlgorithm(FMesh *mesh, std::vector<FMesh::Point> &points, FMesh::Point &start_point, std::vector<std::vector<double>> &distance);

int DijkstraAlgorithm(FMesh *mesh, std::vector<FMesh::Point> &points, FMesh::Point &start_point, std::vector<int> &sequence_number);

//================================dijkstra=====================================






//=============================pcl subsampling=================================

int SubsamplingPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &subsampled_points, pcl::PointCloud<pcl::Normal>::Ptr &subsampled_points_normals);

int SubsamplingPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &subsampled_points, double edge_length);

int SubsamplingPCL(FMesh *mesh, std::vector<FMesh::Point> &subsampled_points, double edge_length);

int SubsamplingPCL(FMesh *mesh, std::vector<FMesh::Point> &subsampled_points, std::vector<FMesh::Normal> &subsampled_points_normals);

//=============================pcl subsampling=================================







//==============================iterative_ransac===============================

int OpenMeshMesh2PCLCloudNormal(FMesh *mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals);

int OpenMeshMesh2PCLCloud(FMesh *mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

int OpenMeshVectorPoint2PCLCloud(std::vector<FMesh::Point> &points_openmesh, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

int PCL2OpenMeshVectorNormal(pcl::PointCloud<pcl::Normal>::Ptr &normal, std::vector<FMesh::Normal> &normal_openmesh);

int PCL2OpenMeshVectorPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<FMesh::Point> &points_openmesh);

//int IterativeRansac(FMesh *mesh, std::vector<FMesh::Point> subsampled_points_openmesh, std::vector<FMesh::Normal> subsampled_normal_openmesh);	//all subsample nodes. WIP. CAUTION: may not use normal.
int IterativeRansac(Tooth* surface, float mean_edge_length);	//mean edge length of the mesh. To eliminate the sample points around the boundary area between two adjacent patches (for CATIA optimizer).
int DeleteBoundaryPoints(std::vector<PatchType> &patchtype, float mean_edge_length);
void GetOMeshPointNNormal(FMesh *mesh, std::vector<FMesh::VertexIter> sample_it, std::vector<FMesh::Point>& OpenMeshPointList, std::vector<FMesh::Normal>& OpenMeshNormalList);

int DeleteCandidatePointsFromPoints(std::vector<FMesh::Point> &points, std::vector<FMesh::Point> candidate_points);

int OutputPatchTypeOnTXT(std::vector<PatchType> patchtype, std::string output_file);
bool Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, PatchType &patchtype_temp, bool second_run);	//second_run=0 for the Iterative RANSAC. second_run=1 for the conservative run.
int Second_Ransac(std::vector<PatchType>& patchtype);	//NOT iterative. Only run RANSAC once on each patch, excluding boundary points.
//==============================iterative_ransac===============================








//=====================Fitting 3D Points(on one plane) To a Line=====================

//flag_head_tail == 0: head and tail are the projection of the first two points in points_list
//flag_head_tail == 1: head and tail are the projection of the head and tail points in points_list
//fitting_mode == 0: 2d PCA
//fitting_mode == 1: Linear Regression(Perpendicular least squares fitting)
//fitting_mode == 2: Linear Regression(Vertical least squares fitting)	//don't use this.
int Fitting3DPointsToLine2D(ON_3dPointArray &points_opennurbs, ON_3dVector normal, double &error, ON_3dPoint &head, ON_3dPoint &tail, bool flag_head_tail, int fitting_mode);
pcl::PointCloud<pcl::PointXYZ>::Ptr OpennurbsArray2PointCloud(ON_3dPointArray &points_opennurbs);
std::vector<double> OpennurbsVector2VectorDouble(ON_3dVector &normal);
pcl::PointXYZ OpennurbsPoint2PCLPoint(ON_3dPoint &point_opennurbs);
int Fitting3DPointsToLine2D(pcl::PointCloud<pcl::PointXYZ>::Ptr &points_list, std::vector<double> plane_normal, double &error,
	pcl::PointXYZ &head, pcl::PointXYZ &tail, bool flag_head_tail, int fitting_mode);
//=====================Fitting 3D Points(on one plane) To a Line=====================










//
////============================PCL visualizer================================
//
//enum camera_position { xy, yz, xz };
//
//boost::shared_ptr<pcl::visualization::PCLVisualizer> visCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_visualize, std::string window_label, camera_position camera_pos);
//void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, std::string label_viewer_window, camera_position camera_pos);
//
//void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string viewer_window_label, camera_position camera_pos);
//
//boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals, std::string window_label, camera_position camera_pos);
//void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, std::string viewer_window_label, camera_position camera_pos);
//
////============================PCL visualizer================================





#endif