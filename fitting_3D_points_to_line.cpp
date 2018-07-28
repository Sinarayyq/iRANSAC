#include "fitting_3D_points_to_line.h"



using namespace std;



//PCA 
int FindBestFittingLine(const std::vector<Eigen::Vector3d> & c, Eigen::Vector3d *centroid, Eigen::Vector3d *axis)
{
	// copy coordinates to  matrix in Eigen format
	size_t num_atoms = c.size();
	if (num_atoms == 0)
	{
		std::cout << "No points to fit." << std::endl;
		return -1;
	}
	Eigen::Matrix< Eigen::Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic > centers(num_atoms, 3);
	for (size_t i = 0; i < num_atoms; ++i) 
		centers.row(i) = c[i];
	
	
	//centroid质心
	(*centroid) = centers.colwise().mean();                  

	//transpose转置阵
	Eigen::MatrixXd centered = centers.rowwise() - (*centroid).transpose(); 
	

	//adjoint共轭矩阵																	  
	Eigen::MatrixXd cov = centered.adjoint() * centered;  
	

	//SelfAdjointEigenSolver:Computes eigenvalues and eigenvectors of selfadjoint matrices
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
	/*std::cout << "centers:" << std::endl << centers << std::endl;
	std::cout << "centroid:" << std::endl << centroid << std::endl;
	std::cout << "centroid.transpose():" << std::endl << centroid.transpose() << std::endl;
	std::cout << "centered:" << std::endl << centered << std::endl;
	std::cout << "centered.adjoint():" << std::endl << centered.adjoint() << std::endl;
	std::cout << "cov:" << std::endl << cov << std::endl;
	std::cout << "eig.eigenvectors():" << std::endl << eig.eigenvectors() << std::endl;
	std::cout << "eig.eigenvectors().col(2):" << std::endl << eig.eigenvectors().col(2) << std::endl;*/

	//eigenvectors特征向量
	(*axis) = eig.eigenvectors().col(2).normalized();       
	std::cout << "centroid:" << std::endl << *centroid << std::endl;
	std::cout << "axis:" << std::endl << *axis << std::endl;

	return 0;
}

std::vector<Eigen::Vector3d> ProjectPointsOntoLine(std::vector<Eigen::Vector3d> points, Eigen::Vector3d centroid, Eigen::Vector3d axis)
{
	std::vector<Eigen::Vector3d> points_projected;
	size_t size = points.size();
	Eigen::Vector3d points_eigen, ap, a = centroid, b = centroid + axis, ab = b - a;
	
	for (size_t i = 0; i < size; i++)
	{
		ap = points[i] - a;
		points_projected.push_back(a + ap.dot(ab) / ab.dot(ab) * ab);
		/*std::cout << "points[" << i << "]:" << std::endl << points[i] << std::endl
			<< "points_projected[" << i << "]:" << std::endl << points_projected[i] << std::endl;*/

	}
	return points_projected;
}

double ComputeDistanceOfTwoPoints(std::vector<Eigen::Vector3d> points, std::vector<Eigen::Vector3d> points_projected)
{
	double sum = 0;
	double error, maxerror = 0;
	size_t size = points.size();
	Eigen::Vector3d pp;
	for (size_t i = 0; i < size; i++)
	{
		pp = points[i] - points_projected[i];
		error = sqrt(pp.dot(pp));
		sum += error;		
		if (error > maxerror)
			maxerror = error;			
	}
	//return sum / size;	//mean error
	return maxerror;
}

double ComputeDistanceOfTwoPointsByLeastSquares(std::vector<Eigen::Vector3d> points, std::vector<Eigen::Vector3d> points_projected)
{
	double sum = 0;
	size_t size = points.size();
	Eigen::Vector3d pp;
	for (size_t i = 0; i < size; i++)
	{
		pp = points[i] - points_projected[i];
		sum += pp.dot(pp);
	}
	return sqrt(sum);
}


int Fitting3DPointsToLine(ON_3dPointArray& points_opennurbs, double &error, ON_3dPoint &head, ON_3dPoint &tail)
{
	size_t size_points = points_opennurbs.Count();

	if (size_points == 0)
	{
		std::cout << "No points to fit." << std::endl;
		return -1;
	}
	std::vector<Eigen::Vector3d> points;
	for (size_t i = 0; i < size_points; i++)
	{
		points.push_back(Eigen::Vector3d(points_opennurbs[i].x, points_opennurbs[i].y, points_opennurbs[i].z));
	}
	
	
	/*
	points.push_back(Eigen::Vector3d(0, 1, 2));
	points.push_back(Eigen::Vector3d(0, 3, 4));
	points.push_back(Eigen::Vector3d(0, 5, 6));

	points.push_back(Eigen::Vector3d(0, 100, 110));
	points.push_back(Eigen::Vector3d(0, 50, 70));
	points.push_back(Eigen::Vector3d(0, 200, 180));

	points.push_back(Eigen::Vector3d(0, -10, -15));
	points.push_back(Eigen::Vector3d(0, -20, -15));
	points.push_back(Eigen::Vector3d(0, -50, -60));
	points.push_back(Eigen::Vector3d(0, 4, 7));*/
	
	Eigen::Vector3d centroid, axis;
	FindBestFittingLine(points, &centroid, &axis);
	std::vector<Eigen::Vector3d> points_projected = ProjectPointsOntoLine(points, centroid, axis);
	double mean_distance = ComputeDistanceOfTwoPoints(points, points_projected);
	//double error_least_squares = ComputeDistanceOfTwoPointsByLeastSquares(points, points_projected);

	error = mean_distance;
	//std::cout << "error:" << *error << std::endl;

	//ON_3dPoint head, tail;
	head.x = points_projected[0].x();
	head.y = points_projected[0].y();
	head.z = points_projected[0].z();
	//tail.x = points_projected[size_points - 1].x();
	//tail.y = points_projected[size_points - 1].y();
	//tail.z = points_projected[size_points - 1].z();

	//use the second point as the tail. For multiple segment intersection. 20180427.
	tail.x = points_projected[1].x();
	tail.y = points_projected[1].y();
	tail.z = points_projected[1].z();


	return 0;

}


