#include "io.h"

//std::string PATH_PCD_DOC;
int PLANE_METHOD_TYPE;
int CYL_METHOD_TYPE;
int CONE_METHOD_TYPE;
double PLANE_TOL;
double CYL_TOL;
double CONE_TOL;
int PLANE_MAX_NUM_ITER;
int CYL_MAX_NUM_ITER;
int CONE_MAX_NUM_ITER;
double CYL_WEIGHT_NORMAL_DISTANCE;
double CONE_WEIGHT_NORMAL_DISTANCE;
double CYL_MIN_RADIUS_LIMIT;
double CYL_MAX_RADIUS_LIMIT;
double CONE_MIN_RADIUS_LIMIT;
double CONE_MAX_RADIUS_LIMIT;
double CONE_MIN_OPENING_ANGLE;
double CONE_MAX_OPENING_ANGLE;
double PERCENTAGE;

const int INF = std::numeric_limits<int>::max();

std::string MASTER_FOLDER;
std::string INPUT_PATH;
std::string OUTPUT_PATH;
std::string FILE_NAME;
double remesh_parameter;

void setParameters(std::vector<std::string>& parameters)
{
	for (std::size_t i = 0; i < parameters.size(); ++i)
	{
		switch (i)
		{
			/*case 0: {PATH_PCD_DOC = parameters[i]; }
			break;*/
		case 0: {
			if (std::stoi(parameters[i]) == 0)
			{
				PLANE_METHOD_TYPE = pcl::SAC_RANSAC;
			}
			else if (std::stoi(parameters[i]) == 1)
			{
				PLANE_METHOD_TYPE = pcl::SAC_PROSAC;
			}
			else if (std::stoi(parameters[i]) == 2)
			{
				PLANE_METHOD_TYPE = pcl::SAC_LMEDS;
			}
			else if (std::stoi(parameters[i]) == 3)
			{
				PLANE_METHOD_TYPE = pcl::SAC_MSAC;
			}
			else if (std::stoi(parameters[i]) == 4)
			{
				PLANE_METHOD_TYPE = pcl::SAC_RRANSAC;
			}
			else if (std::stoi(parameters[i]) == 5)
			{
				PLANE_METHOD_TYPE = pcl::SAC_RMSAC;
			}
			else if (std::stoi(parameters[i]) == 6)
			{
				PLANE_METHOD_TYPE = pcl::SAC_MLESAC;
			}
			else
			{
				PLANE_METHOD_TYPE = INF;
			}
		}
				break;
		case 1: {
			if (std::stoi(parameters[i]) == 0)
			{
				CYL_METHOD_TYPE = pcl::SAC_RANSAC;
			}
			else if (std::stoi(parameters[i]) == 1)
			{
				CYL_METHOD_TYPE = pcl::SAC_PROSAC;
			}
			else if (std::stoi(parameters[i]) == 2)
			{
				CYL_METHOD_TYPE = pcl::SAC_LMEDS;
			}
			else if (std::stoi(parameters[i]) == 3)
			{
				CYL_METHOD_TYPE = pcl::SAC_MSAC;
			}
			else if (std::stoi(parameters[i]) == 4)
			{
				CYL_METHOD_TYPE = pcl::SAC_RRANSAC;
			}
			else if (std::stoi(parameters[i]) == 5)
			{
				CYL_METHOD_TYPE = pcl::SAC_RMSAC;
			}
			else if (std::stoi(parameters[i]) == 6)
			{
				CYL_METHOD_TYPE = pcl::SAC_MLESAC;
			}
			else
			{
				CYL_METHOD_TYPE = INF;
			}
		}
				break;
		case 2: {
			if (std::stoi(parameters[i]) == 0)
			{
				CONE_METHOD_TYPE = pcl::SAC_RANSAC;
			}
			else if (std::stoi(parameters[i]) == 1)
			{
				CONE_METHOD_TYPE = pcl::SAC_PROSAC;
			}
			else if (std::stoi(parameters[i]) == 2)
			{
				CONE_METHOD_TYPE = pcl::SAC_LMEDS;
			}
			else if (std::stoi(parameters[i]) == 3)
			{
				CONE_METHOD_TYPE = pcl::SAC_MSAC;
			}
			else if (std::stoi(parameters[i]) == 4)
			{
				CONE_METHOD_TYPE = pcl::SAC_RRANSAC;
			}
			else if (std::stoi(parameters[i]) == 5)
			{
				CONE_METHOD_TYPE = pcl::SAC_RMSAC;
			}
			else if (std::stoi(parameters[i]) == 6)
			{
				CONE_METHOD_TYPE = pcl::SAC_MLESAC;
			}
			else
			{
				CONE_METHOD_TYPE = INF;
			}
		}
				break;
		case 3: {PLANE_TOL = std::stod(parameters[i]); }
				break;
		case 4: {CYL_TOL = std::stod(parameters[i]); }
				break;
		case 5: {CONE_TOL = std::stod(parameters[i]); }
				break;
		case 6: {PLANE_MAX_NUM_ITER = std::stoi(parameters[i]); }
				break;
		case 7: {CYL_MAX_NUM_ITER = std::stoi(parameters[i]); }
				 break;
		case 8: {CONE_MAX_NUM_ITER = std::stoi(parameters[i]); }
				 break;
		case 9: {CYL_WEIGHT_NORMAL_DISTANCE = std::stod(parameters[i]); }
				 break;
		case 10: {CONE_WEIGHT_NORMAL_DISTANCE = std::stod(parameters[i]); }
				 break;
		case 11: {CYL_MIN_RADIUS_LIMIT = std::stod(parameters[i]); }
				 break;
		case 12: {CYL_MAX_RADIUS_LIMIT = std::stod(parameters[i]); }
				 break;
		case 13: {CONE_MIN_RADIUS_LIMIT = std::stod(parameters[i]); }
				 break;
		case 14: {CONE_MAX_RADIUS_LIMIT = std::stod(parameters[i]); }
				 break;
		case 15: {CONE_MIN_OPENING_ANGLE = std::stod(parameters[i]); }
				 break;
		case 16: {CONE_MAX_OPENING_ANGLE = std::stod(parameters[i]); }
				 break;
		case 17: {PERCENTAGE = std::stod(parameters[i]); }
				 break;
		case 18: {remesh_parameter = std::stod(parameters[i]); }
				 break;

		}
	}
}

bool readParameterFile(std::string parameterTxtFilePath)
{
	std::ifstream infile(parameterTxtFilePath);
	std::string line;
	std::vector<std::string> parameters;
	const size_t num_parameters = 19;
	//size_t n_lines = 0;

	while (std::getline(infile, line))
	{
		if (line[0] != '#')
		{
			parameters.push_back(line);
			//std::istringstream iss(line);
			//int a, b;
			//if (!(iss >> a >> b)) { break; } // error

			// process pair (a,b)
		}
		//++n_lines;
	}

	////Print parameters in cmd window:
	//for (std::size_t i = 0; i < parameters.size(); ++i)
	//{
	//	std::cerr << parameters[i] << std::endl;
	//}
	//std::cerr << "numero di paramentri: " << parameters.size() << std::endl;

	//Check if the parameter file contains the right number of parameters
	if (parameters.size() != num_parameters)
	{
		std::cerr << "ERROR in the text file of parameters!" << std::endl;
		return false;
	}
	//std::cerr << "The parameter file is correct." << std::endl;
	setParameters(parameters);
	return true;
}
