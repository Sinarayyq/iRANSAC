//Sinara YANG
//This functions are used to read the parameters from a txt file and to set the parameters of the algorithm.

//#pragma once //If it is not added there is an error. It could disappear whenever I use this header (?)

// This is start of the header guard.  READ_PARAMETERS_H can be any unique name.  
// By convention, we use the name of the header file.
#ifndef IO_H
#define IO_H

#include <string>
#include <vector>
#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


//extern std::string PATH_PCD_DOC;
extern int PLANE_METHOD_TYPE;
extern int CYL_METHOD_TYPE;
extern int CONE_METHOD_TYPE;
extern double PLANE_TOL;
extern double CYL_TOL;
extern double CONE_TOL;
extern int PLANE_MAX_NUM_ITER;
extern int CYL_MAX_NUM_ITER;
extern int CONE_MAX_NUM_ITER;
extern double CYL_WEIGHT_NORMAL_DISTANCE;
extern double CONE_WEIGHT_NORMAL_DISTANCE;
extern double CYL_MIN_RADIUS_LIMIT;
extern double CYL_MAX_RADIUS_LIMIT;
extern double CONE_MIN_RADIUS_LIMIT;
extern double CONE_MAX_RADIUS_LIMIT;
extern double CONE_MIN_OPENING_ANGLE;
extern double CONE_MAX_OPENING_ANGLE;
extern double PERCENTAGE;

extern const int INF;

extern std::string MASTER_FOLDER;
extern std::string INPUT_PATH;
extern std::string OUTPUT_PATH;
extern std::string FILE_NAME;
extern double remesh_parameter;

// It reads and sets the method parameters from a txt file 
// INPUT: the path corresponding to the txt file
// OUTPUT: it returns true if the file contains the correct number of parameters (21), false otherwise
bool readParameterFile(std::string parameterTxtFilePath);


#endif