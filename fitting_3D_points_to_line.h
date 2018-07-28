//Sinara YANG
//This functions are used to fit 3D points to a line and get sum of distances between points and the fitting line.


#pragma once

#ifndef FITTING_3D_POINTS_TO_LINE_H
#define FITTING_3D_POINTS_TO_LINE_H



#include<iostream>
#include<fstream>
#include<iomanip>
#include<string>
#include<vector>
#include<Eigen/Dense>
#include<Eigen/Core>
#include<math.h>
#include<opennurbs.h>

//int Fitting3DPointsToLine(std::vector<ON_3dPoint> points_opennurbs, double &error, ON_3dPoint &head, ON_3dPoint &tail);
int Fitting3DPointsToLine(ON_3dPointArray& points_opennurbs, double &error, ON_3dPoint &head, ON_3dPoint &tail);

#endif