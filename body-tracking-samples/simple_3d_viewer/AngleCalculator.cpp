#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

// Calculate normal vector of a plane defined by 3 points
Eigen::Vector3d CalculateNormalVector(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3)
{
	Eigen::Vector3d v1 = p1 - p2;
	Eigen::Vector3d v2 = p3 - p2;
	return v1.cross(v2).normalized();
}

// Calculate parameter D of equation of a plane (ax + by + cz + D = 0)
double CalculatePlaneD(const Eigen::Vector3d& point, const Eigen::Vector3d& normal)
{
	return -normal.dot(point);
}

// Project a point onto a plane
Eigen::Vector3d ProjectPointOntoPlane(const Eigen::Vector3d& point, const Eigen::Vector3d& normal, double D)
{
	double distance = normal.dot(point) + D;
	return point - distance * normal;
}

// Calculate angle between three points
double CalculateAngle(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, const Eigen::Vector3d& normal)
{
	Eigen::Vector3d v1 = p1 - p2;
	Eigen::Vector3d v2 = p3 - p2;
	// Check if the points are too close or identical
	double v1_length = v1.norm();
	double v2_length = v2.norm();
	if (v1_length < 1e-10 || v2_length < 1e-10) 
	{
		throw std::invalid_argument("Points are too close or identical");
	}
	// Calculate cosine of the angle
	double cos = v1.dot(v2) / (v1_length * v2_length);
	// Deal with floating point precision issues
	cos = std::min(std::max(cos, -1.0), 1.0);
	// Calculate angle with acos
	double angle_rad = std::acos(cos);
	double angle_deg = angle_rad * 180.0 / M_PI;
	// Calculate the angle direction with cross product
	Eigen::Vector3d cross = v1.cross(v2);

	return cross.dot(normal) > 0 ? angle_deg : -angle_deg;
}

// Calculate angle between projected three points
double CalculateProjectedAngle(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, const Eigen::Vector3d& s1, const Eigen::Vector3d& s2, const Eigen::Vector3d& s3)
{
	// Calculate normal vector of the plane defined by the three points
	Eigen::Vector3d normal = CalculateNormalVector(p1, p2, p3);
	// Calculate parameter D of the plane equation
	double D = CalculatePlaneD(p1, normal);
	// Project the points onto the plane
	Eigen::Vector3d ps1 = ProjectPointOntoPlane(s1, normal, D);
	Eigen::Vector3d ps2 = ProjectPointOntoPlane(s2, normal, D);
	Eigen::Vector3d ps3 = ProjectPointOntoPlane(s3, normal, D);
	
	return CalculateAngle(ps1, ps2, ps3, normal);
}