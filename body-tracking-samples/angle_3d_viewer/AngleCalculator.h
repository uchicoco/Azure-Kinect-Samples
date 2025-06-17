#pragma once

#include <Eigen/Dense>

/**
 * Calculate normal vector of a plane defined by 3 points
 * 
 * @param p1 First point on the plane
 * @param p2 Second point on the plane
 * @param p3 Third point on the plane
 * @return Normalized normal vector of the plane
 */
Eigen::Vector3d CalculateNormalVector(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3);

/**
 * Calculate parameter D of equation of a plane (ax + by + cz + D = 0)
 * 
 * @param point A point on the plane
 * @param normal The normal vector of the plane
 * @return The D parameter value
 */
double CalculatePlaneD(const Eigen::Vector3d& point, const Eigen::Vector3d& normal);

/**
 * Project a point onto a plane
 * 
 * @param point The point to project
 * @param normal The normal vector of the plane
 * @param D The D parameter of the plane equation
 * @return The projected point on the plane
 */
Eigen::Vector3d ProjectPointOntoPlane(const Eigen::Vector3d& point, const Eigen::Vector3d& normal, double D);

/**
 * Calculate angle between three points
 * 
 * @param p1 First point
 * @param p2 Center point (vertex of the angle)
 * @param p3 Third point
 * @param normal Normal vector for determining angle sign
 * @return Angle in degrees (signed)
 * @throws std::invalid_argument if points are too close or identical
 */
double CalculateAngle(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, const Eigen::Vector3d& normal);

/**
 * Calculate angle between projected three points
 * 
 * @param p1 First point defining the projection plane
 * @param p2 Second point defining the projection plane
 * @param p3 Third point defining the projection plane
 * @param s1 First point to be projected and used for angle calculation
 * @param s2 Second point to be projected (vertex of the angle)
 * @param s3 Third point to be projected and used for angle calculation
 * @return Angle in degrees (signed)
 */
double CalculateProjectedAngle(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, 
                               const Eigen::Vector3d& s1, const Eigen::Vector3d& s2, const Eigen::Vector3d& s3);
