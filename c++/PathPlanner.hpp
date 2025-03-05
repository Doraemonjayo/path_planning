/**
 * @file PathPlanner.hpp
 * @author Doraemonjayo
 * @brief 
 * @version 1.0
 * @date 2025-03-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#define _USE_MATH_DEFINES
#include <cmath>
#include <tuple>
#include <vector>
#include <limits>
#include <iostream>
#include <array>
#include <algorithm>
#include <memory>

namespace PathPlanner
{
    double positive_fmod(double x, double y);

    struct Vector2d
    {
        double x, y;

        Vector2d(double x = 0, double y = 0);

        Vector2d operator+(const Vector2d &other) const;
        Vector2d operator-(const Vector2d &other) const;
        Vector2d operator*(double scalar) const;
        Vector2d operator/(double scalar) const;

        double dot(const Vector2d &other) const;
        double norm() const;
        Vector2d normalized() const;
    };

    Vector2d operator*(double scalar, const Vector2d &vector);

    struct pointAndVelocity
    {
        pointAndVelocity(Vector2d point = Vector2d(), double velocity = 0, double angle = 0);
        Vector2d point;
        double velocity;
        double angle;
    };

    class BasePath
    {
    public:
        virtual ~BasePath() = default;

        virtual Vector2d timeToPoint(double time) const = 0;
        virtual Vector2d timeToVelocity(double time) const = 0;
        virtual Vector2d timeToAcceleration(double time) const = 0;
        virtual double pointToTime(const Vector2d &point) const = 0;

        double getDuration() const;
        double getAngle() const;

    protected:
        double duration = 0;
        double angle = 0;
    };

    class StraightPath : public BasePath
    {
    public:
    StraightPath(const Vector2d &startPoint, const Vector2d &endPoint, double startVelocity, double endVelocity, double angle);

        Vector2d timeToPoint(double time) const override;
        Vector2d timeToVelocity(double time) const override;
        Vector2d timeToAcceleration(double time) const override;
        double pointToTime(const Vector2d &point) const override;

    private:
        Vector2d startPoint, endPoint, vector, unitVector;
        double startVelocity, endVelocity, angle, length, acceleration;
    };

    class CornerPath : public BasePath
    {
    public:
        CornerPath(const Vector2d &cornerPoint, const Vector2d &startVector, const Vector2d &endVector,double velocity, double acceleration, double angle);

        Vector2d timeToPoint(double time) const override;
        Vector2d timeToVelocity(double time) const override;
        Vector2d timeToAcceleration(double time) const override;
        double pointToTime(const Vector2d &point) const override;
        double getCutLength() const;

    private:
        Vector2d cornerPoint, startVector, endVector, centerPoint;
        double velocity, acceleration, angle, startAngle, endAngle, deltaAngle, radius, cutLength, arcStartAngle;
    };

    class PointPath : public BasePath
    {
    public:
        PointPath(const Vector2d &point, const Vector2d &velocity, double angle);

        Vector2d timeToPoint(double time) const override;
        Vector2d timeToVelocity(double time) const override;
        Vector2d timeToAcceleration(double time) const override;
        double pointToTime(const Vector2d &point) const override;

    private:
        Vector2d point, velocity;
        double angle, duration;
    };

    class Path
    {
    public:
        Path(const std::vector<pointAndVelocity> &pointsAndVelocities, double maxVelocity, double maxAcceleration, double maxLateralAcceleration);

        Vector2d calcVelocity(const Vector2d &point);
        double getTargetAngle() const;

    private:

        std::vector<pointAndVelocity> pointsAndVelocities;
        double maxVelocity, maxAcceleration, maxLateralAcceleration;
        std::vector<Vector2d> points;
        std::vector<double> velocities;
        std::vector<double> angles;
        std::vector<std::unique_ptr<BasePath>> paths;

        Vector2d forwardVelocity = Vector2d();
        Vector2d lateralVelocity = Vector2d();
        double target_angle = 0;
    };

    double calcAngularVelocity(double current_angle, double target_angle, double max_angular_velocity);
}

#endif // PATH_PLANNER_HPP