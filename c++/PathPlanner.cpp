/**
 * @file PathPlanner.cpp
 * @author Doraemonjayo
 * @brief
 * @version 1.0
 * @date 2025-03-03
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "PathPlanner.hpp"

namespace PathPlanner
{
    double positive_fmod(double x, double y)
    {
        return std::fmod(std::fmod(x, y) + y, y);
    }

    // Vector2d implementation
    Vector2d::Vector2d(double x, double y) : x(x), y(y) {}

    Vector2d Vector2d::operator+(const Vector2d &other) const
    {
        return {x + other.x, y + other.y};
    }

    Vector2d Vector2d::operator-(const Vector2d &other) const
    {
        return {x - other.x, y - other.y};
    }

    Vector2d Vector2d::operator*(double scalar) const
    {
        return {x * scalar, y * scalar};
    }

    Vector2d Vector2d::operator/(double scalar) const
    {
        return {x / scalar, y / scalar};
    }

    double Vector2d::dot(const Vector2d &other) const
    {
        return x * other.x + y * other.y;
    }

    double Vector2d::norm() const
    {
        return std::sqrt(x * x + y * y);
    }

    Vector2d Vector2d::normalized() const
    {
        double n = norm();
        return n > 0 ? (*this / n) : Vector2d();
    }

    Vector2d operator*(double scalar, const Vector2d &vector)
    {
        return vector * scalar;
    }

    // pointAndVelocity implementation
    pointAndVelocity::pointAndVelocity(Vector2d point, double velocity, double angle) : point(point), velocity(velocity), angle(angle) {}

    // BasePath implementation
    double BasePath::getDuration() const
    {
        return duration;
    }

    double BasePath::getAngle() const
    {
        return angle;
    }

    // StraightPath implementation
    StraightPath::StraightPath(const Vector2d &startPoint, const Vector2d &endPoint, double startVelocity, double endVelocity, double angle)
        : startPoint(startPoint), endPoint(endPoint), startVelocity(startVelocity), endVelocity(endVelocity), angle(angle)
    {
        vector = endPoint - startPoint;
        length = vector.norm();
        unitVector = vector.normalized();

        duration = 2 * length / (startVelocity + endVelocity);
        acceleration = (endVelocity - startVelocity) / duration;
    }

    double StraightPath::pointToTime(const Vector2d &point) const
    {
        double time;
        double n = (point - startPoint).dot(unitVector);
        if (acceleration == 0)
        {
            time = n / startVelocity;
        }
        else
        {
            if (startVelocity * startVelocity + 2 * acceleration * n >= 0)
            {
                time = (std::sqrt(startVelocity * startVelocity + 2 * acceleration * n) - startVelocity) / acceleration;
            }
            else
            {
                time = -1;
            }
        }
        if (0 <= time && time <= duration)
        {
            return time;
        }
        else
        {
            return std::numeric_limits<double>::quiet_NaN();
        }
    }

    Vector2d StraightPath::timeToPoint(double time) const
    {
        if (0 <= time && time <= duration)
        {
            return startPoint + unitVector * (startVelocity * time + 0.5 * acceleration * time * time);
        }
        else
        {
            return Vector2d();
        }
    }

    Vector2d StraightPath::timeToVelocity(double time) const
    {
        if (0 <= time && time <= duration)
        {
            return unitVector * (startVelocity + acceleration * time);
        }
        else
        {
            return Vector2d();
        }
    }

    Vector2d StraightPath::timeToAcceleration(double time) const
    {
        if (0 <= time && time <= duration)
        {
            return unitVector * acceleration;
        }
        else
        {
            return Vector2d();
        }
    }

    // CornerPath implementation
    CornerPath::CornerPath(const Vector2d &cornerPoint, const Vector2d &startVector, const Vector2d &endVector, double velocity, double acceleration, double angle)
        : cornerPoint(cornerPoint), startVector(startVector), endVector(endVector), velocity(std::abs(velocity)), acceleration(std::abs(acceleration)), angle(angle)
    {
        startAngle = std::atan2(startVector.y, startVector.x);
        endAngle = std::atan2(endVector.y, endVector.x);
        deltaAngle = positive_fmod(endAngle - startAngle + M_PI, 2 * M_PI) - M_PI;

        radius = velocity * velocity / acceleration;

        if (radius == 0)
        {
            duration = 0;
        }
        else
        {
            duration = deltaAngle * radius / velocity;
        }
        if (duration < 0)
        {
            duration = -duration;
            radius = -radius;
        }

        cutLength = radius * std::tan(deltaAngle / 2);

        centerPoint = cornerPoint + cutLength / std::cos(M_PI / 2 - deltaAngle / 2) * Vector2d(std::cos(startAngle + deltaAngle / 2 + M_PI / 2), std::sin(startAngle + deltaAngle / 2 + M_PI / 2));
        cutLength = std::abs(cutLength);

        arcStartAngle = startAngle - M_PI / 2;
    }

    double CornerPath::pointToTime(const Vector2d &point) const
    {
        double angle = std::atan2(point.y - centerPoint.y, point.x - centerPoint.x) - arcStartAngle;
        angle = positive_fmod(angle + M_PI, 2 * M_PI) - M_PI;
        double time = angle / deltaAngle * duration;
        if (0 <= time && time <= duration)
        {
            return time;
        }
        angle += M_PI;
        angle = positive_fmod(angle + M_PI, 2 * M_PI) - M_PI;
        time = angle / deltaAngle * duration;
        if (0 <= time && time <= duration)
        {
            return time;
        }
        return std::numeric_limits<double>::quiet_NaN();
    }

    Vector2d CornerPath::timeToPoint(double time) const
    {
        if (0 <= time && time <= duration)
        {
            return centerPoint + radius * Vector2d(std::cos(arcStartAngle + time / duration * deltaAngle), std::sin(arcStartAngle + time / duration * deltaAngle));
        }
        return Vector2d();
    }

    Vector2d CornerPath::timeToVelocity(double time) const
    {
        if (0 <= time && time <= duration)
        {
            return velocity * Vector2d(-std::sin(arcStartAngle + time / duration * deltaAngle), std::cos(arcStartAngle + time / duration * deltaAngle));
        }
        return Vector2d();
    }

    Vector2d CornerPath::timeToAcceleration(double time) const
    {
        if (0 <= time && time <= duration)
        {
            return acceleration * Vector2d(-std::cos(arcStartAngle + time / duration * deltaAngle), -std::sin(arcStartAngle + time / duration * deltaAngle));
        }
        return Vector2d();
    }

    double CornerPath::getCutLength() const
    {
        return cutLength;
    }

    // PointPath implementation
    PointPath::PointPath(const Vector2d &point, const Vector2d &velocity, double angle)
        : point(point), velocity(velocity), angle(angle)
    {
        duration = 0;
    }

    double PointPath::pointToTime(const Vector2d &point) const
    {
        return 0;
    }

    Vector2d PointPath::timeToPoint(double time) const
    {
        if (0 <= time && time <= duration)
        {
            return point;
        }
        else
        {
            return Vector2d();
        }
    }

    Vector2d PointPath::timeToVelocity(double time) const
    {
        if (0 <= time && time <= duration)
        {
            return velocity;
        }
        else
        {
            return Vector2d();
        }
    }

    Vector2d PointPath::timeToAcceleration(double time) const
    {
        if (0 <= time && time <= duration)
        {
            return Vector2d(0, 0);
        }
        else
        {
            return Vector2d();
        }
    }

    // Path implementation
    Path::Path(const std::vector<pointAndVelocity> &pointsAndVelocities, double maxVelocity, double maxAcceleration, double maxLateralAcceleration) : pointsAndVelocities(pointsAndVelocities), maxVelocity(maxVelocity), maxAcceleration(maxAcceleration), maxLateralAcceleration(maxLateralAcceleration)
    {
        for (const auto &p : pointsAndVelocities)
        {
            points.push_back(p.point);
            velocities.push_back(p.velocity);
            angles.push_back(p.angle);
        }

        Vector2d vector0 = (points[1] - points[0]).normalized();
        Vector2d vector1 = (points[2] - points[1]).normalized();

        paths.push_back(std::make_unique<PointPath>(points[0], velocities[0] * vector0, angles[0]));

        double cutLength0 = 0;
        auto corner = std::make_unique<CornerPath>(points[1], vector0, vector1, velocities[1], maxAcceleration, angles[1]);
        double cutLength1 = corner->getCutLength();
        paths.push_back(std::make_unique<StraightPath>(points[0] + cutLength0 * vector0, points[1] - cutLength1 * vector0, velocities[0], velocities[1], angles[1]));
        paths.push_back(std::move(corner));

        for (size_t i = 2; i < points.size() - 1; i++)
        {
            vector0 = vector1;
            vector1 = (points[i + 1] - points[i]).normalized();
            cutLength0 = cutLength1;
            corner = std::make_unique<CornerPath>(points[i], vector0, vector1, velocities[i], maxAcceleration, angles[i]);
            cutLength1 = corner->getCutLength();
            paths.push_back(std::make_unique<StraightPath>(points[i - 1] + cutLength0 * vector0, points[i] - cutLength1 * vector0, velocities[i - 1], velocities[i], angles[i]));
            paths.push_back(std::move(corner));
        }

        vector0 = vector1;
        paths.push_back(std::make_unique<StraightPath>(points[points.size() - 2] + cutLength0 * vector0, points[points.size() - 1], velocities[points.size() - 2], velocities[points.size() - 1], angles[points.size() - 1]));
        paths.push_back(std::make_unique<PointPath>(points[points.size() - 1], velocities[points.size() - 1] * vector0, angles[points.size() - 1]));
    }

    Vector2d Path::calcVelocity(const Vector2d &point)
    {
        double min_dist = std::numeric_limits<double>::infinity();
        Vector2d fv(0, 0);
        Vector2d lv(0, 0);
        double ta = 0;

        for (const auto &path : paths)
        {
            double time = path->pointToTime(point);
            if (!std::isnan(time))
            {
                Vector2d p_vec = path->timeToPoint(time);
                Vector2d v_vec = path->timeToVelocity(time);
                double dist = (point - p_vec).norm();
                if (dist < min_dist)
                {
                    min_dist = dist;
                    fv = v_vec;
                    lv = (p_vec - point).normalized();
                    ta = path->getAngle();
                }
            }
        }

        lv = lv * std::min(maxVelocity, std::sqrt(2 * maxLateralAcceleration * min_dist));
        double fv_norm = fv.norm();
        if (fv_norm == 0)
        {
            fv = Vector2d();
        }
        else
        {
            fv = fv / fv_norm * std::min(maxVelocity, std::min(fv_norm, std::sqrt(std::max(maxVelocity * maxVelocity - lv.norm() * lv.norm(), 0.0))));
        }

        forwardVelocity = fv;
        lateralVelocity = lv;

        Vector2d output = fv + lv;

        target_angle = ta;

        if (output.norm() > maxVelocity)
        {
            output = output.normalized() * maxVelocity;
        }

        return output;
    }

    double Path::getTargetAngle() const
    {
        return target_angle;
    }

    const std::vector<std::unique_ptr<BasePath>>& Path::getPaths() const
    {
        return paths;
    }

    double calcAngularVelocity(double target_angle, double feedback_angle, double feedback_angular_velocity, double max_angular_velocity, double max_angular_acceleration, double dt)
    {
        double delta_angle = target_angle - feedback_angle;
        delta_angle = positive_fmod(delta_angle + M_PI, 2 * M_PI) - M_PI;

        double velocity;
        if (delta_angle > 0)
        {
            velocity = std::min(max_angular_velocity, std::sqrt(2 * (max_angular_acceleration / 4) * delta_angle));
        }
        else
        {
            velocity = -std::min(max_angular_velocity, std::sqrt(2 * (max_angular_acceleration / 4) * -delta_angle));
        }

        if (velocity - feedback_angular_velocity > max_angular_acceleration * dt)
        {
            velocity = feedback_angular_velocity + max_angular_acceleration * dt;
        }
        else if (velocity - feedback_angular_velocity < -max_angular_acceleration * dt)
        {
            velocity = feedback_angular_velocity - max_angular_acceleration * dt;
        }

        return velocity;
    }
}