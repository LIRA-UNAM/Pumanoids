#pragma once

#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <string>
#include <stdexcept>
#include "../types.h"
#include "./print.h"

using namespace std;

// Convert degrees to radians.
inline double deg2rad(double deg)
{
    return deg / 180.0 * M_PI;
}

// Convert radians to degrees.
inline double rad2deg(double rad)
{
    return rad / M_PI * 180.0;
}

// Return the arithmetic mean.
inline double mean(double x, double y)
{
    return (x + y) / 2;
}

// Clamp a number to a range.
inline double cap(double x, double upper_limit, double lower_limit)
{
    return max(min(x, upper_limit), lower_limit);
}

// Return the L2 norm of two values.
inline double norm(double x, double y)
{
    return sqrt(x * x + y * y);
}

// Return the L2 norm of a vector.
inline double norm(vector<double> v)
{
    return sqrt(v[0] * v[0] + v[1] * v[1]);
}

// Normalize an angle to [-M_PI, M_PI).
inline double toPInPI(double theta)
{
    int n = static_cast<int>(fabs(theta / 2 / M_PI)) + 1;
    return fmod(theta + M_PI + 2 * n * M_PI, 2 * M_PI) - M_PI;
}

// Return the angle in radians between vector v and the x-axis, in (-M_PI, M_PI].
inline double thetaToX(vector<double> v)
{
    vector<double> x = {1, 0};
    double ang = atan2(v[1], v[0]);
    return toPInPI(ang);
}

// Transform planar coordinates from frame 0 to frame 1, which is rotated by theta relative to frame 0.
inline Point2D transform(Point2D p0, double theta)
{
    Point2D p1;
    p1.x = p0.x * cos(theta) + p0.y * sin(theta);
    p1.y = -p0.x * sin(theta) + p0.y * cos(theta);
    return p1;
}

/**
 * @brief Transform a pose from source frame s to target frame t.
 *
 * @param xs, ys, thetas Pose in the source frame; angles are in radians.
 * @param xst, yst, thetast Source-frame origin pose in the target frame; angles are in radians.
 * @param xt, yt, thetat Output pose in the target frame; angles are in radians.
 */
inline void transCoord(const double &xs, const double &ys, const double &thetas, const double &xst, const double &yst, const double &thetast, double &xt, double &yt, double &thetat)
{
    thetat = toPInPI(thetas + thetast);
    xt = xst + xs * cos(thetast) - ys * sin(thetast);
    yt = yst + xs * sin(thetast) + ys * cos(thetast);
}

/**
 * @brief Transform a two-dimensional pose between coordinate frames.
 * 
 * @param x, y, theta Two-dimensional pose.
 * @param xf, yf, thetaf Frame pose in world coordinates.
 * @param dir "forth" transforms from world to the frame; "back" performs the inverse.
 * 
 * @return Transformed pose as {x, y, theta}.
 */
inline vector<double> trans(double x, double y, double theta, double xf, double yf, double thetaf, string dir = "forth") {
    Eigen::Matrix3d T;
    T << cos(thetaf), -sin(thetaf), xf,
         sin(thetaf),  cos(thetaf), yf,
         0,            0,           1;

    Eigen::Vector3d pose(x, y, 1);

    if (dir == "forth") {
        Eigen::Vector3d transformed_pose = T.inverse() * pose;
        return {transformed_pose(0), transformed_pose(1), toPInPI(theta - thetaf)};
    } else if (dir == "back") {
        Eigen::Vector3d transformed_pose = T * pose;
        return {transformed_pose(0), transformed_pose(1), toPInPI(theta + thetaf)};
    } else {
        throw std::invalid_argument("Invalid direction. Use 'forth' or 'back'.");
    }
}

// Return the cross product of two-dimensional vectors.
inline double crossProduct(const vector<double> a, const vector<double> b) {
    return a[0] * b[1]- a[1] * b[0];
}

inline double innerProduct(const vector<double> a, const vector<double> b) {
    return a[0] * b[0] + a[1] * b[1];
}

// Sigmoid function.
inline double sigmoid(double x, double shift = 0., double scale = 1.) {
    return 1 / (1 + std::exp(scale * (x - shift)));
}

// Return the mean of a vector.
inline double mean(vector<double> v) {
    if (v.size() == 0) return 0.;

    double sum = 0.;
    for (int i = 0; i < v.size(); i++) sum += v[i];
    
    return sum/v.size();
}

inline bool linesIntersect(const Line l1, const Line l2) {
    vector<double> AB = {l1.x1 - l1.x0, l1.y1 - l1.y0};
    vector<double> AC = {l2.x0 - l1.x0, l2.y0 - l1.y0};
    vector<double> AD = {l2.x1 - l1.x0, l2.y1 - l1.y0};
    bool cross1 = crossProduct(AB, AC) * crossProduct(AB, AD) <= 0;

    vector<double> CD = {l2.x1 - l2.x0, l2.y1 - l2.y0};
    vector<double> CA = {l1.x0 - l2.x0, l1.y0 - l2.y0};
    vector<double> CB = {l1.x1 - l2.x0, l1.y1 - l2.y0};
    bool cross2 = crossProduct(CD, CA) * crossProduct(CD, CB) <= 0;

    return cross1 && cross2;
}

inline double angleBetweenLines(const Line l1, const Line l2) {
    vector<double> AB = {l1.x1 - l1.x0, l1.y1 - l1.y0};
    vector<double> CD = {l2.x1 - l2.x0, l2.y1 - l2.y0};
    auto normProdut = norm(AB) * norm(CD);
    if (normProdut == 0) return 0.;

    auto angle = acos((innerProduct(AB, CD)) / normProdut);
    if (angle > M_PI / 2) angle = M_PI - angle;
    
    return angle;
}

inline double lineLength(const Line l) {
    return norm(l.x1 - l.x0, l.y1 - l.y0);
}

// Return a point's signed distance from a line; positive is away from the origin and negative is toward it.
inline double pointPerpDistToLine(const Point2D p, const Line l) {
    if (lineLength(l) == 0) return 0.;

    vector<double> OA = {l.x0, l.y0};
    vector<double> OB = {l.x1, l.y1};
    vector<double> vLine;
    vector<double> vPoint; 
    if (crossProduct( {l.x0, l.y0}, {l.x1, l.y1}) > 0) {
        vLine = {l.x0 - l.x1, l.y0 - l.y1};
        vPoint = {p.x - l.x1, p.y - l.y1};
    } else {
        vLine = {l.x1 - l.x0, l.y1 - l.y0};
        vPoint = {p.x - l.x0, p.y - l.y0};
    }
    return crossProduct(vLine, vPoint) / lineLength(l);
}

inline double pointMinDistToLine(const Point2D p, const Line l) {
    vector<double> AB = {l.x1 - l.x0, l.y1 - l.y0};
    vector<double> AP = {p.x - l.x0, p.y - l.y0};
    if (innerProduct(AB, AP) < 0) return norm(AP[0], AP[1]);

    vector<double> BA = {l.x0 - l.x1, l.y0 - l.y1};
    vector<double> BP = {p.x - l.x1, p.y - l.y1};
    if (innerProduct(BA, BP) < 0) return norm(BP[0], BP[1]);

    // else
    return fabs(pointPerpDistToLine(p, l));
}

inline double boxDistToLine(const BoundingBox b, const Line l) {
    
    // Test whether any of the four edges intersects the line.
    vector<Line> lines = {{b.xmax, b.ymax, b.xmin, b.ymax},
                          {b.xmin, b.ymax, b.xmin, b.ymin},
                          {b.xmin, b.ymin, b.xmax, b.ymin},
                          {b.xmax, b.ymin, b.xmax, b.ymax}};
    for (int i = 0; i < 4; i++) {
        auto line = lines[i];
        if (linesIntersect(line, l)) return 0.;
    }

    // Find the minimum distance from the four points to the line.
    double minDist = 1e9;
    vector<Point2D> points = {{b.xmin, b.ymin}, {b.xmin, b.ymax}, {b.xmax, b.ymin}, {b.xmax, b.ymax}};
    for (int i = 0; i < 4; i++) {
        auto point = points[i];
        auto dist = fabs(pointPerpDistToLine(point, l));
        if (dist < minDist) {
            minDist = dist;
        }
    }
    return minDist; // This is an approximation: intersection uses a segment, while perpendicular distance uses an infinite line.
}

// Extend a line segment by length at both ends.
inline Line extendLine(const Line l, const double length) {
    if (lineLength(l) == 0) return l;

    double dir = atan2(l.y1 - l.y0, l.x1 - l.x0);
    Line res = l;
    res.x0 -= length * cos(dir);
    res.y0 -= length * sin(dir);
    res.x1 += length * cos(dir);
    res.y1 += length * sin(dir);
    return res;
}
    

// Return whether two segments are collinear within angleTolerance.
inline bool isSameLine(const Line line1, const Line line2, const double angleTolerance = 0.1, const double extendLength = 0.5, const double maxPerpDist = 0.2, const double maxPointDist = 0.5) {
    
    auto l1 = extendLine(line1, extendLength);
    auto l2 = extendLine(line2, extendLength);

    Point2D A = {l1.x0, l1.y0};
    Point2D B = {l1.x1, l1.y1};

    return angleBetweenLines(l1, l2) < angleTolerance 
        && fabs(pointPerpDistToLine(A, l2)) < maxPerpDist
        && fabs(pointPerpDistToLine(B, l2)) < maxPerpDist
        && (
            pointMinDistToLine(A, l2) < maxPointDist
            || pointMinDistToLine(B, l2) < maxPointDist
            || linesIntersect(l1, l2)
        );
}

// Estimate the probability that segment l1 is part of segment l2.
inline double probPartOfLine(const Line l1, const Line l2) {
    Point2D p0 = {l1.x0, l1.y0};
    Point2D p1 = {l1.x1, l1.y1};

    auto dist = (pointMinDistToLine(p0, l2) + pointMinDistToLine(p1, l2))/2;
    // return sigmoid(dist, 0.5, 8);
    return cap(1 - dist / 3.0, 1, 0);
}


inline Line mergeLines(const Line l1, const Line l2) {
    Line res{};

    vector<Point2D> points = {{l1.x0, l1.y0}, {l1.x1, l1.y1}, {l2.x0, l2.y0}, {l2.x1, l2.y1}};

    double maxDist = 0;
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 4; j++) {
            auto dist = norm(points[i].x - points[j].x, points[i].y - points[j].y);
            if (dist > maxDist) {
                maxDist = dist;
                res.x0 = points[i].x;
                res.y0 = points[i].y;
                res.x1 = points[j].x;
                res.y1 = points[j].y;
            }
        }
    }
    return res;
}

/**
 * @brief Fit a line y = ax + b.
 * 
 * @param x Independent values.
 * @param y Dependent values.
 * @param calcR2 Whether to calculate R-squared.
 * @return Slope, intercept, and R-squared value.
 */
inline vector<double> linearFit(const vector<double>& x, const vector<double>& y, bool calcR2 = false) {
    const int n = x.size();
    if (n < 2 || n != y.size()) return {0, 0, 0};

    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    
    for(int i = 0; i < n; ++i) {
        sum_x += x[i];
        sum_y += y[i];
        sum_xy += x[i] * y[i];
        sum_xx += x[i] * x[i];
    }
    
    // Calculate the slope and intercept.
    double denominator = n * sum_xx - sum_x * sum_x;
    double a = denominator == 0 ? 0 : (n * sum_xy - sum_x * sum_y) / denominator;
    double b = (sum_y - a * sum_x) / n;
    
    vector<double> res = {a, b};
    
    // Calculate R-squared.
    if (calcR2) {
        double y_mean = sum_y / n;
        double ss_tot = 0, ss_res = 0;
        for(int i = 0; i < n; ++i) {
            double y_pred = a * x[i] + b;
            ss_res += (y[i] - y_pred) * (y[i] - y_pred);
            ss_tot += (y[i] - y_mean) * (y[i] - y_mean);
        }
        double r_squared = ss_tot == 0 ? 0 : 1 - ss_res / ss_tot;
        res.push_back(r_squared);
    }
    
    return res;
}
