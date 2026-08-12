/**
 * @file posPredictor.h
 * @brief Defines PosPredictor for forecasting positions such as the ball position.
 */
#pragma once
#include <Eigen/Dense>
#include <tuple>
#include <cmath>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <mutex>
#include <array>
#include <utils/math.h>
#include <opencv2/opencv.hpp> 
#include <fstream>

using namespace std;

class PosPredictor
{
public:
    PosPredictor(rclcpp::Clock::SharedPtr clock, double maxMemSecs=10, int minDataSize=5, int maxDataSize=200, int speedWindow=5, double stopSpeed=0.1, double speedCliff=0.3, double defaultAcc=-0.4, double defaultProcessNoise=1e-2, double measurementNoiseRatio=0.2) 
        : _clock(clock), _maxMemSecs(maxMemSecs), _minDataSize(minDataSize), _maxDataSize(maxDataSize), _speedWindow(speedWindow), _stopSpeed(stopSpeed), _speedCliff(speedCliff), _defaultAcc(defaultAcc), _defaultProcessNoise(defaultProcessNoise), _measurementNoiseRatio(measurementNoiseRatio) {
        // Initialize the Kalman filter.
        _kf = cv::KalmanFilter(4, 2, 0);  // Four state variables (x, y, vx, vy) and two measurements (x, y)
        
        // Initial state-transition matrix A.
        _kf.transitionMatrix = (cv::Mat_<float>(4, 4) << 
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1);
            
        cv::setIdentity(_kf.measurementMatrix);  // Measurement matrix H
        cv::setIdentity(_kf.processNoiseCov, cv::Scalar::all(_defaultProcessNoise));  // Process noise Q
        cv::setIdentity(_kf.measurementNoiseCov, cv::Scalar::all(_measurementNoiseRatio));  // Measurement noise R
        cv::setIdentity(_kf.errorCovPost, cv::Scalar::all(1));  // Posterior error P
    }

    void add(rclcpp::Time t, double x, double y, double dist) {
        lock_guard<mutex> lock(_mutex);

        // Record the raw sample.
        _t.push_back(t);
        _x.push_back(x);
        _y.push_back(y);

        // Calculate and record velocity over a moving window.
        int n = _t.size();
        int i0 = n - _speedWindow;
        double vx, vy, s;
        if (i0 >= 0) {
            double dt = (t - _t[i0]).seconds();
            double dx = x - _x[i0];
            double dy = y - _y[i0];
            vx = dt > 0 ? dx / dt : 0;
            vy = dt > 0 ? dy / dt : 0;
            s = dt > 0 ? sqrt(vx * vx + vy * vy) : 0;
        } else {
            vx = 0;
            vy = 0;
            s = 0;
        }
        _vx.push_back(vx);
        _vy.push_back(vy);
        _s.push_back(s);
        _curSmoothS = _curSmoothS * 0.8 + s * 0.2;
        _smoothS.push_back(_curSmoothS);
        // Apply the Kalman filter after storing raw data and before removeOldData, then record the filtered values.
        _runKalman(dist);

        // Remove expired samples.
        _removeOldData();
    }

    // Legacy predictor.
    vector<array<double, 2>> predict(double intervalMSecs, int steps, double &s0, double &acc, double &theta, int &dataLen, bool useFiltered=false) {
        lock_guard<mutex> lock(_mutex);
        // _removeOldData();
        if (_t.size() == 0) return {};

        auto params = _getSpeed(useFiltered);
        s0 = params[0];
        theta = params[1];
        dataLen = static_cast<int>(params[2]);
        acc = _defaultAcc;

        double x0 = useFiltered ? _filteredX.back() : _x.back();
        double y0 = useFiltered ? _filteredY.back() : _y.back();
        auto t0 = _t.back();
        auto secsTillStop = acc < 0 ? -s0 / acc : 1000000;
        auto ts = t0 + rclcpp::Duration::from_seconds(secsTillStop); // Estimated stop time

        vector<array<double, 2>> predictions;
        auto now = _clock->now();
        for (int i = 0; i < steps; i++) {
            auto t = now + rclcpp::Duration::from_seconds(intervalMSecs * i / 1000.0);
            // auto t = t0 + rclcpp::Duration::from_seconds(intervalMSecs * i / 1000.0);

            double dt = t > ts ? (ts - t0).seconds() : (t - t0).seconds();
            double s1 = s0 + acc * dt;
            double s = (s0 + s1) / 2;

            double dx = s * cos(theta) * dt;
            double dy = s * sin(theta) * dt;
            array<double, 2> pos = {x0 + dx, y0 + dy};
            predictions.push_back(pos);

            // cout << "t: " << t.nanoseconds() << " t0: " << t0.nanoseconds() << " dt: " << dt << "s0: " << s0 << "s1: " << s1 << "s: " << s << "dx: " << dx << "dy: " << dy << endl;
        }

        return predictions;
    }

    // Preferred predictor.
    tuple<vector<array<double, 2>>, bool, string> predict_filtered(double intervalMSecs, int steps, double acc, int minDataLen=3) {
        lock_guard<mutex> lock(_mutex);
        // _removeOldData();
        if (_t.size() == 0) return {{}, false, "no data"};

        auto params = _getSpeed(true);
        auto s0 = params[0];
        auto theta = params[1];
        auto dataLen = static_cast<int>(params[2]);
        if (dataLen < minDataLen) return {{}, false, "data len < min data len"};

        double x0 = _filteredX.back();
        double y0 = _filteredY.back();
        auto t0 = _t.back();
        auto secsTillStop = acc < 0 ? -s0 / acc : 1000000;
        auto ts = t0 + rclcpp::Duration::from_seconds(secsTillStop); // Estimated stop time

        vector<array<double, 2>> predictions;
        for (int i = 0; i < steps; i++) {
            auto t = t0 + rclcpp::Duration::from_seconds(intervalMSecs * i / 1000.0);

            double dt = t > ts ? (ts - t0).seconds() : (t - t0).seconds();
            double s1 = s0 + acc * dt;
            double s = (s0 + s1) / 2;

            double dx = s * cos(theta) * dt;
            double dy = s * sin(theta) * dt;
            array<double, 2> pos = {x0 + dx, y0 + dy};
            predictions.push_back(pos);

            // cout << "t: " << t.nanoseconds() << " t0: " << t0.nanoseconds() << " dt: " << dt << "s0: " << s0 << "s1: " << s1 << "s: " << s << "dx: " << dx << "dy: " << dy << endl;
        }

        return make_tuple(predictions, true, "");
    }

    /*
    @brief Linear position prediction.
    @param intervalMSecs Prediction interval.
    @param steps Number of prediction steps.
    @param acc Prior acceleration.
    @param trainDataLen Training-data length.
    @param rSquareValve R-squared threshold for the fit.
    @param sameDirValve Direction-consistency threshold.
    @return Predicted (x, y) values per step, success status, and error message.
    */
    tuple<vector<array<double, 2>>, bool, string> predict_linear(double intervalMSecs, int steps, double acc=-0.4, int trainDataLen=5, double rSquareValve=0.9, double sameDirValve=M_PI/2.0) {
        lock_guard<mutex> lock(_mutex);
        if (_t.size() < trainDataLen || _x.size() < trainDataLen || _y.size() < trainDataLen) {
            return make_tuple(vector<array<double, 2>>(), false, format("Insufficient data for linear prediction, need at least %d data points", trainDataLen));
        }
        vector<double> x(_x.end() - trainDataLen, _x.end());
        vector<double> y(_y.end() - trainDataLen, _y.end());
        vector<rclcpp::Time> t(_t.end() - trainDataLen, _t.end());
        
        auto [a, b, rSquare, errMsg] = linear_fit(x, y);
        if (rSquare < rSquareValve) {
            return make_tuple(vector<array<double, 2>>(), false, format("rsquare too small: r^2 = %.2f, need: %.2f", rSquare, rSquareValve));
        }
        if (!isSameDir(x, y, sameDirValve)) {
            return make_tuple(vector<array<double, 2>>(), false, "move direction is not consistent");
        }

        // Otherwise, determine whether the object is decelerating uniformly.
        auto x0 = x[0];
        auto y0 = y[0];
        auto t0 = t[0];
        auto x1 = x.back();
        auto y1 = y.back();
        auto t1 = t.back();

        auto dx = x1 - x0;
        auto dy = y1 - y0;
        auto dt = (t1 - t0).seconds();
        auto speed = sqrt(dx * dx + dy * dy) / (dt + 1e-8);
        speed += acc * dt / 2; // Convert average velocity to terminal velocity.
        auto theta = atan2(dy, dx);

        vector<array<double,2>> res;
        for (int i = 0; i < steps; i++) {
            auto dt2 = intervalMSecs * i / 1000.0;
            auto s2 = speed + dt2 * acc;
            auto dist = s2 > 0 ? (speed + s2) / 2.0 * dt2 : (speed / 2.0) * (speed /fabs(acc) + 1e-8);
            auto x2 = x1 + cos(theta) * dist;
            auto y2 = y1 + sin(theta) * dist;
            res.push_back({x2, y2});
        }
        return make_tuple(res, true, format("rsquare: %.2f, speed: %.2f, theta: %.2f", rSquare, speed, theta));
    }

    void getData(vector<rclcpp::Time> &t, vector<double> &x, vector<double> &y) {
        lock_guard<mutex> lock(_mutex);
        t = _t;
        x = _x;
        y = _y;
    }

    void getData(vector<rclcpp::Time> &t, vector<double> &x, vector<double> &y, vector<double> &s, vector<double> &smoothS, vector<double> &xf, vector<double> &yf, vector<double> &vxf, vector<double> &vyf, vector<double> &sf) {
        lock_guard<mutex> lock(_mutex);
        t = _t;
        x = _x;
        y = _y;
        s = _s;
        smoothS = _smoothS;
        xf = _filteredX;
        yf = _filteredY;
        vxf = _filteredVx;
        vyf = _filteredVy;
        sf = _filteredS;
    }

    void getLastData(rclcpp::Time &t, double &x, double &y, double &s, double &smoothS, double &xf, double &yf, double &vxf, double &vyf, double &sf) {
        lock_guard<mutex> lock(_mutex);
        t = _t.back();
        x = _x.back();
        y = _y.back();
        s = _s.back();
        smoothS = _smoothS.back();
        xf = _filteredX.back();
        yf = _filteredY.back();
        vxf = _filteredVx.back();
        vyf = _filteredVy.back();
        sf = _filteredS.back();
    }

    // Fit a linear equation for x and y and return its R-squared value.
    tuple<double, double, double, string> linear_fit(const vector<double>& x, const vector<double>& y) {
        int n = x.size();
        
        if (n < 2) {
            return make_tuple(0., 0., 0., "At least two data points are required");
        }
        
        if (x.size() != y.size()) {
            return make_tuple(0., 0., 0., "x and y must have the same length");
        }
        
        // Identical x values would make the matrix singular.
        double x_min = *min_element(x.begin(), x.end());
        double x_max = *max_element(x.begin(), x.end());
        if (abs(x_max - x_min) < 1e-10) {
            return make_tuple(0., 0., 0., "Linear regression requires non-identical x values");
        }
        
        // Construct design matrix A = [1, x].
        Eigen::MatrixXd A(n, 2);
        Eigen::VectorXd b(n);
        
        for (int i = 0; i < n; i++) {
            A(i, 0) = 1.0;
            A(i, 1) = x[i];
            b(i) = y[i];
        }
        
        // Solve with QR decomposition for better numerical stability.
        Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);
        
        double intercept = coeffs(0);
        double slope = coeffs(1);
        
        // Calculate R-squared with a division-by-zero guard.
        Eigen::VectorXd y_pred = A * coeffs;
        
        double y_mean = 0.0;
        for (double val : y) {
            y_mean += val;
        }
        y_mean /= n;
        
        double tss = 0.0, rss = 0.0;
        for (int i = 0; i < n; i++) {
            tss += (y[i] - y_mean) * (y[i] - y_mean);
            rss += (y[i] - y_pred(i)) * (y[i] - y_pred(i));
        }
        
        double r_square;
        if (abs(tss) < 1e-10) {
            // For identical y values, define R-squared as 1 (a perfect constant fit).
            r_square = 1.0;
        } else {
            r_square = 1.0 - (rss / tss);
            // Keep R-squared within its valid range.
            r_square = max(0.0, min(1.0, r_square));
        }
        
        return make_tuple(slope, intercept, r_square, "");
    }

    // Check whether the x/y movement direction is consistent across steps.
    bool isSameDir(const vector<double>& x, vector<double>& y, double threshold = M_PI) {
        int n = x.size();
        if (y.size() != n) return false;
        if (n < 3) return false;

        vector<double> dirs = {};
        for (int i = 0; i < n - 1; i++) {
            double dx = x[i + 1] - x[i];
            double dy = y[i + 1] - y[i];
            double dir = atan2(dy, dx);
            dirs.push_back(dir);
        }
        for (int i = 0; i < dirs.size() - 1; i++) {
            auto delta = toPInPI(dirs[i] - dirs[i + 1]);
            if (abs(delta) > threshold) {
                return false;
            }
        }

        return true;
    }

private:
    rclcpp::Clock::SharedPtr _clock;
    mutex _mutex;  // Synchronizes predictor state
    
    double _maxMemSecs; // Maximum sample history in seconds
    int _minDataSize; // Minimum sample count for prediction
    int _maxDataSize; // Maximum sample count used for prediction
    int _speedWindow; // Compare against the sample this many frames earlier to calculate velocity
    double _stopSpeed; // Speeds below this value are considered stopped
    double _speedCliff; // Backward speed drop that marks the beginning of a motion segment
    double _defaultAcc; // Fallback acceleration when a reasonable value cannot be fitted

    vector<rclcpp::Time> _t;
    vector<double> _x;
    vector<double> _y;
    vector<double> _vx;
    vector<double> _vy;
    vector<double> _s; // Speed
    vector<double> _smoothS; // Smoothed speed
    double _curSmoothS = 0; // Running value used to smooth speed
    
    // Kalman-filter state.
    cv::KalmanFilter _kf;
    double _suddenChangeSpeedValve = 1.0; // Sudden speed-change threshold
    int _lastSuddenChangeIndex = 0; // Index of the last sudden speed change
    vector<double> _filteredX;
    vector<double> _filteredY;
    vector<double> _filteredVx;
    vector<double> _filteredVy;
    vector<double> _filteredS;
    double _defaultProcessNoise; // Default process noise
    double _measurementNoiseRatio; // Measurement-noise-to-distance ratio

    // Record Kalman-filtered xf, yf, xvf, yvf, and sf; s is the sum of squared vx/vy, and f means filtered.
    void _runKalman(double dist) {
        if (_t.empty()) return;
        
        auto x = _x.back();
        auto y = _y.back();

        if (_t.size() == 1) {
            _filteredX.push_back(x);
            _filteredY.push_back(y);
            _filteredVx.push_back(0);
            _filteredVy.push_back(0);
            _filteredS.push_back(0);
            return;
        }

        cv::Mat measurement = (cv::Mat_<float>(2, 1) << x, y);

        if ( // Treat a sudden speed change as a new segment and reinitialize the Kalman-filter state.
            _s.size() > 4 && 
            (
                fabs(_s.back() - _s[_s.size() - 2]) > _suddenChangeSpeedValve
                || fabs(_s.back() - _s[_s.size() - 3]) > _suddenChangeSpeedValve
                || fabs(_s.back() - _s[_s.size() - 4]) > _suddenChangeSpeedValve
                )
            ) {
            _kf.statePost = (cv::Mat_<float>(4, 1) << x, y, _vx.back(), _vy.back());
            // _kf.statePost = (cv::Mat_<float>(4, 1) << x, y, 0, 0);
            cv::setIdentity(_kf.errorCovPost, cv::Scalar::all(1));
            _lastSuddenChangeIndex = _t.size() - 1;
        }
        
        float dt = (_t.back() - _t[_t.size() - 2]).seconds();
        
        // Update the state-transition matrix.
        _kf.transitionMatrix = (cv::Mat_<float>(4, 4) << 
            1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1);
        // cv::setIdentity(_kf.measurementNoiseCov, cv::Scalar::all(dist * _measurementNoiseRatio));  // Measurement noise R
        double speedNoise = (_t.size() - _lastSuddenChangeIndex) > 10 ? dist * _measurementNoiseRatio : dist * _measurementNoiseRatio * 0.15;
        _kf.measurementNoiseCov = (cv::Mat_<float>(2, 2) << dist * _measurementNoiseRatio, 0, 0, speedNoise);

        
        // Prediction step, required to update internal state.
        _kf.predict();
        
        // Correction step.
        cv::Mat estimated = _kf.correct(measurement);
        
        _filteredX.push_back(estimated.at<float>(0));
        _filteredY.push_back(estimated.at<float>(1));
        _filteredVx.push_back(estimated.at<float>(2));
        _filteredVy.push_back(estimated.at<float>(3));
        _filteredS.push_back(sqrt(estimated.at<float>(2) * estimated.at<float>(2) + estimated.at<float>(3) * estimated.at<float>(3)));
    }

    // Search backward from the current time for the start of uniform deceleration.
    int _getSegStartIndex(bool useFiltered=false) {
        int n = _t.size();
        if (n == 0) return -1;
        if (useFiltered) {
            // cout << "_t.size(): " << _t.size() << ", _filteredS.size(): " << _filteredS.size() << endl;
            ofstream debugFile("debug.log", ios::app);
            debugFile << "n: " << n << endl;
            double curSpeed = _filteredS.back();
            int i;
            int startIndex = n - 1; 
            for (i = n - 2; i >= 0 && i >= n - _maxDataSize; i--) {
                // debugFile << "i: " << i << ", _filteredS[i]: " << _filteredS[i] << ", curSpeed: " << curSpeed << endl;
                if (_filteredS[i] < curSpeed - 1e-6)  break;
                //else
                curSpeed = _filteredS[i];
                startIndex = i;
            }
            debugFile.close();
            return startIndex;
        }
        
        // else
        double maxSpeed = 0;
        int startIndex = n - 1;
        for (int i = startIndex; i >= 0 && i >= n - _maxDataSize; i--) {
            if (_smoothS[i] > maxSpeed) {
                maxSpeed = _smoothS[i];
                startIndex = i;
            } else if (_smoothS[i] < maxSpeed - _speedCliff) {
                break;
            }
        }
        return startIndex;
    }

    // Return speed, motion direction, and sample count; speed is zero outside uniform deceleration.
    vector<double> _getSpeed(bool useFiltered=false) {
        int n = _t.size();
        if (n == 0) return {0.0, 0.0, -1};

        int startIndex = _getSegStartIndex(useFiltered);
        if (n - startIndex < max(2, _minDataSize)) return {0.0, 0.0, -2};

        auto vec_x = useFiltered ? vector<double>(_filteredX.begin() + startIndex, _filteredX.end()) : vector<double>(_x.begin() + startIndex, _x.end());
        auto vec_y = useFiltered ? vector<double>(_filteredY.begin() + startIndex, _filteredY.end()) : vector<double>(_y.begin() + startIndex, _y.end());

        double x0 = useFiltered ? _filteredX[startIndex] : _x[startIndex];
        double x1= useFiltered ? _filteredX.back() : _x.back();
        if (fabs(x1 - x0) < 1e-6) return {0.0, 0.0, -3}; // Prevent a failed linear fit.

        // Fit a line.
        auto fitRes = linearFit(vec_x, vec_y);
        double a = fitRes[0];
        double b = fitRes[1];
        double y0 = a * x0 + b;
        double y1 = a * x1 + b;
        double theta = atan2(y1 - y0, x1 - x0); // TODO: Direction may be unstable when x1 - x0 is very small.

        double s = useFiltered ? _filteredS.back() : _smoothS.back();

        return {s, theta, 0. + n - startIndex};
    }

    // Unused legacy fit that returns speed, acceleration, and motion direction.
    vector<double> _fitParams(bool useFiltered=false) {
        int n = _t.size();
        if (n == 0) return {0.0, _defaultAcc, 0.0};

        int startIndex = _getSegStartIndex(useFiltered);
        if (startIndex == -1) return {0.0, _defaultAcc, 0.0};
        
        if (n - startIndex < min(2, _minDataSize)) return {useFiltered ? _filteredS.back() : _s.back(), _defaultAcc, 0.0};

        double y0 = useFiltered ? _filteredY[startIndex] : _y[startIndex];
        double x0 = useFiltered ? _filteredX[startIndex] : _x[startIndex];
        double y1 = useFiltered ? _filteredY.back() : _y.back();
        double x1 = useFiltered ? _filteredX.back() : _x.back();

        double theta = atan2(y1 - y0, x1 - x0);

        vector<double> t_vector;
        t_vector.reserve(n - startIndex);
        rclcpp::Time endTime = _t.back();
        for (int i = startIndex; i < n; i++) {
            double timeDiff = (_t[i] - endTime).seconds();
            t_vector.push_back(timeDiff);
        }
        vector<double> s_vector(_s.begin() + startIndex, _s.end());

        auto params = linearFit(t_vector, s_vector);
        double a = params[0];
        double s0 = params[1];
        // double s0 = _s.back();

        return {s0, a, theta};
    }

    void _removeOldData() {
        auto curTime = _clock->now();
        // Find the first non-expired sample.
        size_t firstValidPos = 0;
        while (firstValidPos < _t.size() && 
               curTime - _t[firstValidPos] > rclcpp::Duration::from_seconds(_maxMemSecs)) {
            firstValidPos++;
        }
        
        // Remove all expired samples at once.
        if (firstValidPos > 0) {
            _t.erase(_t.begin(), _t.begin() + firstValidPos);
            _x.erase(_x.begin(), _x.begin() + firstValidPos);
            _y.erase(_y.begin(), _y.begin() + firstValidPos);
            _vx.erase(_vx.begin(), _vx.begin() + firstValidPos);
            _vy.erase(_vy.begin(), _vy.begin() + firstValidPos);
            _s.erase(_s.begin(), _s.begin() + firstValidPos);
            _smoothS.erase(_smoothS.begin(), _smoothS.begin() + firstValidPos);
            _filteredX.erase(_filteredX.begin(), _filteredX.begin() + firstValidPos);
            _filteredY.erase(_filteredY.begin(), _filteredY.begin() + firstValidPos);
            _filteredVx.erase(_filteredVx.begin(), _filteredVx.begin() + firstValidPos);
            _filteredVy.erase(_filteredVy.begin(), _filteredVy.begin() + firstValidPos);
            _filteredS.erase(_filteredS.begin(), _filteredS.begin() + firstValidPos);
        }
    }
};
