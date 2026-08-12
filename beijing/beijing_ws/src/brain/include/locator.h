/**
 * @file locator.h
 * @brief Localizes the robot from field markings using a particle filter.
 */
#pragma once

#include <Eigen/Core>
#include <cstdlib> // for srand and rand
#include <ctime>   // for time
#include <limits>
#include <cmath>
#include <chrono>
#include <rerun.hpp>

#include "types.h"

// #define EPSILON 1e-5

using namespace std;
namespace chr = std::chrono;

// Localization search constraints.
struct PoseBox2D
{
	double xmin;
	double xmax;
	double ymin;
	double ymax;
	double thetamin;
	double thetamax;
};

// Field marking.
struct FieldMarker
{
	char type;		   // L, T, X, or P; P denotes a penalty mark
	double x, y;	   // Marking position (m)
	double confidence; // Detection confidence
};

// Localization result.
struct LocateResult
{
	bool success = false;
	int code = -1;		 // 0 success; 1 no particles; 2 invalid residual; 3 not converged; 4 too few markers; 5 probabilities too low; -1 initial
	double residual = 0; // Mean residual
	Pose2D pose;
	int msecs = 0; // Localization duration
};

/**
 * @class Locator
 * @brief Localize the robot with a particle filter.
 *
 */
class Locator
{
public:
	// Parameters
	double convergeTolerance = 0.2;	 // Converged when every hypothesis x/y/theta range is below this value
	double residualTolerance = 0.4;	 // Reject a converged pose when mean residual per marker exceeds this value
	double maxIteration = 20;		 // Maximum iteration count
	double muOffset = 2.0;			 // Approximate residual mean as min(residuals) - muOffset * std(residuals)
	double numShrinkRatio = 0.85;	 // Particle-count ratio after each resampling
	double offsetShrinkRatio = 0.8;	 // x/y/theta sampling-offset ratio after each resampling
	int minMarkerCnt = 3;		 // Minimum marker count
	double enableLog = false;		 // Enable logging
	string logIP = "127.0.0.1:9876"; // Log server address

	// Working data
	const rerun::RecordingStream log = rerun::RecordingStream("locator", "locator");
	vector<FieldMarker> fieldMarkers;
	FieldDimensions fieldDimensions;
	Eigen::ArrayXXd hypos;				  // Hypotheses: pose x/y/theta, residual, normalized probability, cumulative probability
	PoseBox2D constraints;				  // Localization search constraints
	double offsetX, offsetY, offsetTheta; // Random offset range around particles from the previous iteration
	Pose2D bestPose;					  // Best pose hypothesis for the current localization
	double bestResidual;				  // Minimum residual for the current localization

	void init(FieldDimensions fd, int minMarkerCnt = 4, double residualTolerance = 0.4, double muOffsetParam = 2.0, bool enableLog = false, string logIP = "127.0.0.1:9876");

	/**
	 * @brief Generate all field-marking positions in the field frame.
	 *
	 * @param fieldDimensions Field dimensions.
	 *
	 */
	void calcFieldMarkers(FieldDimensions fd);

	/**
	 * @brief Calculate the robot pose on the field.
	 *
	 * @param fieldDimensions Field dimensions.
	 * @param markers_r Visually detected field markings in the robot frame.
	 * @param constraints Prior search constraints; tighter constraints reduce localization time.
	 *
	 * @return LocateResult
	 *         success: bool
	 *         code: 0 success; 1 no particles; 2 invalid residual; 3 not converged; 4 too few markers; 5 probabilities too low
	 *         residual: Fitting residual.
	 *         Pose2D: Localized pose.
	 */
	LocateResult locateRobot(vector<FieldMarker> markers_r, PoseBox2D constraints, int numParticles = 200, double offsetX = 2.0, double offsetY = 2.0, double offsetTheta = M_PI / 4);

	/**
	 * @brief Generate the initial particles.
	 *
	 * @param constraints Generate pose hypotheses only within these constraints.
	 * @param num Number of hypotheses to generate.
	 *
	 * @return 0 on success, nonzero on failure.
	 *
	 */
	int genInitialParticles(int num = 200);

	/**
	 * @brief Resample particles according to their probabilities.
	 *
	 * @return 0 on success, nonzero on failure.
	 */
	int genParticles();

	/**
	 * @brief Transform observed field markings from the robot frame to the field frame.
	 *
	 * @param FieldMarker marker
	 * @param Pose2D pose
	 *
	 * @return All transformed field markings.
	 *
	 */
	FieldMarker markerToFieldFrame(FieldMarker marker, Pose2D pose);

	/**
	 * @brief Find the minimum distance between an observed marking and all map markings.
	 *
	 * @param marker FieldMarker
	 *
	 * @return Minimum distance.
	 *
	 */
	double minDist(FieldMarker marker);

	/**
	 * @brief Find the pose offset from an observed marking to the nearest map marking.
	 *
	 * @param marker FieldMarker
	 *
	 * @return Signed {dx, dy}, where dx = map marking x - observed marking x.
	 *
	 */
	vector<double> getOffset(FieldMarker marker);

	/**
	 * @brief Calculate the fitting residual between robot-frame observations and field markings.
	 *
	 * @param markers_r Observed markings in the robot frame.
	 * @param pose Robot pose used to transform markers_r to the field frame.
	 *
	 * @return Fitting residual.
	 *
	 */
	double residual(vector<FieldMarker> markers_r, Pose2D pose);

	/**
	 * @brief Check whether the current hypotheses have converged.
	 *
	 * @return bool
	 */
	bool isConverged();

	/**
	 * @brief Calculate probabilities for the current pose hypotheses and store them in probs.
	 *
	 * @return 0 on success, nonzero on failure.
	 */
	int calcProbs(vector<FieldMarker> markers_r);

	/**
	 * @brief Recalibrate x and y from marking positions after convergence.
	 *
	 * @return Recalibrated pose.
	 */
	Pose2D finalAdjust(vector<FieldMarker> markers_r, Pose2D pose);

	/**
	 * @brief Gaussian probability-density function.
	 *
	 * @param r Observed value.
	 * @param mu Distribution mean.
	 * @param sigma Distribution standard deviation.
	 *
	 * @return Probability density.
	 *
	 */
	inline double probDesity(double r, double mu, double sigma)
	{
		if (sigma < 1e-5)
			return 0.0;
		return 1 / sqrt(2 * M_PI * sigma * sigma) * exp(-(r - mu) * (r - mu) / (2 * sigma * sigma));
	};

	/**
	 * @brief Log particles(hypos) to rerun
	 *
	 */
	void logParticles();
};
