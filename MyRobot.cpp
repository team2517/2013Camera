#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Math.h"

#define AREA_MINIMUM 500

	//Structure to represent the scores for the various tests used for target identification
	struct Scores {
		double rectangularity;
		double aspectRatioInner;
		double aspectRatioOuter;
		double xEdge;
		double yEdge;

	};

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */
class RobotDemo : public SimpleRobot {
	RobotDrive myRobot; // robot drive system
	Joystick stick; // only joystick
	Scores *scores;

public:
	RobotDemo(void) :
	myRobot(1, 2), // these must be initialized in the same order
	stick(1) // as they are declared above.
	
	{
		myRobot.SetExpiration(0.1);
	}
	
	

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void) {
		/*
		 myRobot.SetSafetyEnabled(false);
		 myRobot.Drive(-0.5, 0.0); 	// drive forwards half speed
		 Wait(2.0); 				//    for 2 seconds
		 myRobot.Drive(0.0, 0.0); 	// stop robot
		 */
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void) {
		//myRobot.SetSafetyEnabled(true);
		Threshold threshold(60, 100, 90, 255, 20, 255);
		ParticleFilterCriteria2 criteria[] = {
			{	IMAQ_MT_AREA, AREA_MINIMUM, 65535, false, false}
		};

			AxisCamera &camera = AxisCamera::GetInstance();

			while (IsOperatorControl()) {
				//myRobot.ArcadeDrive(stick); // drive with arcade style (use right stick)
				//Wait(0.005);				// wait for a motor update time
				ColorImage *image;
				camera.GetImage(image);

				BinaryImage *thresholdImage = image->ThresholdHSV(threshold);
				BinaryImage *convexHullImage = thresholdImage->ConvexHull(false);
				BinaryImage *filteredImage = convexHullImage->ParticleFilter(
						criteria, 1);
				vector<ParticleAnalysisReport> *reports =
				filteredImage->GetOrderedParticleAnalysisReports();
				scores = new Scores[reports->size()];

				for (unsigned i = 0; i < reports->size(); i++) {
					ParticleAnalysisReport *report = &(reports->at(i));

					scores[i].rectangularity = scoreRectangularity(report);
					scores[i].aspectRatioOuter = scoreAspectRatio(filteredImage,
							report, true);
					scores[i].aspectRatioInner = scoreAspectRatio(filteredImage,
							report, false);
					scores[i].xEdge = scoreXEdge(thresholdImage, report);
					scores[i].yEdge = scoreYEdge(thresholdImage, report);

					if (scoreCompare(scores[i], false)) {
						printf(
								"particle: %d  is a High Goal  centerX: %f  centerY: %f \n",
								i, report->center_mass_x_normalized,
								report->center_mass_y_normalized);
						printf("Distance: %f \n", computeDistance(thresholdImage,
										report, false));
					} else if (scoreCompare(scores[i], true)) {
						printf(
								"particle: %d  is a Middle Goal  centerX: %f  centerY: %f \n",
								i, report->center_mass_x_normalized,
								report->center_mass_y_normalized);
						printf("Distance: %f \n", computeDistance(thresholdImage,
										report, true));
					} else {
						printf(
								"particle: %d  is not a goal  centerX: %f  centerY: %f \n",
								i, report->center_mass_x_normalized,
								report->center_mass_y_normalized);
					}
					printf("rect: %f  ARinner: %f \n", scores[i].rectangularity,
							scores[i].aspectRatioInner);
					printf("ARouter: %f  xEdge: %f  yEdge: %f  \n",
							scores[i].aspectRatioOuter, scores[i].xEdge,
							scores[i].yEdge);
				}.
			}
		}

		/**
		 * Runs during test mode
		 */

	};

	START_ROBOT_CLASS(RobotDemo)
	;

