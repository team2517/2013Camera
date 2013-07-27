#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Math.h"

#define X_IMAGE_RES 320	
#define VIEW_ANGLE 48
#define PI 3.141592653

#define RECTANGULARITY_LIMIT 60
#define ASPECT_RATIO_LIMIT 75
#define X_EDGE_LIMIT 40
#define Y_EDGE_LIMIT 60

#define AREA_MINIMUM 500

#define XMAXSIZE 24
#define XMINSIZE 24
#define YMAXSIZE 24
#define YMINSIZE 48
const double xMax[XMAXSIZE] = { 1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5,
		.5, .5, .5, .5, .5, .5, .5, 1, 1, 1, 1 };
const double xMin[XMINSIZE] = { .4, .6, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1,
		.1, .1, .1, .1, .1, .1, .1, .1, .1, .1, 0.6, 0 };
const double yMax[YMAXSIZE] = { 1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5,
		.5, .5, .5, .5, .5, .5, .5, 1, 1, 1, 1 };
const double yMin[YMINSIZE] = { .4, .6, .05, .05, .05, .05, .05, .05, .05, .05,
		.05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
		.05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
		.05, .05, .05, .05, .05, .05, .05, .05, .6, 0 };

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */
class RobotDemo : public SimpleRobot {

	//Structure to represent the scores for the various tests used for target identification
	struct Scores {
		double rectangularity;
		double aspectRatioInner;
		double aspectRatioOuter;
		double xEdge;
		double yEdge;

	};

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
		ParticleFilterCriteria2 criteria[] = { { IMAQ_MT_AREA, AREA_MINIMUM,
				65535, false, false } };

		AxisCamera &camera = AxisCamera::GetInstance();

		while (IsOperatorControl()) {
			/**
			 * Do the image capture with the camera and apply the algorithm described above. This
			 * sample will either get images from the camera or from an image file stored in the top
			 * level directory in the flash memory on the cRIO. The file name in this case is "testImage.jpg"
			 */
			ColorImage *image;
			

			camera.GetImage(image);				//To get the images from the camera comment the line above and uncomment this one
			BinaryImage *thresholdImage = image->ThresholdHSV(threshold); // get just the green target pixels
			//thresholdImage->Write("/threshold.bmp");
			BinaryImage *convexHullImage = thresholdImage->ConvexHull(false); // fill in partial and full rectangles
			//convexHullImage->Write("/ConvexHull.bmp");
			BinaryImage *filteredImage = convexHullImage->ParticleFilter(
					criteria, 1); //Remove small particles
			//filteredImage->Write("Filtered.bmp");

			vector<ParticleAnalysisReport> *reports =
					filteredImage->GetOrderedParticleAnalysisReports(); //get a particle analysis report for each particle
			scores = new Scores[reports->size()];

			//Iterate through each particle, scoring it and determining whether it is a target or not
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
			}
			printf("\n");

			// be sure to delete images after using them
			delete filteredImage;
			delete convexHullImage;
			delete thresholdImage;
			delete image;

			//delete allocated reports and Scores objects also
			delete scores;
			delete reports;
		}
	}

	/**
	 * Runs during test mode
	 */

};

START_ROBOT_CLASS(RobotDemo)
;

