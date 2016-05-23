//Title:  coordinate_system.cpp
//Author: Nicholas Ballard

#include <stdio.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <iomanip>

using namespace cv;
using namespace std;



// Globals ----------------------------------------------------------------------------------------

int boardHeight = 7;
int boardWidth = 5;
Size cbSize = Size(boardHeight,boardWidth);

string filename = "out_camera_data.yml";

bool doneYet = false;

//default image size
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

//function prototypes
//void generate_boardPoints();


std::vector<Point2d> ReverserMat(std::vector<Point2d> input) {

   std::vector<Point2d> tempMat;
     
   for( int i = 0; i < input.size(); i++ )  
   {  
     tempMat.push_back(input[input.size()-i-1]);  
   }  

   return(tempMat);
 }

 void rotateImage(const Mat &input, Mat &output, double alpha, double beta, double gamma, double dx, double dy, double dz, double f)
  {
    alpha = (alpha - 90.)*CV_PI/180.;
    beta = (beta - 90.)*CV_PI/180.;
    gamma = (gamma - 90.)*CV_PI/180.;
    // get width and height for ease of use in matrices
    double w = (double)input.cols;
    double h = (double)input.rows;
    // Projection 2D -> 3D matrix
    Mat A1 = (Mat_<double>(4,3) <<
              1, 0, -w/2,
              0, 1, -h/2,
              0, 0,    0,
              0, 0,    1);
    // Rotation matrices around the X, Y, and Z axis
    Mat RX = (Mat_<double>(4, 4) <<
              1,          0,           0, 0,
              0, cos(alpha), -sin(alpha), 0,
              0, sin(alpha),  cos(alpha), 0,
              0,          0,           0, 1);
    Mat RY = (Mat_<double>(4, 4) <<
              cos(beta), 0, -sin(beta), 0,
              0, 1,          0, 0,
              sin(beta), 0,  cos(beta), 0,
              0, 0,          0, 1);
    Mat RZ = (Mat_<double>(4, 4) <<
              cos(gamma), -sin(gamma), 0, 0,
              sin(gamma),  cos(gamma), 0, 0,
              0,          0,           1, 0,
              0,          0,           0, 1);
    // Composed rotation matrix with (RX, RY, RZ)
    Mat R = RX * RY * RZ;
    // Translation matrix
    Mat T = (Mat_<double>(4, 4) <<
             1, 0, 0, dx,
             0, 1, 0, dy,
             0, 0, 1, dz,
             0, 0, 0, 1);
    // 3D -> 2D matrix
    Mat A2 = (Mat_<double>(3,4) <<
              f, 0, w/2, 0,
              0, f, h/2, 0,
              0, 0,   1, 0);
    // Final transformation matrix
    Mat trans = A2 * (T * (R * A1));
    // Apply matrix transformation
    warpPerspective(input, output, trans, input.size(), INTER_LANCZOS4);
  }


// Main -------------------------------------------------------------------------------------------
int main()
{
	
	//set up a FileStorage object to read camera params from file
	FileStorage fs;
	fs.open(filename, FileStorage::READ);
	// read camera matrix and distortion coefficients from file
	Mat intrinsics, distortion;
	fs["Camera_Matrix"] >> intrinsics;
	fs["Distortion_Coefficients"] >> distortion;
	// close the input file
	fs.release();




	//set up matrices for storage
	Mat webcamImage, gray, one, dst_img;
	Mat rvec = Mat(Size(3,1), CV_64F);
	Mat tvec = Mat(Size(3,1), CV_64F);

	//setup vectors to hold the chessboard corners in the chessboard coordinate system and in the image
	vector<Point2d> imagePoints, imageFramePoints, imageOrigin;
	vector<Point3d> boardPoints, framePoints;


	//generate vectors for the points on the chessboard
	for (int i=0; i<boardWidth; i++)
	{
		for (int j=0; j<boardHeight; j++)
		{
			boardPoints.push_back( Point3d( double(i), double(j), 0.0) );
		}
	}
	//generate points in the reference frame
	framePoints.push_back( Point3d( 0.0, 0.0, 0.0 ) );
	framePoints.push_back( Point3d( 5.0, 0.0, 0.0 ) );
	framePoints.push_back( Point3d( 0.0, 5.0, 0.0 ) );
	framePoints.push_back( Point3d( 0.0, 0.0, 5.0 ) );


	//set up VideoCapture object to acquire the webcam feed from location 0 (default webcam location)
	VideoCapture capture(0);
	//capture.open(0);
	//set the capture frame size
	capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

	while(!doneYet)
	{
		 //store image to matrix
		 //capture.read(webcamImage);
         capture >> webcamImage;

         flip(webcamImage, webcamImage, 1);
         resize(webcamImage, webcamImage , Size(640, 480), 0, 0, INTER_NEAREST);

		 //make a gray copy of the webcam image
		 cvtColor(webcamImage,gray,COLOR_BGR2GRAY);


		 //detect chessboard corners
		 bool found = findChessboardCorners(gray, cbSize, imagePoints, CALIB_CB_FAST_CHECK);
		 //drawChessboardCorners(webcamImage, cbSize, Mat(imagePoints), found);

		 

		 //find camera orientation if the chessboard corners have been found
		 if ( found )
		 {
		 	 if((imagePoints[34].x + imagePoints[34].y) < (imagePoints[0].x + imagePoints[0].y))
             {
              	imagePoints = ReverserMat(imagePoints);
             }	
			 //find the camera extrinsic parameters
			 solvePnP( Mat(boardPoints), Mat(imagePoints), intrinsics, distortion, rvec, tvec, false );

			 //project the reference frame onto the image
			 projectPoints(framePoints, rvec, tvec, intrinsics, distortion, imageFramePoints );
			 

			 //DRAWING
			 //draw the reference frame on the image
			 circle(webcamImage, (Point) imagePoints[0], 4 ,CV_RGB(255,0,0) );
			 
			 Point one, two, three;
			 one.x=10; one.y=10;
			 two.x = 60; two.y = 10;
			 three.x = 10; three.y = 60;

			 line(webcamImage, one, two, CV_RGB(255,0,0) );
			 line(webcamImage, one, three, CV_RGB(0,255,0) );


			 line(webcamImage, imageFramePoints[0], imageFramePoints[1], CV_RGB(255,0,0), 2 );
			 line(webcamImage, imageFramePoints[0], imageFramePoints[2], CV_RGB(0,255,0), 2 );
			 line(webcamImage, imageFramePoints[0], imageFramePoints[3], CV_RGB(0,0,255), 2 );

			 Mat rmat;
             char str[200];
             Rodrigues(rvec,rmat);
             double roll, pitch, yaw;

             roll = atan2(rmat.at<double>(1,0),rmat.at<double>(0,0));
             pitch = -asin(rmat.at<double>(2,0));
             yaw = atan2(rmat.at<double>(2,1),rmat.at<double>(2,2));

             roll = roll*180/3.1415;
             pitch = pitch*180/3.1415;
             yaw = yaw*180/3.1415;
			 
			 //show the pose estimation data
			 cout << fixed << setprecision(2) << "roll = ["
				  << roll << "] \t" << "pitch = ["
				  << pitch << "]\t" << "yaw = ["
				  << yaw << "]" << endl;

			 
			 //show the pose estimation data
			 cout << fixed << setprecision(2) << "rvec = ["
				  << rvec.at<double>(0,0) << ", "
				  << rvec.at<double>(1,0) << ", "
				  << rvec.at<double>(2,0) << "] \t" << "tvec = ["
				  << tvec.at<double>(0,0) << ", "
				  << tvec.at<double>(1,0) << ", "
				  << tvec.at<double>(2,0) << "]" << endl;

			double alpha = 90 +  pitch/4;
            double beta = (yaw > 0 ? 90 - (180 - abs(yaw))/2:90 + (180 - abs(yaw))/2);  //the rotation around the y axis
            double gamma = 90 - (roll - 90); //the rotation around the z axis
            //double gamma = 90;
               
            rotateImage(webcamImage, dst_img, alpha, beta, gamma, 0, 0, 200, 200);	  

		    namedWindow("OpenCV dst", 0);
		    imshow("OpenCV dst", dst_img);	  
			
		 }

		 //show the image on screen
		 namedWindow("OpenCV Webcam", 0);
		 imshow("OpenCV Webcam", webcamImage);




		 //show the gray image
		 //namedWindow("Gray Image", CV_WINDOW_AUTOSIZE);
		 //imshow("Gray Image", gray);


		 waitKey(10);
	}

	return 0;
}
