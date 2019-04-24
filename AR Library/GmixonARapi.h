#ifndef __ExampleUnmanagedDLL_h__
#define __ExampleUnmanagedDLL_h__

#include <stdio.h>
#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/video/tracking.hpp>
#include "opencv2/gpu/gpu.hpp"

using namespace cv;
using namespace cv::gpu;


using namespace std;

const int MAX_COUNT = 100;
const  Size subPixWinSize(10,10), winSize(31,31);
const TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);

const int API_AR_CPU_SURF = 0;
const int API_AR_GPU_SURF =  1;
const int API_AR_OPTICAL_FLOW = 2;
const int API_AR_SURF_OBJECT_FINDER = 4;

// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the EXAMPLEUNMANAGEDDLL_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// EXAMPLEUNMANAGEDDLL_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef EXAMPLEUNMANAGEDDLL_EXPORTS
	#define EXAMPLEUNMANAGEDDLL_API __declspec(dllexport)
#else
	#define EXAMPLEUNMANAGEDDLL_API __declspec(dllimport)
#endif


class EXAMPLEUNMANAGEDDLL_API apiAR
{

private:

		bool external_input;
		bool tracking_enabled;
        bool object_lost;

        int64 work_begin; // Performance test
        int64 work_end;

        void workBegin();
        void workEnd();
        double getTime();

        bool USE_GPU; //  GPU using.
        bool USE_OPTICAL_FLOW; // Optical flow using.
        bool USE_SURF_OBJECT_LOCATION;

        bool show_test_3D_Object; //
        bool USE_KALMAN_FILTERING;

        bool optical_flow_first;
        bool addRemovePt;

        bool draw_marker_contour;
        bool first;
		int object_not_found_counter;
		int frame_counter;
        int image_width, image_height; // Image size.

        float c_width;
        float c_height;
        float c_deep;

        Mat_<double> camMat;   // Calibration camera matrix.
        Mat_<double> distCoeffs; // Calibration camera distortion coeffs.
        double fx, fy,cx,cy;

        std::vector<cv::Point3f> object_corners_3d; // 3D object virtual points.

      //  GLFWwindow* window;


        CvCapture* vidObj; // OpenCV큦 capture object.

        Mat img_scene; // Grabbed image.
        Mat prev_img_scene;
        Mat img_object; // Model object.


        KalmanFilter KF; // Kalman filter for a more estable object큦 corners detection.
        cv::Mat_<float> measurement; // Measurements vector
        Mat_<float> state; //  (x, y, Vx, Vy) - State vector


        // Optical flow tracker stuff
        Mat gray, prevGray, image;
        vector<Point2f> points[2];
        vector<Point2f> of_model_points;
        bool needToInit;
        cv::Rect myROI;
        std::vector<Point2f> optical_flow_obj_corners;


        SurfFeatureDetector detector; // Surf key point detector
        SurfDescriptorExtractor extractor; // Surf descriptors extractor


//	SURF_GPU  surf; // GPU Surf keypoints and descriptors extractor.
//	GpuMat img1, img2; // Model image and scene image.
//	GpuMat keypoints1GPU, keypoints2GPU;   // keypoints and desriptors or both: model image and scene image.
//	GpuMat descriptors1GPU, descriptors2GPU;




        std::vector<KeyPoint> keypoints_object, keypoints_scene; // CPU keypoints
        Mat descriptors_object, descriptors_scene; // CPU Surf descriptors.
        FlannBasedMatcher matcher;

        cv::Mat_<float> rotMat; // Object큦 3D pose rotation matrix
        cv::Mat_<float> Tvec; // Object큦 3D pose transaltion vector.

        cv::Mat raux,taux; // auxiliary rotation and translation matrix.

        int OpenCam();
        void KalmanInit();

        void InitOpenGL();

        void GetCPUModelKeypoints();  // Gets model object keypoints and descriptors.
//	void GetGPUModelKeypoints();  // Gets model object keypoints and descriptors.
        void GetOpticalFlowModelFeautures(); // Gets model object keypoints for optical flow usage.

        std::vector<Point2f> GetCPUObjectCorners(Mat _img_scene);  // Gets 2D corners from the object in the scene using CPU.
//	std::vector<Point2f> GetGPUObjectCorners(Mat _img_scene);  // Gets 2D corners from the object in the scene using GPU.

        std::vector<cv::Point2f> GetOpticalFlowObjectCorners(Mat _img_scene); // Gets 2D corners using optical flow.


        void FindObject3DPose(std::vector<Point2f> Object2DPoints); // Finds Rotation and translation matrix for 3D pose.

        void RenderTest3DObject();

        double angle( Point pt1, Point pt2, Point pt0 );
        bool isInsideContour(int nvert,vector<Point2f> vertices,Point2f current);

        bool ObjectFound(vector<Point2f> vertices);

        void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ );

		
public:
		apiAR(char * object_image_path, int w, int h,int algortihm_type,bool _external = false);
        ~apiAR(void);

        inline Mat GetRotationMatrix(){return this->rotMat;}
        inline Mat GetTranslationVector(){return this->Tvec;}
		
		char* ReturnString();


        inline Mat GetSceneImage(){return this->img_scene;}
        inline void Set3DTestObjectRendering(bool r){this->show_test_3D_Object = r;}
        inline void SetObjectContourLines(bool l){this->draw_marker_contour = l;}
        inline void SetKalmanFiltering(bool kalman_using){this->USE_KALMAN_FILTERING = kalman_using;}
        inline void InitTracking(){this->needToInit = true;}
        inline bool isObjectLost(){return object_lost;}
		Mat MatFromFloatArray(float * pixels,int lenght);

        void PlaneTracking(float * pixels, int lenght); // arApi main pipeline: TODO:: create independent threads

};

#endif	// __ExampleUnmanagedDLL_h__
