#include <iostream>

#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
#include <windows.h>

#include "GmixonARapi.h"



#ifdef _MANAGED
#pragma managed(push, off)
#endif

BOOL APIENTRY DllMain(
	HMODULE		/*hModule*/,
	DWORD		ul_reason_for_call,
	LPVOID		/*lpReserved*/)
{
	switch (ul_reason_for_call)
	{
		case DLL_PROCESS_ATTACH:
		case DLL_THREAD_ATTACH:
		case DLL_THREAD_DETACH:
		case DLL_PROCESS_DETACH:
			break;
	}
	return TRUE;
}

#ifdef _MANAGED
#pragma managed(pop)
#endif

void apiAR::workBegin()
{
    work_begin = getTickCount();
}

int handleError( int status, const char* func_name,
            const char* err_msg, const char* file_name,
            int line, void* userdata )
{
    //Do nothing -- will suppress console output
    return 0;   //Return value is not used
}

void apiAR::workEnd()
{
    work_end = getTickCount() - work_begin;
}

double apiAR::getTime()
{
    return work_end /((double)cvGetTickFrequency() * 1000.);
}

apiAR::apiAR(char * object_image_path, int w, int h,int algorithm_type, bool _external)
{

	
		
        switch(algorithm_type)
        {
                case 0:
                        USE_GPU = false;
                        USE_OPTICAL_FLOW = false;                        
                        break;

                case 1:
                        USE_GPU = true;
                        USE_OPTICAL_FLOW = false;
                        break;

                case 2:
                        USE_GPU = false;
                        USE_OPTICAL_FLOW = true;
                        USE_SURF_OBJECT_LOCATION = false;
                        break;

                case 6:
                        USE_GPU = false;
                        USE_OPTICAL_FLOW = true;
                        USE_SURF_OBJECT_LOCATION = true;

                default:

                        USE_GPU = false;
                        USE_OPTICAL_FLOW = true;
                        break;

        }

		external_input = _external;

        USE_KALMAN_FILTERING = false;
        object_lost = true;

        optical_flow_first = true;
        show_test_3D_Object = true;
        draw_marker_contour = false;

        object_not_found_counter = 5;

        image_width = w;
        image_height = h;

        first = true;
        needToInit = false;

        c_width = 9;
        c_height = 12;
        c_deep = 0;



		if(OpenCam() || external_input) // If a capture device is ready then proceed
        {
                try
                {

                        img_object = imread("object.JPG",CV_LOAD_IMAGE_GRAYSCALE); // Reading model object image from file.
						
						

                        gray = img_object.clone();

                        // Initilalizing GPU SURF
                //	img1.upload(img_object);
                //	surf.hessianThreshold = 200;



                }
                catch(...)
                {
                        cout << "Unable to load model object image" << endl;
                        tracking_enabled = false;
                        return;

                }

                if(img_object.empty())
                    tracking_enabled = false;
                else
                    tracking_enabled = true;

                if(tracking_enabled)
                {

										
                KalmanInit(); // Inits Kalman filter

                rotMat = Mat_<float>(3,3); // Inits rotation matrix.

                InitOpenGL();

                detector.hessianThreshold = 400;


                GetCPUModelKeypoints(); // Extracting object keypoints and descriptors once using CPU.


        //	GetGPUModelKeypoints();  // Gets model object keypoints and descriptors.

                GetOpticalFlowModelFeautures();
            }

        }
        else
        {


                        cout << "Unable to find an aviable camera" << endl;
                        return ;

        }

}


apiAR::~apiAR(void)
{

        ///  !!!!!! TODO

        cvReleaseCapture(&vidObj);
       // glfwDestroyWindow(window);

//    glfwTerminate();



}

void apiAR::InitOpenGL()
{

/*        if (!glfwInit())
        exit(EXIT_FAILURE);


        window = glfwCreateWindow(image_width, image_height, "Simple example", NULL, NULL);

        glfwHideWindow(window);

    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

        glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

  //  glfwSetKeyCallback(window, key_callback);*/


}


void apiAR::KalmanInit()
{

// Instantiate Kalman Filter with
    // 16 dynamic parameters and 8 measurement parameters,
    // where my measurement is: 4 2D location (one for each object´s corner) of the object,
    // and dynamic is: 2D location and 2D velocity of the corners.
    KF.init(16, 8, 0);

        measurement = Mat_<float>::zeros(8,1);

        KF.statePre.setTo(0);

    KF.statePost.setTo(0);

        /* KF.transitionMatrix = *(Mat_<float>(16, 16) << 1,0,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
                                                                                                        0,1,0,1, 0,0,0,0, 0,0,0,0, 0,0,0,0,
                                                                                                        0,0,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
                                                                                                        0,0,0,1, 0,0,0,0, 0,0,0,0, 0,0,0,0,

                                                                                                        0,0,0,0, 1,0,1,0, 0,0,0,0, 0,0,0,0,
                                                                                                        0,0,0,0, 0,1,0,1, 0,0,0,0, 0,0,0,0,
                                                                                                        0,0,0,0, 0,0,1,0, 0,0,0,0, 0,0,0,0,
                                                                                                        0,0,0,0, 0,0,0,1, 0,0,0,0, 0,0,0,0,

                                                                                                        0,0,0,0, 0,0,0,0, 1,0,1,0, 0,0,0,0,
                                                                                                        0,0,0,0, 0,0,0,0, 0,1,0,1, 0,0,0,0,
                                                                                                        0,0,0,0, 0,0,0,0, 0,0,1,0, 0,0,0,0,
                                                                                                        0,0,0,0, 0,0,0,0, 0,0,0,1, 0,0,0,0,

                                                                                                        0,0,0,0, 0,0,0,0, 0,0,0,0, 1,0,1,0,
                                                                                                        0,0,0,0, 0,0,0,0, 0,0,0,0, 0,1,0,1,
                                                                                                        0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,1,0,
                                                                                                        0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,1);

         KF.measurementMatrix = *(Mat_<float>(8, 16) << 1,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
                                                                                                        0,1,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
                                                                                                        0,0,0,0, 1,0,0,0, 0,0,0,0, 0,0,0,0,
                                                                                                        0,0,0,0, 0,1,0,0, 0,0,0,0, 0,0,0,0,
                                                                                                        0,0,0,0, 0,0,0,0, 1,0,0,0, 0,0,0,0,
                                                                                                        0,0,0,0, 0,0,0,0, 0,1,0,0, 0,0,0,0,
                                                                                                        0,0,0,0, 0,0,0,0, 0,0,0,0, 1,0,0,0,
                                                                                                        0,0,0,0, 0,0,0,0, 0,0,0,0, 0,1,0,0);*/


    setIdentity(KF.transitionMatrix);
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(.1)); //adjust this for faster convergence - but higher noise
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(.1));



}

int apiAR::OpenCam()
{

		if(!external_input)
		{
        try{



        vidObj = cvCaptureFromCAM(0);

        if(vidObj != NULL)
        {
                cvSetCaptureProperty(vidObj,3,image_width);
                cvSetCaptureProperty(vidObj,4,image_height);
        }
        else
        {
//            QMessageBox Msgbox;
//            Msgbox.setText("Unable to find a camera.");
 //           Msgbox.exec();

            return -1;
        }
    }
    catch(...){
        cvReleaseCapture(&vidObj);

        return -1;
        }

        while (!cvGrabFrame(vidObj)){}
    // Retrieve the captured frame
        img_scene = cvRetrieveFrame(vidObj);



        cout << "First frame arrived " << endl;

        image_width = img_scene.cols;
        image_height = img_scene.rows;

		}

		image_width = 640;
        image_height = 480;

				
        camMat = Mat_<double>(3,3);
        distCoeffs = Mat_<double>(5,1);

        cx = image_width/2;
        cy = image_height/2;

        fx = fy = cx/tan(60/2 * CV_PI / 180);
        camMat << fx, 0., img_scene.cols/2.0, 0, fy, img_scene.rows/2.0, 0., 0., 1.;
        distCoeffs << 1.0, 0.2, 0., 0., 0.;

    object_corners_3d.push_back(cvPoint3D32f(c_width,0,0));
    object_corners_3d.push_back(cvPoint3D32f(0.0,0.0,0));
    object_corners_3d.push_back(cvPoint3D32f(0.0,c_height,0));
    object_corners_3d.push_back(cvPoint3D32f(c_width,c_height,0));

        cout << " Camera setup ended" << endl;

        return 1;
}

void apiAR::GetCPUModelKeypoints()
{

        detector.detect( img_object, keypoints_object ); // Detecting model object keypoints.
        extractor.compute( img_object, keypoints_object, descriptors_object ); // Extracting descriptors.

}

/*void apiAR::GetGPUModelKeypoints()
{

        surf(img1, GpuMat(), keypoints1GPU, descriptors1GPU);


}*/

void apiAR::GetOpticalFlowModelFeautures()
{

        goodFeaturesToTrack(img_object, of_model_points, MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
        cornerSubPix(img_object, of_model_points, subPixWinSize, Size(-1,-1), termcrit);


}

std::vector<Point2f> apiAR::GetCPUObjectCorners(Mat _img_scene)
{

  workBegin();

  detector.detect( _img_scene, keypoints_scene );

  extractor.compute( _img_scene, keypoints_scene, descriptors_scene );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  std::vector< DMatch > matches;
  matcher.match( descriptors_object, descriptors_scene, matches );

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_object.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }


  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector< DMatch > good_matches;

  for( int i = 0; i < descriptors_object.rows; i++ )
  { if( matches[i].distance < 3*min_dist )
    { good_matches.push_back( matches[i]); }
  }


  //-- Localize the object from img_1 in img_2
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;

  for( size_t i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }

  workEnd();


  float surf_time = getTime();

  workBegin();


  Mat H = findHomography( obj, scene, CV_RANSAC);



  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = Point(0,0); obj_corners[1] = Point( img_object.cols, 0 );
  obj_corners[2] = Point( img_object.cols, img_object.rows ); obj_corners[3] = Point( 0, img_object.rows );
  std::vector<Point2f> scene_corners(4);

  perspectiveTransform( obj_corners, scene_corners, H);

  workEnd();

  float homography_time = getTime();

 // cout << "Surf time: " << surf_time << endl << "Keypoints total:" << scene.size() << endl << "Homography time: " << homography_time << endl;


  return scene_corners;

}


/*std::vector<Point2f> apiAR::GetGPUObjectCorners(Mat _img_scene)
{

        workBegin();
        Mat gray_scene;
        cvtColor(_img_scene,gray_scene,CV_BGR2GRAY);
        img2.upload(gray_scene);


        surf(img2, GpuMat(), keypoints2GPU, descriptors2GPU);

        gpu:BruteForceMatcher_GPU<L2<float>> matcher;

        GpuMat trainIdx, distance;
    matcher.matchSingle(descriptors1GPU, descriptors2GPU, trainIdx, distance);

    // downloading results
    vector<KeyPoint> keypoints1, keypoints2;
    vector<float> descriptors1, descriptors2;

    vector<DMatch> matches;
    surf.downloadKeypoints(keypoints1GPU, keypoints1);
    surf.downloadKeypoints(keypoints2GPU, keypoints2);

    surf.downloadDescriptors(descriptors1GPU, descriptors1);
    surf.downloadDescriptors(descriptors2GPU, descriptors2);

        matcher.matchDownload(trainIdx, distance, matches);


         double max_dist = 0; double min_dist = 100;
        std::vector< DMatch > good_matches;

        for( int i = 0; i < matches.size(); i++ )
  { if( matches[i].distance < 3*min_dist )
    { good_matches.push_back( matches[i]); }
  }

//-- Localize the object from img_1 in img_2
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;

  for( size_t i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints1[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints2[ good_matches[i].trainIdx ].pt );
  }

  workEnd();

  float surf_time = getTime();

  workBegin();

  Mat H = findHomography( obj, scene, CV_RANSAC );

  workEnd();

  float homography_time = getTime();

//  cout << "Surf time: " << surf_time << endl << "Keypoints total:" << scene.size() << endl << "Homography time: " << homography_time << endl;

  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = Point(0,0); obj_corners[1] = Point( img1.cols, 0 );
  obj_corners[2] = Point( img1.cols, img1.rows ); obj_corners[3] = Point( 0, img1.rows );
  std::vector<Point2f> scene_corners(4);

  perspectiveTransform( obj_corners, scene_corners, H);


  return scene_corners;

}*/

bool apiAR::isInsideContour(int nvert,vector<Point2f> vertices,Point2f current)
{

bool c = false;
int i, j = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
          if ( ((vertices[i].y>current.y) != (vertices[j].y>current.y)) &&
                  (current.x < (vertices[j].x-vertices[i].x) * (current.y-vertices[i].y) / (vertices[j].y-vertices[i].y) + vertices[i].x) )
       c = !c;
  }
  return c;

}

std::vector<cv::Point2f> apiAR::GetOpticalFlowObjectCorners(Mat _img_scene)
{

        std::vector<Point2f> scene_corners(4);

        if(!USE_SURF_OBJECT_LOCATION)
                                myROI = cvRect(image_width/2-0.16*image_width,image_height/2-image_height*0.4
                                                                        ,image_width*0.31,image_height*0.62);
		
        _img_scene.copyTo(image);
		
	if(!external_input)
    cvtColor(image, gray, CV_BGR2GRAY);
	else
	{
		
		gray = image.clone();
	}

        if( needToInit )
        {

                        try
                        {


                                if(USE_SURF_OBJECT_LOCATION)
                                    {
                                        optical_flow_obj_corners = this->GetCPUObjectCorners(_img_scene);

                                      /*  int distx = (optical_flow_obj_corners[2].x - optical_flow_obj_corners[0].x);
                                        int disty = (optical_flow_obj_corners[2].y - optical_flow_obj_corners[0].y);

                                        optical_flow_obj_corners[0].x = optical_flow_obj_corners[0].x + distx/4;
                                        optical_flow_obj_corners[0].y = optical_flow_obj_corners[0].y + disty/4;

                                        optical_flow_obj_corners[2].x = optical_flow_obj_corners[2].x - distx/4;
                                        optical_flow_obj_corners[2].y = optical_flow_obj_corners[2].y - disty/4;


                                        optical_flow_obj_corners[1].x = optical_flow_obj_corners[1].x - distx/4;
                                        optical_flow_obj_corners[1].y = optical_flow_obj_corners[1].y + disty/4;

                                        optical_flow_obj_corners[3].x = optical_flow_obj_corners[3].x + distx/4;
                                        optical_flow_obj_corners[3].y = optical_flow_obj_corners[3].y - disty/4;*/


                                    }


           // automatic initialization
                        Mat mask = gray.clone();

                        if(!USE_SURF_OBJECT_LOCATION)
                        {
                                for(int i = 0; i < mask.rows;i++)
                                        for(int j = 0; j < mask.cols;j++)
                                                if(j >= myROI.x && j <= myROI.x+myROI.width
                                                        && (i>= myROI.y && i<= myROI.y+myROI.height) )
                                                                mask.at<char>(i,j) = 255;
                                                else
                                                                mask.at<char>(i,j) = 0;
                        }
                        else
                        {

                                if(optical_flow_obj_corners.size() > 3)
                                {
                                        for(int i = 0; i < mask.rows;i++)
                                                for(int j = 0; j < mask.cols;j++)
                                                        if(isInsideContour(optical_flow_obj_corners.size(),optical_flow_obj_corners,cvPoint2D32f(j,i)))
                                                                mask.at<char>(i,j) = 255;
                                                        else
                                                                mask.at<char>(i,j) = 0;
                                }


                        }



            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, mask, 3, true, 0.04);

                        if(points[1].size() > 0)
            cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);


                        of_model_points.clear();

                        for(int i = 0; i < points[1].size();i++)
                                of_model_points.push_back(points[1][i]);

                        optical_flow_first = false;


                        }
                        catch(...){ }

        }
         else if( !points[0].empty() )
        {
              vector<uchar> status;
            vector<float> err;

            if(prevGray.empty())
                gray.copyTo(prevGray);
                        calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                 3, termcrit, 0, 0.0001);
            size_t i, k;


            for( i = k = 0; i < points[1].size(); i++ )
            {

                if( !status[i] )
                    continue;

                points[1][k++] = points[1][i];

                 if(!optical_flow_first && this->draw_marker_contour)
                circle( img_scene, points[1][i], 3, Scalar(0,255,0), -1, 8);

				

            }
            points[1].resize(k);
        }


            needToInit = false;

                if(!optical_flow_first && points[1].size() > 0 && of_model_points.size() > 0)
                {

                       //  cv::redirectError(handleError);

                        try
                        {
                                int k = min(points[1].size(),of_model_points.size());

                                points[1].resize(k);
                                of_model_points.resize(k);

                                Mat H; // homography matrix.


								H = findHomography( of_model_points, points[1], CV_LMEDS,100 );

                        //-- Get the corners from the image_1 ( the object to be "detected" )
                          std::vector<Point2f> obj_corners(4);

                          if(!USE_SURF_OBJECT_LOCATION)
                          {
                          obj_corners[0] = Point(myROI.x,myROI.y); obj_corners[1] = Point( myROI.x+myROI.width, myROI.y);
                          obj_corners[2] = Point( myROI.x+myROI.width, myROI.y+myROI.height ); obj_corners[3] = Point( myROI.x, myROI.y+myROI.height );
                          }
                          else
                          {

                                  obj_corners[0] = optical_flow_obj_corners[0];
                                  obj_corners[1] = optical_flow_obj_corners[1];
                                  obj_corners[2] = optical_flow_obj_corners[2];
                                  obj_corners[3] = optical_flow_obj_corners[3];

                          }

                         if(of_model_points.size() > 0)
                         perspectiveTransform( obj_corners, scene_corners, H);

                        }
                        catch(...){}


                }


                if(optical_flow_first)
                        cv::rectangle(img_scene,myROI,cvScalar(255,0,100),2);

        

        std::swap(points[1], points[0]);
		 swap(prevGray, gray);
	
        return scene_corners;

}




void apiAR::FindObject3DPose(std::vector<Point2f> Object2DPoints)
{

  cv::Mat Rvec;

/*  std::vector<Point3d> test;
  test.push_back(cvPoint3D32f(Object2DPoints[0].x,Object2DPoints[0].y,0));
  test.push_back(cvPoint3D32f(Object2DPoints[1].x,Object2DPoints[0].y,0));
  test.push_back(cvPoint3D32f(Object2DPoints[1].x,Object2DPoints[1].y,0));
  test.push_back(cvPoint3D32f(Object2DPoints[0].x,Object2DPoints[1].y,0));*/


  try
  {
          if(!first)
          {
			  cv::solvePnP(object_corners_3d,Object2DPoints,camMat,distCoeffs,raux,taux,true,CV_ITERATIVE);
          }
          else
          {
			  cv::solvePnP(object_corners_3d,Object2DPoints,camMat,distCoeffs,raux,taux,CV_ITERATIVE);
                        first = false;
          }

  raux.convertTo(Rvec,CV_32F);
  taux.convertTo(Tvec ,CV_32F);
  cv::Rodrigues(Rvec, rotMat);

 

  }
  catch(...){


  }


}



void apiAR::PlaneTracking(float * pixels, int lenght)
{

		if(!external_input)
        img_scene = cvQueryFrame(vidObj);
		else
			img_scene = MatFromFloatArray(pixels,lenght);

				
        vector<Point2f> scene_corners;


        if(tracking_enabled)
        {

        if(!USE_OPTICAL_FLOW)
        {
        try
        {
        if(!this->USE_GPU)
        scene_corners = GetCPUObjectCorners(img_scene);
//	else
//	scene_corners = GetGPUObjectCorners(img_scene);
        }
        catch(...){}
        }
        else
        scene_corners = GetOpticalFlowObjectCorners(img_scene);

        bool object_found = false;

        try
        {
        object_found = ObjectFound(scene_corners);
        }
        catch(...){}

        std::vector<Point2f> filtered_scene_corners;

        try
        {
        if(object_found)
        {

                object_not_found_counter = 0;
                object_lost = false;

                Mat prediction = KF.predict();

            measurement(0) = scene_corners[0].x;
                measurement(1) = scene_corners[0].y;
                measurement(2) = scene_corners[1].x;
                measurement(3) = scene_corners[1].y;
                measurement(4) = scene_corners[2].x;
                measurement(5) = scene_corners[2].y;
                measurement(6) = scene_corners[3].x;
                measurement(7) = scene_corners[3].y;

                Mat estimated = KF.correct(measurement);

                filtered_scene_corners.push_back(cvPoint2D32f(estimated.at<float>(0),estimated.at<float>(1)));
                filtered_scene_corners.push_back(cvPoint2D32f(estimated.at<float>(2),estimated.at<float>(3)));
                filtered_scene_corners.push_back(cvPoint2D32f(estimated.at<float>(4),estimated.at<float>(5)));
                filtered_scene_corners.push_back(cvPoint2D32f(estimated.at<float>(6),estimated.at<float>(7)));


        }
        else
        {
            object_not_found_counter++;

                first = true;

                   if(object_not_found_counter < 30)
                   {
                           Mat prediction = KF.predict();

                          filtered_scene_corners.push_back(cvPoint2D32f(prediction.at<float>(0),prediction.at<float>(1)));
                          filtered_scene_corners.push_back(cvPoint2D32f(prediction.at<float>(2),prediction.at<float>(3)));
                          filtered_scene_corners.push_back(cvPoint2D32f(prediction.at<float>(4),prediction.at<float>(5)));
                          filtered_scene_corners.push_back(cvPoint2D32f(prediction.at<float>(6),prediction.at<float>(7)));
                   }
        }
        }
        catch(...){}

        if(draw_marker_contour)
        {
                try
                {

                Scalar color;

                if(USE_GPU)
                        color = cvScalar(0,255,0);
                else
                        color = cvScalar(255,0,0);

                if(USE_KALMAN_FILTERING)
                {
                    color = cvScalar(200,255,50);

                if(filtered_scene_corners.size() > 0)
                {
                line( img_scene, filtered_scene_corners[0], filtered_scene_corners[1], color, 4 );
                line( img_scene, filtered_scene_corners[1], filtered_scene_corners[2], color, 4 );
                line( img_scene, filtered_scene_corners[2], filtered_scene_corners[3], color, 4 );
                line( img_scene, filtered_scene_corners[3], filtered_scene_corners[0], color, 4 );
                }
                }
                else if(scene_corners.size() > 0)
                {
                line( img_scene, scene_corners[0], scene_corners[1], color, 4 );
                line( img_scene, scene_corners[1], scene_corners[2], color, 4 );
                line( img_scene, scene_corners[2], scene_corners[3], color, 4 );
                line( img_scene, scene_corners[3], scene_corners[0], color, 4 );

                circle(img_scene,scene_corners[0],5,cvScalar(255,0,255),2);
                circle(img_scene,scene_corners[2],5,cvScalar(255,255,0),2);


                }

                }
                catch(...){}
        }

        if(object_not_found_counter < 30)
        {
                try
                {

                if(USE_KALMAN_FILTERING)
                {
                FindObject3DPose(filtered_scene_corners);
                }
                else
                {


                FindObject3DPose(scene_corners);

                }

                if(show_test_3D_Object)
                        RenderTest3DObject();
                }
                catch(...){}

				

        }
        else
        {
            object_lost = true;

            if(USE_OPTICAL_FLOW && object_not_found_counter > 30 && object_not_found_counter < 50)
                {

                        needToInit = true;

                }
                else if(USE_OPTICAL_FLOW && object_not_found_counter >= 50)
                {
                        if(object_not_found_counter % 100 == 0)
                                needToInit = true;

                }




        }
		if(frame_counter == 50)
		{
			frame_counter = 0;
			needToInit = true;
		}

		frame_counter++;
    }

    //    cv::imshow("3D Object Test",img_scene);
    //    cvWaitKey(1);

}

void apiAR::RenderTest3DObject()
{

           // GLfloat vertices[] =
//{
//  0,0,0,			c_width,0,0,			c_width,c_height,0,				0,c_height,0,
//  0,0,c_deep,		c_width,0,c_deep,		c_width,c_height,c_deep,		0,c_height,c_deep,
//  0,0,0,			c_width,0,0,			c_width,0,c_deep,				0,0,c_deep,
//  0,c_height,0,		c_width,c_height,0,		c_width,c_height,c_deep,		0,c_height,c_deep,
//  0,0,0,			0,c_height,0,			0,c_height,c_deep,				0,0,c_deep,
//  c_width,0,0,		c_width,c_height,0,		c_width,c_height,c_deep,		c_width,0,c_deep

//};

//GLfloat colors[] =
//{
//0, 0, 0,   0, 0, 1,   0, 1, 1,   0, 1, 0,
//1, 0, 0,   1, 0, 1,   1, 1, 1,   1, 1, 0,
//0, 0, 0,   0, 0, 1,   1, 0, 1,   1, 0, 0,
//0, 1, 0,   0, 1, 1,   1, 1, 1,   1, 1, 0,
//0, 0, 0,   0, 1, 0,   1, 1, 0,   1, 0, 0,
//0, 0, 1,   0, 1, 1,   1, 1, 1,   1, 0, 1
//};

//        glfwGetFramebufferSize(window, &image_width, &image_height);


//        glViewport(0, 0, image_width, image_height);
//        glClear(GL_COLOR_BUFFER_BIT);

//        Mat_<double> persp;
//        double _near = 1.0, _far = 100.0;

//        glMatrixMode(GL_PROJECTION);
//        if(persp.empty()) {
//            persp.create(4,4); persp.setTo(0);

//            persp(0,0) = fx/cx;
//            persp(1,1) = fy/cy;
//            persp(2,2) = -(_far+_near)/(_far-_near);
//            persp(2,3) = -2.0*_far*_near / (_far-_near);
//            persp(3,2) = -1.0;

//            persp = persp.t(); //to col-major
//        }

//        glLoadMatrixd((double*)persp.data);


//        glMatrixMode(GL_MODELVIEW);

//        glPushMatrix();
//                double m[16] = { rotMat(0),-rotMat(3),-rotMat(6),0,
//                                                 rotMat(1),-rotMat(4),-rotMat(7),0,
//                                                 rotMat(2),-rotMat(5),-rotMat(8),0,
//                                                 Tvec(0),-Tvec(1),-Tvec(2),1
//                                                };



//                //Rotate and translate according to result from solvePnP
//        glLoadMatrixd(m);

//        //	glLoadIdentity();


//        /* We have a color array and a vertex array */
//                glEnableClientState(GL_VERTEX_ARRAY);
//                glEnableClientState(GL_COLOR_ARRAY);
//                glVertexPointer(3, GL_FLOAT, 0, vertices);
//                glColorPointer(3, GL_FLOAT, 0, colors);

//                /* Send data : 24 vertices */
//                glDrawArrays(GL_QUADS, 0, 24);

//        /*Draw OpenGl to OpenCv image*/
//                cv::Mat img(image_height, image_width, CV_8UC3);

//                //use fast 4-byte alignment (default anyway) if possible
//                glPixelStorei(GL_PACK_ALIGNMENT, (img.step & 3) ? 1 : 4);

//                //set length of one complete row in destination data (doesn't need to equal img.cols)
//                glPixelStorei(GL_PACK_ROW_LENGTH, img.step/img.elemSize());


//                glReadPixels(0, 0, img.cols, img.rows, GL_BGR, GL_UNSIGNED_BYTE, img.data);



//                cv:flip(img,img,0);
//                cv::add(img_scene,img,img_scene);


//                /* Cleanup states */
//                glDisableClientState(GL_COLOR_ARRAY);
//                glDisableClientState(GL_VERTEX_ARRAY);


//        glfwSwapBuffers(window);
//        glfwPollEvents();




}

double apiAR::angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

bool apiAR::ObjectFound(vector<Point2f> vertices)
{
         double maxCosine = 0;

     for( int j = 2; j < 5; j++ )
     {
     // find the maximum cosine of the angle between joint edges
     double cosine = fabs(angle(vertices[j%4], vertices[j-2], vertices[j-1]));
     maxCosine = MAX(maxCosine, cosine);
     }

     // if cosines of all angles are small
     // (all angles are ~90 degree) then write quandrange
     // vertices to resultant sequence
     if( maxCosine < 0.5 & maxCosine > 0 )
       return true;


        return false;
}


Mat  apiAR::MatFromFloatArray(float * pixels, int lenght)
{

	Mat_<float> test_img(480,640);

	Mat result(480,640,CV_8U);
	
    for(int i = 0; i < lenght;i++)
		test_img(i) = pixels[i];
	
	cv::flip(test_img,test_img,0);
	
	test_img.convertTo(result,CV_8U,255,0);

	return result.clone();
}


char* apiAR::ReturnString()
{
	static char s_chString[] = "Hello there";
	std::cout << "ReturnString() was called" << std::endl;

	return s_chString;
}
