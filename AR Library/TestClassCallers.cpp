#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
#include <windows.h>

#include "TestClassCallers.h"


extern "C" EXAMPLEUNMANAGEDDLL_API apiAR* CreateTestClass(char * object_image_path, int w, int h,int algortihm_type, bool _external)
{
	return new apiAR(object_image_path,w,h,algortihm_type,_external);
}

extern "C" EXAMPLEUNMANAGEDDLL_API void DisposeTestClass(apiAR* pObject)
{
	if(pObject != NULL)
	{
		delete pObject;
		pObject = NULL;
	}
}

extern "C" EXAMPLEUNMANAGEDDLL_API void DoMarkerTracking(apiAR* pObject,float * pixels, int lenght)
{
	if(pObject != NULL)
	{
		pObject->PlaneTracking(pixels,lenght);
	}
}

extern "C" EXAMPLEUNMANAGEDDLL_API void CallGetTx(apiAR* pObject, float * t_vec)
{

	if(pObject != NULL)
	{
		
		float * data = new float[3];

		Mat_<float> T = pObject->GetTranslationVector();

		data[0] = T(0);
		data[1] = T(1);
		data[2] = T(2);

		memcpy(t_vec,data,3*sizeof(float));

	}


}

extern EXAMPLEUNMANAGEDDLL_API void CallGetRotMat(apiAR* pObject, float * rot_mat)
{


	if(pObject != NULL)
	{
		
		float * data = new float[9];

		Mat_<float> T = pObject->GetRotationMatrix();

		data[0] = T(0);
		data[1] = T(1);
		data[2] = T(2);
		data[3] = T(3);
		data[4] = T(4);
		data[5] = T(5);
		data[6] = T(6);
		data[7] = T(7);
		data[8] = T(8);
				
		memcpy(rot_mat,data,9*sizeof(float));

	}





}


extern EXAMPLEUNMANAGEDDLL_API void CallGetFrame(apiAR* pObject, float * red, float * green, float * blue)
{


	Mat img = pObject->GetSceneImage();

	Mat result(480,640,CV_32F);

	img.convertTo(result,CV_32F);

	flip(result,result,0);

	Mat channels[3];
	
	split(result,channels);

	float * datar = new float[307200];
	float * datag = new float[307200];
	float * datab = new float[307200];

	int index = 0;

	for(int i = 0; i < 640;i++)
		for(int j = 0; j < 480;j++)
		{
			datar[index] = (channels[0].at<float>(j,i));
			datag[index] = (channels[1].at<float>(j,i));
			datab[index] = (channels[2].at<float>(j,i));

			index++;
		}
	
	
	
	memcpy(red,datar,307200*sizeof(float));
	memcpy(green,datag,307200*sizeof(float));
	memcpy(blue,datab,307200*sizeof(float));

	delete datar;
	delete datag;
	delete datab;

}

extern "C" EXAMPLEUNMANAGEDDLL_API char* CallReturnString(apiAR* pObject)
{
	if(pObject != NULL)
	{
		return pObject->ReturnString();
	}

	return NULL;
}

extern EXAMPLEUNMANAGEDDLL_API bool CallObjectFound(apiAR* pObject)
{

		
	if(pObject != NULL)
	{
		return !pObject->isObjectLost();
	}

	return false;



}

extern EXAMPLEUNMANAGEDDLL_API void CallSetKalmanFiltering(apiAR* pObject, bool value)
{

	if(pObject != NULL)
	{
		pObject->SetKalmanFiltering(value);
	}



}




extern EXAMPLEUNMANAGEDDLL_API void CallSetContourDrawing(apiAR* pObject, bool value)
{

	if(pObject != NULL)
	{
		pObject->SetObjectContourLines(value);
	}

}

void CallMatFromFloatArray(apiAR* pObject,float * pixels, int lenght)
{

	if(pObject != NULL)
	{
	   pObject->MatFromFloatArray(pixels,lenght);
	}



}