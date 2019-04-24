#ifndef __TestClassCallers_h__
#define __TestClassCallers_h__

#include "GmixonARapi.h"		// needed for EXAMPLEUNMANAGEDDLL_API

#ifdef __cplusplus
extern "C" {
#endif

extern EXAMPLEUNMANAGEDDLL_API apiAR* CreateTestClass(char * object_image_path, int w, int h,int algortihm_type, bool _external);
extern EXAMPLEUNMANAGEDDLL_API void  CallMatFromFloatArray(apiAR* pObject, float * pixels, int lenght);
extern EXAMPLEUNMANAGEDDLL_API void DisposeTestClass(apiAR* pObject);
extern EXAMPLEUNMANAGEDDLL_API void DoMarkerTracking(apiAR* pObject,float * pixels, int lenght);

extern EXAMPLEUNMANAGEDDLL_API char* CallReturnString(apiAR* pObject);

extern EXAMPLEUNMANAGEDDLL_API void CallGetTx(apiAR* pObject, float * t_vec);

extern EXAMPLEUNMANAGEDDLL_API void CallGetFrame(apiAR* pObject, float * red, float * green, float * blue);

extern EXAMPLEUNMANAGEDDLL_API void CallGetRotMat(apiAR* pObject, float * rot_mat);

extern EXAMPLEUNMANAGEDDLL_API bool CallObjectFound(apiAR* pObject);

extern EXAMPLEUNMANAGEDDLL_API void CallSetKalmanFiltering(apiAR* pObject, bool value);

extern EXAMPLEUNMANAGEDDLL_API void CallSetContourDrawing(apiAR* pObject, bool value);

#ifdef __cplusplus
}
#endif

#endif // __TestClassCallers_h__

