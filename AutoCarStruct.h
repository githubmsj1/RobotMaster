#ifndef AUTO_CAR_STRUCT
#define AUTO_CAR_STRUCT

#include <opencv2\opencv.hpp>  

typedef struct 
{
	CvPoint center;
	CvRect bounding;
}DetectObject;


#endif
