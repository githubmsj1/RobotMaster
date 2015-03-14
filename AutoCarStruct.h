#ifndef AUTO_CAR_STRUCT
#define AUTO_CAR_STRUCT

#include <opencv2\opencv.hpp> 
using namespace cv;
using namespace std;


typedef struct 
{
	CvPoint center;
	CvRect bounding;
	RotatedRect boundingR;
	int isObject;
}DetectObject;


#endif

