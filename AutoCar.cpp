#include <opencv2\opencv.hpp>  
#include <iostream>
#include "Serial.h" 
#include <TCHAR.H>
#include <windows.h>
#include "AutoCarStruct.h"

using namespace cv;
using namespace std;

#define ENABLE 0 
#define DISABLE -1

#define DEBUG 1
#define RUNING -1
#define RUN_STATUS DEBUG


#if RUN_STATUS==DEBUG
#define SERIAL_PORT DISABLE
#define FRAME_TEST DISABLE
#else
#define SERIAL_PORT ENABLE
#define FRAME_TEST DISABLE
#endif

#define GROUND_RED 0 
#define GROUND_BLUE 1
#define GROUND_DEFAULT GROUND_RED

#define VIDEO 0
#define CAMERA 1//Note:remember to check the number of device
#define PICTURE 2
#define IMG_SOURCE CAMERA


#define DELAY_TIME 5000

#define NCOM "COM3"



int displayNum=0;
int ground = 0;
int isShowDebug = 1;
int isUartOpen = 1;
//CvPoint center_point[3];
//CvRect bounding[3];
DetectObject candidateObj[3];
const char *srcPath = "6.jpg";
const char *videoPath="v1.wmv";//"1.wmv";

//Declare
int Configure(void);
int Detect_enemy(void);
void ConnectedComponents(IplImage *mask, int poly1_hull0, float perimScale, int number,DetectObject candidateObj[]);
IplImage* doPyrDown(IplImage* in);
int ChooseTarget(DetectObject candidateObj[],DetectObject* detectResult,CvPoint* target);
int ChooseTargetR(DetectObject candidateObj[],DetectObject* detectResult,CvPoint* target);
void imshowN(const char *name,Mat src);
void drawRotateRect(Mat src,RotatedRect rRect,Scalar color);

#pragma region Calibrate parameter
////Calibrate parameter
//float i_matrixs[] = { 598.61, 0, 340.242, 0, 592.345, 238.53, 0, 0, 1 };
//float d_coeffs[] = { -0.00832889, 0.0690866, -0.00387945, 0.000132819 };
//CvMat intrinsic_matrix;
//CvMat distortion_coeffs;
#pragma endregion 


IplImage *src = 0, *ano_color_ima = 0, *sel_channel = 0, *tran_image = 0, *sel_image = 0;
CvCapture *capture;

int main()
{

	//camera calibration
	/*
	IplImage *src;
	IplImage *dst;
	src = cvLoadImage("10.jpg");
	cvInitMatHeader(&intrinsic_matrix,3,3, CV_32FC1, i_matrixs);
	cvInitMatHeader(&distortion_coeffs,1,4, CV_32FC1, d_coeffs);
	cvNamedWindow("src", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("dst", CV_WINDOW_AUTOSIZE);
	
	dst = cvCloneImage(src);
	cvUndistort2(src, dst, &intrinsic_matrix, &distortion_coeffs);
	cvShowImage("src", src);
	cvShowImage("dst",dst);
	cvWaitKey(0);
	*/


	int key = 0;
	unsigned char send_buff[10];
	double time_cnt = 0;
	int time_ms = 0;
	

	CvPoint target = cvPoint(0, 0);

	//cvNamedWindow("Source", 1);
	//cvMoveWindow("Source", 480, 0);


	//Choose the image source
	if(IMG_SOURCE==CAMERA)
		capture = cvCreateCameraCapture(1);
	else if(IMG_SOURCE==VIDEO)
		capture = cvCreateFileCapture(videoPath);
	
	
	//Defult ground type choose  
	ground = GROUND_DEFAULT;
	if (ground == GROUND_BLUE)
		printf("场地强制选择成功：蓝场\n");
	else if (ground == GROUND_RED)
		printf("场地强制选择成功：红场\n");


	cvWaitKey(1000);

	
	

    //Open and the uart and read the ground type 
	if(SERIAL_PORT==ENABLE)
	{
		if (isUartOpen)
		{
			if (Configure())
				return 1;
		}
	}
     

	//Main recognition
	while (true)
	{
		//Reset the display win number
		displayNum=-1;


		

		//Start to count
		time_cnt = (double)cvGetTickCount();
		
		if(IMG_SOURCE==CAMERA||IMG_SOURCE==VIDEO)
		{
			src = cvQueryFrame(capture);
			src = cvQueryFrame(capture);
		}
		else if(IMG_SOURCE==PICTURE)
			src = cvLoadImage(srcPath, CV_LOAD_IMAGE_UNCHANGED);  

		if(src==NULL)
		{
			waitKey(0);
			exit(0);
		}

		imshowN("Srouce",Mat(src));

		Detect_enemy();

		//Draw 3 rects

		

		

		DetectObject* targetObj=new DetectObject;
		ChooseTargetR(candidateObj,targetObj,&target);
		
		//Check whether some valid object is detected
		if(target.x!=0||target.y!=0)
		{
			for(int i=0;i<=2;i++)
			{
				if(candidateObj[i].isObject!=-1)
				{
					//Draw the candidate reactangle with pig
					cvRectangle(src, cvPoint(candidateObj[i].bounding.x, candidateObj[i].bounding.y), cvPoint(candidateObj[i].bounding.x + candidateObj[i].bounding.width, candidateObj[i].bounding.y + candidateObj[i].bounding.height), cvScalar(0, 100, 0),4);
					//Draw the supposed object with pig
					drawRotateRect(src,candidateObj[i].boundingR,Scalar(100,0,0));
				}
			}

			cvCircle(src,targetObj->center,8,cvScalar(0,255,0),-1);
			cvCircle(src,targetObj->boundingR.center,8,cvScalar(255,0,0),-1);

			cvRectangle(src, cvPoint(targetObj->bounding.x, targetObj->bounding.y), cvPoint(targetObj->bounding.x + targetObj->bounding.width, targetObj->bounding.y + targetObj->bounding.height), cvScalar(0, 255, 0),4);
			drawRotateRect(src,targetObj->boundingR,Scalar(255,0,0));
		}
		

		
		imshowN("Source", Mat(src));

		
		
		
		
		

		//Package the data
		send_buff[0] = 0xf2;
		send_buff[1] = (unsigned char) (target.x/4);//0-160
		send_buff[2] = (unsigned char) (target.y/4);//0-120

		//Send coordinate to car
		if(SERIAL_PORT==ENABLE)
		{
			if (isUartOpen)
			{
				Serial_write(&send_buff, 3);
			}
		}

		//End the tick and count the time spent
		time_cnt = (double)cvGetTickCount() - time_cnt;
		time_ms = time_cnt / cvGetTickFrequency() / 1000;


		//Display time
		printf("发送坐标（%d,%d)", send_buff[1], send_buff[2]);
		if (time_ms > 34)
			printf("     运行时间%dms\n", time_ms + 6);
		else
			printf("     运行时间40ms\n");


		//Delay for supervisor or pass when runing
		if(FRAME_TEST==DISABLE)
		{
			key=cvWaitKey(1);
		}
		else
		{
			key=cvWaitKey(-1);
		}
		//click q to quit
		if (key == 'q')
		{
			break;
		}
	}

	cvDestroyAllWindows();
	cvReleaseCapture(&capture);
}

//Choose the best target from candidate
int ChooseTarget(DetectObject candidateObj[],DetectObject* detectResult,CvPoint* target)
{
	if (candidateObj[0].center.x == 0 && candidateObj[0].center.y == 0 && candidateObj[1].center.x == 0 && candidateObj[1].center.y == 0
			&& candidateObj[2].center.x == 0 && candidateObj[2].center.y == 0)
		{
			target->x = 0;
			target->y = 0;
			
			
		}
	else if (candidateObj[0].bounding.height * candidateObj[0].bounding.width > candidateObj[1].bounding.height * candidateObj[1].bounding.width)
		{
			if (candidateObj[0].bounding.height * candidateObj[0].bounding.width > candidateObj[2].bounding.height * candidateObj[2].bounding.width)
			{
			target->x = candidateObj[0].center.x;
			target->y = candidateObj[0].center.y;
			*detectResult=candidateObj[0];
			}
			else
			{
				target->x = candidateObj[2].center.x;
				target->y = candidateObj[2].center.y;
				*detectResult=candidateObj[2];
			}
		}
		else
		{
			if (candidateObj[1].bounding.height * candidateObj[1].bounding.width > candidateObj[2].bounding.height * candidateObj[2].bounding.width)
			{
				target->x = candidateObj[1].center.x;
				target->y = candidateObj[1].center.y;
				*detectResult=candidateObj[1];
			}
			else
			{
				target->x = candidateObj[2].center.x;
				target->y = candidateObj[2].center.y;
				*detectResult=candidateObj[2];
			}
		}

	return 0;

}







//Choose the best target from candidateR(acroding the area)
int ChooseTargetR(DetectObject candidateObj[],DetectObject* detectResult,CvPoint* target)
{
	if (candidateObj[0].center.x == 0 && candidateObj[0].center.y == 0 && candidateObj[1].center.x == 0 && candidateObj[1].center.y == 0
			&& candidateObj[2].center.x == 0 && candidateObj[2].center.y == 0)
		{
			target->x = 0;
			target->y = 0;
			
			
		}
	else if (candidateObj[0].boundingR.size.height * candidateObj[0].boundingR.size.width > candidateObj[1].boundingR.size.height * candidateObj[1].boundingR.size.width)
		{
			if (candidateObj[0].boundingR.size.height * candidateObj[0].boundingR.size.width > candidateObj[2].boundingR.size.height * candidateObj[2].boundingR.size.width)
			{
			target->x = candidateObj[0].center.x;
			target->y = candidateObj[0].center.y;
			*detectResult=candidateObj[0];
			}
			else
			{
				target->x = candidateObj[2].center.x;
				target->y = candidateObj[2].center.y;
				*detectResult=candidateObj[2];
			}
		}
		else
		{
			if (candidateObj[1].boundingR.size.height * candidateObj[1].boundingR.size.width > candidateObj[2].boundingR.size.height * candidateObj[2].boundingR.size.width)
			{
				target->x = candidateObj[1].center.x;
				target->y = candidateObj[1].center.y;
				*detectResult=candidateObj[1];
			}
			else
			{
				target->x = candidateObj[2].center.x;
				target->y = candidateObj[2].center.y;
				*detectResult=candidateObj[2];
			}
		}

	return 0;

}

int Configure(void)
{
	unsigned char send_buff = 0xf1,read_buff = 0;
	int serial_state = 0;

	serial_state = Serial_open(_T(NCOM), 115200);
	if (serial_state)
	{
		printf("打开串口失败\n");
		for (int i = 0; i < 100; i++)
		{
			Beep(2000, 500);
			cvWaitKey(500);
		}
		return 1;
	}
	else
	{
		printf("打开串口成功，等待连接。。。\n");
		if (ground == 0)
		{
			while (true)
			{			
				cvWaitKey(10);
				Serial_write(&send_buff, 1);
				cvWaitKey(10);
				if (Serial_read(&read_buff, 1))
				{

					if (read_buff == GROUND_BLUE)
					{
						ground = GROUND_BLUE;
						printf("场地接收成功:蓝场\n");					
						
						for (int i = 0; i < 3; i++)
						{
							Beep(2000, 200);
							cvWaitKey(200);
						}
						
						break;
					}
					else if (ground == GROUND_RED)
					{
						ground = GROUND_RED;
						printf("场地接收成功:红场\n");

						for (int i = 0; i < 3; i++)
						{
							Beep(2000, 200);
							cvWaitKey(200);
						}

						break;
					}
					else
					{
						printf("场地信息错误，重新同步\n");
					}
				}
			}
		}
	}

	return 0;
}

//图像缩放
IplImage* doPyrDown(IplImage* in)
{
	assert(in->width % 2 == 0 && in->height % 2 == 0);

	IplImage* out = cvCreateImage(
		cvSize(in->width / 2, in->height / 2),
		in->depth,
		in->nChannels
		);
	cvPyrDown(in, out);
	return(out);
};
unsigned char cam_err = 0;

int Detect_enemy(void)
{
//	static unsigned char cam_err = 0;

	
	//check whether the image source is valid 
	if (!src)
	{
		cam_err++;
		if (cam_err == 10)
		{
			printf("摄像头错误错误\n");
			for (int i = 0; i < 100; i++)
			{
				Beep(2000, 500);
				cvWaitKey(500);
			}
			cvWaitKey(0);
		}
		return 1;
	}

	//src = doPyrDown(src);//zoom in the image

	if (!ano_color_ima)
	{
		ano_color_ima = cvCreateImage(cvGetSize(src), 8, 3);
		sel_channel = cvCreateImage(cvGetSize(src), 8, 1);
		sel_image = cvCreateImage(cvGetSize(src), 8, 1);
		tran_image = cvCreateImage(cvGetSize(src), 8, 1);
	}

	//RGB
	//color_type = src;
	//cvSplit(color_type, 0, 0, sel_channel, 0);

	//YCrCb
	cvCvtColor(src, ano_color_ima, CV_BGR2YCrCb);
	if (ground == GROUND_RED)
	{
		cvSplit(ano_color_ima, 0, 0, sel_channel, 0);
	}
	else
	{
		cvSplit(ano_color_ima, 0, sel_channel, 0, 0);
	}

	imshowN("YCrCb",Mat(sel_channel));
	

	
	//HSV
	//cvCvtColor(src, color_type, CV_BGR2HSV);
	//cvSplit(color_type, 0, 0, sel_channel, 0);

	//Binarization
	cvThreshold(sel_channel, sel_image, 180, 255.0, CV_THRESH_BINARY);

	imshowN("YrCb_Binarization",Mat(sel_image));

	//Interval Binarization
	//cvThreshold(sel_channel, tran_image, 100, 0, CV_THRESH_TOZERO);
	//cvThreshold(tran_image, sel_image, 150, 0, CV_THRESH_TOZERO_INV);
	//cvThreshold(sel_channel, sel_image, 100, 255.0, CV_THRESH_BINARY);


	//Clear the last result
	 memset(candidateObj, 0, sizeof(candidateObj));
	 //memset(bounding, 0, sizeof(bounding));
	 
	
	//Connection analysis,3 objects at most，poly fitter
	ConnectedComponents(sel_image, 1, 16, 3,candidateObj); //Source bin image,Flag of method to approximate,Reference of scale of image size,Number of object to detect,Output

	


	//Filter to remove the cyclinder
	for (int i = 0; i < 3; i++)
	{
		if (1.2f*candidateObj[i].bounding.height >= candidateObj[i].bounding.width)
		{
			candidateObj[i].bounding.height = 0;
			candidateObj[i].bounding.width = 0;
			candidateObj[i].center.x = 0;
			candidateObj[i].center.y = 0;
			candidateObj[i].isObject=-1;
			candidateObj[i].boundingR.size.height=0;
			candidateObj[i].boundingR.size.width=0;
			
		}
		else
		{
			candidateObj[i].isObject=0;
		}
	}

	return 0;
}

//Just some convienience macros
#define CV_CVX_WHITE    CV_RGB(0xff,0xff,0xff)
#define CV_CVX_BLACK    CV_RGB(0x00,0x00,0x00)
/*
连通域处理函数
参数mask表示的是需要进行连通域处理二值图像。
参数poly1_hull0表示轮廓边缘是否采用多边形拟合,如果该参数为1，则表示采用多边形拟合，否则采用凸包拟合。
参数是用perimScale来将那些小的轮廓去掉，那些小的轮廓时指它的周长小于(mask长+宽)/perimScale。当然你在其内部代码也可以该为面积来判断。
参数num表示实际需要处理最多的轮廓的个数（如果输入的mask有多个轮廓的话），这里的处理是指计算出这些轮廓的外接矩形和中心点。默认值为0，表示函数内部不需要处理这些外接矩形和中心点。
参数bbs表示的是处理完后对应轮廓的外接矩形，默认值为Rect()，表示不需要返回这些外接矩形。
参数centers表示处理完后对应轮廓的中心点坐标，默认值为Point(-1, -1)，表示不需要返回这些中心点。
*/
void ConnectedComponents(IplImage *mask, int poly1_hull0, float perimScale, int number,DetectObject candidateObj[])
{
	/*下面4句代码是为了兼容原函数接口，即内部使用的是c风格，但是其接口是c++风格的*/
	int *num = &number;
	static CvMemStorage* mem_storage = NULL;
	static CvSeq* contours = NULL;
	//CLEAN UP RAW MASK
	//Morphology Open
	cvMorphologyEx(mask, mask, NULL, NULL, CV_MOP_OPEN, 1);//对输入mask进行开操作，CVCLOSE_ITR为开操作的次数，输出为mask图像
	
	//imshowN("Open operation",Mat(mask));
	//Morphology Close
	cvMorphologyEx(mask, mask, NULL, NULL, CV_MOP_CLOSE, 1);//对输入mask进行闭操作，CVCLOSE_ITR为闭操作的次数，输出为mask图像
	//imshowN("Close operation",Mat(mask));

	//FIND CONTOURS AROUND ONLY BIGGER REGIONS
	if (mem_storage == NULL) 
		mem_storage = cvCreateMemStorage(0);
	else 
		cvClearMemStorage(mem_storage);
	
	imshowN("Morphology Operation",Mat(mask));
	
	//CV_RETR_EXTERNAL=0是在types_c.h中定义的，CV_CHAIN_APPROX_SIMPLE=2也是在该文件中定义的
	CvContourScanner scanner = cvStartFindContours(mask, mem_storage, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);//Find the external contour
	//Here mask will be erased
	
	
	CvSeq* c;
	int numCont = 0;	

	//Substitue the big contour and delete the small ones
	while ((c = cvFindNextContour(scanner)) != NULL)
	{
		double len = cvContourPerimeter(c);//calculate perimeter
		double q = (mask->height + mask->width) / perimScale;   //calculate perimeter len threshold
		if (len < q/4) //Get rid of blob(small object) if it's perimeter is too small
		{
			cvSubstituteContour(scanner, NULL);    //Substitute the uneligible
		}
		else //Smooth it's edges if it's large enough
		{
			CvSeq* c_new;
			if (poly1_hull0) //Polygonal approximation of the segmentation,poly1_hull0 is a flag
				c_new = cvApproxPoly(c, sizeof(CvContour), mem_storage, CV_POLY_APPROX_DP, 2, 0);
			else //Convex Hull of the segmentation
				c_new = cvConvexHull2(c, mem_storage, CV_CLOCKWISE, 1);
			cvSubstituteContour(scanner, c_new); //最开始的轮廓用凸包或者多项式拟合曲线替换
			numCont++;
		}
	}

	contours = cvEndFindContours(&scanner);    //结束轮廓查找操作

	// PAINT THE FOUND REGIONS BACK INTO THE IMAGE
	cvZero(mask);
	IplImage *maskTemp;

	//Calculate the center of mass and bounding of surranding rectangle
	if (*num != 0)
	{
		int N = *num, numFilled = 0, i = 0;
		CvMoments moments;
		double M00, M01, M10;
		maskTemp = cvCloneImage(mask);
		for (i = 0, c = contours; c != NULL; c = c->h_next, i++)        //h_next为轮廓序列中的下一个轮廓
		{
			if (i < N) //Only process up to *num of them
			{
				//CV_CVX_WHITE在本程序中是白色的意思
				//maskTemp is to be the input of moment calculate,which just contain one contour
				cvDrawContours(maskTemp, c, CV_CVX_WHITE, CV_CVX_WHITE, -1, CV_FILLED, 8);
				//Find the center of each contour
				if (1/*centers != &cvPoint(-1, -1)*/)
				{
					cvMoments(maskTemp, &moments, 1);    //Calculate the moment,Moments moments( const CvArr* arr, CvMoments* moments, int binary=0 )
					M00 = cvGetSpatialMoment(&moments, 0, 0); //提取x的0次和y的0次矩
					M10 = cvGetSpatialMoment(&moments, 1, 0); //提取x的1次和y的0次矩
					M01 = cvGetSpatialMoment(&moments, 0, 1); //提取x的0次和y的1次矩
					
					candidateObj[i].center.x= (int)(M10 / M00);    //利用矩的结果求出轮廓的中心点坐标
					candidateObj[i].center.y= (int)(M01 / M00);
				}
				//Bounding rectangles around blobs

				candidateObj[i].bounding = cvBoundingRect(c); //Calcuate the rect of the contour
				CvBox2D bounding1=cvMinAreaRect2(c);
				candidateObj[i].boundingR=RotatedRect(bounding1);


				cvZero(maskTemp);
				numFilled++;
			}

			//Draw filled contours into mask
			cvDrawContours(mask, c, CV_CVX_WHITE, CV_CVX_WHITE, 1, 2, 8); //draw to central mask
			

		} //end looping over contours
		*num = numFilled;
		cvReleaseImage(&maskTemp);
	}
	//If no object is required,just draw the contour
	else
	{
		for (c = contours; c != NULL; c = c->h_next)
		{
			cvDrawContours(mask, c, CV_CVX_WHITE, CV_CVX_BLACK,1, 2, 8);
		}
	}

	imshowN("ConnectedComponents",Mat(sel_image));

}

void imshowN(const char *name,Mat src)
{
	static int row=0;
	static int colum=0;
	const int wNum=3;
	char displayStr[30];
	
	displayNum++;
	row=displayNum/wNum;
	colum=displayNum%wNum;

	
	sprintf(displayStr,"%d:%s",displayNum,name);
	imshow(displayStr,src);
	moveWindow(displayStr,colum*500,row*400);
}

void drawRotateRect(Mat src,RotatedRect rRect,Scalar color)
{
	Point2f vertices[4];
	rRect.points(vertices);

	for (int i = 0; i < 4; i++)
		line(src, vertices[i], vertices[(i+1)%4], color,7);


}