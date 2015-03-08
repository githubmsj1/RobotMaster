#include <opencv2\opencv.hpp>  
#include <iostream>
#include "Serial.h" 
#include <TCHAR.H>
#include <windows.h>
#include "AutoCarStruct.h"

#define DEBUG 1
#define DEBUG_WITHOUT_PORT 0
#define RUNING -1
#define RUN_STATUS DEBUG_WITHOUT_PORT

#define GROUND_RED 0 
#define GROUND_BLUE 1
#define GROUND_DEFAULT GROUND_BLUE

#define VIDEO 0
#define CAMERA 1
#define PICTURE 2
#define IMG_SOURCE VIDEO

#define DELAY_TIME 5000

#define NCOM "COM3"

using namespace cv;
using namespace std;


int ground = 0;
int isShowDebug = 1;
int isUartOpen = 1;
//CvPoint center_point[3];
//CvRect bounding[3];
DetectObject candidateObj[3];
const char *srcPath = "6.jpg";
const char *videoPath="2.flv";

//Declare
int Configure(void);
int Detect_enemy(void);
void ConnectedComponents(IplImage *mask, int poly1_hull0, float perimScale, int number,DetectObject candidateObj[]);
IplImage* doPyrDown(IplImage* in);
int ChooseTarget(DetectObject candidateObj[],DetectObject* detectResult,CvPoint* target);

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

	cvNamedWindow("Source", 1);
	cvMoveWindow("Source", 480, 0);


	//Choose the image source
	if(IMG_SOURCE==CAMERA)
		capture = cvCreateCameraCapture(1);
	else if(IMG_SOURCE==VIDEO)
		capture = cvCreateFileCapture(videoPath);
	
	
	//Defult ground type choose  
	ground = GROUND_DEFAULT;
	if (ground == GROUND_BLUE)
		printf("����ǿ��ѡ��ɹ�������\n");
	else if (ground == GROUND_RED)
		printf("����ǿ��ѡ��ɹ����쳡\n");


	cvWaitKey(1000);

	
	

    //Open and the uart and read the ground type 
	if(RUN_STATUS!=DEBUG_WITHOUT_PORT)
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
		//Start to count
		time_cnt = (double)cvGetTickCount();
		
		if(IMG_SOURCE==CAMERA||IMG_SOURCE==VIDEO)
			src = cvQueryFrame(capture);
		else if(IMG_SOURCE==PICTURE)
			src = cvLoadImage(srcPath, CV_LOAD_IMAGE_UNCHANGED);  

		Detect_enemy();

		//Draw 3 rects

		

		cvNamedWindow("Sel",1);
		cvShowImage("Sel",sel_image);


		DetectObject* targetObj=new DetectObject;
		ChooseTarget(candidateObj,targetObj,&target);
		
		if(target.x!=0||target.y!=0)
		{
			cvCircle(src,targetObj->center,8,cvScalar(0,255,0),-1);
			cvRectangle(src, cvPoint(candidateObj[0].bounding.x, candidateObj[0].bounding.y), cvPoint(candidateObj[0].bounding.x + candidateObj[0].bounding.width, candidateObj[0].bounding.y + candidateObj[0].bounding.height), cvScalar(0, 255, 0),4);
			cvRectangle(src, cvPoint(candidateObj[1].bounding.x, candidateObj[1].bounding.y), cvPoint(candidateObj[1].bounding.x + candidateObj[1].bounding.width, candidateObj[1].bounding.y + candidateObj[1].bounding.height), cvScalar(0, 255, 0),4);
			cvRectangle(src, cvPoint(candidateObj[2].bounding.x, candidateObj[2].bounding.y), cvPoint(candidateObj[2].bounding.x + candidateObj[2].bounding.width, candidateObj[2].bounding.y + candidateObj[2].bounding.height), cvScalar(0, 255, 0),4);
		}
		
		cvShowImage("Source", src);
		
		


		//Package the data
		send_buff[0] = 0xf2;
		send_buff[1] = (unsigned char) (target.x/4);//0-160
		send_buff[2] = (unsigned char) (target.y/4);//0-120

		//Send coordinate to car
		if(RUN_STATUS!=DEBUG_WITHOUT_PORT)
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
		printf("�������꣨%d,%d)", send_buff[1], send_buff[2]);
		if (time_ms > 34)
			printf("     ����ʱ��%dms\n", time_ms + 6);
		else
			printf("     ����ʱ��40ms\n");

		key=cvWaitKey(30);
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




int Configure(void)
{
	unsigned char send_buff = 0xf1,read_buff = 0;
	int serial_state = 0;

	serial_state = Serial_open(_T(NCOM), 115200);
	if (serial_state)
	{
		printf("�򿪴���ʧ��\n");
		for (int i = 0; i < 100; i++)
		{
			Beep(2000, 500);
			cvWaitKey(500);
		}
		return 1;
	}
	else
	{
		printf("�򿪴��ڳɹ����ȴ����ӡ�����\n");
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
						printf("���ؽ��ճɹ�:����\n");					
						
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
						printf("���ؽ��ճɹ�:�쳡\n");

						for (int i = 0; i < 3; i++)
						{
							Beep(2000, 200);
							cvWaitKey(200);
						}

						break;
					}
					else
					{
						printf("������Ϣ��������ͬ��\n");
					}
				}
			}
		}
	}

	return 0;
}

//ͼ������
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
			printf("����ͷ�������\n");
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

	//HSV
	//cvCvtColor(src, color_type, CV_BGR2HSV);
	//cvSplit(color_type, 0, 0, sel_channel, 0);

	//Binarization
	cvThreshold(sel_channel, sel_image, 180, 255.0, CV_THRESH_BINARY);

	//Interval Binarization
	//cvThreshold(sel_channel, tran_image, 100, 0, CV_THRESH_TOZERO);
	//cvThreshold(tran_image, sel_image, 150, 0, CV_THRESH_TOZERO_INV);
	//cvThreshold(sel_channel, sel_image, 100, 255.0, CV_THRESH_BINARY);


	////Clear the last result
	 memset(candidateObj, 0, sizeof(candidateObj));
	 //memset(bounding, 0, sizeof(bounding));
	 
	
	//Connection analysis,3 objects at most��poly fitter
	ConnectedComponents(sel_image, 1, 16, 3,candidateObj); 

	//object verified
	for (int i = 0; i < 3; i++)
	{
		if (1.2f*candidateObj[i].bounding.height >= candidateObj[i].bounding.width)
		{
			candidateObj[i].bounding.height = 0;
			candidateObj[i].bounding.width = 0;
			candidateObj[i].center.x = 0;
			candidateObj[i].center.y = 0;
		}
	}

	return 0;
}

//Just some convienience macros
#define CV_CVX_WHITE    CV_RGB(0xff,0xff,0xff)
#define CV_CVX_BLACK    CV_RGB(0x00,0x00,0x00)
/*
��ͨ������
����mask��ʾ������Ҫ������ͨ�����ֵͼ��
����poly1_hull0��ʾ������Ե�Ƿ���ö�������,����ò���Ϊ1�����ʾ���ö������ϣ��������͹����ϡ�
��������perimScale������ЩС������ȥ������ЩС������ʱָ�����ܳ�С��(mask��+��)/perimScale����Ȼ�������ڲ�����Ҳ���Ը�Ϊ������жϡ�
����num��ʾʵ����Ҫ�������������ĸ�������������mask�ж�������Ļ���������Ĵ�����ָ�������Щ��������Ӿ��κ����ĵ㡣Ĭ��ֵΪ0����ʾ�����ڲ�����Ҫ������Щ��Ӿ��κ����ĵ㡣
����bbs��ʾ���Ǵ�������Ӧ��������Ӿ��Σ�Ĭ��ֵΪRect()����ʾ����Ҫ������Щ��Ӿ��Ρ�
����centers��ʾ��������Ӧ���������ĵ����꣬Ĭ��ֵΪPoint(-1, -1)����ʾ����Ҫ������Щ���ĵ㡣
*/
void ConnectedComponents(IplImage *mask, int poly1_hull0, float perimScale, int number,DetectObject candidateObj[])
{
	/*����4�������Ϊ�˼���ԭ�����ӿڣ����ڲ�ʹ�õ���c��񣬵�����ӿ���c++����*/
	int *num = &number;
	static CvMemStorage*    mem_storage = NULL;
	static CvSeq*            contours = NULL;
	//CLEAN UP RAW MASK
	//Morphology Open
	cvMorphologyEx(mask, mask, NULL, NULL, CV_MOP_OPEN, 1);//������mask���п�������CVCLOSE_ITRΪ�������Ĵ��������Ϊmaskͼ��
	//Morphology Close
	cvMorphologyEx(mask, mask, NULL, NULL, CV_MOP_CLOSE, 1);//������mask���бղ�����CVCLOSE_ITRΪ�ղ����Ĵ��������Ϊmaskͼ��
	
	//FIND CONTOURS AROUND ONLY BIGGER REGIONS
	if (mem_storage == NULL) mem_storage = cvCreateMemStorage(0);
	else cvClearMemStorage(mem_storage);

	//CV_RETR_EXTERNAL=0����types_c.h�ж���ģ�CV_CHAIN_APPROX_SIMPLE=2Ҳ���ڸ��ļ��ж����
	CvContourScanner scanner = cvStartFindContours(mask, mem_storage, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	CvSeq* c;
	int numCont = 0;
	//��while�ڲ�ֻ��ԱȽϴ���������߽����滻����
	while ((c = cvFindNextContour(scanner)) != NULL)
	{
		double len = cvContourPerimeter(c);
		double q = (mask->height + mask->width) / perimScale;   //calculate perimeter len threshold
		if (len < q) //Get rid of blob if it's perimeter is too small
		{
			cvSubstituteContour(scanner, NULL);    //��NULL����ԭ�����Ǹ�����
		}
		else //Smooth it's edges if it's large enough
		{
			CvSeq* c_new;
			if (poly1_hull0) //Polygonal approximation of the segmentation
				c_new = cvApproxPoly(c, sizeof(CvContour), mem_storage, CV_POLY_APPROX_DP, 2, 0);
			else //Convex Hull of the segmentation
				c_new = cvConvexHull2(c, mem_storage, CV_CLOCKWISE, 1);
			cvSubstituteContour(scanner, c_new); //�ʼ��������͹�����߶���ʽ��������滻
			numCont++;
		}
	}

	contours = cvEndFindContours(&scanner);    //�����������Ҳ���
	// PAINT THE FOUND REGIONS BACK INTO THE IMAGE
	cvZero(mask);
	IplImage *maskTemp;

	//CALC CENTER OF MASS AND OR BOUNDING RECTANGLES
	if (*num != 0)
	{
		int N = *num, numFilled = 0, i = 0;
		CvMoments moments;
		double M00, M01, M10;
		maskTemp = cvCloneImage(mask);
		for (i = 0, c = contours; c != NULL; c = c->h_next, i++)        //h_nextΪ���������е���һ������
		{
			if (i < N) //Only process up to *num of them
			{
				//CV_CVX_WHITE�ڱ��������ǰ�ɫ����˼
				cvDrawContours(maskTemp, c, CV_CVX_WHITE, CV_CVX_WHITE, -1, CV_FILLED, 8);
				//Find the center of each contour
				if (1/*centers != &cvPoint(-1, -1)*/)
				{
					cvMoments(maskTemp, &moments, 1);    //����maskͼ�����ߴ�3�׵ľ�
					M00 = cvGetSpatialMoment(&moments, 0, 0); //��ȡx��0�κ�y��0�ξ�
					M10 = cvGetSpatialMoment(&moments, 1, 0); //��ȡx��1�κ�y��0�ξ�
					M01 = cvGetSpatialMoment(&moments, 0, 1); //��ȡx��0�κ�y��1�ξ�
					
					candidateObj[i].center.x= (int)(M10 / M00);    //���þصĽ��������������ĵ�����
					candidateObj[i].center.y= (int)(M01 / M00);
				}
				//Bounding rectangles around blobs

				candidateObj[i].bounding = cvBoundingRect(c); //�������c����Ӿ���

				cvZero(maskTemp);
				numFilled++;
			}
			//Draw filled contours into mask
			cvDrawContours(mask, c, CV_CVX_WHITE, CV_CVX_WHITE, -1, CV_FILLED, 8); //draw to central mask
		} //end looping over contours
		*num = numFilled;
		cvReleaseImage(&maskTemp);
	}
	//ELSE JUST DRAW PROCESSED CONTOURS INTO THE MASK
	else
	{
		for (c = contours; c != NULL; c = c->h_next)
		{
			cvDrawContours(mask, c, CV_CVX_WHITE, CV_CVX_BLACK, -1, CV_FILLED, 8);
		}
	}
}

