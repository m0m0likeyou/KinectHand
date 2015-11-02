#include "stdafx.h"
#include "opencv2/opencv.hpp"
#include <windows.h>
#include <iostream>
#include <fstream>
#include <Kinect.h>// Kinect Header files  也就是曾经的nuiapi.h
using namespace cv;
using namespace std;
ofstream outfile("E:\\test\\hand.txt", ios::in | ios::trunc);
// Safe release for interfaces
template<class Interface>   //什么鬼
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}
/*
int isXY(int x, int y)
{
	if (((x - 112)*(x - 112) + (y - 156)*(y - 156)) < 18) return 1;
	if (((x - 112)*(x - 112) + (y - 356)*(y - 356)) < 18) return 1;
	if (((x - 312)*(x - 312) + (y - 156)*(y - 156)) < 18) return 1;
	if (((x - 312)*(x - 312) + (y - 356)*(y - 356)) < 18) return 1;
	if (((x - 212)*(x - 212) + (y - 256)*(y - 256)) < 18) return 1;
	return 0;
}

int isZ(int x)
{
	if ((x > 990)  & (x < 1010)) return 1;
	if ((x > 1190) & (x < 1210)) return 1;
	if ((x > 1390) & (x < 1410)) return 1;
	if ((x > 1590) & (x < 1610)) return 1;
	if ((x > 1790) & (x < 1810)) return 1;
	if ((x > 1990) & (x < 2010)) return 1;
	return 0;
}

int isHand(Mat_<Vec4b> show,int x,int y)
{
	int judge = 0;
	for (int i = -1; i <= 1; i++)
		for (int j = -1; j <= 1; j++)
		{
			cv::Vec4b val = show.at<cv::Vec4b>(x+5*i, y+5*j);
			int red = val[0];
			int green = val[1];
			int blue = val[2];
			int light = (red * 299 + green * 587 + blue * 114) / 1000;
			if (light > 10) judge++;
		}
	if (judge > 7) return 1;
	return 0;
}*/

//定义Kinect方法类
class Kinect
{
public:
	static const int        cDepthWidth  = 512;
	static const int        cDepthHeight = 424;
	static const int        cColorWidth  = 1920;
	static const int        cColorHeight = 1080;
	Kinect();
	~Kinect();
	HRESULT					InitKinect();//初始化Kinect
	void					Update();	//更新数据
	void					ProcessDepth(const UINT16* pBuffer, RGBQUAD *cBuffer, int nWidth, int nHeight, int cWidth, int cHeight, USHORT nMinDepth, USHORT nMaxDepth);//处理得到的数据
private:
	
	IKinectSensor*          m_pKinectSensor;// Current Kinect
	IDepthFrameReader*      m_pDepthFrameReader;// depth reader
	IBodyFrameReader*		m_pBodyFrameReader;//Body reader   后加
	IColorFrameReader*		m_pColorFrameReader;
	ICoordinateMapper*		m_pMapper;         //map
	RGBQUAD*                m_pDepthRGBX;
	RGBQUAD*                m_pColorRGBX;
	CvPoint                 rhandpoint;
	int                     rhanddepth;
};

//主函数
int main()
{
	Kinect kinect;
	kinect.InitKinect();
	while(1)
	{
		kinect.Update();
		if(waitKey(1) >= 0)//按下任意键退出
		{
			break;
		}
	}

	return 0;
}

Kinect::Kinect()
{
	m_pKinectSensor = NULL;
	m_pDepthFrameReader = NULL; 
	m_pBodyFrameReader = NULL;    //后加
	m_pColorFrameReader = NULL;  //后加
	m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];// create heap storage for color pixel data in RGBX format
	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
}

Kinect::~Kinect()
{
	if (m_pDepthRGBX)
	{
		delete [] m_pDepthRGBX;
		m_pDepthRGBX = NULL;
		delete[] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}

	SafeRelease(m_pDepthFrameReader);// done with color frame reader
	SafeRelease(m_pBodyFrameReader);
	SafeRelease(m_pColorFrameReader);
//	SafeRelease(m_pMapper);

	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();// close the Kinect Sensor
	}
	SafeRelease(m_pKinectSensor);
}

HRESULT	Kinect::InitKinect()
{
	HRESULT hr;
	rhandpoint.x = 0;
	rhandpoint.y = 0;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get the depth reader
		IDepthFrameSource* pDepthFrameSource = NULL;
		IBodyFrameSource* pBodyFrameSource = NULL;   //后加
		IColorFrameSource* pColorFrameSource = NULL;

		hr = m_pKinectSensor->Open();


		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		}

		if (pDepthFrameSource!=NULL)
		{
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}
		SafeRelease(pDepthFrameSource);

		if (SUCCEEDED(hr))
		{
		INT32 nBodyNum = 0;                                          //后加
		m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);     //后加
		pBodyFrameSource->get_BodyCount(&nBodyNum);                  //后加
		pBodyFrameSource->OpenReader(&m_pBodyFrameReader);           //后加
		SafeRelease(pBodyFrameSource);                               //后加
		m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);   //后加
		pColorFrameSource->OpenReader(&m_pColorFrameReader);         //后加
		SafeRelease(pColorFrameSource);                              //后加
//		m_pKinectSensor->get_CoordinateMapper(&m_pMapper);
		}
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		printf("No ready Kinect found! \n");
		return E_FAIL;
	}

	return hr;
}

void Kinect::Update()
{
	if (!m_pDepthFrameReader)
	{
		return;
	}
	if (!m_pColorFrameReader)
	{
		return;
	}

	IDepthFrame* pDepthFrame = NULL;
	IBodyFrame*  pBodyFrame = NULL;
	IColorFrame* pColorFrame = NULL;

	HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);   //记得release IBodyFrame！！！
	if (pBodyFrame!=NULL)
	{
		IBody* pBodies[BODY_COUNT] = { 0 };
		pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBodies); // 更新所有人身体数据
		for (int i = 0; i < BODY_COUNT; ++i)
		{
			IBody* pBody = pBodies[i];
			if (pBody)
			{
				BOOLEAN bTracked = false;
				hr = pBody->get_IsTracked(&bTracked);//判断是否检测到这个人（因为bodyCount是6啊orz）
				if (bTracked)
				{
					Joint joints[JointType_Count];
					FLOAT xr, yr;
/*					HandState leftHandState = HandState_Unknown; //获取手的状态,用不上
					HandState rightHandState = HandState_Unknown;
					pBody->get_HandRightState(&rightHandState); */
					hr = pBody->GetJoints(_countof(joints), joints);
					//tran hand to depth
/*					rhandpoint.y = int(256 + 256 * joints[11].Position.X);//图像上Y是body的x
					rhandpoint.x = int(212 - 212 * joints[11].Position.Y);*/
					rhanddepth = joints[11].Position.Z * 1000;
					yr = -0.647*joints[11].Position.Z + 2.1453;  //转换系数
					xr = -0.6443*joints[11].Position.Z + 1.9619;
					rhandpoint.y = int(256 + 256 * joints[11].Position.X*xr);//图像上Y是body的x
					rhandpoint.x = int(212 - 212 * joints[11].Position.Y*yr);
					
				/*	if (isXY(rhandpoint.x, rhandpoint.y) )//用来校对body到depth
					{
						printf("Person %d : 右手 %2f %2f %2f\n", i, joints[11].Position.X, joints[11].Position.Y, joints[11].Position.Z * 1000);
						if (isZ(rhanddepth))
						{
							outfile << "RightHand  " << joints[11].Position.X << "   " << joints[11].Position.Y << "   " << joints[11].Position.Z * 1000 << endl;
							system("color F0");
							printf("Person %d : 右手 %2f %2f %2f\n", i, joints[11].Position.X, joints[11].Position.Y, joints[11].Position.Z * 1000);
							system("color 0F");
						}
					}*/
				}
			}
		}
	}
	SafeRelease(pBodyFrame);
	if (SUCCEEDED(hr))  hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);//跳过对color的检查（说起来这些函数的hr根本没用嘛- -
	if (SUCCEEDED(hr))  hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);//需要同时采集吗？
	if (SUCCEEDED(hr))  //这一堆在对depthFrame操作，获取其w,h以及最大最小置信范围。
	{
		IFrameDescription* pFrameDescription = NULL;
		int nWidth = 0;
		int nHeight = 0;
		int cWidth = 0;
		int cHeight = 0;
		USHORT nDepthMinReliableDistance = 0;
		USHORT nDepthMaxDistance = 0;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nBufferSize = 0;
        UINT16 *pBuffer = NULL;
		RGBQUAD *cBuffer = NULL;

//========================
		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_FrameDescription(&pFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Width(&cWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Height(&cHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}

		if (SUCCEEDED(hr))
		{
			//这个if是干嘛的真是F了doge
			//把数据从pColorFrame帧数据里放到pBuffer里来。
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&cBuffer));   //看不懂！！！xixi
			}
			else if (m_pColorRGBX)
			{
				cBuffer = m_pColorRGBX;
				nBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(cBuffer), ColorImageFormat_Bgra);
			}
			else
			{
				hr = E_FAIL;
			}
		}



	//========================

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Width(&nWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Height(&nHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
		}

		if (SUCCEEDED(hr))
		{
			//好好说中文不行吗  为得到深度完整范围（包括略不可信的远场深度），我们最大可能的设置了这个阈值。
			// In order to see the full range of depth (including the less reliable far field depth)
			// we are setting nDepthMaxDistance to the extreme potential depth threshold
			//nDepthMaxDistance = USHRT_MAX;
			//nDepthMaxDistance = 0x05f;
			//如果你只想得到可靠深度，就把我下面的下面这行的////去掉。你猜我去不去
			// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
			hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);            
		}



		if (SUCCEEDED(hr))
		{
			ProcessDepth(pBuffer,cBuffer, nWidth, nHeight, cWidth, cHeight, nDepthMinReliableDistance, nDepthMaxDistance);//加入color
			//ProcessDepth( pBuffer, nWidth, nHeight, rhanddepth-50, rhanddepth+50);//手前后
			//ProcessDepth(pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance);//都显示
		}

		SafeRelease(pFrameDescription);
	}

	SafeRelease(pDepthFrame);
	SafeRelease(pColorFrame);
}

void Kinect::ProcessDepth(const UINT16* pBuffer,RGBQUAD *cBuffer, int nWidth, int nHeight, int cWidth, int cHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	// Make sure we've received valid data
	if (m_pDepthRGBX && pBuffer /*&& cBuffer*/ && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
	{
		RGBQUAD* pRGBX = m_pDepthRGBX;

		// end pixel is start + width*height - 1
		const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);
		while (pBuffer < pBufferEnd)
		{
			USHORT depth = *pBuffer;

			//为了转换为一个字节，我们舍弃了大部分重要的位
			// To convert to a byte, we're discarding the most-significant
			// rather than least-significant bits.
			//我们保留了细节，尽管强度会比较渣， 可信距离外的值被设为黑色（0）
			// We're preserving detail, although the intensity will "wrap."
			// Values outside the reliable depth range are mapped to 0 (black).
			//注意：使用本循环中的三目运算符将会降低性能，写产品代码时考虑用查询表
			// Note: Using conditionals in this loop could degrade performance.
			// Consider using a lookup table instead when writing production code.

			BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth %256 ) : 0);

			pRGBX->rgbRed   = intensity;
			pRGBX->rgbGreen = intensity;
			pRGBX->rgbBlue  = intensity;

			++pRGBX;
			++pBuffer;
		}
		// Draw the data with OpenCV
		Mat DepthImage(nHeight, nWidth, CV_8UC4, m_pDepthRGBX);

		//Mat show = DepthImage.clone();
		Mat_<Vec4b> show = DepthImage.clone(); 
/*		for (int i = 0; i<DepthImage.rows; i++)
		{
			uchar* data = show.ptr<uchar>(i);
			for (int j = 0; j<DepthImage.cols; j++)
			{
				if (pow(double(i - rhandpoint.x), 2) + pow(double(j - rhandpoint.y), 2) - 900.0 > 0)
				{
					data[3*j] = 0;
					data[3 * j + 1] = 0;
					data[3 * j + 2] = 0;
				}
			}
		}*/
		DepthSpacePoint dpPoint;
		//ColorSpacePoint* clPoint = {};
//		for (int x = 0; x < DepthImage.rows; x++)//手部
//			for (int y = 0; y < DepthImage.cols; y++)
//				if (pow(double(x - rhandpoint.x), 2) + pow(double(y - rhandpoint.y), 2) - 3600.0 > 0.00000000001)
//					show(x, y) = Vec4b(0, 0, 0);
/*		for (int x = 0; x < DepthImage.rows; x++)//手部
			for (int y = 0; y < DepthImage.cols; y++)
				if (pow(double(x - rhandpoint.x), 2) + pow(double(y - rhandpoint.y), 2) - 3600.0 < 0.00000000001)
				{
					dpPoint.X = x;
					dpPoint.Y = y;
					m_pMapper->MapDepthPointToColorSpace(dpPoint, 1, clPoint);
				}*/
					
/*		if (isHand(show, 212, 256))    //用于采集数据校准body→depth
		{
			printf("Person : 右手 %2d,%2d,%2d\n",rhandpoint.x, rhandpoint.y, rhanddepth);
			if (isZ(rhanddepth))
			{
				outfile << "CT  " << rhandpoint.x << "  " << rhandpoint.y << "  " << rhanddepth << endl;
				system("color F0");
				printf("Person : 右手 %2d,%2d,%2d\n", rhandpoint.x, rhandpoint.y, rhanddepth);
				system("color 0F");
			}
		}
*/		
//以下4行用于绘点
		for (int x = 0; x < DepthImage.rows; x++)//手部
			for (int y = 0; y < DepthImage.cols; y++)
				if (pow(double(x - rhandpoint.x), 2) + pow(double(y - rhandpoint.y), 2) - 25.0 < 0.00000000001)             
					show(x, y) = Vec4b(0, 0, 255);
/*		for (int x = 0; x < DepthImage.rows; x++)//左上
			for (int y = 0; y < DepthImage.cols; y++)
				if (pow(double(x - 112), 2) + pow(double(y - 156), 2) - 100.0 < 0.00000000001)
					show(x, y) = Vec4b(255, 255, 118);

		for (int x = 0; x < DepthImage.rows; x++)//右上
			for (int y = 0; y < DepthImage.cols; y++)
				if (pow(double(x -112), 2) + pow(double(y - 356), 2) - 100.0 < 0.00000000001)
					show(x, y) = Vec4b(255, 255, 118);

		for (int x = 0; x < DepthImage.rows; x++)//左下
			for (int y = 0; y < DepthImage.cols; y++)
				if (pow(double(x - 312), 2) + pow(double(y - 156), 2) - 100.0 < 0.00000000001)
					show(x, y) = Vec4b(255, 255, 118);

		for (int x = 0; x < DepthImage.rows; x++)//右下
			for (int y = 0; y < DepthImage.cols; y++)
				if (pow(double(x - 312), 2) + pow(double(y - 356), 2) - 100.0 < 0.00000000001)
					show(x, y) = Vec4b(255, 255, 118);

		for (int x = 0; x < DepthImage.rows; x++)//中间
			for (int y = 0; y < DepthImage.cols; y++)
				if (pow(double(x - 212), 2) + pow(double(y - 256), 2) - 100.0 < 0.00000000001)
					show(x, y) = Vec4b(255, 255, 118);*/
		imshow("DepthImage", show);
	}
	if (cBuffer && (cWidth == cColorWidth) && (cHeight == cColorHeight))
	{
		// Draw the data with OpenCV
		Mat ColorImage(cHeight, cWidth, CV_8UC4, cBuffer);
		Mat showImage;
		resize(ColorImage, showImage, Size(cWidth / 2, cHeight / 2));
		imshow("ColorImage", showImage);////imshow("ColorImage", ColorImage);
	}
}
