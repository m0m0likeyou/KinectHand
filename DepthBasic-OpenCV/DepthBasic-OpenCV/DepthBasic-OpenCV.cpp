#include "stdafx.h"
#include "opencv2/opencv.hpp"
#include <windows.h>
#include <iostream>
#include <fstream>
#include <Kinect.h>// Kinect Header files  也就是曾经的nuiapi.h
using namespace cv;
using namespace std;
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

//定义Kinect方法类
class Kinect
{
public:
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;
	static const int        cColorWidth = 1920;
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
	CameraSpacePoint        cam_rhandpoint;
	DepthSpacePoint         dps_rhandpoint;
};

//主函数
int main()
{
	Kinect kinect;
	kinect.InitKinect();
	while (1)
	{
		kinect.Update();
		if (waitKey(1) >= 0)//按下任意键退出
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
		delete[] m_pDepthRGBX;
		m_pDepthRGBX = NULL;
		delete[] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}

	SafeRelease(m_pDepthFrameReader);// done with color frame reader
	SafeRelease(m_pBodyFrameReader);
	SafeRelease(m_pColorFrameReader);
	SafeRelease(m_pMapper);

	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();// close the Kinect Sensor
	}
	SafeRelease(m_pKinectSensor);
}

HRESULT	Kinect::InitKinect()
{
	HRESULT hr;
	dps_rhandpoint.X = 0;
	dps_rhandpoint.Y = 0;
	cam_rhandpoint.X = 0;
	cam_rhandpoint.Y = 0;
	cam_rhandpoint.Z = 0;

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

		if (pDepthFrameSource != NULL)
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
			m_pKinectSensor->get_CoordinateMapper(&m_pMapper);
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
	if (pBodyFrame != NULL)
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

/*					HandState leftHandState = HandState_Unknown; //获取手的状态,用不上
					HandState rightHandState = HandState_Unknown;
					pBody->get_HandRightState(&rightHandState); */

//					yr =  -0.647*joints[11].Position.Z + 2.1453;  //转换系数  系统自带真好用/微笑
//					xr = -0.6443*joints[11].Position.Z + 1.9619;
//					dps_rhandpoint.Y = int(256 + 256 * joints[11].Position.X*xr);//图像上Y是body的x
//					dps_rhandpoint.X = int(212 - 212 * joints[11].Position.Y*yr);

					hr = pBody->GetJoints(_countof(joints), joints);
					cam_rhandpoint = joints[11].Position;
					m_pMapper->MapCameraPointToDepthSpace(cam_rhandpoint, &dps_rhandpoint);
//					cout << cam_rhandpoint.Z << endl;
				}
			}
		}
	}
	SafeRelease(pBodyFrame);

	if (SUCCEEDED(hr))  hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);//跳过对color的检查（说起来这些函数的hr根本没用嘛- -
	if (SUCCEEDED(hr))  hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);//需要同时采集吗？
	if (SUCCEEDED(hr))  
	{
		IFrameDescription* pFrameDescription = NULL;
		int nWidth = 0;
		int nHeight = 0;
		USHORT nDepthMinReliableDistance = 0;
		USHORT nDepthMaxDistance = 0;
		UINT nBufferSize = 0;
		UINT16 *pBuffer = NULL;

		int cWidth = 0;
		int cHeight = 0;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		RGBQUAD *cBuffer = NULL;

		//========================color
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
			//把数据从pColorFrame帧数据里放到pBuffer里来。
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&cBuffer));   //看不懂！！
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
		//================color

		//========================这一堆在对depthFrame操作，获取其w,h以及最大最小置信范围。

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
			//为得到深度完整范围（包括略不可信的远场深度），我们最大可能的设置了这个阈值。
			nDepthMaxDistance = USHRT_MAX;
			//nDepthMaxDistance = 0x05f;
			//如果你只想得到可靠深度，就把我下面的下面这行的////去掉。
			//hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
		}
		 
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		}



		if (SUCCEEDED(hr))
		{
			//ProcessDepth(pBuffer, cBuffer, nWidth, nHeight, cWidth, cHeight, int(cam_rhandpoint.Z*1000 - 50), int(cam_rhandpoint.Z * 1000 + 50));//加入color
		  //ProcessDepth( pBuffer, nWidth, nHeight, rhanddepth-50, rhanddepth+50);//手前后

		   ProcessDepth(pBuffer, cBuffer, nWidth, nHeight, cWidth, cHeight, nDepthMinReliableDistance, nDepthMaxDistance);//都显示
		}

		SafeRelease(pFrameDescription);
	}

	SafeRelease(pDepthFrame);
	SafeRelease(pColorFrame);
}

void Kinect::ProcessDepth(const UINT16* pBuffer, RGBQUAD *cBuffer, int nWidth, int nHeight, int cWidth, int cHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	// Make sure we've received valid data
	if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
		if (m_pColorRGBX &&cBuffer && (cWidth == cColorWidth) && (cHeight == cColorHeight))
		{
			cout << "1";
			RGBQUAD* pRGBX = m_pDepthRGBX;
			const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);
			const UINT16* qBuffer = pBufferEnd - (nWidth * nHeight);
			USHORT ppBuffer[424][512] = {};
			ofstream outfile("E:\\test\\12345678.txt", ios::in | ios::trunc);
			int i = 0;
			int j = 0;

			for (int i = 0; i < nWidth; i++)
				for (int j = 0; j < nHeight; j++)
				{
//					ppBuffer[i][j] = depth;
					outfile << ushort(*qBuffer)<<"|";
					++qBuffer;
				}
//			pBuffer = pBufferEnd - (nWidth*nHeight);

			while (pBuffer < pBufferEnd)
			{
				USHORT depth = *pBuffer;
				BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 255):0);
				pRGBX->rgbRed = intensity;
				pRGBX->rgbGreen = intensity;
				pRGBX->rgbBlue = intensity;
//				outfile << ushort(depth)<<"|";
				++pRGBX;
				++pBuffer;
			}

			Mat DepthImage(nHeight, nWidth, CV_8UC4, m_pDepthRGBX);
			Mat_<Vec4b> show = DepthImage.clone();

/*			float sr;
			sr = pow((208.32*pow(2.71828182, -1.316*cam_rhandpoint.Z)), 2);
			for (int x = 0; x < DepthImage.rows; x++)//depth中手部以外抹黑
				for (int y = 0; y < DepthImage.cols; y++)
					if (pow(double(x - dps_rhandpoint.Y), 2) + pow(double(y - dps_rhandpoint.X), 2) - sr > 0.00001)
						show(x, y) = Vec4b(0, 0, 0);*/

			//以下用于手部的绘点
			/*		for (int x = 0; x < DepthImage.rows; x++)
			for (int y = 0; y < DepthImage.cols; y++)
				if (pow(double(x - rhandpoint.x), 2) + pow(double(y - rhandpoint.y), 2) - 25.0 < 0.00000000001)
					show(x, y) = Vec4b(0, 0, 255);*/

			//==============color image============

			// Draw the data with OpenCV

//			m_pColorRGBX = cBuffer;   //各种赋值方式...最后不是这里的问题- -
//			Mat ColorImage(cHeight, cWidth, CV_8UC4, m_pColorRGBX);
//			Mat_<Vec4b> ColorImages = ColorImage.clone();

			Mat ColorImage(cHeight, cWidth, CV_8UC4, cBuffer);
//			Mat ShowImage(cHeight, cWidth, CV_8UC4, Scalar(0, 0, 0));
			Mat_<Vec4b> ColorImages = ColorImage.clone();
//			Mat_<Vec4b> ShowImages = ShowImage.clone();

			DepthSpacePoint dpPoint;
			ColorSpacePoint cPoint;
			cPoint.X = 500;
			cPoint.Y = 500;
//			cout << ColorImages(cPoint.X, cPoint.Y) << endl;

			for (int x = 0; x < 100; x++)//手部  这一段准备将depth里手的点对应到color里
			{
//				uchar* data = ColorImage.ptr<uchar>(x);
				for (int y = 0; y < 100; y++)
				//	if (pow(double(x - dps_rhandpoint.X), 2) + pow(double(y - dps_rhandpoint.Y),0.000001)
				//		if (ppBuffer[x, y] > 0)
						{
							dpPoint.X = x;
							dpPoint.Y = y;
//							cout << ppBuffer[x, y]<<endl;
//							m_pMapper->MapDepthPointToColorSpace(dpPoint, ppBuffer[x, y], &cPoint);
//							ColorImages(cPoint.X, cPoint.Y)= Vec4b(0, 0, 255);
						}
			}

//			resize(showImage, showImag, Size(cWidth / 2, cHeight / 2));
			imshow("ColorImage", ColorImages);
			imshow("DepthImage", show);

//			ofstream outfile("E:\\test\\color.txt", ios::in | ios::trunc);
//			outfile << ColorImages[235,562]<<endl<<ShowImages[235, 562];
//			outfile << ColorImages(10, 80)[0] << endl;// << ColorImages(10, 80)[1] << endl;
		}
}
