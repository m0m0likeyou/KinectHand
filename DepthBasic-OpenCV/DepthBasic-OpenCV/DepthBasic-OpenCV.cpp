#include "stdafx.h"
#include "opencv2/opencv.hpp"
#include <windows.h>
#include <iostream>
#include <fstream>
#include <Kinect.h>// Kinect Header files  Ҳ����������nuiapi.h
using namespace cv;
using namespace std;
// Safe release for interfaces
template<class Interface>   //ʲô��
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

//����Kinect������
class Kinect
{
public:
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;
	static const int        cColorWidth = 1920;
	static const int        cColorHeight = 1080;
	Kinect();
	~Kinect();
	HRESULT					InitKinect();//��ʼ��Kinect
	void					Update();	//��������
//	void					ProcessDepth(const UINT16* pBuffer, RGBQUAD *cBuffer, int nWidth, int nHeight, int cWidth, int cHeight, USHORT nMinDepth, USHORT nMaxDepth);//����õ�������
	void					ProcessFrame(INT64 nTime,
										const UINT16* pDepthBuffer, int nDepthHeight, int nDepthWidth, USHORT nMinDepth, USHORT nMaxDepth,
										const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
										int nBodyCount, IBody** pBodies,
										const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight);
private:

	IKinectSensor*          m_pKinectSensor;// Current Kinect
	IMultiSourceFrameReader*m_pMultiSourceFrameReader;
//	IDepthFrameReader*      m_pDepthFrameReader;// depth reader
//	IBodyFrameReader*		m_pBodyFrameReader;//Body reader   ���
//	IColorFrameReader*		m_pColorFrameReader;
	ICoordinateMapper*		m_pMapper;         //map
	RGBQUAD*                m_pOutputRGBX;
	RGBQUAD*                m_pDepthRGBX;
	RGBQUAD*                m_pColorRGBX;
	RGBQUAD*                m_pBackgroundRGBX;
	CameraSpacePoint        cam_rhandpoint;
	DepthSpacePoint         dps_rhandpoint;
	ColorSpacePoint			clr_rhandpoint;
	DepthSpacePoint*        m_pDepthCoordinates;//��ȵ���ɫ��

	Mat						m_Depth;
	Mat						m_Color;
	Mat						m_BodyIndex;
};

//������
int main()
{
	Kinect kinect;
	kinect.InitKinect();
	while (1)
	{
		kinect.Update();
		if (waitKey(1) >= 0)//����������˳�
		{
			break;
		}
	}

	return 0;
}

Kinect::Kinect()
{
	m_pKinectSensor = NULL;
	m_pMultiSourceFrameReader = NULL;
//	m_pDepthFrameReader = NULL;
//	m_pBodyFrameReader = NULL;    //���
//	m_pColorFrameReader = NULL;  //���
	m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];// create heap storage for color pixel data in RGBX format
	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
	m_pOutputRGBX = new RGBQUAD[cColorWidth * cColorHeight];
	m_pBackgroundRGBX = new RGBQUAD[cColorWidth * cColorHeight];
	m_pDepthCoordinates = new DepthSpacePoint[cColorWidth * cColorHeight];
}

Kinect::~Kinect()
{
	if (m_pDepthRGBX)
	{
		delete[] m_pDepthRGBX;
		m_pDepthRGBX = NULL;
		delete[] m_pColorRGBX;
		m_pColorRGBX = NULL;
		delete[] m_pBackgroundRGBX;
		m_pBackgroundRGBX = NULL;
		delete[] m_pOutputRGBX;
		m_pOutputRGBX = NULL;
	}

//	SafeRelease(m_pDepthFrameReader);// done with color frame reader
//	SafeRelease(m_pBodyFrameReader);
//	SafeRelease(m_pColorFrameReader);
	SafeRelease(m_pMultiSourceFrameReader);
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
		// Initialize the Kinect and get coordinate mapper and the frame reader
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pMapper);
		}

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_Body | FrameSourceTypes::FrameSourceTypes_BodyIndex,
				&m_pMultiSourceFrameReader);
		}
	}
/*	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get the depth reader
//		IDepthFrameSource* pDepthFrameSource = NULL;
//		IBodyFrameSource* pBodyFrameSource = NULL;   //���
//		IColorFrameSource* pColorFrameSource = NULL;

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
			INT32 nBodyNum = 0;                                          //���
			m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);     //���
			pBodyFrameSource->get_BodyCount(&nBodyNum);                  //���
			pBodyFrameSource->OpenReader(&m_pBodyFrameReader);           //���
			SafeRelease(pBodyFrameSource);                               //���
			m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);   //���
			pColorFrameSource->OpenReader(&m_pColorFrameReader);         //���
			SafeRelease(pColorFrameSource);                              //���
			m_pKinectSensor->get_CoordinateMapper(&m_pMapper);
		}
	}*/

	if (!m_pKinectSensor || FAILED(hr))
	{
		printf("No ready Kinect found! \n");
		return E_FAIL;
	}
	return hr;
}

void Kinect::Update()
{
	if (!m_pMultiSourceFrameReader)
	{
		return;
	}
	IMultiSourceFrame* pMultiSourceFrame = NULL;
	IDepthFrame* pDepthFrame = NULL;
	IBodyFrame*  pBodyFrame = NULL;
	IBodyIndexFrame* pBodyIndexFrame = NULL;
	IColorFrame* pColorFrame = NULL;

	HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);   //�ǵ�release IBodyFrame������

	if (SUCCEEDED(hr))//�����Ϣ
	{
		IDepthFrameReference* pDepthFrameReference = NULL;
		hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
		}
		SafeRelease(pDepthFrameReference);
	}

	if (SUCCEEDED(hr))//��ɫ��Ϣ
	{
		IColorFrameReference* pColorFrameReference = NULL;
		hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pColorFrameReference->AcquireFrame(&pColorFrame);
		}
		SafeRelease(pColorFrameReference);
	}

	if (SUCCEEDED(hr))//������Ϣ
	{
		IBodyFrameReference* pBodyFrameReference = NULL;
		hr = pMultiSourceFrame->get_BodyFrameReference(&pBodyFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameReference->AcquireFrame(&pBodyFrame);
		}
		SafeRelease(pBodyFrameReference);
	}

	if (SUCCEEDED(hr))//������Ĥ����
	{
		IBodyIndexFrameReference* pBodyIndexFrameReference = NULL;
		hr = pMultiSourceFrame->get_BodyIndexFrameReference(&pBodyIndexFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrameReference->AcquireFrame(&pBodyIndexFrame);
		}
		SafeRelease(pBodyIndexFrameReference);
	}
	if (SUCCEEDED(hr))
	{
		INT64 nDepthTime = 0;
		IFrameDescription* pDepthFrameDescription = NULL;
		int nDepthWidth = 0;
		int nDepthHeight = 0;
		UINT nDepthBufferSize = 0;
		UINT16 *pDepthBuffer = NULL;
		USHORT nDepthMinReliableDistance = 0;
		USHORT nDepthMaxDistance = 0;

		IFrameDescription* pColorFrameDescription = NULL;
		int nColorWidth = 0;
		int nColorHeight = 0;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nColorBufferSize = 0;
		RGBQUAD *pColorBuffer = NULL;

		IBody* pBodies[BODY_COUNT] = { 0 };

		IFrameDescription* pBodyIndexFrameDescription = NULL;
		int nBodyIndexWidth = 0;
		int nBodyIndexHeight = 0;
		UINT nBodyIndexBufferSize = 0;
		BYTE *pBodyIndexBuffer = NULL;

// get depth frame data 
		hr = pDepthFrame->get_RelativeTime(&nDepthTime);

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameDescription->get_Width(&nDepthWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameDescription->get_Height(&nDepthHeight);
		}

		if (SUCCEEDED(hr))
		{
			;
			hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
			nDepthMinReliableDistance = cam_rhandpoint.Z * 1000 - 100;
		}

		if (SUCCEEDED(hr))
		{
//			nDepthMaxDistance = USHRT_MAX;
			nDepthMaxDistance = cam_rhandpoint.Z * 1000 + 100;
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);
		}

		m_Depth = Mat(nDepthHeight, nDepthWidth, CV_16UC1, pDepthBuffer).clone();//
// get color frame data
		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameDescription->get_Width(&nColorWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameDescription->get_Height(&nColorHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}

		if (SUCCEEDED(hr))//������
		{
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
			}
			else if (m_pColorRGBX)
			{
				pColorBuffer = m_pColorRGBX;
				nColorBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
			}
			else
			{
				hr = E_FAIL;
			}
		}

		m_Color = Mat(nColorHeight, nColorWidth, CV_8UC4, pColorBuffer);///////////////

		// get body index frame data
		if (SUCCEEDED(hr))
		{
			hr = pBodyFrame->GetAndRefreshBodyData(_countof(pBodies), pBodies);
		}
		if (SUCCEEDED(hr))
		{
			for (int i = 0; i < BODY_COUNT; ++i)
			{
				IBody* pBody = pBodies[i];
				if (pBody)
				{
					BOOLEAN bTracked = false;
					hr = pBody->get_IsTracked(&bTracked);//�ж��Ƿ��⵽����ˣ���ΪbodyCount��6��orz��
					if (bTracked)
					{
						Joint joints[JointType_Count];
//						HandState leftHandState = HandState_Unknown; //��ȡ�ֵ�״̬,�ò���
//						HandState rightHandState = HandState_Unknown;
//						pBody->get_HandRightState(&rightHandState); 
//						yr =  -0.647*joints[11].Position.Z + 2.1453;  //ת��ϵ��  ϵͳ�Դ������/΢Ц
//						xr = -0.6443*joints[11].Position.Z + 1.9619;
//						dps_rhandpoint.Y = int(256 + 256 * joints[11].Position.X*xr);//ͼ����Y��body��x
//						dps_rhandpoint.X = int(212 - 212 * joints[11].Position.Y*yr);
						pBody->GetJoints(_countof(joints), joints);
						cam_rhandpoint = joints[7].Position;
						m_pMapper->MapCameraPointToDepthSpace(cam_rhandpoint, &dps_rhandpoint);
						m_pMapper->MapCameraPointToColorSpace(cam_rhandpoint, &clr_rhandpoint);
					}
				}
			}
		}
		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrame->get_FrameDescription(&pBodyIndexFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrameDescription->get_Width(&nBodyIndexWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrameDescription->get_Height(&nBodyIndexHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrame->AccessUnderlyingBuffer(&nBodyIndexBufferSize, &pBodyIndexBuffer);
		}
		m_BodyIndex = Mat(nBodyIndexHeight, nBodyIndexWidth, CV_8UC1, pBodyIndexBuffer);

		if (SUCCEEDED(hr))
		{
			ProcessFrame(nDepthTime, pDepthBuffer, nDepthWidth, nDepthHeight, nDepthMinReliableDistance, nDepthMaxDistance,
				pColorBuffer, nColorWidth, nColorHeight,
				BODY_COUNT, pBodies,
				pBodyIndexBuffer, nBodyIndexWidth, nBodyIndexHeight);
		}

		SafeRelease(pDepthFrameDescription);
		SafeRelease(pColorFrameDescription);
		SafeRelease(pBodyIndexFrameDescription);

		for (int i = 0; i < _countof(pBodies); ++i)
		{
			SafeRelease(pBodies[i]);
		}
	}

	SafeRelease(pDepthFrame);
	SafeRelease(pColorFrame);
	SafeRelease(pBodyFrame);
	SafeRelease(pBodyIndexFrame);
	SafeRelease(pMultiSourceFrame);
//  ����ԭ����
/*	if (SUCCEEDED(hr))
	{
		IBody* pBodies[BODY_COUNT] = { 0 };
		pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBodies); // ������������������
		for (int i = 0; i < BODY_COUNT; ++i)
		{
			IBody* pBody = pBodies[i];
			if (pBody)
			{
				BOOLEAN bTracked = false;
				hr = pBody->get_IsTracked(&bTracked);//�ж��Ƿ��⵽����ˣ���ΪbodyCount��6��orz��
				if (bTracked)
				{
					Joint joints[JointType_Count];

/*					HandState leftHandState = HandState_Unknown; //��ȡ�ֵ�״̬,�ò���
					HandState rightHandState = HandState_Unknown;
					pBody->get_HandRightState(&rightHandState); *

//					yr =  -0.647*joints[11].Position.Z + 2.1453;  //ת��ϵ��  ϵͳ�Դ������/΢Ц
//					xr = -0.6443*joints[11].Position.Z + 1.9619;
//					dps_rhandpoint.Y = int(256 + 256 * joints[11].Position.X*xr);//ͼ����Y��body��x
//					dps_rhandpoint.X = int(212 - 212 * joints[11].Position.Y*yr);

					hr = pBody->GetJoints(_countof(joints), joints);
					cam_rhandpoint = joints[7].Position;
					m_pMapper->MapCameraPointToDepthSpace(cam_rhandpoint, &dps_rhandpoint);
					m_pMapper->MapCameraPointToColorSpace(cam_rhandpoint, &clr_rhandpoint);
//					cout << cam_rhandpoint.Z << endl;
				}
			}
		}
	}
	SafeRelease(pBodyFrame);

	if (SUCCEEDED(hr))  hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);//������color�ļ�飨˵������Щ������hr����û����- -
	if (SUCCEEDED(hr))  hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);//��Ҫͬʱ�ɼ���
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
			//�����ݴ�pColorFrame֡������ŵ�pBuffer������
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&cBuffer));   //����������
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

		//========================��һ���ڶ�depthFrame��������ȡ��w,h�Լ������С���ŷ�Χ��

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
			//Ϊ�õ����������Χ�������Բ����ŵ�Զ����ȣ������������ܵ������������ֵ��
			nDepthMaxDistance = USHRT_MAX;
			//nDepthMaxDistance = 0x05f;
			//�����ֻ��õ��ɿ���ȣ��Ͱ���������������е�////ȥ����
			//hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
		}
		 
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		}



		if (SUCCEEDED(hr))
		{
		  //ProcessDepth(pBuffer, cBuffer, nWidth, nHeight, cWidth, cHeight, int(cam_rhandpoint.Z*1000 - 50), int(cam_rhandpoint.Z * 1000 + 50));//����color
		  //ProcessDepth( pBuffer, nWidth, nHeight, rhanddepth-50, rhanddepth+50);//��ǰ��
		   ProcessDepth(pBuffer, cBuffer, nWidth, nHeight, cWidth, cHeight, int(cam_rhandpoint.Z * 1000 - 60), int(cam_rhandpoint.Z * 1000 + 60));//����ʾ
		}

		SafeRelease(pFrameDescription);
	}

	SafeRelease(pDepthFrame);
	SafeRelease(pColorFrame);*/
}

//void Kinect::ProcessDepth(const UINT16* pBuffer, RGBQUAD *cBuffer, int nWidth, int nHeight, int cWidth, int cHeight, USHORT nMinDepth, USHORT nMaxDepth)
void Kinect::ProcessFrame(INT64 nTime,
						const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight, USHORT nMinDepth, USHORT nMaxDepth,
						const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
						int nBodyCount, IBody** pBodies,
						const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight)
{
	// Make sure we've received valid data
//	if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
//		if (m_pColorRGBX &&cBuffer && (cWidth == cColorWidth) && (cHeight == cColorHeight))
	LARGE_INTEGER qpcNow = { 0 };
	WCHAR szStatusMessage[64];
	if (m_pMapper && m_pDepthCoordinates && m_pOutputRGBX &&
		pDepthBuffer && (nDepthWidth == cDepthWidth) && (nDepthHeight == cDepthHeight) &&
		pColorBuffer && (nColorWidth == cColorWidth) && (nColorHeight == cColorHeight) &&
		pBodyIndexBuffer && (nBodyIndexWidth == cDepthWidth) && (nBodyIndexHeight == cDepthHeight) &&
		m_pDepthRGBX)
	{
		//depth&color
		RGBQUAD* pRGBXX = m_pDepthRGBX;
		const UINT16* pBufferEnd = pDepthBuffer + (nDepthWidth * nDepthHeight);
		const UINT16* qBuffer = pBufferEnd - (nDepthWidth * nDepthHeight);

		while (qBuffer < pBufferEnd)
		{
			USHORT depth = *qBuffer;
			BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? 255 : 0);// (depth % 400) : 0);  //�Ѻڰ׸ĳɻ���,��Z�Ĳ������
			pRGBXX->rgbRed = intensity;// int(6.5*intensity);
			pRGBXX->rgbGreen = intensity;//220*(intensity > 0 ? 1 : 0);
			pRGBXX->rgbBlue = intensity;// 0;
			++pRGBXX;
			++qBuffer;
		}

		//================================
		//ͨ��depth���ֿٳ���
		Mat DepthImage(nDepthHeight, nDepthWidth, CV_8UC4, m_pDepthRGBX);
		float sr;
		sr = pow((208.32*pow(2.71828182, -1.316*cam_rhandpoint.Z)), 2);
		RGBQUAD* pDepthRGBXX = m_pDepthRGBX;
		for (int x = 0; x < DepthImage.rows; x++)//depth���ֲ�����Ĩ��
			for (int y = 0; y < DepthImage.cols; y++)
			{
				if (pow(double(x - dps_rhandpoint.Y), 2) + pow(double(y - dps_rhandpoint.X), 2) - sr > 0.00001)
				{
					pDepthRGBXX->rgbBlue = 0;
					pDepthRGBXX->rgbGreen = 0;
					pDepthRGBXX->rgbRed = 0;
				}
				++pDepthRGBXX;
			}
		Mat_<Vec4b> show = DepthImage.clone();
		cout << show(dps_rhandpoint.Y, dps_rhandpoint.X)<< endl;
		imshow("DEPTH", show);

		//===========================
		HRESULT hr = m_pMapper->MapColorFrameToDepthSpace(nDepthWidth * nDepthHeight, (UINT16*)pDepthBuffer, nColorWidth * nColorHeight, m_pDepthCoordinates);
		if (FAILED(hr))
		{
			return;
		}
		//��depthΪ����ѭ��
		for (int colorIndex = 0; colorIndex < (nColorWidth*nColorHeight); ++colorIndex)
		{
			// default setting source to copy from the background pixel
			const RGBQUAD* pSrc = m_pBackgroundRGBX + colorIndex;
			DepthSpacePoint p = m_pDepthCoordinates[colorIndex];
			// Values that are negative infinity means it is an invalid color to depth mapping so we
			// skip processing for this pixel
			if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
			{
				int depthX = static_cast<int>(p.X + 0.5f);
				int depthY = static_cast<int>(p.Y + 0.5f);
				if ((depthX >= 0 && depthX < nDepthWidth) && (depthY >= 0 && depthY < nDepthHeight))
				{
					BYTE player = pBodyIndexBuffer[depthX + (depthY * cDepthWidth)];
					uchar* p;
					p = DepthImage.ptr<uchar>(depthY);
					// if we're tracking a player for the current pixel, draw from the color camera

					if ((int)p[depthX * 4] != 0)
					{
						pSrc = m_pColorRGBX + colorIndex;
					}
				}
			}
			m_pOutputRGBX[colorIndex] = *pSrc;
		}

		/*
		//��BodyIndexΪ����ѭ��
		for (int colorIndex = 0; colorIndex < (nColorWidth*nColorHeight); ++colorIndex)
		{
			// default setting source to copy from the background pixel
			const RGBQUAD* pSrc = m_pBackgroundRGBX + colorIndex;

			DepthSpacePoint p = m_pDepthCoordinates[colorIndex];

			// Values that are negative infinity means it is an invalid color to depth mapping so we
			// skip processing for this pixel
			if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
			{
				int depthX = static_cast<int>(p.X + 0.5f);
				int depthY = static_cast<int>(p.Y + 0.5f);

				if ((depthX >= 0 && depthX < nDepthWidth) && (depthY >= 0 && depthY < nDepthHeight))
				{
					BYTE player = pBodyIndexBuffer[depthX + (depthY * cDepthWidth)];
					// if we're tracking a player for the current pixel, draw from the color camera
					if (player != 0xff)
					{
						pSrc = m_pColorRGBX + colorIndex;
					}
				}
			}
			m_pOutputRGBX[colorIndex] = *pSrc;
		}*/
	//ȷ��������׼ȷ

		 //imshow("color", m_Color);

	    Mat ColorImage( cColorHeight, cColorWidth, CV_8UC4, m_pOutputRGBX);
		Mat ColorImages;
		resize(ColorImage, ColorImages, Size(cColorWidth / 2, cColorHeight / 2));
		Mat showImage;
		resize(m_Color, showImage, Size(cColorWidth / 2, cColorHeight / 2));
		imshow("Color", showImage);////imshow("ColorImage", ColorImage);
//		imshow("Depth", m_Depth);
		imshow("BodyIndex", m_BodyIndex);
		imshow("Coloraa", ColorImages);
		waitKey(1);
	}
}
/* 		{
			RGBQUAD* pRGBX = m_pDepthRGBX;
			const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);
			const UINT16* qBuffer = pBufferEnd - (nWidth * nHeight);
//			USHORT ppBuffer[424][512] = {};
//			ofstream outfile("E:\\test\\12345678.txt", ios::in | ios::trunc);
			int i = 0;
			int j = 0;

//			for (int i = 0; i < nWidth; i++)
//				for (int j = 0; j < nHeight; j++)
//				{
//					outfile << ushort(*qBuffer)<<"|";
//					++qBuffer;
//				}

			while (pBuffer < pBufferEnd)
			{
				USHORT depth = *pBuffer;
				BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? 255 : 0);// (depth % 400) : 0);  //�Ѻڰ׸ĳɻ���,��Z�Ĳ������
				pRGBX->rgbRed = intensity;// int(6.5*intensity);
				pRGBX->rgbGreen = intensity;//220*(intensity > 0 ? 1 : 0);
				pRGBX->rgbBlue = intensity;// 0;
				++pRGBX;
				++pBuffer;
			}

			Mat DepthImage(nHeight, nWidth, CV_8UC4, m_pDepthRGBX);
			Mat_<Vec4b> show = DepthImage.clone();
			//show.at<int>(0, 1);
			float sr;
			sr = pow((208.32*pow(2.71828182, -1.316*cam_rhandpoint.Z)), 2);
			for (int x = 0; x < DepthImage.rows; x++)//depth���ֲ�����Ĩ��
				for (int y = 0; y < DepthImage.cols; y++)
				{
					if (pow(double(x - dps_rhandpoint.Y), 2) + pow(double(y - dps_rhandpoint.X), 2) - sr > 0.00001)
						show(x, y) = Vec4b(0, 0, 0);
				}
			
/*			cout << show(180, 150)<<endl;
			uchar* p;
			p =DepthImage.ptr<uchar>(180);
			cout << (int)p[150 * 4];  ��ɶ������õ�0ͨ��ֵ*/

			//��������������ֲ��Ļ��
/*			for (int x = 0; x < DepthImage.rows; x++)
				for (int y = 0; y < DepthImage.cols; y++)
					if (pow(double(x - rhandpoint.x), 2) + pow(double(y - rhandpoint.y), 2) - 25.0 < 0.00000000001)
						show(x, y) = Vec4b(0, 0, 255);
			//==============color image============

			// Draw the data with OpenCV
			Mat ColorImage(cHeight, cWidth, CV_8UC4, cBuffer);
			Mat_<Vec4b> ColorImages = ColorImage.clone();

//=================��������
			DOUBLE cr = 110.03*pow(cam_rhandpoint.Z, -1.27);
			cr = pow(cr, 2);
//     		for (int x = 0; x < ColorImages.rows; x++)//�ֲ�  ��������ɫ
//			{
//				for (int y = 0; y < ColorImages.cols; y++)
//					if (pow(double(x - clr_rhandpoint.Y), 2) + pow(double(y - clr_rhandpoint.X), 2) - cr > 0.00001)
//						ColorImages(x, y) = Vec4b(0, 0, 0);
//			}
//			for (int x = 0; x < ColorImages.rows; x++)//�ֲ�  ��������ɫ
//			{
//				for (int y = 0; y < ColorImages.cols; y++)
//					if (pow(double(x - clr_rhandpoint.Y), 2) + pow(double(y - clr_rhandpoint.X), 2) - 100 < 0.00001)
//						ColorImages(x, y) = Vec4b(0, 0, 255);
//			}
//			cout << cam_rhandpoint.Z << endl;

//================================��ԣ���������

			if (m_pBackgroundRGBX) //��ʼ��background
			{
				const RGBQUAD c_green = { 0, 0, 0 };
				// Fill in with a background colour of green if we can't load the background image
				for (int i = 0; i < cColorWidth * cColorHeight; ++i)
				{
					m_pBackgroundRGBX[i] = c_green;
				}
			}

			HRESULT hr = m_pMapper->MapColorFrameToDepthSpace(nWidth * nHeight, (UINT16*)qBuffer, cWidth * cHeight, m_pDepthCoordinates);
			if (SUCCEEDED(hr))
			 for (int colorIndex = 0; colorIndex < (cColorWidth*cColorHeight); ++colorIndex)
            {
                // default setting source to copy from the background pixel
                const RGBQUAD* pSrc = m_pBackgroundRGBX + colorIndex;
                DepthSpacePoint p = m_pDepthCoordinates[colorIndex];
                if (p.X != -numeric_limits<float>::infinity() && p.Y != -numeric_limits<float>::infinity())
                {
                    int depthX = static_cast<int>(p.X + 0.5f);
                    int depthY = static_cast<int>(p.Y + 0.5f);

//                  if ((depthX >= 0 && depthX < cDepthWidth) && (depthY >= 0 && depthY < cDepthHeight))
					if ((depthX >= 0 && depthX < cDepthWidth) && (depthY >= 0 && depthY < cDepthHeight))
                    {
						uchar* p;
						p = DepthImage.ptr<uchar>(depthY);
						int pdepth=p[depthX * 4];
                        if (pdepth>0)
                        {
							pSrc = m_pColorRGBX + colorIndex;
                        }
                    }
                }
            }
			Mat ColorImagess(cHeight, cWidth, CV_8UC4, m_pBackgroundRGBX);

//			resize(showImage, showImag, Size(cWidth / 2, cHeight / 2));
			imshow("DepthImage", show);
//			imshow("ColorImage", ColorImages);
			imshow("ColorImage",ColorImagess);

//			ofstream outfile("E:\\test\\color.txt", ios::in | ios::trunc);
//			outfile << ColorImages[235,562]<<endl<<ShowImages[235, 562];
//			outfile << ColorImages(10, 80)[0] << endl;// << ColorImages(10, 80)[1] << endl;
		}
}*/