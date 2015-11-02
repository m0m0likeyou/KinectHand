#include "stdafx.h"
#include "opencv2/opencv.hpp"
#include <windows.h>
#include <iostream>
#include <fstream>
#include <Kinect.h>// Kinect Header files  Ҳ����������nuiapi.h
using namespace cv;
using namespace std;
ofstream outfile("E:\\test\\hand.txt", ios::in | ios::trunc);
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
	void					ProcessDepth(const UINT16* pBuffer, RGBQUAD *cBuffer, int nWidth, int nHeight, int cWidth, int cHeight, USHORT nMinDepth, USHORT nMaxDepth);//����õ�������
private:

	IKinectSensor*          m_pKinectSensor;// Current Kinect
	IDepthFrameReader*      m_pDepthFrameReader;// depth reader
	IBodyFrameReader*		m_pBodyFrameReader;//Body reader   ���
	IColorFrameReader*		m_pColorFrameReader;
	ICoordinateMapper*		m_pMapper;         //map
	RGBQUAD*                m_pDepthRGBX;
	RGBQUAD*                m_pColorRGBX;
	CameraSpacePoint        cam_rhandpoint;
	DepthSpacePoint         dps_rhandpoint;
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
	m_pDepthFrameReader = NULL;
	m_pBodyFrameReader = NULL;    //���
	m_pColorFrameReader = NULL;  //���
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
		IBodyFrameSource* pBodyFrameSource = NULL;   //���
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
	HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);   //�ǵ�release IBodyFrame������
	if (pBodyFrame != NULL)
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
					FLOAT xr, yr;
/*					HandState leftHandState = HandState_Unknown; //��ȡ�ֵ�״̬,�ò���
					HandState rightHandState = HandState_Unknown;
					pBody->get_HandRightState(&rightHandState); */
					hr = pBody->GetJoints(_countof(joints), joints);
					cam_rhandpoint = joints[11].Position;
//					yr =  -0.647*joints[11].Position.Z + 2.1453;  //ת��ϵ��  ϵͳ�Դ������/΢Ц
//					xr = -0.6443*joints[11].Position.Z + 1.9619;
//					dps_rhandpoint.Y = int(256 + 256 * joints[11].Position.X*xr);//ͼ����Y��body��x
//					dps_rhandpoint.X = int(212 - 212 * joints[11].Position.Y*yr);
					m_pMapper->MapCameraPointToDepthSpace(cam_rhandpoint, &dps_rhandpoint);
					cout << cam_rhandpoint.Z << endl;
					/*	if (isXY(rhandpoint.x, rhandpoint.y) )//����У��body��depth
					{
					printf("Person %d : ���� %2f %2f %2f\n", i, joints[11].Position.X, joints[11].Position.Y, joints[11].Position.Z * 1000);
					if (isZ(rhanddepth))
					{
					outfile << "RightHand  " << joints[11].Position.X << "   " << joints[11].Position.Y << "   " << joints[11].Position.Z * 1000 << endl;
					system("color F0");
					printf("Person %d : ���� %2f %2f %2f\n", i, joints[11].Position.X, joints[11].Position.Y, joints[11].Position.Z * 1000);
					system("color 0F");
					}
					}*/
				}
			}
		}
	}
	SafeRelease(pBodyFrame);
	if (SUCCEEDED(hr))  hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);//������color�ļ�飨˵������Щ������hr����û����- -
	if (SUCCEEDED(hr))  hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);//��Ҫͬʱ�ɼ���
	if (SUCCEEDED(hr))  //��һ���ڶ�depthFrame��������ȡ��w,h�Լ������С���ŷ�Χ��
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
			//���if�Ǹ��������F��doge
			//�����ݴ�pColorFrame֡������ŵ�pBuffer������
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&cBuffer));   //������������xixi
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
			//�ú�˵���Ĳ�����  Ϊ�õ����������Χ�������Բ����ŵ�Զ����ȣ������������ܵ������������ֵ��
			// In order to see the full range of depth (including the less reliable far field depth)
			// we are setting nDepthMaxDistance to the extreme potential depth threshold
			//nDepthMaxDistance = USHRT_MAX;
			//nDepthMaxDistance = 0x05f;
			//�����ֻ��õ��ɿ���ȣ��Ͱ���������������е�////ȥ���������ȥ��ȥ
			// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
			hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		}



		if (SUCCEEDED(hr))
		{
			ProcessDepth(pBuffer, cBuffer, nWidth, nHeight, cWidth, cHeight, int(cam_rhandpoint.Z*1000 - 50), int(cam_rhandpoint.Z * 1000 + 50));//����color
		  //ProcessDepth( pBuffer, nWidth, nHeight, rhanddepth-50, rhanddepth+50);//��ǰ��
		  // ProcessDepth(pBuffer, cBuffer, nWidth, nHeight, cWidth, cHeight, nDepthMinReliableDistance, nDepthMaxDistance);//����ʾ
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
			RGBQUAD* pRGBX = m_pDepthRGBX;
			// end pixel is start + width*height - 1
			const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);
			const UINT16* ppBuffer = pBufferEnd - (nWidth * nHeight);
			while (pBuffer < pBufferEnd)
			{
				USHORT depth = *pBuffer;

				//Ϊ��ת��Ϊһ���ֽڣ����������˴󲿷���Ҫ��λ
				// To convert to a byte, we're discarding the most-significant
				// rather than least-significant bits.
				//���Ǳ�����ϸ�ڣ�����ǿ�Ȼ�Ƚ����� ���ž������ֵ����Ϊ��ɫ��0��
				// We're preserving detail, although the intensity will "wrap."
				// Values outside the reliable depth range are mapped to 0 (black).
				//ע�⣺ʹ�ñ�ѭ���е���Ŀ��������ή�����ܣ�д��Ʒ����ʱ�����ò�ѯ��
				// Note: Using conditionals in this loop could degrade performance.
				// Consider using a lookup table instead when writing production code.

				BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? 255 : 0);// (depth % 256) : 0);

				pRGBX->rgbRed = intensity;
				pRGBX->rgbGreen = intensity;
				pRGBX->rgbBlue = intensity;
				++pRGBX;
				++pBuffer;
			}
			Mat DepthImage(nHeight, nWidth, CV_8UC4, m_pDepthRGBX);
			Mat_<Vec4b> show = DepthImage.clone();

/*			for (int i = 0; i<DepthImage.rows; i++)
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
			float sr;
			sr = pow((208.32*pow(2.71828182, -1.316*cam_rhandpoint.Z)), 2);
			for (int x = 0; x < DepthImage.rows; x++)//depth���ֲ�����Ĩ��
				for (int y = 0; y < DepthImage.cols; y++)
					if (pow(double(x - dps_rhandpoint.Y), 2) + pow(double(y - dps_rhandpoint.X), 2) - sr > 0.00001)
						show(x, y) = Vec4b(0, 0, 0);


			//==============color image============

			// Draw the data with OpenCV

//			m_pColorRGBX = cBuffer;
//			Mat ColorImage(cHeight, cWidth, CV_8UC4, m_pColorRGBX);
//			Mat_<Vec4b> ColorImages = ColorImage.clone();

			Mat ColorImage(cHeight, cWidth, CV_8UC4, cBuffer);
//			Mat ShowImage(cHeight, cWidth, CV_8UC4, Scalar(0, 0, 0));
			Mat_<Vec4b> ColorImages = ColorImage.clone();
//			Mat_<Vec4b> ShowImages = ShowImage.clone();
			float* pData = (float*)ColorImages.data;
//			float* sData = (float*)ShowImages.data;



//==============color image copy============
			DepthSpacePoint dpPoint;
			ColorSpacePoint cPoint;
			for (int x = (dps_rhandpoint.X - 60); x < (dps_rhandpoint.X + 60); x++)//�ֲ�  ��һ��׼����depth���ֵĵ��Ӧ��color��
			{
				uchar* data = ColorImage.ptr<uchar>(x);
				for (int y = 0; y < DepthImage.cols; y++)
					if (pow(double(x - dps_rhandpoint.X), 2) + pow(double(y - dps_rhandpoint.Y), 2) - 3600.0 < 0.000001)
						if (ppBuffer[x, y] > 0)
						{
							dpPoint.X = x;
							dpPoint.Y = y;
							m_pMapper->MapDepthPointToColorSpace(dpPoint, ppBuffer[x, y], &cPoint);
//							cout<<ColorImages.at<uchar>(int(cPoint.X), int(cPoint.Y));
//							pData = ColorImages.ptr<float>(int(cPoint.X));
//							sData = ShowImages.ptr<float>(int(cPoint.X));
//							sData[int(cPoint.Y)]=pData[int(cPoint.Y)];
//							showImage(x,y) = showImages(x,y);
//							uchar* data2 = ColorImages.ptr<uchar>(int(cPoint.X));
//							data[3 * int(cPoint.Y)] = data2[3 * int(cPoint.Y)];
//							data[3 * int(cPoint.Y) + 1] = data2[3 * int(cPoint.Y) + 1];
//							data[3 * int(cPoint.Y) + 2] = data2[3 * int(cPoint.Y) + 2];
//							ShowImages.at<Vec4b>(int(cPoint.X), int(cPoint.Y))[0] = ColorImages(int(cPoint.X), int(cPoint.Y))[0];
//							ShowImages(int(cPoint.X), int(cPoint.Y))[1] = ColorImages(int(cPoint.X), int(cPoint.Y))[1];
//							ShowImages(int(cPoint.X), int(cPoint.Y))[2] = ColorImages(int(cPoint.X), int(cPoint.Y))[2];
						}
			}
				

			/*//���ڲɼ�����У׼body��depth
			if (isHand(show, 212, 256))
			{
			printf("Person : ���� %2d,%2d,%2d\n",rhandpoint.x, rhandpoint.y, rhanddepth);
			if (isZ(rhanddepth))
			{
			outfile << "CT  " << rhandpoint.x << "  " << rhandpoint.y << "  " << rhanddepth << endl;
			system("color F0");
			printf("Person : ���� %2d,%2d,%2d\n", rhandpoint.x, rhandpoint.y, rhanddepth);
			system("color 0F");
			}
			}
			*/
			//�������ڻ��
			/*		for (int x = 0; x < DepthImage.rows; x++)//�ֲ�
			for (int y = 0; y < DepthImage.cols; y++)
			if (pow(double(x - rhandpoint.x), 2) + pow(double(y - rhandpoint.y), 2) - 25.0 < 0.00000000001)
			show(x, y) = Vec4b(0, 0, 255);*/
			//		resize(showImage, showImag, Size(cWidth / 2, cHeight / 2));
//			imshow("ColorImage", ColorImages);
			imshow("DepthImage", show);
//			ofstream outfile("E:\\test\\color.txt", ios::in | ios::trunc);
//			outfile << ColorImages[235,562]<<endl<<ShowImages[235, 562];
//			outfile << ColorImages(10, 80)[0] << endl;// << ColorImages(10, 80)[1] << endl;
		}
}
