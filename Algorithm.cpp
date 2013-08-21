/************************************************************************/
/* pbImage 19/08/2013                                                   */
/************************************************************************/
#include "Algorithm.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

#ifdef _DEBUG
#undef THIS_FILE
static const char THIS_FILE [] = __FILE__;
#define DEBUG_PRINT(fmt, ...) printf("File: %s, Line: %d \n"fmt, THIS_FILE, __LINE__, ##__VA_ARGS__)
#else
#define DEBUG_PRINT
#endif

/* calculate the threshold of binary by otsu algorithm
 * @param _image: input image
 * @param threshold: 
 */
void GetBinaryThreshold(cv::Mat &_image, double &threshold)
{
	if (_image.empty())
	{
		DEBUG_PRINT("image is empty!\n");
		threshold = 0;
		return;
	}
	Mat image;
	if (1 != _image.channels())
	{
		cvtColor(_image, image, CV_BGR2GRAY);
	}
	else if ( 1 == _image.channels())
	{
		_image.copyTo(image);
	}
	threshold = cv::threshold(image, image, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	return;
}
/* Adaptive binary image by otsu
 * @param _srcImg: input 1-channel image
 * @param _dstImg: output binary image
 **/
void AdaptiveBinary(cv::Mat &_srcImg, cv::Mat &_dstImg)
{
	if (_srcImg.empty() || _srcImg.channels() != 1)
	{
		DEBUG_PRINT("_srcImg is invalid!\n");
		_dstImg = Mat::zeros(_srcImg.rows, _srcImg.cols, _srcImg.type());
		return;
	}
#if 0
	uchar *ptr = _srcImg.datastart;
	uchar *ptr_end = _srcImg.dataend;
	while (ptr < ptr_end)
	{
		if (0 == *ptr)
		{
			uchar tmp = *ptr_end;
			*ptr_end = *ptr;
			*ptr = tmp;
			ptr_end--;
		}
		else
		{
			ptr++;
		}

	}
	Mat nz = Mat(vector<uchar> (_srcImg.datastart, ptr_end), true);
	double threshold = cv::threshold(nz, nz, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
#else
	double threshold = cv::threshold(_srcImg, _srcImg, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
#endif
	cv::threshold(_srcImg, _dstImg, threshold, 255, CV_THRESH_BINARY);
	return;
}

/* binary image by local algorithm
 * @param _srcImg: input 8-bit image
 * @param _dstImg: output image, same channel and type of _srcImg
 * @param maxValue: used by CV_THRESHOLD_BINARY & CV_THRESHOLD_BINARY_INV
 * @param blocksize: neighbor block size(3*3  5*5 7*7 e.g...), the default value is 3*3
 * @param C: value be minus. e.g: for the mean algorithm, first calculate the mean value
 *			 of a neighbor (3*3), and then minus C, the result will be the threshold, so, 
 *			 C maybe zero or a nagative value. The default value is 5.
 **/
void AdaptiveBinaryByNeighbor(cv::Mat &_srcImg, cv::Mat &_dstImg, double maxValue, int blocksize, int C)
{
	if (_srcImg.empty() || 1 != _srcImg.channels())
	{
		DEBUG_PRINT("_srcImg is invalid1!\n");
		_dstImg = Mat::zeros(_srcImg.rows, _srcImg.cols, _srcImg.type());
		return;
	}
	cv::adaptiveThreshold(_srcImg, _dstImg, maxValue, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, blocksize, C);
	return;
}
void FilterDenoise(cv::Mat &_srcImg, cv::Mat &_dstImg, int FLAG, int blocksize)
{
	if (_srcImg.empty())
	{
		DEBUG_PRINT("_srcImg is invalid!\n");
		return;
	}
	if (_srcImg.channels() != 1)
	{
		DEBUG_PRINT("_srcImg.channels = 1\n");
		return;
	}
	if (blocksize <= 0)
	{
		DEBUG_PRINT("block size invalid!\n");
		return;
	}
	switch (FLAG)
	{
	case BLUR:
		blur(_srcImg, _dstImg, Size(blocksize, blocksize) );
		break;
	case BILATERAL:

		break;
	case GAUSSIAN:
		GaussianBlur(_srcImg, _dstImg, Size(blocksize, blocksize), 0);
	default:
		break;
	}
	return;
}
void FilterDenoiseEx(cv::Mat &_srcImg, cv::Mat &_dstImg, int thld)
{
	if (_srcImg.empty())
	{
		DEBUG_PRINT("_srcImg invalid!\n");
		return;
	}
	Mat reversImg;
	reversImg = _srcImg.clone();
	//bitwise_not(_srcImg, reversImg);
	vector<vector<Point>> contours;
	ExtractContours(reversImg, contours);
	if (!contours.empty())
	{
		for (int i = 0; i < contours.size(); i++)
		{
			if (contours[i].size() > 0 && contours[i].size() < ( _srcImg.cols * _srcImg.rows))
			{
				double perimter = arcLength(contours[i], true);
				if (perimter > 50 && perimter < (_srcImg.cols + _srcImg.rows))
				{
					drawContours(_srcImg, contours, i, Scalar(255, 0, 0), 2);
				}
			}
		}
	}
	imshow("contour", _srcImg);
	return;
}
void ExtractContours(cv::Mat &img, std::vector < std::vector < cv::Point >> &contours)
{
	if (img.empty())
	{
		DEBUG_PRINT("param invalid!\n");
		contours.clear();
		return;
	}
	contours.clear();
	// 提取轮廓的同时，img被修改
	findContours(img, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
	return;
}