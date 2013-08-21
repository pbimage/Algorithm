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
/* Local adaptive binary algorithm--niblack.
 * @param _srcImg: 
 * @param _dstImg:
 * @param blocksize: size of neighbor block
 * @param k: when object is light, k is [0, 1);  default: k = 0.2
 *			 when object is black, k is (-1, 0]; default: k = -0.2
 */
void AdaptiveBinaryByNiblack(cv::Mat &_srcImg, cv::Mat &_dstImg, const int blocksize, const float k)
{
	if (_srcImg.empty())
	{
		DEBUG_PRINT("_srcImg invalid!\n");
		_dstImg = _srcImg;
		return;
	}
	if (_srcImg.channels() != 1)
	{
		DEBUG_PRINT("_srcImg.channels is not 1, Do not support!");
		return;
	}
	// sum: 32f or 64f;
	// sqsum: 64f;
	Mat sum, sqsum;

	integral(_srcImg, sum, sqsum); // 2ms

	int niblack_thld = 0;
	double mean = 0.0f, stdVar = 0.0f;
	_dstImg = Mat::zeros(_srcImg.size(), _srcImg.type());
	Rect block = Rect(0, 0, 0, 0);
	int left_x, right_x, top_y, bottom_y;
	int block_w, block_h;
	
	for (int r = 0; r < _srcImg.rows; r++)
	{
		uchar* ptr = _srcImg.ptr<uchar>(r);
		uchar* dPtr = _dstImg.ptr<uchar>(r);
		for (int c = 0; c < _srcImg.cols; c++)
		{
			left_x = c - blocksize / 2;
			right_x = left_x + blocksize;
			top_y = r - blocksize / 2;
			bottom_y = top_y + blocksize;
			if (left_x < 0)
			{
				left_x = 0;
			}
			if (right_x > _srcImg.cols)
			{
				right_x = _srcImg.cols;
			}
			if (top_y < 0)
			{
				top_y = 0;
			}
			if (bottom_y > _srcImg.rows)
			{
				bottom_y = _srcImg.rows;
			}
			block_w = right_x - left_x;
			block_h = bottom_y - top_y;
			block = Rect(left_x, top_y, block_w, block_h);
			CalcVarianceAndSD(block, sum, sqsum, mean, stdVar);
			niblack_thld = cvRound( mean + k * stdVar );
			if (ptr[c] > niblack_thld)
			{
				dPtr[c] = 255;
			}
		}
	}
	return;
}
void AdaptiveBinaryByNiblackEx(cv::Mat &_srcImg, cv::Mat &_dstImg, const int blocksize, const float k)
{
	int height = _srcImg.rows;
	int width = _srcImg.cols;
	_dstImg = Mat::zeros(_srcImg.rows, _srcImg.cols, _srcImg.type());
	for (int i = 0; i < height; i = i + blocksize)
	{
		for (int j = 0; j < width; j = j + blocksize)
		{

			int m = i + blocksize;
			if (m >= height)
				m = height - 1;

			int n = j + blocksize;
			if (n >= width)
				n = width - 1;

			double meanVal = 0;
			double stdVal = 0;
			int p = 0;

			for (int k = i; k < m; k++)
			{
				for (int l = j; l < n; l++)
				{
					meanVal = meanVal + _srcImg.at<uchar>(k, l);//image.getPixel(k, l);
					p++;
				}
			}
			meanVal = meanVal/p;

			for (int k = i; k < m; k++)
			{
				for (int l = j; l < n; l++)
				{
					stdVal = stdVal + pow(_srcImg.at<uchar>(k, l) - meanVal, 2.0);//Math.pow(image.getPixel(k, l) - meanVal, 2.0);
				}
			}
			stdVal = sqrt(stdVal / p);//Math.sqrt(stdVal/p);
			double threshold = 0.0f;
			threshold = (meanVal + k * stdVal);
			for (int k = i; k < m; k++)
			{
				for (int l = j; l < n; l++)
				{
					if (_srcImg.at<uchar>(k, l) > threshold)
					{
						_dstImg.at<uchar>(k, l) = 255;
					}
// 					if (image.getPixel(k, l) <= threshold)
// 						image.setBinaryPixel(k, l, (short) 0);
// 					else
// 						image.setBinaryPixel(k, l, (short) 255);
				}
			}
		}
	}
}
inline void CalcVarianceAndSD(cv::Rect &block, cv::Mat &sum, cv::Mat &sqsum, double &mean, double &stdvar)
{	
	double brs = sum.at<int>(block.y+block.height,block.x+block.width);			// D
	double bls = sum.at<int>(block.y+block.height,block.x);						// C
	double trs = sum.at<int>(block.y,block.x+block.width);						// B
	double tls = sum.at<int>(block.y,block.x);									// A
	double brsq = sqsum.at<double>(block.y+block.height,block.x+block.width);	// D
	double blsq = sqsum.at<double>(block.y+block.height,block.x);				// C
	double trsq = sqsum.at<double>(block.y,block.x+block.width);				// B
	double tlsq = sqsum.at<double>(block.y,block.x);							// A
	mean = (brs + tls-trs-bls)/((double)block.area() + 1);				// D + A - B - C
	double sqmean = (brsq+tlsq-trsq-blsq)/((double)block.area() + 1);	// D + A - B - C
	stdvar = sqrt(sqmean - mean * mean);

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
	//reversImg = _srcImg.clone();
	bitwise_not(_srcImg, reversImg);
	vector<vector<Point>> contours;
	ExtractContours(reversImg, contours);
	if (!contours.empty())
	{
		for (int i = 0; i < contours.size(); i++)
		{
			double perimter = arcLength(contours[i], true);
			if (perimter > 300 && perimter < (_srcImg.cols + _srcImg.rows))
			{
				drawContours(_srcImg, contours, i, Scalar(255, 0, 0), 2);
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