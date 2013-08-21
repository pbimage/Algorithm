/************************************************************************/
/* pbImage 19/08/2013                                                   */
/************************************************************************/
#ifndef __ALGORITHM_H__
#define __ALGORITHM_H__

#include <opencv2/opencv.hpp>

#define ADAPTIVE_THRESHOLD_MEAN			CV_ADAPTIVE_THRESH_MEAN_C		// 0
#define ADAPTIVE_THRESHOLD_GAUSSIAN		CV_ADAPTIVE_THRESH_GAUSSIAN_C	// 1

#define BLUR		10	// blur�˲�
#define BILATERAL	11	// ˫���˲�
#define GAUSSIAN	12	// ��˹�˲�
// �����ֵ����ֵ
void GetBinaryThreshold(cv::Mat &_image, double &threshold);
// ����Ӧ��ֵ��
void AdaptiveBinary(cv::Mat &_srcImg, cv::Mat &_dstImg);
void AdaptiveBinaryByNeighbor(cv::Mat &_srcImg, cv::Mat &_dstImg, double maxValue, int blocksize = 3, int C = 5);

// �˲�
void FilterDenoise(cv::Mat &_srcImg, cv::Mat &_dstImg, int FLAG, int blocksize = 5);
void FilterDenoiseEx(cv::Mat &_srcImg, cv::Mat &_dstImg, int thld);
void ExtractContours(cv::Mat &img, std::vector < std::vector < cv::Point >> &contours);

#endif // __ALGORITHM_H__