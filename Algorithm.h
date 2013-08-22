/************************************************************************/
/* pbImage 19/08/2013                                                   */
/************************************************************************/
#ifndef __ALGORITHM_H__
#define __ALGORITHM_H__

#include <opencv2/opencv.hpp>

#define ADAPTIVE_THRESHOLD_MEAN			CV_ADAPTIVE_THRESH_MEAN_C		// 0
#define ADAPTIVE_THRESHOLD_GAUSSIAN		CV_ADAPTIVE_THRESH_GAUSSIAN_C	// 1

// Linear Filters mask
/*
 *  a    -b    c
 * -b    x    -d
 *  c    d    -a
 */

#define BLUR		10	// blur滤波
#define BILATERAL	11	// 双边滤波
#define GAUSSIAN	12	// 高斯滤波
// 计算二值化阈值
void GetBinaryThreshold(cv::Mat &_image, double &threshold);
// 自适应二值化
void AdaptiveBinary(cv::Mat &_srcImg, cv::Mat &_dstImg);
void AdaptiveBinaryByNeighbor(cv::Mat &_srcImg, cv::Mat &_dstImg, double maxValue, int blocksize = 3, int C = 5);
// niblack
void AdaptiveBinaryByNiblack(cv::Mat &_srcImg, cv::Mat &_dstImg, const int blocksize = 3, const float k = 0.2);
void AdaptiveBinaryByNiblackEx(cv::Mat &_srcImg, cv::Mat &_dstImg, const int blocksize, const float k = 0.2);
inline void CalcVarianceAndSD(cv::Rect &block, cv::Mat &sum, cv::Mat &sqsum, double &mean, double &stdvar);

// 滤波
void FilterDenoise(cv::Mat &_srcImg, cv::Mat &_dstImg, int FLAG, int blocksize = 5);
void FilterDenoiseEx(cv::Mat &_srcImg, cv::Mat &_dstImg, int thld);
void ExtractContours(cv::Mat &img, std::vector < std::vector < cv::Point >> &contours);

#endif // __ALGORITHM_H__