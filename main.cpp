#include <iostream>
#include "opencv2/opencv.hpp"
#include "Algorithm.h"
#include <windows.h>

using namespace std;
using namespace cv;

int main()
{
	string src_name = "1.jpg";
	string dst_name = "";
	Mat src_img = imread(src_name, 0);
	Mat dst_img;
	if (src_img.empty())
	{
		cout << "Load image failed!" << endl;
		return -1;
	}

	double thld = 0.0f;
	//GetBinaryThreshold(src_img, thld);
	//AdaptiveBinaryByNeighbor(src_img, dst_img, 255, 65, 15);

	LARGE_INTEGER freq, start, end;
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&start);

	//AdaptiveBinary(src_img, dst_img);
	//FilterDenoise(dst_img, dst_img, BLUR, 5);
	//FilterDenoiseEx(dst_img, dst_img, 100);
	QueryPerformanceCounter(&end);
	cout << "time: " << (end.QuadPart - start.QuadPart) * 1000000 / freq.QuadPart << ".ns" << endl;

	imshow("Adaptive Threshold", dst_img);

	waitKey();

	return 0;
}