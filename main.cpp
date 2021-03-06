﻿#include <iostream>
#include "opencv2/opencv.hpp"
#include "Algorithm.h"
#include <windows.h>
#include "GeometricMatch.h"

using namespace std;
using namespace cv;

float FILTER_KENERL_3_3[] = 
{
	1,  1,  1,
	1,  9,  1,
	1,  1,  1
};
int main()
{
	Mat src_learnMat = imread("learn.bmp", 0);
	Mat src_matchMat = imread("Image1A.bmp", 0);

	imshow("src_learn", src_learnMat);
	//imshow("src_match", src_matchMat);

	RTable rtable;
	vector< int > learn_contourpoints_count;
	EdgeLearn(src_learnMat, rtable, learn_contourpoints_count);

	Rect objBox = Rect(0, 0, 0, 0);

	LARGE_INTEGER freq, start, end;
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&start);

	EdgeMatch(src_matchMat, objBox, rtable, learn_contourpoints_count);

	QueryPerformanceCounter(&end);
	cout << "time: " << (end.QuadPart - start.QuadPart) * 1000 / freq.QuadPart << ".ms" << endl;

	rectangle(src_matchMat, objBox, Scalar::all(0), 2);
	imshow("match result", src_matchMat);
	cout<<"box: "<<objBox.x<<" "<<objBox.y<<" "<<objBox.width<<" "<<objBox.height<<endl;
	waitKey();
	return 0;
}
#if 0
int main()
{
	string src_name = "geometric_lighting.png";
	string dst_name = "";
	Mat src_img = imread(src_name, 0);
	Mat dst_img;
	if (src_img.empty())
	{
		cout << "Load image failed!" << endl;
		return -1;
	}
	imshow("src", src_img);
	double thld = 0.0f;
	AdaptiveBinaryByNiblackEx(src_img, dst_img, 150, -0.2);
	AdaptiveBinaryByNeighbor(src_img, dst_img, 255, 3, 10);
	
	imshow("binary", dst_img);
	LARGE_INTEGER freq, start, end;
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&start);

	Mat kernel;
	GenerateKernel(kernel, FILTER_KENERL_3_3, 3);
	//FilterImage(dst_img, dst_img, kernel);
	Laplacian(dst_img, dst_img, -1);
	QueryPerformanceCounter(&end);
	cout << "time: " << (end.QuadPart - start.QuadPart) * 1000000 / freq.QuadPart << ".ns" << endl;

	imshow("filter", dst_img);

	waitKey();

	return 0;
}
#endif