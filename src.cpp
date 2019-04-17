#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cmath>

using namespace std;
using namespace cv;

//parameters of judge function
int const k1 = 10;
int const k2 = 20000;

Mat afm_src, Optical_src, Optical_dst;
Mat src, dst_gray, afm_dst;
Mat test;


int x1, x2, x3, y_0, y2, y3;

int thresh = 5;
int MaxThresh = 255;
void Trackbar(int, void*, Mat dst);
double Similarity(Mat image_r, int size_r, Mat image_s, int size_s);
double getA(double** arcs, int n);
void  getAStart(double** arcs, int n, double** ans);
bool GetMatrixInverse(double** src, int n, double** des);
double P(int *center_area, Mat image_a, Mat image_b, int size,int len);
double Angle_sum(Mat dst);
int *Center_area(Mat dst);
int Count_location(Mat dst);


int main()
{
	/*Mat a = imread("afm_two.png", 0);
	Mat b = imread("Optical_2.png", 0);
	int count_location = Count_location(a);
	cout << P(Center_area(a), a, b, 56,count_location)<<endl;
	waitKey(0);*/
	afm_src = imread("\images\\Square_gray.png");
	Optical_src = imread("\images\\Optical_image.png");
	flip(afm_src,afm_src, 1);
	//GaussianBlur(dst, dst, Size(7, 7), 0, 0);
	bilateralFilter(afm_src, src, 60, 60, 25);
	bilateralFilter(Optical_src, Optical_dst, 60, 60, 25);
	cvtColor(src, dst_gray, CV_BGR2GRAY); 
	cvtColor(Optical_dst, Optical_dst, CV_BGR2GRAY);
	Mat element = getStructuringElement(MORPH_RECT, Size(10, 10));
	erode(dst_gray, afm_dst, element);
	dilate(afm_dst, afm_dst, element);
	erode(Optical_dst, Optical_dst, element);
	dilate(Optical_dst, Optical_dst, element);
	//resize(src, src, cv::Size(415, 350));
	//resize(dst, dst, cv::Size(415, 350));
	resize(src, src, cv::Size(56, 56));
	resize(afm_dst, afm_dst, cv::Size(56, 56));
	resize(Optical_dst, Optical_dst, cv::Size(830, 700));
	resize(Optical_src, Optical_src, cv::Size(830, 700));
	////////////////////////////
	double min = 0.0, afm_max = 0.0,optical_min,optical_max;
	double* afm_minp = &min;
	double* afm_maxp = &afm_max;
	double* optical_minp = &optical_min;
	double* optical_maxp = &optical_max;
	minMaxIdx(afm_dst, afm_minp, afm_maxp);
	minMaxIdx(Optical_dst, optical_minp, optical_maxp);

	threshold(afm_dst, afm_dst, afm_max / 2 + 20, 255, CV_THRESH_BINARY);
	threshold(Optical_dst, Optical_dst, optical_max / 2 + 20, 255, CV_THRESH_BINARY);


	imshow("two value", afm_dst);
	imwrite("optical_two.png", Optical_dst);
	imwrite("afm_two.png", afm_dst);

	vector<Point> Matching_center;
	vector<double> Score;
	for(int j=0;j<600;j+=5)
	{
		for(int i=0;i<700;i+=5)
		{
			Rect rect(i, j, 56, 56);
			if (Angle_sum(Optical_dst(rect)) != 0)
			{
				double angle_diff=abs(Angle_sum(Optical_dst(rect))-Angle_sum(afm_dst));
				Point p;
				if(angle_diff==0)
				{
					int *a;
					int count_location = Count_location(afm_dst);
					a = Center_area(afm_dst);
					Score.push_back(P(Center_area(afm_dst), afm_dst, Optical_dst(rect), 56, count_location));
					p.x=i+27;
					p.y=j+27;
					Matching_center.push_back(p);
				}
			}
		}
	}
	double min_score=Score[0];
	int min_index;
	for(int i=0;i<Score.size();i++)
	{
		if (Score[i] < min_score)
		{
			min_score = Score[i];
			min_index = i;
		}
	}
	for (int i = 0; i < Matching_center.size(); i++)
	{
		cout << "the " << i << " center point: " << Matching_center[i].x << ", " << Matching_center[i].y << endl;
		cout << "the score is: " << Score[i] << endl;
	}
	circle(Optical_src, Matching_center[min_index], 2, Scalar(0, 0, 255), 2);
	rectangle(Optical_src, Point(Matching_center[min_index].x-27, Matching_center[min_index].y-27), Point(Matching_center[min_index].x + 27, Matching_center[min_index].y + 27), Scalar(0, 0, 255), 1, 8, 0);

	imshow("optical_dst", Optical_src);
	imwrite("Matching result.png", Optical_src);

	waitKey(0);
	return 0;
}

void Trackbar(int, void*, Mat dst)
{
	int s1 = 0;
	int s2 = 0;
	int s3 = 0;
	int s4 = 0;
	int s5 = 0;
	int s6 = 0;
	int s7 = 0;
	int s8 = 0;
	int s9 = 0;
	Mat imageSource;
	imageSource = dst.clone();
	/////////////////////////////////////

	y_0 = dst.rows / 3;
	y2 = dst.rows * 2 / 3;
	y3 = dst.rows - 1;
	x1 = dst.cols / 3;
	x2 = dst.cols * 2 / 3;
	x3 = dst.cols - 1;

	cv::Point p1 = cv::Point(x1, 0);
	cv::Point p2 = cv::Point(x2, 0);
	cv::Point p3 = cv::Point(0, y_0);
	cv::Point p4 = cv::Point(0, y2);
	cv::Point p5 = cv::Point(x1, y3);
	cv::Point p6 = cv::Point(x2, y3);
	cv::Point p7 = cv::Point(x3, y_0);
	cv::Point p8 = cv::Point(x3, y2);


	cv::line(imageSource, p1, p5, cv::Scalar(0, 0, 255), 3, 4);
	cv::line(imageSource, p2, p6, cv::Scalar(0, 0, 255), 3, 4);
	cv::line(imageSource, p3, p7, cv::Scalar(0, 0, 255), 3, 4);
	cv::line(imageSource, p4, p8, cv::Scalar(0, 0, 255), 3, 4);
	////////////////////////////////////////////////////use corner compute every area point number
	vector<Point2f> corners;
	goodFeaturesToTrack(dst, corners, thresh, 0.01, 10, Mat());
	for (int i = 0; i < corners.size(); i++)
	{
		circle(imageSource, corners[i], 2, Scalar(0, 0, 255), 2);
	}
	imshow("corner", imageSource);

	for (int i = 0; i < corners.size(); i++)
	{
		if (corners[i].x < x1)
		{
			if (corners[i].y < y_0)
			{
				s4++;
			}
			else
			{
				if (corners[i].y < y2)
				{
					s5++;
				}
				else
				{
					s6++;
				}
			}
		}
		else
		{
			if (corners[i].x < x2)
			{
				if (corners[i].y < y_0)
				{
					s3++;
				}
				else
				{
					if (corners[i].y < y2)
					{
						s9++;
					}
					else
					{
						s7++;
					}
				}
			}
			else
			{
				if (corners[i].y < y_0)
				{
					s2++;
				}
				else
				{
					if (corners[i].y < y2)
					{
						s1++;
					}
					else
					{
						s8++;
					}
				}
			}
		}
	}
	cout << "s1: " << s1 << endl;
	cout << "s2: " << s2 << endl;
	cout << "s3: " << s3 << endl;
	cout << "s4: " << s4 << endl;
	cout << "s5: " << s5 << endl;
	cout << "s6: " << s6 << endl;
	cout << "s7: " << s7 << endl;
	cout << "s8: " << s8 << endl;
	cout << "s9: " << s9 << endl;
}

double Similarity(Mat image_r, int size_r, Mat image_s, int size_s)
{
	int temp = 0;
	//////compute Cr
	double ** Cr;
	Cr = new double*[2];
	for (int i = 0; i < 2; i++)
	{
		Cr[i] = new double[2];
	}
	for (int j = 0; j < 2; j++)
	{
		for (int k = 0; k < 2; k++)
		{
			Cr[j][k] = 0;
		}
	}
	int num_a = 0;
	for (int i = 0; i < image_r.rows; i++)
	{
		for (int j = 0; j < image_r.cols; j++)
		{
			if (float(image_r.at<uchar>(i, j)) == 0)
			{
				num_a++;
			}
		}
	}
	if (num_a == 0)
	{
		return 10;
	}
	cout << "num_a: " << num_a << endl;
	int  ** zr;
	zr = new int*[num_a];
	for (int i = 0; i < num_a; i++)
	{
		zr[i] = new int[2];
	}
	
	for (int i = 0; i < image_r.rows; i++)
	{
		for (int j = 0; j < image_r.cols; j++)
		{
			if (int(image_r.at<uchar>(i, j)) == 0)
			{
				zr[temp][0] = i;
				zr[temp][1] = j;
				temp++;
			}
		}
	}
	double mean_r[2] = { 0,0 };
	double sum_r[2] = { 0,0 };
	
	for (int i = 0; i < num_a; i++)
	{
		sum_r[0] += zr[i][0];
		sum_r[1] += zr[i][1];
	}
	mean_r[0] = sum_r[0] / num_a;
	mean_r[1] = sum_r[1] / num_a;

	for (int i = 0; i < num_a; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			for (int k = 0; k < 2; k++)
			{
				Cr[j][k] += ((zr[i][j] - mean_r[j]) * (zr[i][k] - mean_r[k]))/num_a;
			}
		}
	}
	
	/////////////////////////compute Cs

	double ** Cs;
	Cs = new double*[2];
	for (int i = 0; i < 2; i++)
	{
		Cs[i] = new double[2];
	}
	for (int j = 0; j < 2; j++)
	{
		for (int k = 0; k < 2; k++)
		{
			Cs[j][k] = 0;
		}
	}
	int num_b = 0;
	for (int i = 0; i < image_s.rows; i++)
	{
		for (int j = 0; j < image_s.cols; j++)
		{
			if (float(image_s.at<uchar>(i, j)) == 0)
			{
				num_b++;
			}
		}
	}
	if (num_b == 0)
	{
		return 10;
	}
	cout << "num_b: " << num_b << endl;
	int  ** zs;
	zs = new int*[num_b];
	for (int i = 0; i < num_b; i++)
	{
		zs[i] = new int[2];
	}

	temp = 0;
	for (int i = 0; i < image_s.rows; i++)
	{
		for (int j = 0; j < image_s.cols; j++)
		{
			if (int(image_s.at<uchar>(i, j)) == 0)
			{
				zs[temp][0] = i;
				zs[temp][1] = j;
				temp++;
			}
		}
	}
	double mean_s[2] = { 0,0 };
	double sum_s[2] = { 0,0 };

	for (int i = 0; i < num_b; i++)
	{
		sum_s[0] += zs[i][0];
		sum_s[1] += zs[i][1];
	}
	mean_s[0] = sum_s[0] / num_b;
	mean_s[1] = sum_s[1] / num_b;

	for (int i = 0; i < num_b; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			for (int k = 0; k < 2; k++)
			{
				Cs[j][k] += ((zs[i][j] - mean_s[j]) * (zs[i][k] - mean_s[k])) / num_b;
			}
		}
	}


	

	for (int i = 0; i < 2; i++)
	{
		mean_r[i] -= mean_s[i];//mean_r now is mean_r-mean_s!!!!!!!!!!!!!!!!!!!
		for (int j = 0; j < 2; j++)
		{
			Cr[i][j] += Cs[i][j];//Cr now is the sum of Cr and Cs!!!!!!!!!!!!!!!!
		}
	}
	double col_sum[2] = { 0,0 };
	double **invCr;
	invCr = new double *[2];
	for (int i = 0; i < 2; i++)
	{
		invCr[i] = new double [2];
	}
	GetMatrixInverse(Cr, 2, invCr);
	
	for (int j = 0; j < 2; j++)
	{
		double sum = 0;
		for (int i = 0; i < 2; i++)
		{
			sum += invCr[i][j] * mean_r[i];
		}
		col_sum[j] = sum;
	}
	double temp_sum = 0;
	for (int i = 0; i < 2; i++)
	{
		temp_sum += col_sum[i] * mean_r[i];
	}
	return sqrt(temp_sum);
}


void  getAStart(double** arcs, int n, double** ans)
{
	if (n == 1)
	{
		ans[0][0] = 1;
		return;
	}
	int i, j, k, t;
	double **temp;
	for (int i = 0; i < n; i++)
	{
		temp = new double*[n];
		for (int j = 0; j < n; j++)
		{
			temp[j] = new double[n];
		}
	}
	for (i = 0; i<n; i++)
	{
		for (j = 0; j<n; j++)
		{
			for (k = 0; k<n - 1; k++)
			{
				for (t = 0; t<n - 1; t++)
				{
					temp[k][t] = arcs[k >= i ? k + 1 : k][t >= j ? t + 1 : t];
				}
			}


			ans[j][i] = getA(temp, n - 1);
			if ((i + j) % 2 == 1)
			{
				ans[j][i] = -ans[j][i];
			}
		}
	}
}

double getA(double** arcs, int n)
{
	if (n == 1)
	{
		return arcs[0][0];
	}
	double ans = 0;
	double **temp;
	for (int i = 0; i < n; i++)
	{
		temp = new double*[n];
		for (int j = 0; j < n; j++)
		{
			temp[j] = new double[n];
		}
	}
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			temp[i][j] = 0.0;
		}
	}
	int i, j, k;
	for (i = 0; i<n; i++)
	{
		for (j = 0; j<n - 1; j++)
		{
			for (k = 0; k<n - 1; k++)
			{
				temp[j][k] = arcs[j + 1][(k >= i) ? k + 1 : k];

			}
		}
		double t = getA(temp, n - 1);
		if (i % 2 == 0)
		{
			ans += arcs[0][i] * t;
		}
		else
		{
			ans -= arcs[0][i] * t;
		}
	}
	return ans;
}


bool GetMatrixInverse(double** src, int n, double** des)
{
	double flag = getA(src, n);
	double **t;
	for (int i = 0; i < n; i++)
	{
		t = new double*[n];
		for (int j = 0; j < n; j++)
		{
			t[j] = new double[n];
		}
	}
	if (flag == 0)
	{
		return false;
	}
	else
	{
		getAStart(src, n, t);
		for (int i = 0; i<n; i++)
		{
			for (int j = 0; j<n; j++)
			{
				des[i][j] = t[i][j] / flag;
			}

		}
	}


	return true;

}


double P(int *center_area,Mat image_a,Mat image_b,int size,int len)
{
	cout << "len size is: " << len << endl;
	int s = size / 3;
	Rect rect1(2*s, s, s, s);
	Rect rect2(2*s, 0, s, s);
	Rect rect3(s, 0, s, s);
	Rect rect4(0, 0, s, s);
	Rect rect5(0, s, s, s);
	Rect rect6(0, 2*s, s, s);
	Rect rect7(s, 2*s, s, s);
	Rect rect8(2*s, 2*s, s, s);
	Rect rect9(s, s, s, s);
	vector<double> score_sim;
	for (int i = 0; i < len; i++)
	{
		switch (center_area[i]+1)
		{
		case 1:
			score_sim.push_back(Similarity(image_a(rect1), s, image_b(rect1), s));
			break;
		case 2:
			score_sim.push_back(Similarity(image_a(rect2), s, image_b(rect2), s));
			break;
		case 3:
			score_sim.push_back(Similarity(image_a(rect3), s, image_b(rect3), s));
			break;
		case 4:
			score_sim.push_back(Similarity(image_a(rect4), s, image_b(rect4), s));
			break;
		case 5:
			score_sim.push_back(Similarity(image_a(rect5), s, image_b(rect5), s));
			break;
		case 6:
			score_sim.push_back(Similarity(image_a(rect6), s, image_b(rect6), s));
			break;
		case 7:
			score_sim.push_back(Similarity(image_a(rect7), s, image_b(rect7), s));
			break;
		case 8:
			score_sim.push_back(Similarity(image_a(rect8), s, image_b(rect8), s));
			break;
		case 9:
			score_sim.push_back(Similarity(image_a(rect9), s, image_b(rect9), s));
			break;
		default:
			break;
		}
	}
	for (int i = 0; i < len; i++)
	{
		cout << score_sim[i] << " ";
	}
	cout << endl;
	double area_sum = 0;
	for (int i = 0; i < score_sim.size(); i++)
	{
		area_sum += score_sim[i];
	}
	return area_sum/score_sim.size();
}


double Angle_sum(Mat dst)
{
	///////////////////////////use findcontour
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	vector<Point> meanpoints;
	findContours(dst, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
	Mat imageContours = Mat::zeros(dst.size(), CV_8UC1);
	Mat Contours = Mat::zeros(dst.size(), CV_8UC1);
	Mat Separated = Mat::zeros(dst.size(), CV_8UC1);
	for (int i = 0; i < contours.size(); i++)
	{
		for (int j = 0; j < contours[i].size(); j++)
		{
			Point P = Point(contours[i][j].x, contours[i][j].y);
			Contours.at<uchar>(P) = 255;
		}
		drawContours(imageContours, contours, i, Scalar(255), 1, 8, hierarchy);
	}
	//imshow("contour image", imageContours);
	//imwrite("contour image.png", imageContours);
	//imshow("point of contours", Contours);
	//imwrite("point of contours.png", Contours);
	
	for (int i = 1; i < contours.size(); i++)
	{
		cout << "the size of " << i << " contour is: " << contours[i].size() << endl;
		int sum_x = 0;
		int sum_y = 0;
		Point meanpoint;
		for (int j = 0; j < contours[i].size(); j++)
		{
			sum_x += contours[i][j].x;
			sum_y += contours[i][j].y;
		}
		meanpoint = Point(sum_x / contours[i].size(), sum_y / contours[i].size());
		if (contours[i].size() > 10)
		{
			meanpoints.push_back(meanpoint);
		}
		else continue;
	}
	
	{
		y_0 = dst.rows / 3;
		y2 = dst.rows * 2 / 3;
		y3 = dst.rows - 1;
		x1 = dst.cols / 3;
		x2 = dst.cols * 2 / 3;
		x3 = dst.cols - 1;

		cv::Point p1 = cv::Point(x1, 0);
		cv::Point p2 = cv::Point(x2, 0);
		cv::Point p3 = cv::Point(0, y_0);
		cv::Point p4 = cv::Point(0, y2);
		cv::Point p5 = cv::Point(x1, y3);
		cv::Point p6 = cv::Point(x2, y3);
		cv::Point p7 = cv::Point(x3, y_0);
		cv::Point p8 = cv::Point(x3, y2);


		cv::line(Separated, p1, p5, cv::Scalar(255, 255, 255), 1, 4);
		cv::line(Separated, p2, p6, cv::Scalar(255, 255, 255), 1, 4);
		cv::line(Separated, p3, p7, cv::Scalar(255, 255, 255), 1, 4);
		cv::line(Separated, p4, p8, cv::Scalar(255, 255, 255), 1, 4);

	}
	Mat Meanpoint_image = Separated.clone();
	/*for (int i = 0; i < meanpoints.size(); i++)
	{
		circle(Separated, meanpoints[i], 2, Scalar(255, 255, 255), 2);
	}*/
	//imshow("Separated image", Separated);
	//imwrite("separated image.png", Separated);

	///////////////////////////////////////computer the center
	vector<Point> MeanPoints;
	y_0 = dst.rows / 3;
	y2 = dst.rows * 2 / 3;
	y3 = dst.rows - 1;
	x1 = dst.cols / 3;
	x2 = dst.cols * 2 / 3;
	x3 = dst.cols - 1;

	int sum_x[9] = { 0,0,0,0,0,0,0,0,0 };
	int sum_y[9] = { 0,0,0,0,0,0,0,0,0 };
	int sum_num[9] = { 0,0,0,0,0,0,0,0,0 };

	for (int i = 0; i < meanpoints.size(); i++)
	{
		if (meanpoints[i].x < x1)
		{
			if (meanpoints[i].y < y_0)
			{
				sum_x[3] += meanpoints[i].x;
				sum_y[3] += meanpoints[i].y;
				sum_num[3]++;
			}
			else
			{
				if (meanpoints[i].y < y2)
				{

					sum_x[4] += meanpoints[i].x;
					sum_y[4] += meanpoints[i].y;
					sum_num[4]++;
				}
				else
				{

					sum_x[5] += meanpoints[i].x;
					sum_y[5] += meanpoints[i].y;
					sum_num[5]++;
				}
			}
		}
		else
		{
			if (meanpoints[i].x < x2)
			{
				if (meanpoints[i].y < y_0)
				{

					sum_x[2] += meanpoints[i].x;
					sum_y[2] += meanpoints[i].y;
					sum_num[2]++;
				}
				else
				{
					if (meanpoints[i].y < y2)
					{

						sum_x[8] += meanpoints[i].x;
						sum_y[8] += meanpoints[i].y;
						sum_num[8]++;
					}
					else
					{

						sum_x[6] += meanpoints[i].x;
						sum_y[6] += meanpoints[i].y;
						sum_num[6]++;
					}
				}
			}
			else
			{
				if (meanpoints[i].y < y_0)
				{

					sum_x[1] += meanpoints[i].x;
					sum_y[1] += meanpoints[i].y;
					sum_num[1]++;
				}
				else
				{
					if (meanpoints[i].y < y2)
					{

						sum_x[0] += meanpoints[i].x;
						sum_y[0] += meanpoints[i].y;
						sum_num[0]++;
					}
					else
					{

						sum_x[7] += meanpoints[i].x;
						sum_y[7] += meanpoints[i].y;
						sum_num[7]++;
					}
				}
			}
		}
	}

	for (int i = 0; i < 9; i++)
	{
		if (sum_num[i] != 0)
		{
			Point temp = Point(sum_x[i] / sum_num[i], sum_y[i] / sum_num[i]);
			MeanPoints.push_back(temp);
			cout << "the " << i << " area x: " << temp.x << endl;
			cout << "the " << i << " area y: " << temp.y << endl;
		}
	}

	for (int i = 0; i < MeanPoints.size(); i++)
	{
		circle(Meanpoint_image, MeanPoints[i], 2, Scalar(255, 255, 255), 2);
circle(afm_src, MeanPoints[i], 2, Scalar(0, 0, 255), 2);
	}
	//imshow("Mean Points", Meanpoint_image);
	//imwrite("mean points.png", Meanpoint_image);
	//imshow("src", afm_src);
	//imwrite("src.png", afm_src);
	//imwrite("centerPoint.png", Triangle_src);

	////////////////////////////////compute the angles
	vector<float> Angles;

	if (MeanPoints.size() > 2)
	{
		Angles.push_back(acos((((MeanPoints.back().x - MeanPoints[0].x)*(MeanPoints[1].x - MeanPoints[0].x) + (MeanPoints.back().y - MeanPoints[0].y)*(MeanPoints[1].y - MeanPoints[0].y)) / (sqrt(pow((MeanPoints.back().x - MeanPoints[0].x), 2) + pow((MeanPoints.back().y - MeanPoints[0].y), 2))*sqrt(pow((MeanPoints[1].x - MeanPoints[0].x), 2) + pow((MeanPoints[1].y - MeanPoints[0].y), 2))))) * 180 / 3.1415926);

		for (int i = 1; i < MeanPoints.size() - 1; i++)
		{
			Angles.push_back(acos((((MeanPoints[i - 1].x - MeanPoints[i].x)*(MeanPoints[i + 1].x - MeanPoints[i].x) + (MeanPoints[i - 1].y - MeanPoints[i].y)*(MeanPoints[i + 1].y - MeanPoints[i].y)) / (sqrt(pow((MeanPoints[i - 1].x - MeanPoints[i].x), 2) + pow((MeanPoints[i - 1].y - MeanPoints[i].y), 2))*sqrt(pow((MeanPoints[i + 1].x - MeanPoints[i].x), 2) + pow((MeanPoints[i + 1].y - MeanPoints[i].y), 2))))) * 180 / 3.1415926);
		}
		Angles.push_back(acos((((MeanPoints[MeanPoints.size() - 2].x - MeanPoints.back().x)*(MeanPoints[0].x - MeanPoints.back().x) + (MeanPoints[MeanPoints.size() - 2].y - MeanPoints.back().y)*(MeanPoints[0].y - MeanPoints.back().y)) / (sqrt(pow((MeanPoints[MeanPoints.size() - 2].x - MeanPoints.back().x), 2) + pow((MeanPoints[MeanPoints.size() - 2].y - MeanPoints.back().y), 2))*sqrt(pow((MeanPoints[0].x - MeanPoints.back().x), 2) + pow((MeanPoints[0].y - MeanPoints.back().y), 2))))) * 180 / 3.1415926);
		for (int i = 0; i < Angles.size(); i++)
		{
			cout << "the " << i << " angle degree is: " << Angles[i] << endl;
		}
		double angle_sum = 0;
		for (int i = 0; i < Angles.size(); i++)
		{
			angle_sum += Angles[i];
		}
		return angle_sum;
	}
	else
	{
		return 0;
	}

}

int *Center_area(Mat dst)
{
	///////////////////////////use findcontour
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	vector<Point> meanpoints;
	findContours(dst, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
	Mat imageContours = Mat::zeros(dst.size(), CV_8UC1);
	Mat Contours = Mat::zeros(dst.size(), CV_8UC1);
	Mat Separated = Mat::zeros(dst.size(), CV_8UC1);
	for (int i = 0; i < contours.size(); i++)
	{
		for (int j = 0; j < contours[i].size(); j++)
		{
			Point P = Point(contours[i][j].x, contours[i][j].y);
			Contours.at<uchar>(P) = 255;
		}
		drawContours(imageContours, contours, i, Scalar(255), 1, 8, hierarchy);
	}

	for (int i = 1; i < contours.size(); i++)
	{
		cout << "the size of " << i << " contour is: " << contours[i].size() << endl;
		int sum_x = 0;
		int sum_y = 0;
		Point meanpoint;
		for (int j = 0; j < contours[i].size(); j++)
		{
			sum_x += contours[i][j].x;
			sum_y += contours[i][j].y;
		}
		meanpoint = Point(sum_x / contours[i].size(), sum_y / contours[i].size());
		if (contours[i].size() > 10)
		{
			meanpoints.push_back(meanpoint);
		}
		else continue;
	}

	{
		y_0 = dst.rows / 3;
		y2 = dst.rows * 2 / 3;
		y3 = dst.rows - 1;
		x1 = dst.cols / 3;
		x2 = dst.cols * 2 / 3;
		x3 = dst.cols - 1;

		cv::Point p1 = cv::Point(x1, 0);
		cv::Point p2 = cv::Point(x2, 0);
		cv::Point p3 = cv::Point(0, y_0);
		cv::Point p4 = cv::Point(0, y2);
		cv::Point p5 = cv::Point(x1, y3);
		cv::Point p6 = cv::Point(x2, y3);
		cv::Point p7 = cv::Point(x3, y_0);
		cv::Point p8 = cv::Point(x3, y2);


		cv::line(Separated, p1, p5, cv::Scalar(255, 255, 255), 1, 4);
		cv::line(Separated, p2, p6, cv::Scalar(255, 255, 255), 1, 4);
		cv::line(Separated, p3, p7, cv::Scalar(255, 255, 255), 1, 4);
		cv::line(Separated, p4, p8, cv::Scalar(255, 255, 255), 1, 4);

	}
	Mat Meanpoint_image = Separated.clone();

	///////////////////////////////////////computer the center
	vector<Point> MeanPoints;
	y_0 = dst.rows / 3;
	y2 = dst.rows * 2 / 3;
	y3 = dst.rows - 1;
	x1 = dst.cols / 3;
	x2 = dst.cols * 2 / 3;
	x3 = dst.cols - 1;

	int sum_x[9] = { 0,0,0,0,0,0,0,0,0 };
	int sum_y[9] = { 0,0,0,0,0,0,0,0,0 };
	int sum_num[9] = { 0,0,0,0,0,0,0,0,0 };

	for (int i = 0; i < meanpoints.size(); i++)
	{
		if (meanpoints[i].x < x1)
		{
			if (meanpoints[i].y < y_0)
			{
				sum_x[3] += meanpoints[i].x;
				sum_y[3] += meanpoints[i].y;
				sum_num[3]++;
			}
			else
			{
				if (meanpoints[i].y < y2)
				{

					sum_x[4] += meanpoints[i].x;
					sum_y[4] += meanpoints[i].y;
					sum_num[4]++;
				}
				else
				{

					sum_x[5] += meanpoints[i].x;
					sum_y[5] += meanpoints[i].y;
					sum_num[5]++;
				}
			}
		}
		else
		{
			if (meanpoints[i].x < x2)
			{
				if (meanpoints[i].y < y_0)
				{

					sum_x[2] += meanpoints[i].x;
					sum_y[2] += meanpoints[i].y;
					sum_num[2]++;
				}
				else
				{
					if (meanpoints[i].y < y2)
					{

						sum_x[8] += meanpoints[i].x;
						sum_y[8] += meanpoints[i].y;
						sum_num[8]++;
					}
					else
					{

						sum_x[6] += meanpoints[i].x;
						sum_y[6] += meanpoints[i].y;
						sum_num[6]++;
					}
				}
			}
			else
			{
				if (meanpoints[i].y < y_0)
				{

					sum_x[1] += meanpoints[i].x;
					sum_y[1] += meanpoints[i].y;
					sum_num[1]++;
				}
				else
				{
					if (meanpoints[i].y < y2)
					{

						sum_x[0] += meanpoints[i].x;
						sum_y[0] += meanpoints[i].y;
						sum_num[0]++;
					}
					else
					{

						sum_x[7] += meanpoints[i].x;
						sum_y[7] += meanpoints[i].y;
						sum_num[7]++;
					}
				}
			}
		}
	}


	vector<int> location;
	for (int i = 0; i < 9; i++)
	{
		if (sum_num[i] != 0)
		{
			Point temp = Point(sum_x[i] / sum_num[i], sum_y[i] / sum_num[i]);
			MeanPoints.push_back(temp);
			cout << "the " << i << " area x: " << temp.x << endl;
			cout << "the " << i << " area y: " << temp.y << endl;
			location.push_back(i);
		}
	}
	for (int i = 0; i < MeanPoints.size(); i++)
	{
		circle(Meanpoint_image, MeanPoints[i], 2, Scalar(255, 255, 255), 2);
		circle(afm_src, MeanPoints[i], 2, Scalar(0, 0, 255), 2);
	}
	imshow("Mean Points", Meanpoint_image);
	cout << "location size: " << location.size() << endl;
	int *center_area;
	if (location.size() != 0)
	{
		center_area = new int[location.size()];
		for (int i = 0; i < location.size(); i++)
		{
			center_area[i] = location[i];
			cout << location[i] << " ";
		}
		cout << endl;
	}
	return center_area;
}

int Count_location(Mat dst)
{
	///////////////////////////use findcontour
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	vector<Point> meanpoints;
	findContours(dst, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
	Mat imageContours = Mat::zeros(dst.size(), CV_8UC1);
	Mat Contours = Mat::zeros(dst.size(), CV_8UC1);
	Mat Separated = Mat::zeros(dst.size(), CV_8UC1);
	for (int i = 0; i < contours.size(); i++)
	{
		for (int j = 0; j < contours[i].size(); j++)
		{
			Point P = Point(contours[i][j].x, contours[i][j].y);
			Contours.at<uchar>(P) = 255;
		}
		drawContours(imageContours, contours, i, Scalar(255), 1, 8, hierarchy);
	}

	for (int i = 1; i < contours.size(); i++)
	{
		cout << "the size of " << i << " contour is: " << contours[i].size() << endl;
		int sum_x = 0;
		int sum_y = 0;
		Point meanpoint;
		for (int j = 0; j < contours[i].size(); j++)
		{
			sum_x += contours[i][j].x;
			sum_y += contours[i][j].y;
		}
		meanpoint = Point(sum_x / contours[i].size(), sum_y / contours[i].size());
		if (contours[i].size() > 10)
		{
			meanpoints.push_back(meanpoint);
		}
		else continue;
	}

	{
		y_0 = dst.rows / 3;
		y2 = dst.rows * 2 / 3;
		y3 = dst.rows - 1;
		x1 = dst.cols / 3;
		x2 = dst.cols * 2 / 3;
		x3 = dst.cols - 1;

		cv::Point p1 = cv::Point(x1, 0);
		cv::Point p2 = cv::Point(x2, 0);
		cv::Point p3 = cv::Point(0, y_0);
		cv::Point p4 = cv::Point(0, y2);
		cv::Point p5 = cv::Point(x1, y3);
		cv::Point p6 = cv::Point(x2, y3);
		cv::Point p7 = cv::Point(x3, y_0);
		cv::Point p8 = cv::Point(x3, y2);


		cv::line(Separated, p1, p5, cv::Scalar(255, 255, 255), 1, 4);
		cv::line(Separated, p2, p6, cv::Scalar(255, 255, 255), 1, 4);
		cv::line(Separated, p3, p7, cv::Scalar(255, 255, 255), 1, 4);
		cv::line(Separated, p4, p8, cv::Scalar(255, 255, 255), 1, 4);

	}
	Mat Meanpoint_image = Separated.clone();

	///////////////////////////////////////computer the center
	vector<Point> MeanPoints;
	y_0 = dst.rows / 3;
	y2 = dst.rows * 2 / 3;
	y3 = dst.rows - 1;
	x1 = dst.cols / 3;
	x2 = dst.cols * 2 / 3;
	x3 = dst.cols - 1;

	int sum_x[9] = { 0,0,0,0,0,0,0,0,0 };
	int sum_y[9] = { 0,0,0,0,0,0,0,0,0 };
	int sum_num[9] = { 0,0,0,0,0,0,0,0,0 };

	for (int i = 0; i < meanpoints.size(); i++)
	{
		if (meanpoints[i].x < x1)
		{
			if (meanpoints[i].y < y_0)
			{
				sum_x[3] += meanpoints[i].x;
				sum_y[3] += meanpoints[i].y;
				sum_num[3]++;
			}
			else
			{
				if (meanpoints[i].y < y2)
				{

					sum_x[4] += meanpoints[i].x;
					sum_y[4] += meanpoints[i].y;
					sum_num[4]++;
				}
				else
				{

					sum_x[5] += meanpoints[i].x;
					sum_y[5] += meanpoints[i].y;
					sum_num[5]++;
				}
			}
		}
		else
		{
			if (meanpoints[i].x < x2)
			{
				if (meanpoints[i].y < y_0)
				{

					sum_x[2] += meanpoints[i].x;
					sum_y[2] += meanpoints[i].y;
					sum_num[2]++;
				}
				else
				{
					if (meanpoints[i].y < y2)
					{

						sum_x[8] += meanpoints[i].x;
						sum_y[8] += meanpoints[i].y;
						sum_num[8]++;
					}
					else
					{

						sum_x[6] += meanpoints[i].x;
						sum_y[6] += meanpoints[i].y;
						sum_num[6]++;
					}
				}
			}
			else
			{
				if (meanpoints[i].y < y_0)
				{

					sum_x[1] += meanpoints[i].x;
					sum_y[1] += meanpoints[i].y;
					sum_num[1]++;
				}
				else
				{
					if (meanpoints[i].y < y2)
					{

						sum_x[0] += meanpoints[i].x;
						sum_y[0] += meanpoints[i].y;
						sum_num[0]++;
					}
					else
					{

						sum_x[7] += meanpoints[i].x;
						sum_y[7] += meanpoints[i].y;
						sum_num[7]++;
					}
				}
			}
		}
	}


	vector<int> location;
	for (int i = 0; i < 9; i++)
	{
		if (sum_num[i] != 0)
		{
			Point temp = Point(sum_x[i] / sum_num[i], sum_y[i] / sum_num[i]);
			MeanPoints.push_back(temp);
			cout << "the " << i << " area x: " << temp.x << endl;
			cout << "the " << i << " area y: " << temp.y << endl;
			location.push_back(i);
		}
	}
	for (int i = 0; i < MeanPoints.size(); i++)
	{
		circle(Meanpoint_image, MeanPoints[i], 2, Scalar(255, 255, 255), 2);
		circle(afm_src, MeanPoints[i], 2, Scalar(0, 0, 255), 2);
	}
	return location.size();
}


/*double **res;
for (int i = 0; i < 3; i++)
{
res = new double*[3];
for (int j = 0; j < 3; j++)
{
res[j] = new double[3];
}
}
double **srr;
for (int i = 0; i < 3; i++)
{
srr = new double*[3];
for (int j = 0; j < 3; j++)
{
srr[j] = new double[3];
}
}

for (int i = 0; i < 3; i++)
{
for (int j = 0; j < 3; j++)
{
cout << srr[i][j] << " ";
}
cout << endl;
}
GetMatrixInverse(srr,3, res);

for (int i = 0; i < 3; i++)
{
for (int j = 0; j < 3; j++)
{
cout << res[i][j] << " ";
}
cout << endl;
}*/
//Mat test = dst.clone();
//resize(dst, test, cv::Size(56, 56));
//imshow("test", test);
//Mat imm = imread("\images\\Optical_image.png");
//resize(imm, imm, cv::Size(830, 700));
//imshow("source", imm);
//for (int i = 0;i<dst.rows;i++)
//{
//	for (int j = 0; j < dst.cols; j++)
//	{
//		if (int(dst.at<uchar>(i, j)) > max/2+20)//*2/3)//ȡ������ص�0.5+20��Ϊ��ֵ
//		{
//			dst.at<uchar>(i, j) = 255;
//		}
//		else
//		{
//			dst.at<uchar>(i, j) = 0;
//		}
//		//cout << int(dst.at<uchar>(i, j)) << " ";
//	}
//	//cout << endl;
//}
