#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>   
#include <opencv2/core/core.hpp>  
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <math.h>
#include <string>    
#include<ctime> // calculate the cost of time ...
#include <tag_detector/blur_window_size.h>
using namespace cv;
using namespace std;

static cv::Mat image; // 存贮订阅的图像	

/*
*****订阅话题-回调函数******
*/

static void Image_Callback( const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr; // 声明一个CvImage指针的实例
	try{
				//image = cv_bridge::toCvShare(msg, "rgb8")->image; //获取到img并显示出，try保证在未启动pub节点时节点不会shutdown
				//cv::imshow("SUB", images);
				//cv::waitKey(1);
				cv_ptr =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
			}
	catch(cv_bridge::Exception& e)
		{
				if(!image.empty())
				{
					ROS_INFO_STREAM("convert format to cv_image fail...");// 接收到了图片但是格式不对
				}
				else
				{
						ROS_INFO_STREAM("no image date subscribed, plz run the image_pub node...");//没有接受到图片
				}
			return;
		}
	image = cv_ptr->image;
}

/*
*****锐化图片******
*/
static cv::Mat deblur(cv::Mat &img)
{
	
	Mat res_image;
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;
	Mat grad;
	Sobel(img, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT); // x方向
	Sobel(img, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT); // y方向
	convertScaleAbs(grad_x, abs_grad_x);
	convertScaleAbs(grad_y, abs_grad_y);
	addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
	ROS_INFO_STREAM(" find rectangle fail, try to use deblur function ...");
	//imshow("Deblur", grad);
	//cv::waitKey(10000000);
	return grad;
}


/*
*****寻找矩形******
*/

static void findSquares( const Mat& img, vector<vector<Point> >& squares )
{
	int k = 1;
	squares.clear();
	Mat gray_img, result;
	vector<vector<cv::Point> > contours;
	//ROS_INFO_STREAM("here 0 ...");

	cvtColor(img, gray_img, CV_RGB2GRAY); //转换为灰度图
	//DEBLUR:
	threshold(gray_img, result, 100, 255, CV_THRESH_BINARY); //二值化
	//ROS_INFO_STREAM("here 1 ...");
	findContours(result, contours, RETR_LIST, CHAIN_APPROX_SIMPLE); //提起所有轮廓
	//ROS_INFO_STREAM("here 2 ...");
	Mat res_image = Mat::zeros(result.size(), CV_8UC3);
	//ROS_INFO_STREAM("here 3 ...");
	vector<Point> approx;
	double d1, d2;
	//ROS_INFO_STREAM("contours.size: "<< contours.size());
	for (int i = 0; i < contours.size(); i++) //提取四边形
	{ 
		drawContours(res_image, contours, i, (0, 255, 0), 2); 
		double epsilon = 0.01 * arcLength(contours[i], true);
		approxPolyDP(contours[i], approx, epsilon, true); // 逼近
		if( approx.size() == 4 )
		{
			d1 = sqrt(pow((approx[1].x - approx[2].x), 2) + pow((approx[1].y - approx[2].y), 2));
			d2 = sqrt(pow((approx[1].x - approx[3].x), 2) + pow((approx[1].y - approx[3].y), 2));
			if(d1 * d2 > 1000) // 删除面积很小的干扰轮廓
				{
					squares.push_back(approx);
					//ROS_INFO_STREAM("push_back approx ..."<< approx.size());
				}
		}
		/*										// 根据检测出矩形的数量决定是否调用锐化函数，锐化这里好像写的不太好，或许只需锐化一次，不需要goto语句
		if(squares.size() < 25)
		{
			gray_img = deblur(gray_img);
			k++;
			if(k < 5) goto DEBLUR;
		}
		*/
	}
	ROS_INFO_STREAM("squares_size: "<< squares.size());

}

/*
*****绘制矩形-可视化******
*/

static void drawSquares( Mat& image, const vector<vector<Point> >& squares ,double time) // 画出矩形
	{
		for( size_t i = 0; i < squares.size(); i++ )
		{
			const Point* p = &squares[i][0];
			int n = (int)squares[i].size();
			if (p-> x > 3 && p->y > 3) polylines(image, &p, &n, 1, true, Scalar(0, 255, 0), 3, LINE_AA);
		}
		const std::string& new_time = std::to_string(time);
		putText(image, new_time, Point(50, 60), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 255), 4, 8); //时间显示
		imshow("SUB", image);
		cv::waitKey(1);
	}

/*
*****FFT变换******
*/

static  cv::Mat fft1(cv::Mat& img) 
{ 
	cvtColor(img, img, CV_RGB2GRAY); //转换为灰度图
	int M = getOptimalDFTSize( img.rows );       // 获得最佳DFT尺寸，为2的次方  
	int N = getOptimalDFTSize( img.cols );       //同上  
	Mat padded;  
	copyMakeBorder(img, padded, 0, M - img.rows, 0, N - img.cols, BORDER_CONSTANT, Scalar::all(0)); // 补零 
	Mat planes[] = { Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F) };  
	Mat complexImg;                                                                                                      
	merge(planes, 2, complexImg);  // 合并  

	dft(complexImg, complexImg);  // FFT变换， dft需要一个2通道的Mat  

	// compute log(1 + sqrt(Re(DFT(img))**2 + Im(DFT(img))**2))  
	split(complexImg, planes);   // 分离通道， planes[0] 为实数部分，planes[1]为虚数部分  
	magnitude(planes[0], planes[1], planes[0]);  // 求模  sqrt(x^2 + y^2)
	Mat mag = planes[0];  
	mag += Scalar::all(1);                                                                                              
	log(mag, mag);								// 模的对数 

	mag = mag( Rect(0, 0, mag.cols & -2, mag.rows & -2) );    //保证偶数 
	int cx = mag.cols / 2;  
	int cy = mag.rows / 2;  
	//对傅立叶变换的图像进行重排，4个区块，从左到右，从上到下 分别为q0, q1, q2, q3  
	// 对调q0和q3, q1和q2 ,方便观察
	Mat tmp;  
	Mat q0(mag, Rect(0, 0, cx, cy));  
	Mat q1(mag, Rect(cx, 0, cx, cy));  
	Mat q2(mag, Rect(0, cy, cx, cy));  
	Mat q3(mag, Rect(cx, cy, cx, cy));  
	q0.copyTo(tmp);  
	q3.copyTo(q0);  
	tmp.copyTo(q3);  
	q1.copyTo(tmp);  
	q2.copyTo(q1);  
	tmp.copyTo(q2);  

	normalize(mag, mag, 0, 1, CV_MINMAX);   // 规范化值到 0~1 显示图片的需要 
	return mag; 
}


/*
*****计算图像平均梯度-均值-方差******
*/
/*
*****评估图像******
*
*均值反映了图像的亮度，均值越大说明图像亮度越大，反之越小；
*标准差反映了图像像素值与均值的离散程度，标准差越大说明图像的质量越好；
*平均梯度反映了图像的清晰度和纹理变化，平均梯度越大说明图像越清晰；
*/
static double evaluate_pic(cv::Mat &img)
{
	int rows = img.rows - 1;
	int cols = img.cols - 1;
	Mat mat_mean, mat_stddev;
	double m, s;
	double tmp;
	double ds = 0;
	cv::meanStdDev(img, mat_mean, mat_stddev); //计算均值与标准差
	m = mat_mean.at<double>(0, 0);
	s = mat_stddev.at<double>(0, 0);
	for (int i = 0; i < rows; i++)  // 计算平均梯度
	{ 
		for (int j = 0; j < cols; j++) 
			{ 
				double dx = img.at<double>(i, j + 1) - img.at<double>(i, j); 
				double dy = img.at<double>(i + 1, j) - img.at<double>(i, j); 
				double ds = std::sqrt((dx*dx + dy*dy) / 2);
				//ROS_INFO_STREAM("ds:" << ds); 
				tmp += ds; 
			} 
	} 
	double imageAvG = tmp / (rows*cols);
	double value = imageAvG;
	//ROS_INFO_STREAM("mean" << m);
	//ROS_INFO_STREAM("s:" << s);
	//ROS_INFO_STREAM("imageAvG:" << imageAvG);
	//cout << path << "mean_gradient：" << imageAvG << endl;
	return value;
}




/*
*****main函数******
*/

int main(int argc, char** argv)
{
	clock_t startTime, endTime;
	vector<vector<Point> > squares;
	ros::init(argc, argv, "node_2");
	ros::NodeHandle nh; 
	//cv::namedWindow("SUB");
	//cv::namedWindow("FFT");  
	cv::startWindowThread(); 
	ros::Subscriber image_subcscriber = nh.subscribe("/image_raw", 1, Image_Callback);
	ros::ServiceClient client = nh.serviceClient<tag_detector::blur_window_size>("/set_blur_window_size");
	double total_time;
	ros::Rate loop_rate(20);
	while(ros::ok())
	{
		if (argc != 3)  							// 输入参数提示：发布service->request 内容
		{
		  ROS_INFO_STREAM("Warning: need another two params for gauss_blur : kenerl_x , kenerl_y ");
		  return 1;
		}

		tag_detector::blur_window_size srv;
		srv.request.a = atoll(argv[1]);
		srv.request.b = atoll(argv[2]);
		if (client.call(srv))
  	{
   		ROS_INFO_STREAM("WINDOW_SIZE: " << srv.request.a);
  	}

		ros::spinOnce();

		if(image.empty())								// 判断是否接受到图片
		{
			ROS_INFO_STREAM("no images recieved，plz start pub node first..."); 
		}
		else
		{
			cv::Mat img_fft;
			double total_score;
			image.copyTo(img_fft);       // 新copy一个img用来做FFT
			img_fft = fft1(img_fft);
			//total_score = evaluate_pic(image);
			//ROS_INFO_STREAM("Total_score: " << total_score);
			startTime = clock(); 
			findSquares(image, squares); // 仅仅考虑计算时间
			endTime = clock();
			total_time = (double)(endTime - startTime) / CLOCKS_PER_SEC;
			drawSquares(image, squares,total_time);
			
			//imshow("FFT", img_fft);
			//cv::waitKey(1);
			//ROS_INFO_STREAM("Here we go ...");
		}
		//cv::imshow("SUB", images);
		//cv::waitKey(1);
		loop_rate.sleep();
	}
  //cv::destroyWindow("SUB");
	cv::destroyWindow("FFT");
	return 0;
}
