#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>   
#include <opencv2/core/core.hpp>  
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <string>  
#include <tag_detector/blur_window_size.h>
static double sigma1 = 3.0; // for gaussianBlur x, y
static double sigma2 = 3.0;
static int g_size1;
static int g_size2;
static bool service_flag = false;
/*
	原来打算用 createTrackbar 函数内的callback 与service的callback结合的方式调用高斯滤波器，后面脑子短路了 ：（
*/
bool Set_window_size(tag_detector::blur_window_sizeRequest &req, tag_detector::blur_window_sizeRequest &res) 
{ 
	 
	g_size1 = req.a;
	g_size2 = res.b;
	service_flag = true;
	return true; 
}


//*********************主函数**********************//

int main(int argc, char** argv)
{
	
	cv::VideoCapture cap("/home/dev/Documents/Bito_ws/src/tag_detector/dataset/demo.mp4");
	cv::Mat image = cv::imread("/home/dev/Documents/Bito_ws/src/tag_detector/dataset/test.png");
	int blur_size = 1;
	if(image.empty()) // 检查是否读取到图片
	{
		std::cout << "fail to load image ..." << std::endl;
		return -1;	
	}
	//cv::imshow("demo", image);
	//cv::waitKey(30000); // 节点启动：显示图片，等待按键按下...
	//cv::destroyWindow("demo");
	
	if(!cap.isOpened()) // 检查是否加载视频
	{
		std::cout<< "load video fail ..." << std::endl;
		return -1;
	}

	ros::init(argc, argv, "node_1");
	ros::NodeHandle nh;
	ros::NodeHandle nh2;
	ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/image_raw", 10);
	ros::ServiceServer service = nh.advertiseService("/set_blur_window_size", Set_window_size);
	//ros::ServiceClient client = nh2.serviceClient<blur_window_size::blur_window_size>("accept_size");
	//blur_window_size::blur_window_size srv;
	cv::Mat frame;
	ros::Rate loop_rate(20);
	while(ros::ok())
	{
		ros::spinOnce();
		cap.read(frame);
		if(frame.empty()) // 也可以用try_catch...
		{
			break;
		}
		else
		{

			if(service_flag)
			{
				GaussianBlur(frame, frame, cv::Size(g_size1 * 2 + 1, g_size2 * 2 + 1), sigma1, sigma2);
				//cv::putText(frame, "gaussianBlur" + std::to_string(g_size1), cv::Point(50, 60), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 4, 8); //时间显示
			}
			ROS_INFO_STREAM("start pub img_data..." << service_flag);
/*
			cv::namedWindow("Gauss_Blur", WINDOW_AUTOSIZE); 

			if (client.call(srv))  // 客户端
			{
				if(srv.repose.name)  // name为true
				{
					blur_size = srv.request.size; // 

				}
				
				cv::createTrackbar("Window_size：", "Gauss_Blur", &blur_size, 40 , Gauss_Blur_Callback);
				Gauss_Blur_Callback(blur_size, 0);
			}
*/
			//cv::imshow("PUB", frame);
			//cv::waitKey(1); 	//用来可视化发布的图像，推荐在Rviz中可视化，可以减少延迟
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg(); // 图片格式转换
			image_pub.publish(msg);
			loop_rate.sleep();
		}
	}
	
return 0;
}
