#include<iostream>
#include<string>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cv.h>

using namespace std;

int main()
{
	cv::VideoCapture capture;
	cv::Mat input, convertedOutput,imgThreshed;

    string videoPath = "http://10.0.1.178/mjpg/video.mjpg";

    if(!capture.open(videoPath))
    {
    	cout<<"Video Not Found"<<endl;
        return -1;
    }

    while(true)
    {
    	capture>>input;  //Read a frame from the video

        if(input.empty()) //Check if the frame has been read correctly or not
        {
        	cout<<"Capture Finished"<<endl;
            break;
        }

        cv::cvtColor(input,convertedOutput,CV_BGR2HSV);
	  	cv::inRange(convertedOutput, cv::Scalar(159, 135, 135), cv::Scalar(179, 255, 255), imgThreshed);
        cv::imshow("Output",imgThreshed);
        cv::waitKey(10);
    }

    capture.release();

}
