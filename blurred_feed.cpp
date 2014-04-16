#include <iostream>
#include <stdio.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;    // std:cout, std::endl
using namespace cv;     


int main()
{
	CvCapture* RTfeed = cvCreateCameraCapture(0); // CV_CAP_ANY to use any camera connected to the machine
	cvNamedWindow("RTfeed");
        cvNamedWindow("blurred feed", CV_WINDOW_AUTOSIZE);
        cvNamedWindow("resized_half", CV_WINDOW_AUTOSIZE);
        cvNamedWindow("resized_1.5", CV_WINDOW_AUTOSIZE);

        IplImage* frame;
    
	Mat manipulate_frame, blurred_img,  resize_sm, resize_large;
   
	char c;
      
	for(;;){
	    frame = cvQueryFrame(RTfeed);
            manipulate_frame = frame;
             
	    if(!frame || !manipulate_frame.data) {
		cout << "camera not detected" << endl;
	        break;
	    }
	   	
	   cvShowImage ("RTfeed", frame);
		  
	   blur(manipulate_frame, blurred_img, Size(15,15)); //kernel size = 15x15
           
 	   resize(blurred_img, resize_sm, Size(), 0.5,0.5,CV_INTER_AREA);	   
           resize(blurred_img, resize_large, Size() , 1.5,1.5, CV_INTER_LINEAR);  // scale the window by a factor of 1.5 -----CV_INTER_CUBIC(slow) or CV_INTER_LINEAR(faster but looks ok) 
	   imshow("blurred feed", blurred_img);          
	   imshow("resized_half", resize_sm);		
           imshow("resized_1.5", resize_large);

         
	
 
           c = cvWaitKey(33);
	   if (c==27) break;
	}
       
        cvReleaseCapture(&RTfeed);
	cvDestroyAllWindows();

	 
}
