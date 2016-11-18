#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

Point point1, point2; /* vertical points of the bounding box */
int drag = 0;
Rect rect; /* bounding box */
Mat img, roiImg; /* roiImg - the part of the image in the bounding box */
int select_flag = 0;

void mouseHandler(int event, int x, int y, int flags, void* param)
{
    if (event == CV_EVENT_LBUTTONDOWN && !drag)
    {
        /* left button clicked. ROI selection begins */
        point1 = Point(x, y);
        drag = 1;
    }

    if (event == CV_EVENT_MOUSEMOVE && drag)
    {
        /* mouse dragged. ROI being selected */
        Mat img1 = img.clone();
        point2 = Point(x, y);
        rectangle(img1, point1, point2, CV_RGB(255, 0, 0), 3, 8, 0);
        imshow("image", img1);
    }

    if (event == CV_EVENT_LBUTTONUP && drag)
    {
        point2 = Point(x, y);
        rect = Rect(point1.x,point1.y,x-point1.x,y-point1.y);
        drag = 0;
        roiImg = img(rect);
        cv::imwrite("/home/emilio/Escritorio/target.png", roiImg);
        exit(0);
    }

    if (event == CV_EVENT_LBUTTONUP)
    {
        /* ROI selected */
        select_flag = 1;
        drag = 0;
    }
}

int main()
{
    int k;
    VideoCapture cap = VideoCapture(0); /* Start webcam */
    cap >> img; /* get image(Mat) */
    imshow("image", img);
    while(1)
    {
        cap >> img;
        cvSetMouseCallback("image", mouseHandler, NULL);
        if (select_flag == 1)
        {
            imshow("ROI", roiImg); /* show the image bounded by the box */
        }
        rectangle(img, rect, CV_RGB(255, 0, 0), 3, 8, 0);
        imshow("image", img);
        k = waitKey(10);
        if (k == 27)
        {
            break;
        }
    }
    return 0;
}
