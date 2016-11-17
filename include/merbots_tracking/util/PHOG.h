#ifndef _PHOG_H
#define _PHOG_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>

class PHOG
{
public:
    PHOG(const int bins = 60);
    ~PHOG();

    void describe(const cv::Mat& image, cv::Mat& desc);
    static double distChiSquare(const cv::Mat& a, const cv::Mat& b);

private:
    int nbins;

    void getHistogram(const cv::Mat& edges, const cv::Mat& ors, const cv::Mat& mag, int startX, int startY, int width, int height, cv::Mat& hist);
};

#endif /* _PHOG_H */
