#include <merbots_tracking/util/PHOG.h>

PHOG::PHOG(const int bins) :
      nbins(bins)
{
}

PHOG::~PHOG()
{
}

void PHOG::describe(const cv::Mat &image, cv::Mat &desc)
{
    cv::Mat img = image;
    if (img.channels() > 1)
    {
        // Convert the image to grayscale
        cv::cvtColor(img, img, CV_BGR2GRAY);
    }

    // Mean and Standard Deviation
    cv::Scalar cvMean;
    cv::Scalar cvStddev;
    cv::meanStdDev(img, cvMean, cvStddev);
    double mean = cvMean(0);

    // Apply Canny Edge Detector
    cv::Mat edges;
    // Reduce noise with a kernel 3x3    
    cv::blur(img, edges, cv::Size(3,3));
    // Canny detector    
    cv::Canny(edges, edges, 0.66 * mean, 1.33 * mean);

    //  Computing the gradients.
    // Generate grad_x and grad_y
    cv::Mat grad_x, grad_y;

    // Gradient X    
    cv::Sobel(img, grad_x, CV_32F, 1, 0, 3);

    // Gradient Y
    cv::Sobel(img, grad_y, CV_32F, 0, 1, 3);

    // Total Gradient (approximate)
    cv::Mat grad_m = cv::abs(grad_x) + cv::abs(grad_y);

    // Computing orientations
    cv::Mat grad_o;
    cv::phase(grad_x, grad_y, grad_o, true);

    // Quantizing orientations into bins.
    double w = 360.0 / (double)nbins;
    grad_o = grad_o / w;

    // Creating the descriptor.
    desc = cv::Mat::zeros(1, nbins + 4 * nbins + 16 * nbins, CV_32F);
    int width = image.cols;
    int height = image.rows;

    // Level 0
    cv::Mat chist = desc.colRange(0, nbins);
    getHistogram(edges, grad_o, grad_m, 0, 0, width, height, chist);

    // Level 1
    chist = desc.colRange(nbins, 2 * nbins);
    getHistogram(edges, grad_o, grad_m, 0, 0, width / 2, height / 2, chist);

    chist = desc.colRange(2 * nbins, 3 * nbins);
    getHistogram(edges, grad_o, grad_m, 0, width / 2, width / 2, height / 2, chist);

    chist = desc.colRange(3 * nbins, 4 * nbins);
    getHistogram(edges, grad_o, grad_m, height / 2, 0, width / 2, height / 2, chist);

    chist = desc.colRange(4 * nbins, 5 * nbins);
    getHistogram(edges, grad_o, grad_m, height / 2, width / 2, width / 2, height / 2, chist);

    // Level 2
    int wstep = width / 4;
    int hstep = height / 4;
    int binPos = 5; // Next free section in the histogram
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            chist = desc.colRange(binPos * nbins, (binPos + 1) * nbins);
            getHistogram(edges, grad_o, grad_m, i * hstep, j * wstep, wstep, hstep, chist);
            binPos++;
        }
    }

    // Normalizing the histogram.
    cv::Mat_<float> sumMat;
    cv::reduce(desc, sumMat, 1, CV_REDUCE_SUM);
    float sum = sumMat.at<float>(0, 0);
    desc = desc / sum;
}

double PHOG::distChiSquare(const cv::Mat &a, const cv::Mat &b)
{
    int sz = a.cols;
    cv::Mat_<float> chsvals = cv::Mat::zeros(1, sz, CV_32F);

    //#pragma omp parallel for
    for (int i = 0; i < sz; i++)
    {
        float hi = a.at<float>(0, i);
        float hj = b.at<float>(0, i);
        if (hi > 0 && hj > 0)
        {
            chsvals(0, i) = ((hi - hj) * (hi - hj)) / (hi + hj);
        }

        //std::cout << hi << " " << hj <<  ": " << chsvals(0, i) << std::endl;
    }
    //exit(0);

    cv::Mat sumMat;
    cv::reduce(chsvals, sumMat, 1, CV_REDUCE_SUM);
    return (double)sumMat.at<float>(0, 0);
}

void PHOG::getHistogram(const cv::Mat &edges, const cv::Mat &ors, const cv::Mat &mag, int startX, int startY, int width, int height, cv::Mat &hist)
{
    // Find and increment the right bin(s)
    for (int x = startX; x < startX + height; x++)
    {
        for (int y = startY; y < startY + width; y++)
        {
            if (edges.at<uchar>(x,y) > 0)
            {
                int bin = (int)std::floor(ors.at<float>(x, y));
                hist.at<float>(0, bin) = hist.at<float>(0, bin) + mag.at<float>(x, y);
            }
        }
    }
}
