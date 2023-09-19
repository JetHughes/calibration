#include <opencv2/opencv.hpp>
#include <stdio.h>

const int NUM_IMAGES = 31;
const cv::Size PATTERN_SIZE(9, 6);
const float SQUARE_WIDTH = 20.0f;  // mm
const cv::Size IMAGE_SIZE(2592, 1944);


void drawCorners(cv::Mat& image, char *fname)
{
    std::vector<cv::Point2f> corners;
    bool patternWasFound = cv::findChessboardCorners(image, PATTERN_SIZE, corners);
    if (patternWasFound == false)
    {
        std::cout << "No pattern found" << std::endl;
        return;
    } 
    //cv::drawChessboardCorners(image, PATTERN_SIZE, corners, patternWasFound);
    //cv::imshow(fname, image);
    //cv::waitKey();
    //cv::destroyAllWindows();
}

std::vector<cv::Mat> loadImages(char *path)
{
    std::vector<cv::Mat> images;
    char fname[255];
    for (int i = 0; i < NUM_IMAGES; i++)
    {
        sprintf_s(fname, path, i);
        cv::Mat image = cv::imread(fname);

        if (image.empty())
        {
            std::cerr << "could not load image from " << fname << std::endl;
        }
        else
        {
            cv::Mat bwImage;
            cv::cvtColor(image, bwImage, cv::COLOR_RGB2GRAY);
            image = bwImage;
            drawCorners(image, fname);
            std::cout << "loaded: " << fname << std::endl;
            images.push_back(image);
        }
    }
    return images;
}


void fillCheckerboard(std::vector<cv::Point3f> &checkerboardPattern)
{
    for (int y = 0; y < PATTERN_SIZE.height; ++y)
    {
        for (int x = 0; x < PATTERN_SIZE.width; ++x)
        {
            cv::Point3f point(x * SQUARE_WIDTH, y * SQUARE_WIDTH, .0f);
            checkerboardPattern.push_back(point);
        }
    }
}


double calibrateSingle(std::vector<cv::Mat> &images)
{
    std::vector<double> distCoeffs;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<cv::Point3f> checkerboardPattern;    
    fillCheckerboard(checkerboardPattern);
    cv::Mat K;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    std::cout << "find image and objects points from " << images.size() << " images" << std::endl;
    for (const auto &image : images)
    {
        std::vector<cv::Point2f> corners;
        cv::findChessboardCorners(image, PATTERN_SIZE, corners);
        printf(".");
        imagePoints.push_back(corners);
        objectPoints.push_back(checkerboardPattern);
    }
    
    std::cout << "start calibration" << std::endl;
    double err = cv::calibrateCamera(objectPoints, imagePoints, IMAGE_SIZE, K, distCoeffs, rvecs, tvecs);

    std::cout << "done" << std::endl;
    std::cout << "\nCamera matrix\n"
              << K << std::endl;
    std::cout << "\nError: " << err << std::endl;
    return err;
}

void calibrateMany() {

}

int main(int argc, char *argv[])
{
    std::vector<cv::Mat> images = loadImages("C:/Users/HCIadmin/jet/egocentric-vision/calibration-images/bl/bl_%d.jpg");
    std::cout << "loaded images" << std::endl;

    calibrateSingle(images);

    return 0;
}