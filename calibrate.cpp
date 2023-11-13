#include <opencv2/opencv.hpp>
#include <stdio.h>

const int NUM_IMAGES = 28; // Number of images used for intrinsic calibration
const int EXT_IMG_NUM = 0; // Index of image to use for extrinsic calibration
const bool PREVIEW_IMAGES = false;
const bool SHOW_REPROJECTION = false;

// Checkerboard parameters
const cv::Size PATTERN_SIZE(10, 7); 
const float SQUARE_WIDTH = 33.5f;  // mm

// cornerSubPix configuration parameters
cv::TermCriteria TERMCRIT(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
cv::Size WINSIZE(9, 9), ZERO_ZONE(-1, -1);

void drawCorners(cv::Mat& image, char *fname)
{
    std::vector<cv::Point2f> corners;
    bool patternWasFound = cv::findChessboardCorners(image, PATTERN_SIZE, corners);
    if (!patternWasFound)
    {
        std::cout << "No pattern found" << std::endl;
        return;
    } 
    cv::drawChessboardCorners(image, PATTERN_SIZE, corners, patternWasFound);
    cv::resize(image, image, cv::Size(), 0.5, 0.5);
    cv::imshow(fname, image);
    cv::waitKey();
    cv::destroyAllWindows();
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
            if (PREVIEW_IMAGES) {
                image = bwImage;
                drawCorners(image, fname);   
            }
            std::cout << "loaded: " << fname << std::endl;
            images.push_back(bwImage);
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

cv::Point2d calibrate(std::vector<cv::Mat> &images, cv::Size size)
{
    std::vector<double> distCoeffs;
    std::vector<cv::Mat> tvecs; 
    std::vector<cv::Mat> rvecs;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<cv::Point3f> checkerboardPattern;    
    fillCheckerboard(checkerboardPattern);

    //float data[10] = {1,0,0,
    //                  0,1,0,
    //                  0,0,1};
    //cv::Mat K(3, 3, CV_32FC1, &data);
    cv::Mat K;

    // Find image and object points
    std::cout << "\nfind image and objects points from " << images.size() << " images";
    for (const auto &image : images)
    {
        std::vector<cv::Point2f> corners;
        cv::findChessboardCorners(image, PATTERN_SIZE, corners);
        cv::cornerSubPix(image, corners, WINSIZE, ZERO_ZONE, TERMCRIT);
        printf(".");
        imagePoints.push_back(corners);
        objectPoints.push_back(checkerboardPattern);
    }
    
    //Calibrate
    std::cout << "\nCalibrate" << std::endl;
    double err = cv::calibrateCamera(objectPoints, imagePoints, size, K, distCoeffs, rvecs, tvecs);
    cv::Mat tvec = tvecs[EXT_IMG_NUM]; // Choose one image for extrinsic
    cv::Mat rvec = rvecs[EXT_IMG_NUM];
    std::cout << "Error: " << err << std::endl;
    std::cout << "tvec" << tvec << std::endl;
    std::cout << "rvec" << rvec << std::endl;
    std::cout << "k" << K << std::endl;


    // -----Visually Check Calibration by reprojecting object points onto image-----
    if (SHOW_REPROJECTION) {
        // Reproject points
        cv::Mat extImg_gray = images[EXT_IMG_NUM].clone();
        cv::Mat extImg(extImg_gray.size(), CV_8UC3); // add colour channel to draw projected points as red dots
        cv::cvtColor(extImg_gray, extImg, cv::COLOR_GRAY2BGR);

        std::vector<cv::Point2f> projectedPoints;
        cv::projectPoints(objectPoints[EXT_IMG_NUM], rvec, tvec, K, distCoeffs, projectedPoints);
        for (const cv::Point2f& point : projectedPoints) {
            cv::circle(extImg, point, 2, cv::Scalar(0, 0, 255), -1); // Red circle
        }
        cv::resize(extImg, extImg, cv::Size(), 0.5, 0.5);
        cv::imshow("original image with reprojected points", extImg);

        // Undistort image
        cv::Mat undistortedImg(extImg.size(), CV_8UC3);
        cv::undistort(extImg, undistortedImg, K, distCoeffs);
        cv::imshow("undistorted image with reprojected points", undistortedImg);

        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    // return the x and y coords of the camera. Assume difference in z coord is negligible
    return cv::Point2f(tvec.at<double>(0), tvec.at<double>(1));
}

void visualiseCalibration(std::vector<cv::Point2f> points) {
    int windowSize = 480;
    int scaleFactor = 3;  // if you dont see the points on the window you might need to change the scale
    cv::Mat image(windowSize, windowSize, CV_8UC1, 255);;

    // Calculate the center of the cameras
    cv::Point2f center(0, 0);
    for (const cv::Point2f& point : points) {
        center.x += point.x;
        center.y += point.y;
    }
    center.x /= points.size();
    center.y /= points.size();

    for (const cv::Point2d& point : points) {
        // move cameras to the center of the window
        cv::Point2d relativePosition((point.x-center.x)*scaleFactor+windowSize/2, 
                                     (point.y-center.y)*scaleFactor+windowSize/2);

        // Round values
        std::string roundedX = std::to_string(std::round((point.x - points[0].x) * 10.0) / 10.0);
        std::string roundedY = std::to_string(std::round((point.y - points[0].y) * 10.0) / 10.0);
        roundedX.erase(roundedX.find_last_not_of('0') + 1, std::string::npos);
        roundedY.erase(roundedY.find_last_not_of('0') + 1, std::string::npos);
        std::string text = "(" + roundedX + ", " + roundedY + ")";

        // Draw points and labels
        cv::circle(image, relativePosition, 5, 0, -1);
        cv::putText(image, text, cv::Point(relativePosition.x, relativePosition.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, 0, 1);
    }
    cv::imshow("Calibration", image);
    cv::waitKey(0);
}

int main(int argc, char *argv[])
{
    std::vector<cv::Mat> imagesBL = loadImages("./images/images/bl/bl_%02d.jpg");
    std::vector<cv::Mat> imagesBR = loadImages("./images/images/br/br_%02d.jpg");
    std::vector<cv::Mat> imagesTL = loadImages("./images/images/tl/tl_%02d.jpg");
    std::vector<cv::Mat> imagesTR = loadImages("./images/images/tr/tr_%02d.jpg");
    //std::vector<cv::Mat> imagesZR = loadImages("./images/images/zedleft/zedleft_%02d.jpg");
    //std::vector<cv::Mat> imagesZL = loadImages("./images/images/zedright/zedright_%02d.jpg");
    std::cout << "loaded images" << std::endl;

    std::vector<cv::Point2f> cameraLocations2D;

    const cv::Size imgSizeCam(1944, 2592);
    const cv::Size imgSizeZed(2208, 1242);

    cameraLocations2D.push_back(calibrate(imagesBL, imgSizeCam));
    cameraLocations2D.push_back(calibrate(imagesBR, imgSizeCam));
    cameraLocations2D.push_back(calibrate(imagesTL, imgSizeCam));
    cameraLocations2D.push_back(calibrate(imagesTR, imgSizeCam));
    //cameraLocations2D.push_back(calibrate(imagesZR, imgSizeZed));
    //cameraLocations2D.push_back(calibrate(imagesZL, imgSizeZed));

    visualiseCalibration(cameraLocations2D);

    return 0;
}