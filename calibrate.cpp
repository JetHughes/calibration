#include <opencv2/opencv.hpp>
#include <stdio.h>

const int NUM_IMAGES = 30;
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
            //image = bwImage;
            //drawCorners(image, fname);   
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


double calibrate(std::vector<cv::Mat> &images)
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


void calibrateTwo(std::vector<cv::Mat>& images_A, std::vector<cv::Mat>& images_B) {
    std::cout << "start stero cailbration" << std::endl;

    cv::Mat K_A;
    cv::Mat K_B;

    std::vector<double> distCoeffs_A;
    std::vector<double> distCoeffs_B;

    std::vector<std::vector<cv::Point2f>> imagePoints_A;
    std::vector<std::vector<cv::Point2f>> imagePoints_B;

    std::vector<std::vector<cv::Point3f>> objectPoints;

    std::vector<cv::Point3f> checkerboardPattern;
    fillCheckerboard(checkerboardPattern);

    std::vector<cv::Mat> rvecs_A;
    std::vector<cv::Mat> tvecs_A;

    std::vector<cv::Mat> rvecs_B;
    std::vector<cv::Mat> tvecs_B;

    std::cout << "find image and objects points from A: " << images_A.size() << " images";
    for (const auto& image : images_A)
    {
        std::vector<cv::Point2f> corners;
        cv::findChessboardCorners(image, PATTERN_SIZE, corners);
        printf(".");
        imagePoints_A.push_back(corners);
        objectPoints.push_back(checkerboardPattern);
    }

    std::cout << "\nfind image and objects points from B: " << images_B.size() << " images";
    for (const auto& image : images_B)
    {
        std::vector<cv::Point2f> corners;
        cv::findChessboardCorners(image, PATTERN_SIZE, corners);
        // cv::cornerSubPix(image, corners, IMAGE_SIZE, );
        printf(".");
        imagePoints_B.push_back(corners);
        //objectPoints.push_back(checkerboardPattern);
    }


    std::cout << "\ncalibrate A" << std::endl;
    double err1 = cv::calibrateCamera(objectPoints, imagePoints_A, IMAGE_SIZE, K_A, distCoeffs_A, rvecs_A, tvecs_A);

    std::cout << "calibrate B" << std::endl;
    double err2 = cv::calibrateCamera(objectPoints, imagePoints_B, IMAGE_SIZE, K_B, distCoeffs_B, rvecs_B, tvecs_B);

    cv::Mat R, T, E, F;

    std::cout << "calibrate Stereo" << std::endl;
    double err = cv::stereoCalibrate(objectPoints, imagePoints_A, imagePoints_B, K_A, distCoeffs_A, K_B, distCoeffs_B, IMAGE_SIZE, R, T, E, F);

    std::cout << "Results:" <<std::endl;
    std::cout << "\nR" << R << std::endl;
    std::cout << "\nT" << T << std::endl;
    std::cout << "\nE" << E << std::endl;
    std::cout << "\nF" << F << std::endl;
    std::cout << "\nError: " << err <<std::endl;
}


void calibrateMany(std::vector<std::vector<cv::Mat>> cameras) 
{
    std::vector<cv::Mat> K(cameras.size());
    std::vector<std::vector<double>> distCoeffs(cameras.size());
    std::vector<std::vector<std::vector<cv::Point2f>>> imagePoints(cameras.size());
    std::vector<std::vector<cv::Mat>> tvecs(cameras.size());
    std::vector<std::vector<cv::Mat>> rvecs(cameras.size());

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<cv::Point3f> checkerboardPattern;
    fillCheckerboard(checkerboardPattern);

    std::cout << "\ncalibrating with " << cameras[0].size() << " images" << std::endl;
    for (size_t i = 0; i < cameras.size(); ++i) {
        std::cout << "\nfind image points for camera " << i;
        for (const auto& image : cameras[i])
        {
            std::vector<cv::Point2f> corners;
            cv::findChessboardCorners(image, PATTERN_SIZE, corners);
            printf(".");
            imagePoints[i].push_back(corners);

            // Get object points only from the first camera
            if (i == 0) {
                objectPoints.push_back(checkerboardPattern);
            }
        }
        std::cout << "calibrate camera " << i << std::endl;
        double err1 = cv::calibrateCamera(objectPoints, imagePoints[i], IMAGE_SIZE, K[i], distCoeffs[i], rvecs[i], tvecs[i]);

        if (i != 0) {

            cv::Mat R, T, E, F;
            std::cout << "Stereo calibrate 0 with " << i << std::endl;
            double err = cv::stereoCalibrate(objectPoints, imagePoints[0], imagePoints[i], K[0], distCoeffs[0], K[i], distCoeffs[i], IMAGE_SIZE, R, T, E, F);

            std::cout << "Results:" << std::endl;
            std::cout << "R" << R << std::endl;
            std::cout << "T" << T << std::endl;
            std::cout << "E" << E << std::endl;
            std::cout << "F" << F << std::endl;
            std::cout << "Error: " << err << std::endl;
        }
    }



}


int main(int argc, char *argv[])
{
    std::vector<cv::Mat> imagesBL = loadImages("C:/Users/HCIadmin/jet/Calibration/calibration-images/bl/bl_%d.jpg");
    std::vector<cv::Mat> imagesBR = loadImages("C:/Users/HCIadmin/jet/Calibration/calibration-images/br/br_%d.jpg");
    std::vector<cv::Mat> imagesTL = loadImages("C:/Users/HCIadmin/jet/Calibration/calibration-images/tl/tl_%d.jpg");
    std::vector<cv::Mat> imagesTR = loadImages("C:/Users/HCIadmin/jet/Calibration/calibration-images/tr/tr_%d.jpg");
    std::cout << "loaded images" << std::endl;

    std::vector<std::vector<cv::Mat>> allImages = { imagesBL, imagesBR, imagesTL, imagesTR };

    //calibrate(imagesBL);
    //calibrateTwo(imagesBL, imagesTL);
    //calibrateTwo(imagesBL, imagesTR);

    calibrateMany(allImages);

    return 0;
}