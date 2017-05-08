#include <iostream>
#include <zed/Camera.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#define DEPTH_CLAMP_VALUE 10000

sl::zed::Camera* getCamera()
{
        // Initialisation des parametres
        sl::zed::InitParams parameters;
        parameters.mode = sl::zed::MODE::QUALITY;        // NONE, PERFORMANCE, MEDIUM ou QUALITY
        parameters.unit = sl::zed::UNIT::MILLIMETER;
        parameters.verbose = 1;
        parameters.device = 0;
        
        
        // Initialisation de la camera
        sl::zed::Camera* zedCamera = new sl::zed::Camera(sl::zed::ZEDResolution_mode::VGA);        // VGA, HD720, HD1080 ou HD2K
        sl::zed::ERRCODE err = zedCamera->init(parameters);
        
        if (err != sl::zed::ERRCODE::SUCCESS)
        {
                // on quitte le programmme en cas d'erreur
                std::cout << errcode2str(err) << std::endl;
                delete zedCamera;
                exit(1);
        }
        
        zedCamera->setDepthClampValue(DEPTH_CLAMP_VALUE);
        return zedCamera;
}

std::vector<cv::Point> getZonePoints(const cv::Mat& pixels, cv::Mat& pointsAreComputed, const cv::Point& initialPoint)
{
        std::vector<cv::Point> pointsToCompute = {initialPoint};
        std::vector<cv::Point> zonePoints;
        
        while (pointsToCompute.size() > 0)
        {
                zonePoints.push_back(pointsToCompute[0]);
                
                if (pointsToCompute[0].x <= 0 || pointsToCompute[0].y <= 0 || pointsToCompute[0].x >= pixels.size().width - 1 || pointsToCompute[0].y >= pixels.size().height - 1)
                {
                        pointsToCompute.erase(pointsToCompute.begin());
                        continue;
                }

                for (int i = -1; i < 2; ++i)
                {
                        for (int j = -1; j < 2; ++j)
                        {
                                cv::Point position(pointsToCompute[0].x + i, pointsToCompute[0].y + j);
                                if (pixels.at<uchar>(position) > 0 && pointsAreComputed.at<uchar>(position) == 0)
                                {
                                        pointsToCompute.push_back(position);
                                        pointsAreComputed.at<uchar>(position) = 255;
                                }
                        }
                }

                pointsToCompute.erase(pointsToCompute.begin());
        }

        return zonePoints;
}

int main(int argc, char **argv)
{
        srand(time(0));

        // Initialisation de l'affichage
        sl::zed::Camera* zedCamera = getCamera();
        int width = zedCamera->getImageSize().width;
        int height = zedCamera->getImageSize().height;
        cv::Size size(width, height); // taille de l'image

        cv::Mat depthImage(size, CV_8UC1); // image de profondeur
        cv::Mat oldDepthImage(size, CV_8UC1); // image de profondeur
        cv::Mat pixelsInMovement(size, CV_8UC1); // pixels en mouvement en blanc
        cv::Mat zonesImage(size, CV_8UC3); // zones en mouvement
        cv::Mat colorImage(size, CV_8UC4); // image avec le mouvement en couleur
        
        
        // Boucle d'affichage
        bool programShouldBeClosed = false;
        int count = 0;
        clock_t beginTime = clock();
        clock_t totalTime = 0;
        float fps = 0;
        int imageCountToSave = 0;
        const int totalImageCountToSave = 100;
        int limit = 80;

        ros::init(argc, argv, "cam");
        ros::NodeHandle n;
        ros::Publisher pub = n.advertise<std_msgs::String>("claques", 10);
        
        while (!programShouldBeClosed && ros::ok())
        {
                zedCamera->grab(sl::zed::SENSING_MODE::FILL);        // STANDARD ou FILL
                
                depthImage.copyTo(oldDepthImage);
                slMat2cvMat(zedCamera->normalizeMeasure(sl::zed::MEASURE::DEPTH)).copyTo(depthImage);        // DEPTH, CONFIDENCE ou DISPARITY
                slMat2cvMat(zedCamera->retrieveImage(sl::zed::SIDE::LEFT)).copyTo(colorImage);
                
                // Calcul de la difference de profondeur entre l'image actuelle et l'ancienne
                for (int x = 0; x < width && count > 0; ++x)
                {
                        for (int y = 0; y < height; ++y)
                        {
                                int difference = (int)depthImage.at<uchar>(cv::Point(4 * x, y)) - (int)oldDepthImage.at<uchar>(cv::Point(4 * x, y));

                                if (difference > limit)        // on enleve le bruit par une limite
                                        pixelsInMovement.at<uchar>(cv::Point(x, y)) = 255;
                                else
                                        pixelsInMovement.at<uchar>(cv::Point(x, y)) = 0;
                        }
                }

                int morphSize = 50;
                cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morphSize + 1, 2 * morphSize + 1), cv::Point(morphSize, morphSize));
                morphologyEx(pixelsInMovement, pixelsInMovement, cv::MORPH_CLOSE, element);

                

                int zoneCount = 0;
                cv::Mat pointsAreComputed(zonesImage.size(), CV_8UC1);

                for (int x = 0; x < width && count > 0; ++x)
                {
                        for (int y = 0; y < height; ++y)
                        {
                                cv::Point point(x, y);
                                zonesImage.at<cv::Vec3b>(point) = cv::Vec3b(0, 0, 0);
                                pointsAreComputed.at<uchar>(point) = 0;
                        }
                }

                // Recuperation des zones de mouvement
                int zoneId = 0;
                int globalMinDistance = INT_MAX;

                for (int x = 0; x < width && count > 0; ++x)
                {
                        for (int y = 0; y < height; ++y)
                        {
                                if (pixelsInMovement.at<uchar>(cv::Point(x, y)) > 0 && pointsAreComputed.at<uchar>(cv::Point(x, y)) == 0)
                                {
                                        std::vector<cv::Point> zonePoints = getZonePoints(pixelsInMovement, pointsAreComputed, cv::Point(x, y));
                                        cv::Vec3b color(rand() % 255 + 1, rand() % 255 + 1, rand() % 255 + 1);

                                        int minDistance = INT_MAX;

                                        for (const cv::Point& point : zonePoints)
                                        {
                                                zonesImage.at<cv::Vec3b>(point) = color;

                                                int difference = (int)depthImage.at<uchar>(cv::Point(4 * x, y)) - (int)oldDepthImage.at<uchar>(cv::Point(4 * x, y));                                                
                                                if (difference > limit)
                                                {
                                                        minDistance = std::min(minDistance, 255 - (int)depthImage.at<uchar>(cv::Point(4 * x, y)));
                                                }
                                        }

                                        cv::putText(zonesImage, std::to_string(minDistance), cv::Point(0, zoneId * 50 + 50), cv::FONT_HERSHEY_SIMPLEX, 2, color, 3);
                                        globalMinDistance = std::min(globalMinDistance, minDistance);
                                        ++zoneId;
                                }
                        }
                }

                // Mise en valeur du mouvement sur l'image couleur
                for (int x = 0; x < width && count > 0; ++x)
                {
                        for (int y = 0; y < height; ++y)
                        {
                                if (zonesImage.at<cv::Vec3b>(cv::Point(x, y))[0] == 0)
                                        colorImage.at<cv::Vec4b>(cv::Point(x, y)) = cv::Vec4b(0, 0, 0, 255);
                        }
                }

                if (globalMinDistance < 256)
                {
                        std::cout << "=> " << globalMinDistance / 255. * DEPTH_CLAMP_VALUE / 1000 << std::endl;
                        std_msgs::String msg;
                        std::stringstream ss;
                        ss << globalMinDistance / 255. * DEPTH_CLAMP_VALUE / 1000;
                        msg.data = ss.str();

                        pub.publish(msg);
                }

                // Affichage des images
                //cv::imshow("Couleur", colorImage);
                cv::imshow("Profondeur", depthImage);
                //cv::imshow("Resultat 3", zonesImage);



                // Récupération des inputs
                char key = cv::waitKey(5);
                if (key == 'q')
                        programShouldBeClosed = true;
                else if (key == 'z')
                        limit += 1;
                else if (key == 's')
                        limit -= 1;
                else if (key == 'p' && imageCountToSave == 0)
                        imageCountToSave = totalImageCountToSave;

                if (imageCountToSave > 0)
                {
                        cv::imwrite("images/depth" + std::to_string(totalImageCountToSave - imageCountToSave) + ".jpg", depthImage);
                        cv::imwrite("images/old_depth" + std::to_string(totalImageCountToSave - imageCountToSave) + ".jpg", oldDepthImage);
                        cv::imwrite("images/color" + std::to_string(totalImageCountToSave - imageCountToSave) + ".jpg", colorImage);
                        --imageCountToSave;

                        if (imageCountToSave == 0)
                                std::cout << "Image saved in images/zones*.jpg" << std::endl;
                        else
                                std::cout << "Left : " + std::to_string(imageCountToSave) << std::endl;
                }

                //std::cout << limit << std::endl;
                
                // Calcul des FPS
                count++;
                totalTime += clock() - beginTime;
                
                if (count % 100 == 0)
                {
                        std::cout << 10000 * CLOCKS_PER_SEC / totalTime  << std::endl;
                        beginTime = clock();
                        totalTime = 0;
                }
        }
        
        delete zedCamera;
        return 0;
}

