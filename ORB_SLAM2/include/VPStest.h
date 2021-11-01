#pragma once

#include<System.h>
#include<opencv2/core/core.hpp>


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

cv::Mat InputQueryImg(std::string QueryFile);
std::vector<cv::KeyPoint> ORBFeatureExtract(cv::Mat img);
cv::Mat ORBDescriptor(cv::Mat img, std::vector<cv::KeyPoint> keypoints);
std::vector<cv::DMatch> ORBDescriptorMatch(cv::Mat trainDescriptor, cv::Mat queryDescriptor);

