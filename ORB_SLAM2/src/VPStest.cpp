#include "VPStest.h"


cv::Mat InputQueryImg(std::string QueryFile)
{ 
    cv::Mat image;
    cv::VideoCapture video;
    if(!video.open(QueryFile)){
        std::cout << " No query image " << std::endl;
        cv::waitKey();
    }
    video >> image;

    return image;
}

std::vector<cv::KeyPoint> ORBFeatureExtract(cv::Mat img)
{
    cv::Ptr<cv::ORB> orb = cv::ORB::create(4000);
    vector<cv::KeyPoint> keypoints;
    orb->detect(img, keypoints);
    
    return keypoints;
}

cv::Mat ORBDescriptor(cv::Mat img, std::vector<cv::KeyPoint> keypoints)
{
    cv::Ptr<cv::ORB> orb = cv::ORB::create(4000);
    cv::Mat descriptors;
    orb->compute(img, keypoints, descriptors);

    return descriptors;

}

std::vector<cv::DMatch> ORBDescriptorMatch(cv::Mat queryDescriptor, cv::Mat trainDescriptor)
{
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    std::vector<cv::DMatch> matches;               
    matcher->match(queryDescriptor, trainDescriptor, matches);

    return matches;

}

// void CaculatePoseByPnP()
// {

// }