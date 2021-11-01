/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>


#include<opencv2/core/core.hpp>
#include "Converter.h"
#include "VPStest.h"
#include<System.h>
#include <ctime>


using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    // Main loop
    cv::Mat imLeft, imRight;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    // SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    // Parameter
    // MP size , MP id(getobservation), pos(getworldpos)
    // Keyframe size
    // mnvisible
    // mobservations (getmappoints)
    // discriptor(getdescriptor)
std::vector<ORB_SLAM2::KeyFrame*> AllKFptr = SLAM.mpMap->GetAllKeyFrames();
std::vector<ORB_SLAM2::MapPoint*> AllMpptr = SLAM.mpMap->GetAllMapPoints();
map<ORB_SLAM2::KeyFrame*, size_t> MPIdx = AllMpptr[5]->GetObservations();
map<ORB_SLAM2::KeyFrame, size_t>::iterator itr;


std::cout << " MP size : " << SLAM.mpMap->MapPointsInMap() << std::endl;
std::cout << " KF size : " << SLAM.mpMap->KeyFramesInMap() << std::endl;
std::cout << " MP size : " << AllMpptr.size() << std::endl;
std::cout << " KF size : " << AllKFptr.size() << std::endl;
std::cout << " MP observation count : " << AllMpptr[5]->Observations() << std::endl;
std::cout << " MP observation count : " << AllMpptr[6]->Observations() << std::endl;
std::cout << " MP observation count : " << AllMpptr[5]->mnId << std::endl;
std::cout << " MP observation count : " << AllMpptr[5]->mnFirstKFid << std::endl;
std::cout << " MP observation count : " << AllMpptr[6]->mnId << std::endl;
std::cout << " MP observation count : " << AllMpptr[6]->mnFirstKFid << std::endl;
std::cout << " 3dpoint descriptor : " << AllMpptr[0]->GetDescriptor() << std::endl;

// cv::Mat Descriptors;
// Descriptors = AllMpptr[0]->GetDescriptor();
// for(int i = 0; i < 5; i++){
//     cv::Mat descriptor = AllMpptr[i + 1]->GetDescriptor();
//     cv::vconcat(Descriptors, descriptor, Descriptors);
// }
// std::cout << " 3dpoint descriptor : " << Descriptors << std::endl;

// for(auto i : MPIdx)
// {
//     std::cout << "KF id? " << i.first->mnId << "  MP id? : " << i.second << std::endl;
//     bool a = AllMpptr[5]->IsInKeyFrame(i.first);
//     std::cout << a << std::endl;
// }

    // Camera Parameter
    // 00-02
    // float fx(718.856), fy(718.856), cx(607.193), cy(185.216);

    // 04-12
    float fx(707.0912), fy(707.09126), cx(601.8873), cy(183.1104);
    
    // Map Compression
std::cout << "Map Compression ... " << std::endl;
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);



    int PointCloudNum = AllMpptr.size();
    
std::cout << " Create Variables ... " << std:: endl;
    // Create Variables
    std::vector<GRBVar> x = CreateVariablesBinaryVector(PointCloudNum, model);

std::cout << " Set Objective ... " << std:: endl;
    // Set Objective
    Eigen::Matrix<double, Eigen::Dynamic, 1> q = CalculateObservationCountWeight(SLAM.mpMap);
    SetObjectiveILP(x, q, model);

std::cout << " Add Constraint ... " << std:: endl;    
    // Add Constraint
    Eigen::MatrixXd A =CalculateVisibilityMatrix(SLAM.mpMap);
    AddConstraint(SLAM.mpMap, model, A, x);

    std::cout << std::endl;
std::cout << " Optimize model ... " << std:: endl;
    // Optimize model
    model.optimize();

    std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

    std::cout << std::endl;

    // Storage Ordinary 3d point and Descriptor
    std::vector<cv::Mat> Mp;
    cv::Mat Descriptors;
    
    Descriptors = AllMpptr[0]->GetDescriptor();
    Mp.push_back(AllMpptr[0]->GetWorldPos());
    for(int i = 1; i < AllMpptr.size(); i++){
        cv::Mat pos = AllMpptr[i]->GetWorldPos();
        Mp.push_back(pos);
        cv::Mat descriptor = AllMpptr[i]->GetDescriptor();
        cv::vconcat(Descriptors, descriptor, Descriptors);
    }

    std::cout << " Before Cp 3d Point num : " << Mp.size() << std::endl;
    std::cout << " Before Cp Descriptors num : " << Descriptors.size() << std::endl;

    // Storage Cp point and Descriptor
    // std::vector<Eigen::Matrix<double,3,1>> CpMp;
    std::vector<cv::Mat> CpMp;
    cv::Mat CpDescriptors;
    std::vector<int> CpIdx;
    for(int i = 0; i < x.size(); i++){
        if(x[i].get(GRB_DoubleAttr_X) == 1){
            CpIdx.push_back(i);
        }
    }
    CpDescriptors = AllMpptr[CpIdx[0]]->GetDescriptor();
    CpMp.push_back(AllMpptr[CpIdx[0]]->GetWorldPos());

    for(int i = 1; i < CpIdx.size(); i++){
        
            cv::Mat pos = AllMpptr[CpIdx[i]]->GetWorldPos();
            CpMp.push_back(pos);
            cv::Mat descriptor = AllMpptr[i]->GetDescriptor();
            cv::vconcat(CpDescriptors, descriptor, CpDescriptors);
            
    }

    std::cout << " Cp 3d Point num : " << CpMp.size() << std::endl;
    std::cout << " Cp Descriptors num : " << CpDescriptors.size() << std::endl;
            
            
            
            
        


    

    // Visualize CompressionMap


        DrawMapCompression(CpMp);

    cv::waitKey();
    
        // Load Query Img
        std::cout << " Input Query Img " << std::endl;
        std::string QueryPath = "/media/donghoon/1462-5978/kitti/sequences/00/image_0/%06d.png";
        cv::VideoCapture video;
        if(!video.open(QueryPath)){
            std::cout << " No query image " << std::endl;
            return -1;
        }

    ///////// VPS TEST //////////
    while(true)
    {
        cv::Mat QueryImg;
        video >> QueryImg;
        if(QueryImg.empty()) {
            std::cout << " Finish Visual Localization " << std::endl; 
            break;
        }
        if (QueryImg.channels() > 1) cv::cvtColor(QueryImg, QueryImg, cv::COLOR_RGB2GRAY);

        // cv::Mat QueryImg = InputQueryImg(QueryPath);


        // Extract ORB Feature and Destriptor
        std::cout << " Extract ORB Feature and Descriptor " << std::endl;
        std::vector<cv::KeyPoint> QKeypoints;
        cv::Mat QDescriptor;
        QKeypoints = ORBFeatureExtract(QueryImg);
        QDescriptor = ORBDescriptor(QueryImg, QKeypoints);

        // Descriptor Match between Query Descriptor and 3d point DB Descriptor
        std::cout << " Match ORB Descriptor between query and 3d point " << std::endl;
        std::vector<cv::DMatch> matches, Cpmatches;
        
        matches.clear();
        Cpmatches.clear();

        matches = ORBDescriptorMatch(QDescriptor, Descriptors);
        Cpmatches = ORBDescriptorMatch(QDescriptor, CpDescriptors);
        std::sort(matches.begin(), matches.end());
        std::sort(Cpmatches.begin(), Cpmatches.end());
        
        std::vector<cv::DMatch> Goodmatches(matches.begin(), matches.begin() + 50); 
        std::vector<cv::DMatch> GoodCpmatches(Cpmatches.begin(), Cpmatches.begin() + 50);

        for(int i = 0; i < Goodmatches.size(); i ++){
            std::cout << Goodmatches[i].distance << "      "; 
        }
    std::cout << std::endl;
        for(int i = 0; i < GoodCpmatches.size(); i ++){
            std::cout << GoodCpmatches[i].distance << "      "; 
        }
        // Match id for SolvePnP
        std::vector<cv::Point3f> Match3dPoints, CpMatch3dPoints;
        std::vector<cv::Point2f> MatchQ2dPoints, CpMatchQ2dPoints;
        
        Match3dPoints.clear();
        CpMatch3dPoints.clear();
        MatchQ2dPoints.clear();
        CpMatchQ2dPoints.clear();
        
        for(int i = 0; i < 50; i++){
            cv::Point3f Match3dPoint( Mp[matches[i].trainIdx].at<float>(0, 0), 
                            Mp[matches[i].trainIdx].at<float>(1, 0), 
                            Mp[matches[i].trainIdx].at<float>(2, 0));
            cv::Point3f CpMatch3dPoint( CpMp[Cpmatches[i].trainIdx].at<float>(0, 0), 
                            CpMp[Cpmatches[i].trainIdx].at<float>(1, 0), 
                            CpMp[Cpmatches[i].trainIdx].at<float>(2, 0));                        
            Match3dPoints.push_back(Match3dPoint);
            CpMatch3dPoints.push_back(CpMatch3dPoint);
            
            cv::Point2f MatchQ2dPoint(QKeypoints[matches[i].queryIdx].pt);
            cv::Point2f CpMatchQ2dPoint(QKeypoints[Cpmatches[i].queryIdx].pt);
            MatchQ2dPoints.push_back(MatchQ2dPoint);
            CpMatchQ2dPoints.push_back(CpMatchQ2dPoint);

        }

        // PnP Pose Estimation
        
        cv::Mat R, T, RT, inliers;
        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = fx;
        K.at<float>(1,1) = fy;
        K.at<float>(0,2) = cx;
        K.at<float>(1,2) = cy;
        
        cv::solvePnPRansac(Match3dPoints, MatchQ2dPoints, K, cv::noArray(), R, T, false, 100, 3.0F, 0.99, inliers, 0 );
        cv::Rodrigues(R, R);
        cv::hconcat(R, T, RT);
        std::cout << " Before Map Compression Query Img Pose (After PnP): " << std::endl << RT << std:: endl;
        std::cout << " Before Map Compression PnP Inlier Ratio : " << 100 * inliers.rows / Match3dPoints.size() << std::endl;

        cv::Mat R_, T_, RT_, inliers_;
        cv::solvePnPRansac(CpMatch3dPoints, CpMatchQ2dPoints, K, cv::noArray(), R_, T_, false, 100, 3.0F, 0.99, inliers_, 0 );
        cv::Rodrigues(R_, R_);
        cv::hconcat(R_, T_, RT_);
        std::cout << " After Map Compression Query Img Pose (After PnP): " << std::endl << RT_ << std:: endl;
        std::cout << " After Map Compression PnP Inlier Ratio : " << 100 * inliers_.rows / CpMatch3dPoints.size() << std::endl;
    }
    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    // const int nTimes = vTimestamps.size();
    const int nTimes = 1000;

    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
