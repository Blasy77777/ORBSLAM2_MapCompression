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
#include<System.h>

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
    SLAM.Shutdown();

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
std::cout << " MP observation count : " << AllMpptr[5]->nNextId << std::endl;
std::cout << " MP observation count : " << AllMpptr[5]->mnFirstKFid << std::endl;
std::cout << " MP observation count : " << AllMpptr[6]->mnId << std::endl;
std::cout << " MP observation count : " << AllMpptr[6]->nNextId << std::endl;
std::cout << " MP observation count : " << AllMpptr[6]->mnFirstKFid << std::endl;


for(auto i : MPIdx)
{
    std::cout << "KF id? " << i.first->mnId << "  MP id? : " << i.second << std::endl;
    bool a = AllMpptr[5]->IsInKeyFrame(i.first);
    std::cout << a << std::endl;
}



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
    cv::waitKey();

    // std::vector<Eigen::Matrix<double,3,1>> CpMp;
    std::vector<cv::Mat> CpMp;
    std::vector<cv::Mat> CpDescriptors;

    for(int i = 0; i < x.size(); i++){
        if(x[i].get(GRB_DoubleAttr_X) == 1){
            cv::Mat pos = AllMpptr[i]->GetWorldPos();
            CpMp.push_back(pos);
            CpDescriptors.push_back(AllMpptr[i]->GetDescriptor());
        }
    }
    // for(int i = 0; i < AllMpptr.size(); i++){
    //         cv::Mat pos = AllMpptr[i]->GetWorldPos();
    //         CpMp.push_back(pos);
    //         CpDescriptors.push_back(AllMpptr[i]->GetDescriptor());
        
    // }
    


    DrawMapCompression(CpMp);
    // DrawCompressionMapPoints(CpMp);
    cv::waitKey();
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

    const int nTimes = vTimestamps.size();
    // const int nTimes = 100;

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
