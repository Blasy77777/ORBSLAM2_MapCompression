#include "gurobi_helper.h"



std::vector<GRBVar> CreateVariablesBinaryVector(int PointCloudNum, GRBModel& model_)
{
    
    std::vector<GRBVar> x;
    x.resize(PointCloudNum);
    for(int i = 0; i < PointCloudNum; i++ )
    {
        x[i] = model_.addVar(-5.0, 3.0, 0.0, GRB_BINARY);
    }

    return x;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> CalculateObservationCountWeight(ORB_SLAM2::Map* map_data)
{
    Eigen::Matrix<double, Eigen::Dynamic, 1> q;
    int PointCloudNum_ = map_data->MapPointsInMap();
    q.resize(PointCloudNum_);
    int KeyframeNum = map_data->KeyFramesInMap();
    std::vector<ORB_SLAM2::MapPoint*> AllMpptr = map_data->GetAllMapPoints();
    
    for(int i = 0; i < PointCloudNum_; i++)
    {
        q[i] = (double)AllMpptr[i]->GetObservations().size() / (double)KeyframeNum;
    }
    return q;
}

void SetObjectiveILP(std::vector<GRBVar> x_, Eigen::Matrix<double, Eigen::Dynamic, 1> q_, GRBModel& model_)
{
    GRBLinExpr obj = 0;
    for(int i = 0; i < x_.size(); i++)
    {
        obj += q_[i] * x_[i];
    } 
    model_.setObjective(obj);
    
}

Eigen::MatrixXd CalculateVisibilityMatrix(ORB_SLAM2::Map* map_data)
{

    std::vector<ORB_SLAM2::MapPoint*> AllMpptr = map_data->GetAllMapPoints();
    std::vector<ORB_SLAM2::KeyFrame*> AllKFptr = map_data->GetAllKeyFrames();
    Eigen::MatrixXd A(map_data->KeyFramesInMap(), map_data->MapPointsInMap()); 
    A.setZero();
    for(int i = 0; i < A.rows(); i++ )
        {
            for(int j = 0; j < A.cols(); j++)
            {
                
                bool IsInKF = AllMpptr[j]->IsInKeyFrame(AllKFptr[i]);
                if(IsInKF){
                    A(i, j) = 1.0;
                }    
            }

        }
    return A;
}

void AddConstraint(ORB_SLAM2::Map* map_data, GRBModel& model_, Eigen::MatrixXd A, std::vector<GRBVar> x)
{
    GRBLinExpr constraint = 0;
    double b = 30.0;
    for(int i = 0; i < map_data->KeyFramesInMap(); i++)
    {
       constraint.clear();
       for(int j = 0; j < map_data->MapPointsInMap(); j++)
       {
        
        constraint += A(i, j) * x[j];

       }
       model_.addConstr(constraint >= b);
    }
}
