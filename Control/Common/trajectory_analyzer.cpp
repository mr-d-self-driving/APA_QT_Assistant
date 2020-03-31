#include "trajectory_analyzer.h"

TrajectoryAnalyzer::TrajectoryAnalyzer()
{
//    track_list = new TrackLinkList();
}

void TrajectoryAnalyzer::Init(TrackLinkList *list)
{
    _trajectory_points_list = list;
}
/**
 * @brief CalculateNearestPoint：计算最近的目标点
 * @param list：目标曲线数据集
 * @param a_track：当前车辆跟踪位置
 * @return 返回离车辆后轴最近的目标曲线点
 */
TargetTrack TrajectoryAnalyzer::CalculateNearestPointByPosition(const double x, const double y)const
{
    Node<TargetTrack>* track_index_node;
    Vector2d temp_track;
    TargetTrack index_node;
    float min_value;

    temp_track.setX(static_cast<float>(x));
    temp_track.setY(static_cast<float>(y));
    index_node.point.setX(0.0);
    index_node.point.setY(0.0);
    index_node.yaw = 0.0f;
    index_node.curvature = 0.0f;
    if(!_trajectory_points_list)
    {
        return index_node;
    }
    if(_trajectory_points_list->Length() < 1)
    {
        return index_node;
    }

    track_index_node = _trajectory_points_list->getHeadNode();
    min_value = (track_index_node->data.point - temp_track).Length();
    index_node = track_index_node->data;
    while(track_index_node->next != NULL)
    {
        if((track_index_node->data.point - temp_track).Length() < min_value)
        {
            min_value = (track_index_node->data.point - temp_track).Length();
            index_node = track_index_node->data;
        }
        track_index_node = track_index_node->next;
    }
    return index_node;
}

double TrajectoryAnalyzer::DistanceToEnd(const double x, const double y)const
{
    return static_cast<double>((_trajectory_points_list->getEndNode()->data.point - Vector2d(static_cast<float>(x),static_cast<float>(y))).Length());
}
