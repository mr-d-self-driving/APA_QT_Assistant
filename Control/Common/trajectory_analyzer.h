#ifndef TRAJECTORYANALYZER_H
#define TRAJECTORYANALYZER_H

#include <QMainWindow>

#include "math.h"
#include "Common/Math/vector_2d.h"
#include "Common/Utils/Inc/link_list.h"
#include "Common/VehicleState/GeometricTrack/geometric_track.h"
#include "Common/Configure/Configs/vehilce_config.h"

typedef struct _TargetTrack
{
    Vector2d point;
    float    yaw;
    float    curvature;
    float    velocity;
}TargetTrack;

/**
 * @brief The TrackLinkList class：曲线跟踪目标曲线链表
 */
class TrackLinkList : public LinkList<TargetTrack>{};

class TrajectoryAnalyzer
{
public:
    TrajectoryAnalyzer();

    void Init(TrackLinkList *list);

    /**
     * @brief CalculateNearestPoint：计算最近的目标点
     * @param list：目标曲线数据集
     * @param a_track：当前车辆跟踪位置
     * @return 返回离车辆后轴最近的目标曲线点
     */
    TargetTrack CalculateNearestPointByPosition(const double x, const double y)const;

    double DistanceToEnd(const double x, const double y)const;
private:
    Node<TargetTrack>* _track_node;
    TrackLinkList    * _track_list;
    TrackLinkList    * _trajectory_points_list;
};

#endif // TRAJECTORYANALYZER_H
