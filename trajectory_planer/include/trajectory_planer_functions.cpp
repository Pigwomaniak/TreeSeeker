#include <trajectory_planer_functions.h>

#include <algorithm>


#define MODE 0
// 0 for localmode 1 for gps mode

object_global_localizator_msgs::ObjectsGlobalPositions objGlobPos;
bool readFlag;

std::vector<TreeObejctPosition> treePosVec;
bool trajectoryRecalculateFlag;
Point droneOldPos(0,0);
size_t goalPointId;
bool goolFlag;


sensor_msgs::NavSatFix global_position;
nav_msgs::Odometry local_position;


ros::Publisher goal_pos_pub;

trajectory_planer_msgs::TrajectoryPlaner achievePos;
bool readAchievePos;



void new_Point_cb(const object_global_localizator_msgs::ObjectsGlobalPositions::ConstPtr& msg){
    objGlobPos = *msg;
    readFlag = true;
    //ROS_INFO("global pos read");
}

void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    global_position = *msg;
    //ROS_INFO("global pos read");
}

void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg){
    local_position = *msg;
    //ROS_INFO("local pos read");
}

void achieve_point_cb(const trajectory_planer_msgs::TrajectoryPlaner::ConstPtr& msg){
    achievePos = *msg;
    readAchievePos = true;
    //ROS_INFO("local pos read");
}

void resetReadFlag()
{
    readFlag = false;
    readAchievePos = false;
    
}

void resetGoolFlag()
{
    goolFlag = false;
}

bool checkReadFlag()
{
    return readFlag;
}

void processReadPoints()
{
    for (const auto & pointObj : objGlobPos.ObjectsGlobalPositions) 
    {
        unsigned short id = pointObj.idClassObject;
        double distance = pointObj.distanceDroneToObject;

#if MODE == 0
        Point p (pointObj.globalPositionLocal.x, pointObj.globalPositionLocal.y);
#else
        Point p (pointObj.latitude, pointObj.longitude);
#endif

        bool succes = false;
        for(auto& treePos:treePosVec)
        {
            //ROS_INFO("id: %d v id: %d",id,treePos.getId());
            //ROS_INFO("x: %f v x: %f",p.getPos1(),treePos.getPoint().getPos1());
            //ROS_INFO("y: %f v y: %f",p.getPos2(),treePos.getPoint().getPos2());
            if(treePos.addIfInclude(p,id,1.0/distance))
            {
                //ROS_INFO("add id: %d p1: %f p2: %f",id,p.getPos1(),p.getPos2());
                succes = true;
                break;
            }
        }

        if(!succes)
        {

#if MODE == 0
        double radius = 2.0;
#else
        double radius = 0.001;
#endif

            TreeObejctPosition top (id,p,radius,1.0/distance);
            treePosVec.push_back(top);
            trajectoryRecalculateFlag = true;

            Point p1 = top.getPoint();
            ROS_INFO("new id: %d p1: %f p2: %f",id,p1.getPos1(),p1.getPos2());
            ROS_INFO("new id: %d p1: %f p2: %f",id,p.getPos1(),p.getPos2());
        }

    }
}

void findTrajectory()
{

#if MODE == 0
    Point dronePos(local_position.pose.pose.position.x,local_position.pose.pose.position.y);
#else
    Point dronePos(global_position.latitude,global_position.longitude);
#endif

    if(dronePos.countDistance(droneOldPos)>= pow(treePosVec[0].getRadius(),2))
    {
        droneOldPos = dronePos;
        trajectoryRecalculateFlag = true;
    }



    if(trajectoryRecalculateFlag)
    {
        trajectoryRecalculateFlag = false;
        std::vector<Point> points;

        for(const auto& treePos:treePosVec)
        {
            if((treePos.getId()== 1 || treePos.getId()== 2) && !treePos.isVisited())
            {
                points.push_back(treePos.getPoint());
            }
        }

        if (points.size()==0) {
            goolFlag = false;
            return;
        }

        std::vector<size_t> trajectory = findBestTrajectory(points,dronePos);

        Point goalPoint = points[trajectory[0]];
        for(size_t i = 0; i < treePosVec.size(); i++)
        {
            if(goalPoint.countDistance(treePosVec[i].getPoint())<= pow(treePosVec[0].getRadius(),2))
            {
                goalPointId = i;
                break;
            }
        }
        
        goolFlag = true;

    }


}

std::vector<size_t> findBestTrajectory(const std::vector<Point>& points, const Point& dronePos)
{

    std::vector<size_t> result;
    double minCost = 1e300;

    for(size_t i = 0; i < points.size(); i++)
    {
        Point next = points[i];
        std::vector<size_t> v = {i};

        double cost = next.countDistance(dronePos);

        findLoverCost(points,v,cost,minCost,result);
    }

    return result;

}

void findLoverCost (const std::vector<Point>& points, std::vector<size_t>& v, double actualCost, double& minCost, std::vector<size_t>& result)
{
    if(v.size() == points.size())
    {
        if(actualCost<minCost)
        {
            result = v;
            minCost = actualCost;
        }
    }
    else
    {
        for(size_t i = 0; i < points.size(); i++)
        {
            if(std::find(v.begin(), v.end(), i) == v.end())
            {
                Point last = points[*(v.end()-1)];
                Point next = points[i];

                actualCost += next.countDistance(last);
                v.push_back(i);

                findLoverCost(points,v,actualCost,minCost,result);

            } 
        }
    }
}

void init_publisher(ros::NodeHandle controlNode){
    goal_pos_pub = controlNode.advertise<trajectory_planer_msgs::TrajectoryPlaner>("/trajectory_planer/next_waypoint", 1);
}

void sendOutMessage()
{
    if(goolFlag)
    {
        trajectory_planer_msgs::TrajectoryPlaner outMessage;

#if MODE == 0
        outMessage.mode = "local";
#else
        outMessage.mode = "global";
#endif

        Point p = treePosVec[goalPointId].getPoint();
        outMessage.pos1 = p.getPos1();
        outMessage.pos2 = p.getPos2();
        outMessage.idClassObject = treePosVec[goalPointId].getId();
        outMessage.updateCounter = treePosVec[goalPointId].getUpdateCounter();

        //ROS_INFO("Goal p1: %f p2: %f", p.getPos1(), p.getPos2());

        goal_pos_pub.publish(outMessage);
    }
}

void printInfo()
{
    if(goolFlag)
    {
        ROS_INFO("Actual Points");
        for(const auto& treePos:treePosVec)
        {
                Point p = treePos.getPoint();
                ROS_INFO("ID %d Goal p1: %f p2: %f",treePos.getId(), p.getPos1(), p.getPos2());
        }
        ROS_INFO("Goal has index %d",int(goalPointId));
        Point p = treePosVec[goalPointId].getPoint();
        ROS_INFO("Goal p1: %f p2: %f", p.getPos1(), p.getPos2());
    }
}


void setVisitedPoint()
{
    if(readAchievePos)
    {
        readAchievePos=false;
        Point p (achievePos.pos1, achievePos.pos2);

        for(auto& treePos:treePosVec)
        {
            if(p.countDistance(treePos.getPoint())<= pow(treePosVec[0].getRadius(),2))
            {
                treePos.setVisited();
            }
        }
        trajectoryRecalculateFlag = true;
    }
}
