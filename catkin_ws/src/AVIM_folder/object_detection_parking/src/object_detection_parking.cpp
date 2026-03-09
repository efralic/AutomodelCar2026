#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>
#include <cmath>
#include <vector>
#include <map>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include "object_detection/points_objects.h"
#include <math.h>
#include <limits>

#define UNCLASSIFIED -1
#define CORE_POINT    1
#define BORDER_POINT  2
#define NOISE        -2
#define SUCCESS       0
#define FAILURE      -3

typedef std::vector<cv::Point2f> vector_point;
typedef std::vector<vector_point> obstacles;
typedef std::vector<cv::Point> Cluster;

// Rango reducido para estacionamiento (44 cm max)
float RANGE      = 0.44f;
float RANGE_MIN  = 0.18f;
float INITIAL_RANGE_ANGLE = 0.0f;
float END_RANGE_ANGLE     = 360.0f;

class PointC {
public:
    float x, y;
    int   clusterID;
    PointC(float x, float y): x(x), y(y), clusterID(UNCLASSIFIED){}
};

class DBSCAN {
public:
    std::vector<PointC> m_points;
    DBSCAN(unsigned int minPts, float eps, const std::vector<cv::Point> &points_){
        m_minPoints = minPts;
        m_epsilon   = eps;
        for(auto p : points_)
            m_points.push_back({(float)p.x,(float)p.y});
        m_pointSize = m_points.size();
    }
    ~DBSCAN(){}
    int run();
    std::vector<int> calculateCluster(PointC point);
    int expandCluster(PointC point, int clusterID);
    inline double calculateDistance(PointC a, PointC b);
    void getCluster(std::map<int,std::vector<cv::Point>> &out);
private:
    unsigned int m_pointSize, m_minPoints;
    float m_epsilon;
};

int DBSCAN::run(){
    int clusterID=0;
    for(auto iter=m_points.begin();iter!=m_points.end();++iter)
        if(iter->clusterID==UNCLASSIFIED)
            if(expandCluster(*iter,clusterID)!=FAILURE)
                clusterID++;
    return 0;
}

int DBSCAN::expandCluster(PointC point, int clusterID){
    std::vector<int> seeds=calculateCluster(point);
    if(seeds.size()<m_minPoints){point.clusterID=NOISE;return FAILURE;}
    int indexCorePoint=0,index=0;
    for(auto it=seeds.begin();it!=seeds.end();++it){
        m_points.at(*it).clusterID=clusterID;
        if(m_points.at(*it).x==point.x&&m_points.at(*it).y==point.y)
            indexCorePoint=index;
        index++;
    }
    seeds.erase(seeds.begin()+indexCorePoint);
    for(size_t i=0,n=seeds.size();i<n;++i){
        std::vector<int> nb=calculateCluster(m_points.at(seeds[i]));
        if(nb.size()>=m_minPoints)
            for(auto ni:nb)
                if(m_points.at(ni).clusterID==UNCLASSIFIED||
                   m_points.at(ni).clusterID==NOISE){
                    if(m_points.at(ni).clusterID==UNCLASSIFIED){seeds.push_back(ni);n=seeds.size();}
                    m_points.at(ni).clusterID=clusterID;
                }
    }
    return SUCCESS;
}

std::vector<int> DBSCAN::calculateCluster(PointC point){
    int index=0;
    std::vector<int> clusterIndex;
    for(auto iter=m_points.begin();iter!=m_points.end();++iter){
        if(calculateDistance(point,*iter)<=m_epsilon) clusterIndex.push_back(index);
        index++;
    }
    return clusterIndex;
}

inline double DBSCAN::calculateDistance(PointC a,PointC b){
    return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}

void DBSCAN::getCluster(std::map<int,std::vector<cv::Point>> &cp){
    for(auto p:m_points) cp[p.clusterID].push_back(cv::Point((int)p.x,(int)p.y));
}

class ObjectDetection {
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    std::vector<cv::Point> points_;
    int MINIMUM_POINTS, EPSILON;
    std::map<int,std::vector<cv::Point>> clusters_points;
    std::vector<cv::Point> points_centroids;
    cv::Mat image;
    ros::Publisher pub;
public:
    ObjectDetection(): nh(){
        MINIMUM_POINTS = 1;
        EPSILON        = 10;
        pub      = nh.advertise<object_detection::points_objects>("objects_points",1);
        scan_sub = nh.subscribe("/scan",1,&ObjectDetection::laser_msg_Callback,this);
        ROS_INFO("ObjectDetection PARKING ready. Listening to /scan");
    }

    void laser_msg_Callback(const sensor_msgs::LaserScan::ConstPtr& scan){
        vector_point scan_points;
        object_detection::points_objects points_msg;
        points_msg.another_field = 0;

        int num_ranges = (int)scan->ranges.size();
        if(num_ranges == 0) return;

        float angle_min_rad = scan->angle_min;
        float angle_inc_rad = scan->angle_increment;

        // ── FIX: índice entero correcto (igual que object_detection.cpp) ─
        for(int idx = 0; idx < num_ranges; idx++){
            float angle_rad = angle_min_rad + idx * angle_inc_rad;
            while(angle_rad <  0.0f)      angle_rad += 2.0f*M_PI;
            while(angle_rad >= 2.0f*M_PI) angle_rad -= 2.0f*M_PI;
            float angle_deg = angle_rad * 180.0f / M_PI;

            bool in_roi;
            if(INITIAL_RANGE_ANGLE < END_RANGE_ANGLE)
                in_roi=(angle_deg>=INITIAL_RANGE_ANGLE&&angle_deg<END_RANGE_ANGLE);
            else
                in_roi=(angle_deg>=INITIAL_RANGE_ANGLE||angle_deg<END_RANGE_ANGLE);
            if(!in_roi) continue;

            float r = scan->ranges[idx];
            if(!std::isfinite(r)) continue;
            if(r < RANGE_MIN || r > RANGE) continue;

            scan_points.push_back(cv::Point2f(angle_deg, r));
        }

        get_object_points(scan_points);

        if(points_centroids.size() > 0){
            points_msg.another_field = points_centroids.size();
            for(auto point : points_centroids){
                geometry_msgs::Point point_msg;
                transform_point(point, point_msg);
                point_msg.z = 0;
                points_msg.points.push_back(point_msg);
                ROS_INFO("Parking obstacle: dist=%.2f cm, angle=%.1f deg",
                         point_msg.x, point_msg.y);
            }
            pub.publish(points_msg);
        }
    }

    void get_object_points(const vector_point& scan_points){
        points_.clear(); clusters_points.clear();
        for(auto point : scan_points){
            float r = point.y * 100.0f;
            float x = r*cos((180.0f-point.x)*M_PI/180.0f);
            float y = r*sin((180.0f-point.x)*M_PI/180.0f);
            x=400.0f+x; x=(x<800)?x:800; x=(x>=0)?x:0;
            y=400.0f+y; y=(y<800)?y:800; y=(y>=0)?y:0;
            points_.push_back(cv::Point((int)x,(int)y));
        }
        DBSCAN dbScan(MINIMUM_POINTS,EPSILON,points_);
        dbScan.run();
        dbScan.getCluster(clusters_points);
        get_centroids_objects(clusters_points);
    }

    void transform_point(cv::Point &point, geometry_msgs::Point &point_msg){
        float adj=point.y-400, opp=point.x-400;
        adj=(adj==0.0f)?0.000001f:adj;
        if(point.x>=0&&point.x<=400&&point.y>=0&&point.y<=400)
            point_msg.y=atan(opp/adj)*57.295779f;
        else if(point.x>=0&&point.x<=400&&point.y>=400&&point.y<=800)
            point_msg.y=180.0f+atan(opp/adj)*57.295779f;
        else if(point.x>=400&&point.x<=800&&point.y>=400&&point.y<=800)
            point_msg.y=180.0f+atan(opp/adj)*57.295779f;
        else if(point.x>=400&&point.x<=800&&point.y>=0&&point.y<=400)
            point_msg.y=360.0f+atan(opp/adj)*57.295779f;
        if(point_msg.y<0) point_msg.y=-point_msg.y;
        point_msg.x=cv::norm(point-cv::Point(400,400));
    }

    void get_centroids_objects(const std::map<int,std::vector<cv::Point>> &cp){
        points_centroids.clear();
        for(auto cluster:cp){
            if(cluster.first==NOISE) continue;
            int x=0,y=0,n=(int)cluster.second.size();
            if(n==0) continue;
            for(auto p:cluster.second){x+=p.x;y+=p.y;}
            points_centroids.push_back(cv::Point(x/n,y/n));
        }
    }
};

int main(int argc,char **argv){
    ros::init(argc,argv,"ObjectDetectionParking");
    ROS_INFO("Object detection PARKING node running...");
    ObjectDetection *od=new ObjectDetection;
    ros::spin();
    return 0;
}
