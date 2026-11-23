#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include <iostream>
#include "eigen3/Eigen/Core"

void pointsTransfer(geometry_msgs::Point &point, const Eigen::Vector3f &pointEigen)
{
    point.x = pointEigen(0);
    point.y = pointEigen(1);
    point.z = pointEigen(2);
}

geometry_msgs::Point expandGeometry(const Eigen::Vector3f &base_point, 
                        const Eigen::Vector3f &corenor_point1,
                        const Eigen::Vector3f &corenor_point2)
{
    const float gains = 0.2;   //内外轮廓的长度
    geometry_msgs::Point outline_point;
    const Eigen::Vector3f unit_vector = base_point + gains * (base_point - corenor_point1).normalized()
                        + gains * (base_point - corenor_point2).normalized();
    // ROS_INFO("unit_vector = [%f]\t[%f]\t[%f]", unit_vector(0), unit_vector(1), unit_vector(2));
    pointsTransfer(outline_point, unit_vector);
    return outline_point;
}


void pointsTransfer1(geometry_msgs::Point &point, const Eigen::Vector3f &point1Eigen)//----------one
{
    point.x = point1Eigen(0);
    point.y = point1Eigen(1);
    point.z = point1Eigen(2);
}

geometry_msgs::Point expandGeometry1(const Eigen::Vector3f &base_point, 
                        const Eigen::Vector3f &corenor_point1,
                        const Eigen::Vector3f &corenor_point2)
{
    const float gains = 0.2;   //内外轮廓的长度
    geometry_msgs::Point outline1_point;
    const Eigen::Vector3f unit_vector = base_point + gains * (base_point - corenor_point1).normalized()
                        + gains * (base_point - corenor_point2).normalized();
    // ROS_INFO("unit_vector = [%f]\t[%f]\t[%f]", unit_vector(0), unit_vector(1), unit_vector(2));
    pointsTransfer1(outline1_point, unit_vector);
    return outline1_point;
}



void pointsTransfer2(geometry_msgs::Point &point, const Eigen::Vector3f &point2Eigen)//----------two
{
    point.x = point2Eigen(0);
    point.y = point2Eigen(1);
    point.z = point2Eigen(2);
}

geometry_msgs::Point expandGeometry2(const Eigen::Vector3f &base_point, 
                        const Eigen::Vector3f &corenor_point1,
                        const Eigen::Vector3f &corenor_point2)
{
    const float gains = 0.2;   //内外轮廓的长度
    geometry_msgs::Point outline2_point;
    const Eigen::Vector3f unit_vector = base_point + gains * (base_point - corenor_point1).normalized()
                        + gains * (base_point - corenor_point2).normalized();
    // ROS_INFO("unit_vector = [%f]\t[%f]\t[%f]", unit_vector(0), unit_vector(1), unit_vector(2));
    pointsTransfer2(outline2_point, unit_vector);
    return outline2_point;
}

void pointsTransfer3(geometry_msgs::Point &point, const Eigen::Vector3f &point3Eigen)//----------three
{
    point.x = point3Eigen(0);
    point.y = point3Eigen(1);
    point.z = point3Eigen(2);
}

geometry_msgs::Point expandGeometry3(const Eigen::Vector3f &base_point, 
                        const Eigen::Vector3f &corenor_point1,
                        const Eigen::Vector3f &corenor_point2)
{
    const float gains = 0.2;   //内外轮廓的长度
    geometry_msgs::Point outline3_point;
    const Eigen::Vector3f unit_vector = base_point + gains * (base_point - corenor_point1).normalized()
                        + gains * (base_point - corenor_point2).normalized();
    // ROS_INFO("unit_vector = [%f]\t[%f]\t[%f]", unit_vector(0), unit_vector(1), unit_vector(2));
    pointsTransfer3(outline3_point, unit_vector);
    return outline3_point;
}

void pointsTransfer4(geometry_msgs::Point &point, const Eigen::Vector3f &point4Eigen)//----------four
{
    point.x = point4Eigen(0);
    point.y = point4Eigen(1);
    point.z = point4Eigen(2);
}

geometry_msgs::Point expandGeometry4(const Eigen::Vector3f &base_point, 
                        const Eigen::Vector3f &corenor_point1,
                        const Eigen::Vector3f &corenor_point2)
{
    const float gains = 0.2;   //内外轮廓的长度
    geometry_msgs::Point outline4_point;
    const Eigen::Vector3f unit_vector = base_point + gains * (base_point - corenor_point1).normalized()
                        + gains * (base_point - corenor_point2).normalized();
    // ROS_INFO("unit_vector = [%f]\t[%f]\t[%f]", unit_vector(0), unit_vector(1), unit_vector(2));
    pointsTransfer4(outline4_point, unit_vector);
    return outline4_point;
}

void pointsTransfer5(geometry_msgs::Point &point, const Eigen::Vector3f &point5Eigen)//----------five
{
    point.x = point5Eigen(0);
    point.y = point5Eigen(1);
    point.z = point5Eigen(2);
}

geometry_msgs::Point expandGeometry5(const Eigen::Vector3f &base_point, 
                        const Eigen::Vector3f &corenor_point1,
                        const Eigen::Vector3f &corenor_point2)
{
    const float gains = 0.2;   //内外轮廓的长度
    geometry_msgs::Point outline5_point;
    const Eigen::Vector3f unit_vector = base_point + gains * (base_point - corenor_point1).normalized()
                        + gains * (base_point - corenor_point2).normalized();
    // ROS_INFO("unit_vector = [%f]\t[%f]\t[%f]", unit_vector(0), unit_vector(1), unit_vector(2));
    pointsTransfer5(outline5_point, unit_vector);
    return outline5_point;
}







int main(int argc, char** argv)
{
    ros::init(argc, argv, "rvizrect_node");
    ros::NodeHandle _nh;
    ros::Publisher rect_pub = _nh.advertise<visualization_msgs::Marker>
            ("/rviz/rectangle_map", 10);

    Eigen::Vector3f point1_leftup, point1_leftdown, point1_rightup, point1_rightdown; //---
    _nh.param<float>("/point_leftup_x",point1_leftup(0) ,1.0f);//---
    _nh.param<float>("/point_leftup_y",point1_leftup(1) ,1.0f);//---
    _nh.param<float>("/point_leftup_z",point1_leftup(2) ,1.0f);//---
    _nh.param<float>("/point_leftdown_x",point1_leftdown(0),1.0f);
    _nh.param<float>("/point_leftdown_y",point1_leftdown(1),1.0f);
    _nh.param<float>("/point_leftdown_z",point1_leftdown(2),1.0f);
    _nh.param<float>("/point_rightup_x",point1_rightup(0),1.0f);
    _nh.param<float>("/point_rightup_y",point1_rightup(1),1.0f);
    _nh.param<float>("/point_rightup_z",point1_rightup(2),1.0f);
    _nh.param<float>("/point_rightdown_x",point1_rightdown(0),1.0f);
    _nh.param<float>("/point_rightdown_y",point1_rightdown(1),1.0f);
    _nh.param<float>("/point_rightdown_z",point1_rightdown(2),1.0f);



    Eigen::Vector3f point2_leftup, point2_leftdown, point2_rightup, point2_rightdown;
    _nh.param<float>("/point1_leftup_x",point2_leftup(0) ,1.0f);//---new---
    _nh.param<float>("/point1_leftup_y",point2_leftup(1) ,1.0f);//---
    _nh.param<float>("/point1_leftup_z",point2_leftup(2) ,1.0f);//---
    _nh.param<float>("/point1_leftdown_x",point2_leftdown(0),1.0f);
    _nh.param<float>("/point1_leftdown_y",point2_leftdown(1),1.0f);
    _nh.param<float>("/point1_leftdown_z",point2_leftdown(2),1.0f);
    _nh.param<float>("/point1_rightup_x",point2_rightup(0),1.0f);
    _nh.param<float>("/point1_rightup_y",point2_rightup(1),1.0f);
    _nh.param<float>("/point1_rightup_z",point2_rightup(2),1.0f);
    _nh.param<float>("/point1_rightdown_x",point2_rightdown(0),1.0f);
    _nh.param<float>("/point1_rightdown_y",point2_rightdown(1),1.0f);
    _nh.param<float>("/point1_rightdown_z",point2_rightdown(2),1.0f);

    Eigen::Vector3f point3_leftup, point3_leftdown, point3_rightup, point3_rightdown;
    _nh.param<float>("/point2_leftup_x",point3_leftup(0) ,1.0f);//---new---
    _nh.param<float>("/point2_leftup_y",point3_leftup(1) ,1.0f);//---
    _nh.param<float>("/point2_leftup_z",point3_leftup(2) ,1.0f);//---
    _nh.param<float>("/point2_leftdown_x",point3_leftdown(0),1.0f);
    _nh.param<float>("/point2_leftdown_y",point3_leftdown(1),1.0f);
    _nh.param<float>("/point2_leftdown_z",point3_leftdown(2),1.0f);
    _nh.param<float>("/point2_rightup_x",point3_rightup(0),1.0f);
    _nh.param<float>("/point2_rightup_y",point3_rightup(1),1.0f);
    _nh.param<float>("/point2_rightup_z",point3_rightup(2),1.0f);
    _nh.param<float>("/point2_rightdown_x",point3_rightdown(0),1.0f);
    _nh.param<float>("/point2_rightdown_y",point3_rightdown(1),1.0f);
    _nh.param<float>("/point2_rightdown_z",point3_rightdown(2),1.0f);

    Eigen::Vector3f point4_leftup, point4_leftdown, point4_rightup, point4_rightdown;
    _nh.param<float>("/point3_leftup_x",point4_leftup(0) ,1.0f);//---new---
    _nh.param<float>("/point3_leftup_y",point4_leftup(1) ,1.0f);//---
    _nh.param<float>("/point3_leftup_z",point4_leftup(2) ,1.0f);//---
    _nh.param<float>("/point3_leftdown_x",point4_leftdown(0),1.0f);
    _nh.param<float>("/point3_leftdown_y",point4_leftdown(1),1.0f);
    _nh.param<float>("/point3_leftdown_z",point4_leftdown(2),1.0f);
    _nh.param<float>("/point3_rightup_x",point4_rightup(0),1.0f);
    _nh.param<float>("/point3_rightup_y",point4_rightup(1),1.0f);
    _nh.param<float>("/point3_rightup_z",point4_rightup(2),1.0f);
    _nh.param<float>("/point3_rightdown_x",point4_rightdown(0),1.0f);
    _nh.param<float>("/point3_rightdown_y",point4_rightdown(1),1.0f);
    _nh.param<float>("/point3_rightdown_z",point4_rightdown(2),1.0f);

    Eigen::Vector3f point5_leftup, point5_leftdown, point5_rightup, point5_rightdown;
    _nh.param<float>("/point4_leftup_x",point5_leftup(0) ,1.0f);//---new---
    _nh.param<float>("/point4_leftup_y",point5_leftup(1) ,1.0f);//---
    _nh.param<float>("/point4_leftup_z",point5_leftup(2) ,1.0f);//---
    _nh.param<float>("/point4_leftdown_x",point5_leftdown(0),1.0f);
    _nh.param<float>("/point4_leftdown_y",point5_leftdown(1),1.0f);
    _nh.param<float>("/point4_leftdown_z",point5_leftdown(2),1.0f);
    _nh.param<float>("/point4_rightup_x",point5_rightup(0),1.0f);
    _nh.param<float>("/point4_rightup_y",point5_rightup(1),1.0f);
    _nh.param<float>("/point4_rightup_z",point5_rightup(2),1.0f);
    _nh.param<float>("/point4_rightdown_x",point5_rightdown(0),1.0f);
    _nh.param<float>("/point4_rightdown_y",point5_rightdown(1),1.0f);
    _nh.param<float>("/point4_rightdown_z",point5_rightdown(2),1.0f);

    Eigen::Vector3f point6_leftup, point6_leftdown, point6_rightup, point6_rightdown;
    _nh.param<float>("/point5_leftup_x",point6_leftup(0) ,1.0f);//---new---
    _nh.param<float>("/point5_leftup_y",point6_leftup(1) ,1.0f);//---
    _nh.param<float>("/point5_leftup_z",point6_leftup(2) ,1.0f);//---
    _nh.param<float>("/point5_leftdown_x",point6_leftdown(0),1.0f);
    _nh.param<float>("/point5_leftdown_y",point6_leftdown(1),1.0f);
    _nh.param<float>("/point5_leftdown_z",point6_leftdown(2),1.0f);
    _nh.param<float>("/point5_rightup_x",point6_rightup(0),1.0f);
    _nh.param<float>("/point5_rightup_y",point6_rightup(1),1.0f);
    _nh.param<float>("/point5_rightup_z",point6_rightup(2),1.0f);
    _nh.param<float>("/point5_rightdown_x",point6_rightdown(0),1.0f);
    _nh.param<float>("/point5_rightdown_y",point6_rightdown(1),1.0f);
    _nh.param<float>("/point5_rightdown_z",point6_rightdown(2),1.0f);





    visualization_msgs::Marker rectangle_msg;
    geometry_msgs::Point rectpoint_msg_leftup;
    geometry_msgs::Point rectpoint_msg_rightup;
    geometry_msgs::Point rectpoint_msg_leftdown;
    geometry_msgs::Point rectpoint_msg_rightdown;

    pointsTransfer(rectpoint_msg_leftup, point1_leftup);//---
    pointsTransfer(rectpoint_msg_rightup, point1_rightup);
    pointsTransfer(rectpoint_msg_leftdown, point1_leftdown);
    pointsTransfer(rectpoint_msg_rightdown, point1_rightdown);

    rectangle_msg.action = visualization_msgs::Marker::ADD;
    rectangle_msg.id = 0;
    rectangle_msg.scale.x = 0.1;
    rectangle_msg.header.frame_id = "world";
    rectangle_msg.type = visualization_msgs::Marker::LINE_STRIP;
    rectangle_msg.color.r = 1.0f;
    rectangle_msg.color.a = 1.0f; 
    rectangle_msg.pose.orientation.w = 1.0f;
    rectangle_msg.points.emplace_back(rectpoint_msg_leftdown);
    rectangle_msg.points.emplace_back(rectpoint_msg_leftup);
    rectangle_msg.points.emplace_back(rectpoint_msg_rightup);
    rectangle_msg.points.emplace_back(rectpoint_msg_rightdown);
    rectangle_msg.points.emplace_back(rectpoint_msg_leftdown);

    rectangle_msg.id = 1;               // 外轮廓
    // rectangle_msg.color.r = 0.0f;
    // rectangle_msg.color.b = 1.0f;
    geometry_msgs::Point rectpoint_msg_leftup_outline;
    geometry_msgs::Point rectpoint_msg_rightup_outline;
    geometry_msgs::Point rectpoint_msg_leftdown_outline;
    geometry_msgs::Point rectpoint_msg_rightdown_outline;
    rectpoint_msg_leftup_outline = expandGeometry(point1_leftup , point1_leftdown, point1_rightup);//---
    rectpoint_msg_rightup_outline = expandGeometry(point1_rightup, point1_rightdown, point1_leftup);//---
    rectpoint_msg_leftdown_outline = expandGeometry(point1_leftdown, point1_leftup, point1_rightdown);//---
    rectpoint_msg_rightdown_outline = expandGeometry(point1_rightdown, point1_rightup, point1_leftdown);
    rectangle_msg.points.emplace_back(rectpoint_msg_leftdown_outline);
    rectangle_msg.points.emplace_back(rectpoint_msg_leftup_outline);
    rectangle_msg.points.emplace_back(rectpoint_msg_rightup_outline);
    rectangle_msg.points.emplace_back(rectpoint_msg_rightdown_outline);
    rectangle_msg.points.emplace_back(rectpoint_msg_leftdown_outline);


    visualization_msgs::Marker rectangle1_msg;//--------------------new
    geometry_msgs::Point rectpoint1_msg_leftup;
    geometry_msgs::Point rectpoint1_msg_rightup;
    geometry_msgs::Point rectpoint1_msg_leftdown;
    geometry_msgs::Point rectpoint1_msg_rightdown;

    pointsTransfer(rectpoint1_msg_leftup, point2_leftup);//---
    pointsTransfer(rectpoint1_msg_rightup, point2_rightup);
    pointsTransfer(rectpoint1_msg_leftdown, point2_leftdown);
    pointsTransfer(rectpoint1_msg_rightdown, point2_rightdown);

    rectangle1_msg.action = visualization_msgs::Marker::ADD;
    rectangle1_msg.id = 2;
    rectangle1_msg.scale.x = 0.1;
    rectangle1_msg.header.frame_id = "world";
    rectangle1_msg.type = visualization_msgs::Marker::LINE_STRIP;
    rectangle1_msg.color.r = 0.0f;
    rectangle1_msg.color.g = 1.0f;
    rectangle1_msg.color.b = 0.5f;
    rectangle1_msg.color.a = 1.0f; 
    rectangle1_msg.pose.orientation.w = 1.0f;
    rectangle1_msg.points.emplace_back(rectpoint1_msg_leftdown);
    rectangle1_msg.points.emplace_back(rectpoint1_msg_leftup);
    rectangle1_msg.points.emplace_back(rectpoint1_msg_rightup);
    rectangle1_msg.points.emplace_back(rectpoint1_msg_rightdown);
    rectangle1_msg.points.emplace_back(rectpoint1_msg_leftdown);

    rectangle1_msg.id = 3;               // 外轮廓
    // rectangle_msg.color.r = 0.0f;
    // rectangle_msg.color.b = 1.0f;
    geometry_msgs::Point rectpoint1_msg_leftup_outline;
    geometry_msgs::Point rectpoint1_msg_rightup_outline;
    geometry_msgs::Point rectpoint1_msg_leftdown_outline;
    geometry_msgs::Point rectpoint1_msg_rightdown_outline;
    rectpoint_msg_leftup_outline = expandGeometry(point2_leftup , point2_leftdown, point2_rightup);//---
    rectpoint_msg_rightup_outline = expandGeometry(point2_rightup, point2_rightdown, point2_leftup);//---
    rectpoint_msg_leftdown_outline = expandGeometry(point2_leftdown, point2_leftup, point2_rightdown);//---
    rectpoint_msg_rightdown_outline = expandGeometry(point2_rightdown, point2_rightup, point2_leftdown);
    rectangle1_msg.points.emplace_back(rectpoint_msg_leftdown_outline);
    rectangle1_msg.points.emplace_back(rectpoint_msg_leftup_outline);
    rectangle1_msg.points.emplace_back(rectpoint_msg_rightup_outline);
    rectangle1_msg.points.emplace_back(rectpoint_msg_rightdown_outline);
    rectangle1_msg.points.emplace_back(rectpoint_msg_leftdown_outline);



    visualization_msgs::Marker rectangle2_msg;//--------------------new
    geometry_msgs::Point rectpoint2_msg_leftup;
    geometry_msgs::Point rectpoint2_msg_rightup;
    geometry_msgs::Point rectpoint2_msg_leftdown;
    geometry_msgs::Point rectpoint2_msg_rightdown;

    pointsTransfer(rectpoint2_msg_leftup, point3_leftup);//---
    pointsTransfer(rectpoint2_msg_rightup, point3_rightup);
    pointsTransfer(rectpoint2_msg_leftdown, point3_leftdown);
    pointsTransfer(rectpoint2_msg_rightdown, point3_rightdown);

    rectangle2_msg.action = visualization_msgs::Marker::ADD;
    rectangle2_msg.id = 4;
    rectangle2_msg.scale.x = 0.1;
    rectangle2_msg.header.frame_id = "world";
    rectangle2_msg.type = visualization_msgs::Marker::LINE_STRIP;
    rectangle2_msg.color.b = 1.0f;
    rectangle2_msg.color.a = 1.0f; 
    rectangle2_msg.pose.orientation.w = 1.0f;
    rectangle2_msg.points.emplace_back(rectpoint2_msg_leftdown);
    rectangle2_msg.points.emplace_back(rectpoint2_msg_leftup);
    rectangle2_msg.points.emplace_back(rectpoint2_msg_rightup);
    rectangle2_msg.points.emplace_back(rectpoint2_msg_rightdown);
    rectangle2_msg.points.emplace_back(rectpoint2_msg_leftdown);

    rectangle2_msg.id = 5;               // 外轮廓
    // rectangle_msg.color.r = 0.0f;
    // rectangle_msg.color.b = 1.0f;
    geometry_msgs::Point rectpoint2_msg_leftup_outline;
    geometry_msgs::Point rectpoint2_msg_rightup_outline;
    geometry_msgs::Point rectpoint2_msg_leftdown_outline;
    geometry_msgs::Point rectpoint2_msg_rightdown_outline;
    rectpoint_msg_leftup_outline = expandGeometry(point3_leftup , point3_leftdown, point3_rightup);//---
    rectpoint_msg_rightup_outline = expandGeometry(point3_rightup, point3_rightdown, point3_leftup);//---
    rectpoint_msg_leftdown_outline = expandGeometry(point3_leftdown, point3_leftup, point3_rightdown);//---
    rectpoint_msg_rightdown_outline = expandGeometry(point3_rightdown, point3_rightup, point3_leftdown);
    rectangle2_msg.points.emplace_back(rectpoint_msg_leftdown_outline);
    rectangle2_msg.points.emplace_back(rectpoint_msg_leftup_outline);
    rectangle2_msg.points.emplace_back(rectpoint_msg_rightup_outline);
    rectangle2_msg.points.emplace_back(rectpoint_msg_rightdown_outline);
    rectangle2_msg.points.emplace_back(rectpoint_msg_leftdown_outline);


    visualization_msgs::Marker rectangle3_msg;//--------------------new
    geometry_msgs::Point rectpoint3_msg_leftup;
    geometry_msgs::Point rectpoint3_msg_rightup;
    geometry_msgs::Point rectpoint3_msg_leftdown;
    geometry_msgs::Point rectpoint3_msg_rightdown;

    pointsTransfer(rectpoint3_msg_leftup, point4_leftup);//---
    pointsTransfer(rectpoint3_msg_rightup, point4_rightup);
    pointsTransfer(rectpoint3_msg_leftdown, point4_leftdown);
    pointsTransfer(rectpoint3_msg_rightdown, point4_rightdown);

    rectangle3_msg.action = visualization_msgs::Marker::ADD;
    rectangle3_msg.id = 6;
    rectangle3_msg.scale.x = 0.1;
    rectangle3_msg.header.frame_id = "world";
    rectangle3_msg.type = visualization_msgs::Marker::LINE_STRIP;
    rectangle3_msg.color.r = 1.0f;
    rectangle3_msg.color.a = 1.0f; 
    rectangle3_msg.pose.orientation.w = 1.0f;
    rectangle3_msg.points.emplace_back(rectpoint3_msg_leftdown);
    rectangle3_msg.points.emplace_back(rectpoint3_msg_leftup);
    rectangle3_msg.points.emplace_back(rectpoint3_msg_rightup);
    rectangle3_msg.points.emplace_back(rectpoint3_msg_rightdown);
    rectangle3_msg.points.emplace_back(rectpoint3_msg_leftdown);

    rectangle3_msg.id = 7;               // 外轮廓
    // rectangle_msg.color.r = 0.0f;
    // rectangle_msg.color.b = 1.0f;
    geometry_msgs::Point rectpoint3_msg_leftup_outline;
    geometry_msgs::Point rectpoint3_msg_rightup_outline;
    geometry_msgs::Point rectpoint3_msg_leftdown_outline;
    geometry_msgs::Point rectpoint3_msg_rightdown_outline;
    rectpoint_msg_leftup_outline = expandGeometry(point4_leftup , point4_leftdown, point4_rightup);//---
    rectpoint_msg_rightup_outline = expandGeometry(point4_rightup, point4_rightdown, point4_leftup);//---
    rectpoint_msg_leftdown_outline = expandGeometry(point4_leftdown, point4_leftup, point4_rightdown);//---
    rectpoint_msg_rightdown_outline = expandGeometry(point4_rightdown, point4_rightup, point4_leftdown);
    rectangle3_msg.points.emplace_back(rectpoint_msg_leftdown_outline);
    rectangle3_msg.points.emplace_back(rectpoint_msg_leftup_outline);
    rectangle3_msg.points.emplace_back(rectpoint_msg_rightup_outline);
    rectangle3_msg.points.emplace_back(rectpoint_msg_rightdown_outline);
    rectangle3_msg.points.emplace_back(rectpoint_msg_leftdown_outline);


    visualization_msgs::Marker rectangle4_msg;//--------------------new
    geometry_msgs::Point rectpoint4_msg_leftup;
    geometry_msgs::Point rectpoint4_msg_rightup;
    geometry_msgs::Point rectpoint4_msg_leftdown;
    geometry_msgs::Point rectpoint4_msg_rightdown;

    pointsTransfer(rectpoint4_msg_leftup, point5_leftup);//---
    pointsTransfer(rectpoint4_msg_rightup, point5_rightup);
    pointsTransfer(rectpoint4_msg_leftdown, point5_leftdown);
    pointsTransfer(rectpoint4_msg_rightdown, point5_rightdown);

    rectangle4_msg.action = visualization_msgs::Marker::ADD;
    rectangle4_msg.id = 8;
    rectangle4_msg.scale.x = 0.1;
    rectangle4_msg.header.frame_id = "world";
    rectangle4_msg.type = visualization_msgs::Marker::LINE_STRIP;
    rectangle4_msg.color.g = 1.0f;
    rectangle4_msg.color.a = 1.0f; 
    rectangle4_msg.pose.orientation.w = 1.0f;
    rectangle4_msg.points.emplace_back(rectpoint4_msg_leftdown);
    rectangle4_msg.points.emplace_back(rectpoint4_msg_leftup);
    rectangle4_msg.points.emplace_back(rectpoint4_msg_rightup);
    rectangle4_msg.points.emplace_back(rectpoint4_msg_rightdown);
    rectangle4_msg.points.emplace_back(rectpoint4_msg_leftdown);

    rectangle4_msg.id = 9;               // 外轮廓
    // rectangle_msg.color.r = 0.0f;
    // rectangle_msg.color.b = 1.0f;
    geometry_msgs::Point rectpoint4_msg_leftup_outline;
    geometry_msgs::Point rectpoint4_msg_rightup_outline;
    geometry_msgs::Point rectpoint4_msg_leftdown_outline;
    geometry_msgs::Point rectpoint4_msg_rightdown_outline;
    rectpoint_msg_leftup_outline = expandGeometry(point5_leftup , point5_leftdown, point5_rightup);//---
    rectpoint_msg_rightup_outline = expandGeometry(point5_rightup, point5_rightdown, point5_leftup);//---
    rectpoint_msg_leftdown_outline = expandGeometry(point5_leftdown, point5_leftup, point5_rightdown);//---
    rectpoint_msg_rightdown_outline = expandGeometry(point5_rightdown, point5_rightup, point5_leftdown);
    rectangle4_msg.points.emplace_back(rectpoint_msg_leftdown_outline);
    rectangle4_msg.points.emplace_back(rectpoint_msg_leftup_outline);
    rectangle4_msg.points.emplace_back(rectpoint_msg_rightup_outline);
    rectangle4_msg.points.emplace_back(rectpoint_msg_rightdown_outline);
    rectangle4_msg.points.emplace_back(rectpoint_msg_leftdown_outline);


    visualization_msgs::Marker rectangle5_msg;//--------------------new
    geometry_msgs::Point rectpoint5_msg_leftup;
    geometry_msgs::Point rectpoint5_msg_rightup;
    geometry_msgs::Point rectpoint5_msg_leftdown;
    geometry_msgs::Point rectpoint5_msg_rightdown;

    pointsTransfer(rectpoint5_msg_leftup, point6_leftup);//---
    pointsTransfer(rectpoint5_msg_rightup, point6_rightup);
    pointsTransfer(rectpoint5_msg_leftdown, point6_leftdown);
    pointsTransfer(rectpoint5_msg_rightdown, point6_rightdown);

    rectangle5_msg.action = visualization_msgs::Marker::ADD;
    rectangle5_msg.id = 10;
    rectangle5_msg.scale.x = 0.1;
    rectangle5_msg.header.frame_id = "world";
    rectangle5_msg.type = visualization_msgs::Marker::LINE_STRIP;
    rectangle5_msg.color.b = 1.0f;
    rectangle5_msg.color.a = 1.0f; 
    rectangle5_msg.pose.orientation.w = 1.0f;
    rectangle5_msg.points.emplace_back(rectpoint5_msg_leftdown);
    rectangle5_msg.points.emplace_back(rectpoint5_msg_leftup);
    rectangle5_msg.points.emplace_back(rectpoint5_msg_rightup);
    rectangle5_msg.points.emplace_back(rectpoint5_msg_rightdown);
    rectangle5_msg.points.emplace_back(rectpoint5_msg_leftdown);

    rectangle5_msg.id = 11;               // 外轮廓
    // rectangle_msg.color.r = 0.0f;
    // rectangle_msg.color.b = 1.0f;
    geometry_msgs::Point rectpoint5_msg_leftup_outline;
    geometry_msgs::Point rectpoint5_msg_rightup_outline;
    geometry_msgs::Point rectpoint5_msg_leftdown_outline;
    geometry_msgs::Point rectpoint5_msg_rightdown_outline;
    rectpoint_msg_leftup_outline = expandGeometry(point6_leftup , point6_leftdown, point6_rightup);//---
    rectpoint_msg_rightup_outline = expandGeometry(point6_rightup, point6_rightdown, point6_leftup);//---
    rectpoint_msg_leftdown_outline = expandGeometry(point6_leftdown, point6_leftup, point6_rightdown);//---
    rectpoint_msg_rightdown_outline = expandGeometry(point6_rightdown, point6_rightup, point6_leftdown);
    rectangle5_msg.points.emplace_back(rectpoint_msg_leftdown_outline);
    rectangle5_msg.points.emplace_back(rectpoint_msg_leftup_outline);
    rectangle5_msg.points.emplace_back(rectpoint_msg_rightup_outline);
    rectangle5_msg.points.emplace_back(rectpoint_msg_rightdown_outline);
    rectangle5_msg.points.emplace_back(rectpoint_msg_leftdown_outline);




    ros::Rate rate(10);
    while(ros::ok()){
        ros::spinOnce();
        rect_pub.publish(rectangle_msg);
        rect_pub.publish(rectangle1_msg);
        rect_pub.publish(rectangle2_msg);
        rect_pub.publish(rectangle3_msg);
        rect_pub.publish(rectangle4_msg);
        rect_pub.publish(rectangle5_msg);
        rate.sleep();
    }
    return 1;
}