#pragma once

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Logger/AP_Logger.h>
#include <stdlib.h>
#include <cmath>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>

#define ARM_BOOM_DEGREE_MIN   -18*M_PI/180           //挖掘机大臂最小角度  radians?degrees?
#define ARM_BOOM_DEGREE_MAX   60.6*M_PI/180           //挖掘机大臂最大角度  radians?degrees?
#define ARM_STICK_DEGREE_MIN   -153.12*M_PI/180           //挖掘机斗杆最小角度  radians?degrees?
#define ARM_STICK_DEGREE_MAX   -43.3*M_PI/180           //挖掘机斗杆最大角度  radians?degrees?
#define ARM_BUCKET_DEGREE_MIN   58.35*M_PI/180           //挖掘机铲斗最小角度  radians?degrees?
#define ARM_BUCKET_DEGREE_MAX   -103.5*M_PI/180           //挖掘机铲斗最大角度  radians?degrees?


class AP_ArmLocation_Excavator
{
public:
    AP_ArmLocation_Excavator() :
        _distance_boom_to_slewing(45),
        _distance_stick_to_boom(470),
        _distance_bucket_to_stick(200),
        _distance_tip_to_bucket(148.2),
        _distance_slewing_to_ground(120),
        _angle_boom_up(28*M_PI/180),
        _angle_SRotation_Rod_BRotation(182*M_PI/180),
        _angle_RodBegin_Rod_SRotation(8*M_PI/180),
        _angle_Rod_BRotation_SRotation(4*M_PI/180),
        _angle_BRodLink_BRotation_tip(95*M_PI/180),
        _distance_BRotation_to_Rod(31.4),
        _distance_BRodLink_to_BRodLink(44.1),
        _distance_RodTip_to_BRodLink(55.1),
        _distance_RodTip_to_Rod(53.6)
    {
        _singleton = this;
    }
    
    /* Do not allow copies */
    AP_ArmLocation_Excavator(const AP_ArmLocation_Excavator &other) = delete;
    AP_ArmLocation_Excavator &operator=(const AP_ArmLocation_Excavator&) = delete;

    typedef enum{
        boom_up,
        boom_down,
        stick_up,
        stick_down,
        bucket_up,
        bucket_down
    }Oil_Cylinder;

    //return tip to body position vector
    Vector3f bucket_position_body() const { return _position_tip_to_body; }
    //return tip position in ned to vector position_vec if location exist
    bool bucket_position_ned(Vector3f position_vec);

    //update main function
    void update_position();
/*
Determine whether each cylinder is within the stroke range
*/
    bool stop_arm(AP_ArmLocation_Excavator::Oil_Cylinder oil_cylinder)const;

    static AP_ArmLocation_Excavator *get_singleton(void)
    {
        return _singleton;
    }


private:

    enum class Loc_Status{
        GOOD,
        BAD,
    };
    void update_boom_state();
    void update_stick_state();
    void update_bucket_state();

        //Calculate the three-dimensional coordinates of the tooth tip relative to the body
    void cal_bucket_position_body();

    //Calculate the angle of the boom, stick, bucket relative to the fuselage
    void get_3angle_to_body(Vector3f euler_boom, Vector3f euler_stick, Vector3f euler_bucket, float angle_slewing, 
    float angle_yaw);

    Vector3f  _position_tip_to_body; ///The position coordinates of the tooth tip relative to the car body

    Vector3f  _position_tip_to_ned;//The coordinates of the tooth tip in the global coordinate system

/*
The following specifies some parameters for calculating the position of the excavator working device, 
which are only relevant to the model of the excavator
*/
    float   _distance_boom_to_slewing;///大臂转动原点到回转台中心的距离
    float   _distance_stick_to_boom;///斗杆转动原点到大臂转动原点的距离
    float   _distance_bucket_to_stick;//铲斗转动原点到斗杆转动原点的距离
    float   _distance_tip_to_bucket;///齿尖到铲斗转动原点的距离
    float   _distance_slewing_to_ground;///回转中心到地面的距离
    float   _angle_boom_up;///大臂三角形上小角角度
    float   _angle_SRotation_Rod_BRotation;///斗杆旋转中心、活塞杆固定点以及铲斗旋转中心构成的夹角
    float   _angle_RodBegin_Rod_SRotation;///斗杆活塞杆起始点、活塞杆固定点以及斗杆旋转中心构成的夹角
    float   _angle_Rod_BRotation_SRotation;///活塞杆固定点、铲斗旋转中心以及斗杆旋转中心构成的角度
    float   _angle_BRodLink_BRotation_tip; //铲斗与活塞杆的连接点、铲斗旋转中心以及铲斗齿尖构成角度
    float   _distance_BRotation_to_Rod;///铲斗旋转中心、活塞杆固定点间的距离
    float   _distance_BRodLink_to_BRodLink;///铲斗旋转中心、铲斗与活塞杆的连接点间距离
    float   _distance_RodTip_to_BRodLink;///活塞杆顶点、铲斗与活塞杆的连接点间距离
    float   _distance_RodTip_to_Rod;///活塞杆顶点、活塞杆固定点间的距离

/*
Some intermediate variables that are required for calculations
*/
    float   _angle_boom_to_body;///The angle of the boom relative to the fuselage
    float   _angle_stick_to_body;///The angle of the stick relative to the fuselage
    float   _angle_bucket_to_body;///The angle of the bucket relative to the fuselage
    float   _angle_slewing_to_body;///The angle of the swing relative to the fuselage

    float  _angle_boom_to_slewing;
    float  _angle_stick_to_boom;
    float  _angle_bucket_to_stick;

    //Converted into sensor data in the body's coordinate system
    Vector3f _euler_boom_body_from_sensor;
    Vector3f _euler_stick_body_from_sensor;
    Vector3f _euler_bucket_body_from_sensor;

    Vector3f _body_position;

    //Structures related to overrun and location data
    struct State
    {
        bool boom_out_range_up;
        bool boom_out_range_down;
        bool stick_out_range_up;
        bool stick_out_range_down;
        bool bucket_out_range_up;
        bool bucket_out_range_down;
        enum Loc_Status body_loc_status;
    }state;

    static AP_ArmLocation_Excavator *_singleton;  

};

namespace AP
{
AP_ArmLocation_Excavator *excavator_arm();
};