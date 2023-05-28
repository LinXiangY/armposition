#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include <cmath>

#include "AP_ArmPosition.h"
#include <cmath>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Inclination/AP_Inclination.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

bool AP_ArmLocation_Excavator::stop_arm(AP_ArmLocation_Excavator::Oil_Cylinder oil_cylinder)const{
    switch(oil_cylinder){
    case Oil_Cylinder::boom_up: return state.boom_out_range_up;
    case Oil_Cylinder::boom_down: return state.boom_out_range_down;
    case Oil_Cylinder::stick_up: return state.stick_out_range_up;
    case Oil_Cylinder::stick_down: return state.stick_out_range_down;
    case Oil_Cylinder::bucket_up: return state.bucket_out_range_up;
    case Oil_Cylinder::bucket_down: return state.bucket_out_range_down;
    default: return true;
    }
    return true;
}

bool AP_ArmLocation_Excavator::bucket_position_ned(Vector3f position_vec){
    if(state.body_loc_status == Loc_Status::GOOD){
        position_vec = _position_tip_to_ned;
        return true;
    }
    return false;
}

void AP_ArmLocation_Excavator::update_position()
{
    Inclination *inclination = AP::inclination();
    if(inclination == nullptr){
        return;
    }
    AP_AHRS &ahrs = AP::ahrs();

    _euler_boom_body_from_sensor = ahrs.earth_to_body(inclination->get_deg_location(Boom));
    _euler_stick_body_from_sensor = ahrs.earth_to_body(inclination->get_deg_location(Forearm));
    _euler_bucket_body_from_sensor = ahrs.earth_to_body(inclination->get_deg_location(Bucket));
    
    get_3angle_to_body(_euler_boom_body_from_sensor,_euler_stick_body_from_sensor,_euler_bucket_body_from_sensor,0,ahrs.get_yaw());
    _angle_boom_to_slewing = _angle_boom_to_body;
    update_boom_state();
    _angle_stick_to_boom = _angle_stick_to_body - _angle_boom_to_slewing;
    update_stick_state();
    _angle_bucket_to_stick = _angle_bucket_to_body - _angle_stick_to_boom;
    update_bucket_state();
    cal_bucket_position_body();
    AP::logger().Write("ARMPOSI", "TimeUS,x,y,z",
                   "sm", // units: seconds, meters
                   "FB", // mult: 1e-6, 1e-2
                   "Qfff", // format: uint64_t, float, float, float
                   AP_HAL::micros64(),
                   (float)_position_tip_to_body.x,
                   (float)_position_tip_to_body.y,
                   (float)_position_tip_to_body.z);
    Location current_loc;
    if(!ahrs.get_position(current_loc)){
        state.body_loc_status = Loc_Status::BAD;
        return;
    }
    if(current_loc.get_vector_from_origin_NEU(_body_position)){
        state.body_loc_status = Loc_Status::GOOD;
        _position_tip_to_ned.x = cosf(ahrs.get_yaw()) * _position_tip_to_body.x - sinf(ahrs.get_yaw()) * _position_tip_to_body.y + _body_position.x;
        _position_tip_to_ned.y = sinf(ahrs.get_yaw()) * _position_tip_to_body.x + cosf(ahrs.get_yaw()) * _position_tip_to_body.y + _body_position.y;
        _position_tip_to_ned.z = _position_tip_to_body.z;
    }
}


//Determine whether the arm exceeds the limit
void AP_ArmLocation_Excavator::update_boom_state()
{
    state.boom_out_range_up = false;
    state.boom_out_range_down = false;
    if(_angle_boom_to_slewing >= ARM_BOOM_DEGREE_MAX){
        _angle_boom_to_slewing = ARM_BOOM_DEGREE_MAX;
        state.boom_out_range_up = true;
    }else if(_angle_boom_to_slewing <= ARM_BOOM_DEGREE_MIN){
        _angle_boom_to_slewing = ARM_BOOM_DEGREE_MIN;
        state.boom_out_range_down = true;
    }else{
        return;
    }
}
void AP_ArmLocation_Excavator::update_stick_state()
{
    state.stick_out_range_up = false;
    state.stick_out_range_down = false;
    if(_angle_stick_to_boom >= ARM_STICK_DEGREE_MAX){
        _angle_stick_to_boom = ARM_STICK_DEGREE_MAX;
        state.stick_out_range_up = true;
    }else if(_angle_stick_to_boom <= ARM_STICK_DEGREE_MIN){
        _angle_stick_to_boom = ARM_STICK_DEGREE_MIN;
        state.stick_out_range_down = true;
    }else{
        return;
    }
}
void AP_ArmLocation_Excavator::update_bucket_state()
{
    state.bucket_out_range_up = false;
    state.bucket_out_range_down = false;
    if(_angle_bucket_to_stick >= ARM_BUCKET_DEGREE_MAX){
        _angle_bucket_to_stick = ARM_BUCKET_DEGREE_MAX;
        state.bucket_out_range_up = true;
    }else if(_angle_bucket_to_stick <= ARM_BUCKET_DEGREE_MIN){
        _angle_bucket_to_stick = ARM_BUCKET_DEGREE_MIN;
        state.bucket_out_range_down = true;
    }else{
        return;
    }
}


//Calculate the angle of the boom, stick, bucket relative to the fuselage
void AP_ArmLocation_Excavator::get_3angle_to_body(Vector3f euler_boom, Vector3f euler_stick, Vector3f euler_bucket, float angle_slewing, float angle_phi)
{
    _angle_boom_to_body = radians(euler_boom.x + _angle_boom_up);
    _angle_stick_to_body = radians(euler_stick.x);
    float angle_RodBegin_Rod_RodTip = M_PI + _angle_stick_to_body - _angle_RodBegin_Rod_SRotation - radians(euler_bucket.x);
    float angle_BRotation_Rod_RodTip = M_2PI - _angle_SRotation_Rod_BRotation -
        _angle_RodBegin_Rod_SRotation - angle_RodBegin_Rod_RodTip;
    float distance_BRotation_RodTip = sqrt(_distance_BRotation_to_Rod * _distance_BRotation_to_Rod
        + _distance_RodTip_to_Rod * _distance_RodTip_to_Rod
        - 2 * _distance_BRotation_to_Rod * _distance_RodTip_to_Rod * cosf(angle_BRotation_Rod_RodTip));
    float angle_RodTip_BRotation_Rod = acosf((distance_BRotation_RodTip * distance_BRotation_RodTip
        + _distance_BRotation_to_Rod * _distance_BRotation_to_Rod
        - _distance_RodTip_to_Rod * _distance_RodTip_to_Rod) /
        (2 * distance_BRotation_RodTip * _distance_BRotation_to_Rod));
    float angle_RodTip_BRotation_BRodLink = acosf((_distance_BRodLink_to_BRodLink * _distance_BRodLink_to_BRodLink
        + distance_BRotation_RodTip * distance_BRotation_RodTip
        - _distance_RodTip_to_BRodLink * _distance_RodTip_to_BRodLink) /
        (2 * _distance_BRodLink_to_BRodLink * _distance_RodTip_to_BRodLink));
    float angle_SRotation_BRotation_tip = M_2PI - _angle_Rod_BRotation_SRotation - angle_RodTip_BRotation_Rod
        - angle_RodTip_BRotation_BRodLink - _angle_BRodLink_BRotation_tip;
    _angle_bucket_to_body = _angle_stick_to_body + angle_SRotation_BRotation_tip - M_PI;
    if(_angle_bucket_to_body<(-M_PI)){
        _angle_bucket_to_body += M_PI;
    }
    if(_angle_bucket_to_body>M_PI){
        _angle_bucket_to_body -= M_PI;
    }
    _angle_slewing_to_body = angle_slewing - angle_phi;
}
//Calculate the three-dimensional coordinates of the tooth tip relative to the body
void AP_ArmLocation_Excavator::cal_bucket_position_body()
{
        ///位置计算
        _position_tip_to_body.x = cosf(_angle_slewing_to_body) *
            (_distance_tip_to_bucket * cosf(_angle_bucket_to_body) + _distance_bucket_to_stick * cosf(_angle_stick_to_body)
                + _distance_stick_to_boom * cosf(_angle_boom_to_body) + _distance_boom_to_slewing);
        _position_tip_to_body.y = sinf(_angle_slewing_to_body) *
            (_distance_tip_to_bucket * cosf(_angle_bucket_to_body) + _distance_bucket_to_stick * cosf(_angle_stick_to_body)
                + _distance_stick_to_boom * cosf(_angle_boom_to_body) + _distance_boom_to_slewing);
        _position_tip_to_body.z = _distance_tip_to_bucket * sinf(_angle_bucket_to_body) + _distance_bucket_to_stick * sinf(_angle_stick_to_body)
            + _distance_stick_to_boom * sinf(_angle_boom_to_body) + _distance_slewing_to_ground;
}

AP_ArmLocation_Excavator *AP_ArmLocation_Excavator::_singleton;

namespace AP
{

AP_ArmLocation_Excavator *excavator_arm()
{
    return AP_ArmLocation_Excavator::get_singleton();
}

}
