#include "faraday_state_publisher/faraday_state_publisher.h"

//limb1-4 link1 initial pose in the base_link
const tf::Vector3 faraday_state_publisher::limb1_link1_position(
        0.257, 0.12829, 0.087145);
const tf::Vector3 faraday_state_publisher::limb1_link1_orientation(
        0.0, 0.0, -2.0944);

const tf::Vector3 faraday_state_publisher::limb2_link1_position(
        0.24382, -0.17253, 0.085659);
const tf::Vector3 faraday_state_publisher::limb2_link1_orientation(
        0.0, 0.0, 2.618);

const tf::Vector3 faraday_state_publisher::limb3_link1_position(
        -0.24382, -0.17253, 0.085659);
const tf::Vector3 faraday_state_publisher::limb3_link1_orientation(
        0.0, 0.0, 0.5236);

const tf::Vector3 faraday_state_publisher::limb4_link1_position(
        -0.257, 0.12829, 0.087145);
const tf::Vector3 faraday_state_publisher::limb4_link1_orientation(
        0.0, 0.0, -1.0472);

//limb5 link1-3 initial orientation in the base_link
const tf::Matrix3x3 faraday_state_publisher::limb5_link1_rotation(
        0.0, -1.0, 0.0,
        0.0,  0.0, 1.0,
       -1.0,  0.0, 0.0);
const tf::Vector3 faraday_state_publisher::limb5_link1_position(
        -0.175, -0.09, -0.046);

const tf::Matrix3x3 faraday_state_publisher::limb5_link2_rotation(
        0.0,  0.0, -1.0,
        1.0,  0.0,  0.0,
        0.0, -1.0,  0.0);
const tf::Vector3 faraday_state_publisher::limb5_link2_position(
        0.0, -0.069, -0.0935);

const tf::Matrix3x3 faraday_state_publisher::limb5_link3_rotation(
        0.0, -1.0, 0.0,
        1.0,  0.0, 0.0,
        0.0,  0.0, 1.0);

const double faraday_state_publisher::limb5_link1_y=0.09;

const double faraday_state_publisher::limb5_link2_y=0.069;

const double faraday_state_publisher::limb5_link3_y=0.033;

const double faraday_state_publisher::limb5_link3_z=0.265;




