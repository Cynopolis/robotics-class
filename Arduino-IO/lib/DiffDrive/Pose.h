#pragma once

// typedef struct pose
typedef struct Pose{
    // x and y are in mm
    float x = 0;
    float y = 0;
    // theta is in radians
    float theta = 0;
    // d_x, d_y, and d_theta are the change in position and theta in mm/s and radians/s
    float d_x = 0;
    float d_y = 0;
    float d_theta = 0;

    // Add a function to add two poses together
    Pose operator+(const Pose &other){
        Pose newPose;
        newPose.x = this->x + other.x;
        newPose.y = this->y + other.y;
        newPose.theta = this->theta + other.theta;
        newPose.d_x = this->d_x + other.d_x;
        newPose.d_y = this->d_y + other.d_y;
        newPose.d_theta = this->d_theta + other.d_theta;
        return newPose;
    }

    // Add a function to subtract two poses together
    Pose operator-(const Pose &other){
        Pose newPose;
        newPose.x = this->x - other.x;
        newPose.y = this->y - other.y;
        newPose.theta = this->theta - other.theta;
        newPose.d_x = this->d_x - other.d_x;
        newPose.d_y = this->d_y - other.d_y;
        newPose.d_theta = this->d_theta - other.d_theta;
        return newPose;
    }

    // Add a function to make a pose equal to another pose
    Pose operator=(const Pose &other){
        this->x = other.x;
        this->y = other.y;
        this->theta = other.theta;
        this->d_x = other.d_x;
        this->d_y = other.d_y;
        this->d_theta = other.d_theta;
        return *this;
    }

    // add a function to multiply the pose by a constant
    Pose operator*(const float constant){
        Pose newPose;
        newPose.x = this->x * constant;
        newPose.y = this->y * constant;
        newPose.theta = this->theta * constant;
        newPose.d_x = this->d_x * constant;
        newPose.d_y = this->d_y * constant;
        newPose.d_theta = this->d_theta * constant;
        return newPose;
    }

    // add a function to divide the pose by a constant
    Pose operator/(const float constant){
        Pose newPose;
        newPose.x = this->x / constant;
        newPose.y = this->y / constant;
        newPose.theta = this->theta / constant;
        newPose.d_x = this->d_x / constant;
        newPose.d_y = this->d_y / constant;
        newPose.d_theta = this->d_theta / constant;
        return newPose;
    }
} Pose;