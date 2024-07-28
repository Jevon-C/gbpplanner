/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once

#include <vector>
#include <raylib.h>  // raylib.h includes the definitions for Camera, Vector2, Vector3, etc.
#include <raymath.h> // raymath.h includes the math functions for vectors
#include <rcamera.h> // rcamera.h includes the camera control functions
#include "Globals.h" // Assuming globals is a separate header defining the globals
#include "rlights.h" // Assuming rlights.h defines Light and MAX_LIGHTS

/**************************************************************************/
// Graphics class that deals with the nitty-gritty of display.
// Camera is also included here. You can set different camera positions/trajectories
// and then during simulation cycle through them using the SPACEBAR

// Please note: Raylib camera defines the world with positive X = right, positive Y = down, and positive Z = into-plane
/**************************************************************************/
class Graphics
{
public:
    // Constructor
    Graphics(Image obstacleImg);
    ~Graphics();

    Image obstacleImg_;      // Image representing obstacles in the environment
    Texture2D texture_img_;  // Raylib Texture created from obstacleImg
    Model robotModel_;       // Raylib Model representing a robot. This can be changed.
    Model groundModel_;      // Model representing the ground plane
    Vector3 groundModelpos_; // Ground plane position
    Shader lightShader_;     // Light shader

    Camera camera3d = {0}; // Define the camera to look into our 3d world
    // These represent a set of camera transition frames.
    std::vector<Vector3> camera_positions_;
    std::vector<Vector3> camera_ups_;
    std::vector<Vector3> camera_targets_;
    int camera_idx_ = 0;
    uint32_t camera_clock_ = 0;
    bool camera_transition_ = false;

    // Task Models and Colors
    Model fireModel_;
    Model robberyModel_;
    Color fireColor_;
    Color robberyColor_;

    // Function to update camera based on mouse and key input
    void update_camera();

    // Function to draw the environment and objects
    void draw();
};
