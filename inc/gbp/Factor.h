/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Utils.h>
#include <gbp/GBPCore.h>
#include <memory>
#include <vector>
#include <raylib.h>

// Forward declaration of Simulator
class Simulator;

extern Globals globals;

using Eigen::seqN;
using Eigen::seq;
using Eigen::last;

class Variable;     // Forward declaration

// Types of factors defined. Default is DEFAULT_FACTOR
enum FactorType {DEFAULT_FACTOR, DYNAMICS_FACTOR, INTERROBOT_FACTOR, OBSTACLE_FACTOR};

/*****************************************************************************************/
// Factor used in GBP
/*****************************************************************************************/
class Factor {
    public:
    Simulator* sim_;                            // Pointer to simulator
    int f_id_;                                  // Factor id
    int r_id_;                                  // Robot id this factor belongs to
    Key key_;                                   // Factor key = {r_id_, f_id_}
    int other_rid_;                             // id of other connected robot (if this is an inter-robot factor)
    int n_dofs_;                                // n_dofs of the variables connected to this factor
    Eigen::VectorXd z_;                         // Measurement
    Eigen::MatrixXd h_, J_;                     // Stored values of measurement function h_func_() and Jacobian J_func_()
    Eigen::VectorXd X_;                         // Stored linearisation point
    Eigen::MatrixXd meas_model_lambda_;         // Precision of measurement model
    Mailbox inbox_, outbox_, last_outbox_;      
    FactorType factor_type_ = DEFAULT_FACTOR; 
    float delta_jac=1e-8;                       // Delta used for first order jacobian calculation
    bool initialised_ = false;                  // Becomes true when Jacobian calculated for the first time
    bool linear_ = false;                       // True is factor is linear (avoids recomputation of Jacobian)
    bool skip_flag = false;                     // Flag to skip factor update if required
    virtual bool skip_factor(){                 // Default function to set skip flag
        skip_flag = false;
        return skip_flag;
    };
    std::vector<std::shared_ptr<Variable>> variables_;    // Vector of pointers to the connected variables. Order of variables matters

    // Function declarations
    Factor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
            float sigma, const Eigen::VectorXd& measurement, 
            int n_dofs=4);

    ~Factor();

    void draw();

    virtual Eigen::MatrixXd h_func_(const Eigen::VectorXd& X) = 0;

    virtual Eigen::MatrixXd J_func_(const Eigen::VectorXd& X);

    Eigen::MatrixXd jacobianFirstOrder(const Eigen::VectorXd& X0);

    virtual Eigen::VectorXd residual(){return z_ - h_;};

    bool update_factor();

    Message marginalise_factor_dist(const Eigen::VectorXd &eta, const Eigen::MatrixXd &Lam, int var_idx, int marg_idx);
};

/********************************************************************************************/
/********************************************************************************************/
//                      CUSTOM FACTORS SPECIFIC TO THE PROBLEM
// Create a new factor definition as shown with these examples.
// You may create a new factor_type_, in the enum in Factor.h (optional, default type is DEFAULT_FACTOR)
// Create a measurement function h_func_() and optionally Jacobian J_func_().

class DynamicsFactor: public Factor {
public:
    DynamicsFactor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
        float sigma, const Eigen::VectorXd& measurement, float dt);

    // Constant velocity model
    Eigen::MatrixXd h_func_(const Eigen::VectorXd& X);
    Eigen::MatrixXd J_func_(const Eigen::VectorXd& X);
};

class InterrobotFactor: public Factor {
public:
    double safety_distance_;

    InterrobotFactor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
        float sigma, const Eigen::VectorXd& measurement, 
        float robot_radius);

    Eigen::MatrixXd h_func_(const Eigen::VectorXd& X);
    Eigen::MatrixXd J_func_(const Eigen::VectorXd& X);
    bool skip_factor();
};

class ObstacleFactor: public Factor {
public:
    Image* p_obstacleImage_;

    ObstacleFactor(Simulator* sim, int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
        float sigma, const Eigen::VectorXd& measurement, Image* p_obstacleImage);

    Eigen::MatrixXd h_func_(const Eigen::VectorXd& X);
};
