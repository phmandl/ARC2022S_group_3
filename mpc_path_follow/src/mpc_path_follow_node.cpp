// Additional STUFF
#include <iostream>

// ROS CORE
#include <ros/ros.h>

// ROS MSGS
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

// MPC
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/MatrixFunctions>
#include <OsqpEigen/OsqpEigen.h>
#include <chrono>

using namespace Eigen;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

struct params  
{  
    double tau = 0.5; // s - drive train time constant
    double C_alpha_f = 19000; // Np/rad - cornering stiffnes front
    double C_alpha_r = 33000; // Np/rad - cornering stiffnes rear
    double m = 1575; // kg
    double L_f = 1.2; // m - CoM to front
    double L_r = 1.6; // m - CoM to rear
    double Iz = 2875; // Nms^2 - yaw moment

    // system size
    const int nx = 6;
    int nu = 2;
    int ny = 3;
    int nz = 1;

    // MPC STUFF
    double Ts = 0.1; //s - sampling Time
    double Tl = 3; // s - look-ahead time
    const int Np = Tl/Ts;
    int Nc = Np;
    int variables = Nc*nu;

    // Weighting
    double R1 = 1; // weighting: delta
    double R2 = 2; // weighting: ax
    double Q1 = 500; // V-ref weight
    double Q2 = 50; // e1 weight
    double Q3 = 1e3; // e2 weight

    // Constraints
    double axMin = -3; // m/s^2
    double axMax = +3; // m/s^2
    double deltaMin = -50.0/180.0*M_PI; // rad
    double deltaMax = +50.0/180.0*M_PI; // rad
};

struct System {
    MatrixXd A;
    MatrixXd B;
    MatrixXd E;
    MatrixXd C;
};

struct QPmatrizen {
    SparseMatrix<double> hessian;
    VectorXd gradient;
};

struct constraints {
    VectorXd lowerBound;
    VectorXd upperBound;
    SparseMatrix<double> linearMatrix;
};



class mpcPathFollow {
// This class make path and speed-control
private:
    ros::NodeHandle n;

    // Subscriber
    // ros::Subscriber odom_sub;
    // ros::Subscriber laser_sub;

    // Publisher
    // ros::Publisher break_bool_pub;

    // Declare some variables
    double speed;

    // MPC Stuff
    params data;
    System cont;
    System dis;
    QPmatrizen qp_matrizen;
    constraints cons;
    OsqpEigen::Solver solver;
    VectorXd QPSolution;  // controller input and QPSolution vector
    VectorXd xk;
    VectorXd v_ref;
    VectorXd curvature;

public:
    mpcPathFollow() { // Constructor of the class
        // Initilization Params
        n = ros::NodeHandle();
        speed = 0.0;

        // Make Subscribers
        // laser_sub = n.subscribe("scan",1,&Safety::scan_callback,this);
        // odom_sub = n.subscribe("odom",1,&Safety::odom_callback,this);

        // Make Publishers
        // break_bool_pub = n.advertise<std_msgs>("brake_bool",1);

        // MPC STUFF
        xk = VectorXd::Zero(data.nx,1);
        v_ref = VectorXd::Zero(data.Np,1);
        curvature = VectorXd(data.Np,1);

        // Just for tryout --> should normally come from planner and simulation
        v_ref = VectorXd::Ones(data.Np)*2.7778; // desired velocity for the prediction-horizon
        xk << 0, 0, 0, 0, 0, 0.6568; // current state
        curvature = VectorXd::Zero(data.Np)/10;

        // System-Matrix
        cont = setDynamicsMatrices(data);

        // Discrtize system !!
        dis = setDiscreteSystem(cont,data,0.1);

        // Build Hessian, f, constraint matrix etc.
        qp_matrizen = setHessianGradient(dis,data,xk,curvature,v_ref);

        // Make constraints
        cons = setLowerUpperBounds(data);

        // OSQP - solve the problem
        solver.settings()->setWarmStart(true);
        solver.settings()->setVerbosity(false); // disable solver feeback

        solver.data()->setNumberOfVariables(data.Nc*data.nu);
        solver.data()->setNumberOfConstraints(data.Np*data.nu);
        if(!solver.data()->setHessianMatrix(qp_matrizen.hessian)) ROS_INFO_STREAM("MPC: Error in setup");
        if(!solver.data()->setGradient(qp_matrizen.gradient)) ROS_INFO_STREAM("MPC: Error in setup");
        if(!solver.data()->setLinearConstraintsMatrix(cons.linearMatrix)) ROS_INFO_STREAM("MPC: Error in setup");
        if(!solver.data()->setLowerBound(cons.lowerBound)) ROS_INFO_STREAM("MPC: Error in setup");
        if(!solver.data()->setUpperBound(cons.upperBound)) ROS_INFO_STREAM("MPC: Error in setup");

        // instantiate the solver
        if(!solver.initSolver()) ROS_INFO_STREAM("MPC: Error in setup");

    }

    void solveMPC() {
        // Update hessian --> next step
        dis = setDiscreteSystem(cont,data,0.1); // Update first the discrete system ---> velocity update!
        qp_matrizen = setHessianGradient(dis,data,xk,curvature,v_ref); // Build hessian etc
        if(!solver.updateHessianMatrix(qp_matrizen.hessian)) ROS_INFO_STREAM("MPC: Error in setup");
        if(!solver.updateGradient(qp_matrizen.gradient)) ROS_INFO_STREAM("MPC: Error in setup");

        // Solve
        if(!solver.solve()) {
            ROS_INFO_STREAM("MPC: Solver faild");
        };
        QPSolution = solver.getSolution();

    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // TODO: update current speed
        // double x_velocity = odom_msg->twist.twist.linear.x;
        // speed = x_velocity;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    }

    // MPC STUFF HERE -- WE SHOULD PUT THIS IN AN OUTHER FILE AND CLASS
    // ----------------------------------------------------------------
    System setDynamicsMatrices(params &data) {
        System cont;
        cont.A = MatrixXd::Zero(data.nx,data.nx);
        cont.B = MatrixXd::Zero(data.nx,data.nu);
        cont.E = MatrixXd::Zero(data.nx,data.nz);
        cont.C = MatrixXd::Zero(data.ny,data.nx);

        // BUILD SYSTEM MATRIX
        // ---------------------------------------------------------------------------
        cont.A(0,0) = -1/data.tau;
        cont.A(1,0) = 1;
        cont.A(2,2) = -(2*data.C_alpha_f + 2*data.C_alpha_r)/data.m; // missing 1/Vx
        cont.A(2,3) = -(2*data.C_alpha_f*data.L_f - 2*data.C_alpha_r*data.L_r)/data.m; // missing 1/Vx - Vx
        cont.A(3,2) = -(2*data.C_alpha_f*data.L_f - 2*data.C_alpha_r*data.L_r)/data.Iz; // missing 1/Vx
        cont.A(3,3) = -(2.0*data.C_alpha_f*data.L_f*data.L_f + 2.0*data.C_alpha_r*data.L_r*data.L_r)/data.Iz; // missing 1/Vx
        cont.A(4,2) = 1;
        cont.A(4,5) = 1; // missing 1*Vx
        cont.A(5,3) = 1;

        cont.B(0,0) = 1/data.tau;
        cont.B(2,1) = 2*data.C_alpha_f/data.m;
        cont.B(3,1) = 2*data.L_f*data.C_alpha_f/data.Iz;
        
        cont.E(5) = -1; // missing 1*Vx

        cont.C(0,1) = 1;
        cont.C(1,4) = 1;
        cont.C(2,5) = 1;

        return cont;
    }

    System setDiscreteSystem(System &cont, struct params &data, double Vx) {
        System dis;
        dis.A = cont.A;
        dis.B = cont.B;
        dis.E = cont.E;
        dis.C = cont.C;

        dis.A(2,2) = cont.A(2,2)/Vx;
        dis.A(2,3) = cont.A(2,3)/Vx - Vx;
        dis.A(3,2) = cont.A(3,2)/Vx;
        dis.A(3,3) = cont.A(3,3)/Vx;
        dis.A(4,5) = cont.A(4,5)*Vx;
        dis.E(5) = cont.E(5)*Vx;

        MatrixXd As = MatrixXd::Zero(data.nx + data.nu, data.nx + data.nu); // super A
        
        As.block(0,0,data.nx,data.nx) = dis.A;
        As.block(0,data.nx,data.nx,data.nu) = dis.B;
        As.block(data.nx,data.nx,data.nu,data.nu) = MatrixXd::Identity(data.nu,data.nu);
        MatrixXd expmAsTs = (As*data.Ts).exp();
        dis.A = expmAsTs.block(0,0,data.nx,data.nx);
        dis.B = expmAsTs.block(0,data.nx,data.nx,data.nu);

        As = MatrixXd::Zero(data.nx + data.nz, data.nx + data.nz); // super A
        As.block(0,0,data.nx,data.nx) = dis.A;
        As.block(0,data.nx,data.nx,data.nz) = dis.E;
        As.block(data.nx,data.nx,data.nz,data.nz) = MatrixXd::Identity(data.nz,data.nz);
        expmAsTs = (As*data.Ts).exp();
        dis.E = expmAsTs.block(0,data.nx,data.nx,data.nz);
        
        return dis;
    }

    QPmatrizen setHessianGradient(System &dis, const params &data, VectorXd &xk, VectorXd &curvature, VectorXd &v_ref) {
        QPmatrizen out;

        // start with creating F
        MatrixXd F = MatrixXd::Zero(data.Np*data.ny,data.nx);
        MatrixXd Apow = MatrixXd::Identity(data.nx,data.nx);
        for (size_t i = 0; i < data.Np; i++)
        {
            Apow = Apow*dis.A;
            // F.block(data.ny*i,0,data.ny,data.nx) = dis.C*dis.A.pow(i + 1);
            F.block(data.ny*i,0,data.ny,data.nx) = dis.C*Apow;
        }

        // Make Phi_u
        MatrixXd Phi_u = MatrixXd::Zero(data.Np*data.ny, data.Nc*data.nu);
        MatrixXd firstCol = MatrixXd::Zero(data.Np*data.ny, data.nu);
        firstCol.block(0,0,data.ny,data.nu) = dis.C*dis.B;
        for (size_t i = 1; i < data.Np; i++)
        {
            firstCol.block(data.ny*i,0,data.ny,data.nu) = F.block(data.ny*(i - 1),0,data.ny,data.nx)*dis.B;
        }
        for (size_t i = 0; i < data.Nc; i++)
        {
            Phi_u.block(data.ny*i, data.nu*i, data.Np*data.ny - data.ny*i, data.nu) = firstCol.block(0, 0, data.Np*data.ny - data.ny*i, data.nu);
        }

        // // Phi_z
        MatrixXd Phi_z = MatrixXd::Zero(data.Np*data.ny, data.Np*data.nz);
        firstCol = MatrixXd::Zero(data.Np*data.ny, data.nz);
        firstCol.block(0,0,data.ny,data.nz) = dis.C*dis.E;
        for (size_t i = 1; i < data.Np; i++)
        {
            firstCol.block(data.ny*i,0,data.ny,data.nz) = F.block(data.ny*(i - 1),0,data.ny,data.nx)*dis.E;
        }
        for (size_t i = 0; i < data.Np; i++)
        {
            Phi_z.block(data.ny*i, data.nz*i, data.Np*data.ny - data.ny*i, data.nz) = firstCol.block(0, 0, data.Np*data.ny - data.ny*i, data.nz);
        }

        // Make big weighting matrix
        Vector2d R(data.R1,data.R2);
        Vector3d Q(data.Q1,data.Q2,data.Q3);
        MatrixXd bigR = MatrixXd::Zero(data.nu*data.Nc, data.nu*data.Nc);
        MatrixXd bigQ = MatrixXd::Zero(data.ny*data.Np, data.ny*data.Np);
        bigR.diagonal() << R.replicate(data.Nc,1);
        bigQ.diagonal() << Q.replicate(data.Np,1);

        MatrixXd reference = MatrixXd::Zero(data.ny*data.Np,1) ; // 1:3:60 --> v_ref / 2:3:60 --> lateral ref e1 / 3:3:60 --> yaw ref e2
        for (size_t i = 0; i < data.ny*data.Np; i = i + data.ny)
        {
            reference(i) = v_ref(i/data.ny);
        }

        SparseMatrix<double> spH, spf, spPhi_u, spBigR, spBigQ; //without allocation
        spPhi_u = Phi_u.sparseView();
        spBigR = bigR.sparseView();
        spBigQ = bigQ.sparseView();

        // Calculate hessian with sparse --> faster
        spH = 2.0*(spPhi_u.transpose()*spBigQ*spPhi_u + spBigR);

        // Make vector f
        VectorXd f(data.Nc*data.nu,1);
        spf = 2.0*spPhi_u.transpose()*spBigQ*(F*xk + Phi_z*curvature - reference).sparseView();
        f = VectorXd(spf);

        // assign output
        out.hessian = spH;
        out.gradient = f;
        return out;
    }

    constraints setLowerUpperBounds(params &data) {
        constraints out;

        // evaluate the lower and the upper inequality vectors
        VectorXd lowerInequality = VectorXd::Zero(data.Np*data.nu);
        VectorXd upperInequality = VectorXd::Zero(data.Np*data.nu);
        for (size_t i = 0; i < data.Np*data.nu; i += data.nu)
        {
            // constraints for first input (a_x)
            lowerInequality(i) = data.axMin;
            upperInequality(i) = data.axMax;

            // constraints for second input (delta)
            lowerInequality(i+1) = data.deltaMin;
            upperInequality(i+1) = data.deltaMax;
        }
        
        // populate linear constraint matrix
        SparseMatrix<double> A(data.Np*data.nu,data.Np*data.nu);
        for (size_t i = 0; i < data.Np*data.nu; i++)
        {
            A.insert(i,i) = -1;
        }
        
        // asssert output
        out.linearMatrix = A;
        out.lowerBound = lowerInequality;
        out.upperBound = upperInequality;
        return out;
    }

};

int main(int argc, char ** argv){
    //Initialize and start the node
    ros::init(argc, argv, "mpcPathFollow");

    // Init Controller
    auto controller = mpcPathFollow();

    // DEBUG MESSAGE
    ROS_INFO_STREAM("starting with safety brake");

    // Make ROS-Rate-Update-Rate
    ros::Rate loop_rate(10);
    int count=0;

    

    // START ROS-LOOP
    while (ros::ok())
    {

        // RUN MPC
        auto t1 = high_resolution_clock::now();
        controller.solveMPC();
        auto t2 = high_resolution_clock::now();
        duration<double, std::milli> ms_double = (t2 - t1)/1;

        std::cout << "\nMPC runtime: " << ms_double.count() << "ms\n";

        // Step once and wait time
        ros::spinOnce();
        loop_rate.sleep();

    }
    
    return 0;
}
