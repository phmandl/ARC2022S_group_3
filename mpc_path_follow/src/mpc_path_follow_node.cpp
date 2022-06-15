// Additional STUFF
#include <iostream>

// ROS CORE
#include <ros/ros.h>

// ROS MSGS
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Path.h>

// TF
#include <tf/transform_datatypes.h>

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
    // PARAMETERS FROM PARAMS.YAML
    double tau = 0.1; // s - drive train time constant
    double C_alpha_f = 4.718; // Np/rad - cornering stiffnes front
    double C_alpha_r = 5.4562; // Np/rad - cornering stiffnes rear
    double m = 3.47; // kg
    double L_f = 0.15875; // m - CoM to front
    double L_r = 0.17145; // m - CoM to rear
    double Iz = .04712; // Nms^2 - yaw moment

    // system size
    const int nx = 6;
    int nu = 2;
    int ny = 3;
    int nz = 1;

    // MPC STUFF
    double Ts = 0.05; //s - sampling Time
    double Tl = 0.5; // s - look-ahead time
    const int Np = Tl/Ts;
    int Nc = Np;
    int variables = Nc*nu;

    // Weighting
    double R1 = 5; // weighting: delta --> don't use steering too much
    double R2 = 1; // weighting: ax --> don't accel to much
    double Q1 = 1; // V-ref weight --> track vref
    double Q2 = 2; // e1 weight --> reduce lateral error
    double Q3 = 1; // e2 weight --> reduce heading error 1e3

    // Constraints
    double axMin = -3; // m/s^2
    double axMax = +3; // m/s^2
    double deltaMin = -1*0.4189; // -50.0/180.0*M_PI; // rad
    double deltaMax = +1*0.4189; // +50.0/180.0*M_PI; // rad
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
    ros::Subscriber odom_sub;
    ros::Subscriber imu_sub;

    // Subscribers for optimised path
    ros::Subscriber path_sub;

    // Publisher
    ros::Publisher drive_pub;

    // Declare some variables
    double speed;
    double steering;
    double accel;
    double Vx;
    double VxDot;
    double Vy;
    double psiDot;
    double e1;
    double e2;

    // Imu Stuff
    double xpos;
    double ypos;
    double roll;
    double pitch;
    double yaw;

    // MPC Stuff
    System cont;
    System dis;
    QPmatrizen qp_matrizen;
    constraints cons;
    OsqpEigen::Solver solver;
    VectorXd QPSolution;  // controller input and QPSolution vector
    VectorXd xk;

    // Drive Topic
    ackermann_msgs::AckermannDriveStamped drive_msg;

    // PATH DATA
    VectorXd xpos_mpc;
    VectorXd ypos_mpc;
    VectorXd vref_mpc;
    VectorXd curv_mpc;
    VectorXd ax_mpc;

public:

    // MPC Stuff
    params data;

    mpcPathFollow() { // Constructor of the class
        // Initilization Params
        n = ros::NodeHandle();
        speed = 0.0;
        steering = 0.0;
        accel = 0.0;
        Vx = 0.0;
        VxDot = 0.0;
        Vy = 0.0;
        psiDot = 0.0;
        e1 = 0.0;
        e2 = 0.0;
        xpos = 0.0;
        ypos = 0.0;
        roll = 0.0;
        pitch = 0.0;
        yaw = 0.0;
        
        // MPC STUFF
        xk = VectorXd::Zero(data.nx,1);
        xpos_mpc = VectorXd::Zero(data.Np,1);
        ypos_mpc = VectorXd::Zero(data.Np,1);
        vref_mpc = VectorXd::Zero(data.Np,1);
        curv_mpc = VectorXd::Zero(data.Np,1);
        ax_mpc = VectorXd::Zero(data.Np,1);

        // Make Subscribers
        odom_sub = n.subscribe("odom",1,&mpcPathFollow::odom_callback,this);
        imu_sub = n.subscribe("imu",1,&mpcPathFollow::imu_callback,this);
        path_sub = n.subscribe("mpc_path",1,&mpcPathFollow::path_callback,this);

        // Make Publishers
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 1);

        // Drive message
        drive_msg.header.frame_id = "map";
        drive_msg.drive.steering_angle = 0.0;
        drive_msg.drive.acceleration = 0.0;

        // System-Matrix
        cont = setDynamicsMatrices(data);

        // Discrtize system !!
        dis = setDiscreteSystem(cont,data,0.1);

        // Build Hessian, f, constraint matrix etc.
        qp_matrizen = setHessianGradient(dis,data,xk,curv_mpc,vref_mpc);

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
        // What is needed: xk, velocity (for linearisation), curvature (for path tracking), vref (vector)
        // Size of stuff:  6,  1, Np, Np
        // STATE: VxDot, Vx, Vy, psiDot, e1, e2

        // Get current xk: calculate lateral error and heading error
        xk << VxDot, Vx, Vy, psiDot, e1, e2;
        // xk << 0, Vx, 0, 0, e1, e2;
        
        // Check for zero velocity
        double v_lin = 0.0;
        if (std::abs(Vx) < 0.1) {
            v_lin = 0.1;
        } else {
            v_lin = Vx;
        }
    
        // Update hessian --> next step
        dis = setDiscreteSystem(cont,data,v_lin); // Update first the discrete system ---> velocity update!
        qp_matrizen = setHessianGradient(dis,data,xk,curv_mpc,vref_mpc); // Build hessian etc
        if(!solver.updateHessianMatrix(qp_matrizen.hessian)) ROS_INFO_STREAM("MPC: Error in setup");
        if(!solver.updateGradient(qp_matrizen.gradient)) ROS_INFO_STREAM("MPC: Error in setup");

        // Solve
        if(!solver.solve()) {
            ROS_INFO_STREAM("MPC: Solver faild");
        };
        QPSolution = solver.getSolution();

        // Extract control commands
        accel = QPSolution(0);
        steering = QPSolution(1);

        // PUBLISH COMMANDS
        // ROS_INFO_STREAM("MPC: accel " << accel << ", steering " << steering);

        // Publish drive message, don't forget to limit the steering angle.
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.drive.steering_angle = steering;
        drive_msg.drive.acceleration = accel;
        drive_msg.drive.speed = vref_mpc(0);
        drive_pub.publish(drive_msg);   
    }

    void path_callback(const nav_msgs::Path::ConstPtr &path_msg) {

        // Zero pose carrys the error
        e1 = path_msg->poses[0].pose.orientation.z;
        e2 = path_msg->poses[0].pose.orientation.w;

        for (int i = 0; i < data.Np; i++) {
            xpos_mpc[i] = path_msg->poses[i].pose.position.x;
            ypos_mpc[i] = path_msg->poses[i].pose.position.y;
            vref_mpc[i] = path_msg->poses[i].pose.position.z;
            ax_mpc[i] = path_msg->poses[i].pose.orientation.x;
            curv_mpc[i] = path_msg->poses[i].pose.orientation.y;
        }

    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // Kalamnfilter hierher --> um auch Vy und psiDot richtig zu schätzen!
        Vx = odom_msg->twist.twist.linear.x;
        
        Vy = odom_msg->twist.twist.linear.y; // Ist empty ...
        psiDot = odom_msg->twist.twist.angular.z;
        xpos = odom_msg->pose.pose.position.x;
        ypos = odom_msg->pose.pose.position.y;

        tf::Quaternion q(odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w);

        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        yaw = wrap_to_pi(yaw);

        // ROS_INFO_STREAM(Vy << " " << VxDot << " ");
    }

    double wrap_to_pi(double val) {
        return std::fmod(val + M_PI, 2*M_PI) - M_PI;
    }

    void imu_callback(const sensor_msgs::Imu::ConstPtr &imu_msg) {
        // Kalamnfilter hierher --> um auch Vy und psiDot richtig zu schätzen!
        VxDot = imu_msg->linear_acceleration.x;
        // psiDot = imu_msg->angular_velocity.z;
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
    ROS_INFO_STREAM("MPC: STARTING CONTROLLER!");

    // Make ROS-Rate-Update-Rate
    ros::Rate loop_rate(1/controller.data.Ts);

    // START ROS-LOOP
    while (ros::ok())
    {

        // RUN MPC
        // auto t1 = high_resolution_clock::now();
        controller.solveMPC();
        // auto t2 = high_resolution_clock::now();
        // duration<double, std::milli> ms_double = (t2 - t1)/1;

        // std::cout << "\nMPC runtime: " << ms_double.count() << "ms\n";

        // Step once and wait time
        ros::spinOnce();
        loop_rate.sleep();

    }
    
    return 0;
}
