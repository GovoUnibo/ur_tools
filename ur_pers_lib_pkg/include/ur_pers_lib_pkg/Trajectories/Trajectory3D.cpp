#include "ur_pers_lib_pkg/Trajectories/Trajectory3D.hpp"

using namespace std;
using namespace Eigen;
using namespace MyQuaternion;


namespace Traj{
    Trajectory3D::Trajectory3D()
    : Traj::ScaledTrajectory()
    {}

    Trajectory3D::Trajectory3D(TrajParam traj_param, string traj_type)
    : Traj::ScaledTrajectory(traj_param, traj_type)
    {   }

    Trajectory3D::~Trajectory3D()
    {}

    void Trajectory3D::resize_pos_vectors(int num_of_points){
        this->position_x.resize(num_of_points);
        this->position_y.resize(num_of_points);
        this->position_z.resize(num_of_points);
    }

    void Trajectory3D::resize_vel_vectors(int num_of_points){
        this->velocity_x.resize(num_of_points);
        this->velocity_y.resize(num_of_points);
        this->velocity_z.resize(num_of_points);
    }

    void Trajectory3D::resize_acc_vectors(int num_of_points){
        this->acceleration_x.resize(num_of_points);
        this->acceleration_y.resize(num_of_points);
        this->acceleration_z.resize(num_of_points);
    }

    void Trajectory3D::resize_rot_vectors(int num_of_points){
        this->theta_x.resize(num_of_points);
        this->theta_y.resize(num_of_points);
        this->theta_z.resize(num_of_points);
        this->omega_x.resize(num_of_points);
        this->omega_y.resize(num_of_points);
        this->omega_z.resize(num_of_points);
    }

    vector<Vector3d> Trajectory3D::get3DTrajPositionPoints(Eigen::Vector3d start_pose, Eigen::Vector3d end_pose){
        int num_of_points = ScaledTrajectory::getNumOfPosPoints();
        this->resize_pos_vectors(num_of_points);
        vector<Vector3d> position(num_of_points);

        this->position_x = ScaledTrajectory::scalePosition(start_pose(0), end_pose(0));
        this->position_y = ScaledTrajectory::scalePosition(start_pose(1), end_pose(1));
        this->position_z = ScaledTrajectory::scalePosition(start_pose(2), end_pose(2));

        for (int i = 0; i < num_of_points; i++)
            position[i] = Vector3d(position_x[i], position_y[i], position_z[i]);

        return position;
    }

    vector<Vector3d>Trajectory3D::get3DTrajVelocityPoints(Eigen::Vector3d start_pose, Eigen::Vector3d end_pose){
        int num_of_points = ScaledTrajectory::getNumOfPosPoints();
        this->resize_vel_vectors(num_of_points);

        vector<Vector3d> velocity(num_of_points);

        this->velocity_x = ScaledTrajectory::scaleVelocity(start_pose(0), end_pose(0));
        this->velocity_y = ScaledTrajectory::scaleVelocity(start_pose(1), end_pose(1));
        this->velocity_z = ScaledTrajectory::scaleVelocity(start_pose(2), end_pose(2));

        for (int i = 0; i < num_of_points; i++)
            velocity[i] = Vector3d(velocity_x[i], velocity_y[i], velocity_z[i]);

        return velocity;
    }

    vector<Vector3d> Trajectory3D::get3DTrajAccelerationPoints(Eigen::Vector3d start_pose, Eigen::Vector3d end_pose){
        int num_of_points = ScaledTrajectory::getNumOfPosPoints();
        this->resize_acc_vectors(num_of_points);

        vector<Vector3d> acceleration(num_of_points);

        this->acceleration_x = ScaledTrajectory::scaleAcceleration(start_pose(0), end_pose(0));
        this->acceleration_y = ScaledTrajectory::scaleAcceleration(start_pose(1), end_pose(1));
        this->acceleration_z = ScaledTrajectory::scaleAcceleration(start_pose(2), end_pose(2));

        for (int i = 0; i < num_of_points; i++)
            acceleration[i] = Vector3d(acceleration_x[i], acceleration_y[i], acceleration_z[i]);

        return acceleration;
    }

    void  Trajectory3D::rotTraj(){
        RPY::RPY_Angle_s angles;
        
        int num_of_points = ScaledTrajectory::getNumOfPosPoints();

        this->resize_rot_vectors(num_of_points);

        vector<double> s = ScaledTrajectory::getScaledPositionPoints();

        Quaternion_s firs_point_on_sphere = this->q0;
        Quaternion_s second_point_on_sphere = this->qf;
        AngVel::AngularVelocity_struct omega;

        
        for (int i = 0; i < num_of_points; i++){
            second_point_on_sphere = slerp(this->q0, this->qf, s[i]);
            angles = second_point_on_sphere.getRPY();
            this->theta_x[i] = angles.roll;
            this->theta_y[i] = angles.pitch;
            this->theta_z[i] = angles.yaw;

            omega = rotationVelocity(firs_point_on_sphere, second_point_on_sphere, this->time_discretizzation);
            
            this->omega_x[i] = omega.w_x;
            this->omega_y[i] = omega.w_y;
            this->omega_z[i] = omega.w_z;
            
            firs_point_on_sphere = second_point_on_sphere;
        }
    }

    vector<Vector3d> Trajectory3D::get3DRotPositionPoints(Vector3d start_rot, Vector3d end_rot)//theta_x, theta_y, theta_z
    {
        this->q0 = Quaternion_s(start_rot(0), start_rot(1), start_rot(2));
        this->qf = Quaternion_s(end_rot(0), end_rot(1), end_rot(2));

        int num_of_points = ScaledTrajectory::getNumOfPosPoints();
        vector<Vector3d> angle(num_of_points);

        this->rotTraj();

        
        for (int i = 0; i < num_of_points; i++)
            angle[i] = Vector3d(this->theta_x[i], this->theta_y[i], this->theta_z[i]);
        
        return angle;

    }

    vector<Vector3d> Trajectory3D::get3DTrajAngularVelocityPoints(Vector3d start_rot, Vector3d end_rot){
        this->q0 = Quaternion_s(start_rot(0), start_rot(1), start_rot(2));
        this->qf = Quaternion_s(end_rot(0), end_rot(1), end_rot(2));

        int num_of_points = ScaledTrajectory::getNumOfPosPoints();
        vector<Vector3d> omega(num_of_points);

        this->rotTraj();

        
        for (int i = 0; i < num_of_points; i++)
            omega[i] = Vector3d(this->omega_x[i], this->omega_y[i], this->omega_z[i]);
        
        return omega;
    }

    vector<VectorXd> Trajectory3D::getPosRotTrajPoints(VectorXd start_pose, VectorXd end_pose){
        int num_of_points = ScaledTrajectory::getNumOfPosPoints();
        vector<VectorXd> pose_points(num_of_points, VectorXd::Zero(6));

        vector<Vector3d> position = this->get3DTrajPositionPoints(Vector3d(start_pose(0), start_pose(1), start_pose(2)), Vector3d(end_pose(0), end_pose(1), end_pose(2)));
        vector<Vector3d> angle = this->get3DRotPositionPoints(Vector3d(start_pose(3), start_pose(4), start_pose(5)), Vector3d(end_pose(3), end_pose(4), end_pose(5)));
        
        for (int i = 0; i < num_of_points; i++){
            pose_points[i].head(3) = position[i];
            pose_points[i].tail(3) = angle[i];
        }
       
        return pose_points;
    }

} // namespace Traj3D