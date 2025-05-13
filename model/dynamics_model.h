#ifndef _DYN_MODEL_H
#define _DYN_MODEL_H

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Addons;
using namespace RigidBodyDynamics::Utils;

namespace Dynamics
{    
    class DModel
    {
    private:
        Model* _rbdl_model;
        unsigned int _dof;
        int _q_size;
    public:
        
        DModel(std::string, Eigen::Vector3d& , Eigen::Quaterniond& ,
                bool floating_base = false, bool _verbose = true);
        ~DModel();

        void setGravity(Eigen::Vector3d& _g);

        unsigned int linkId(std::string& _link_name);

        unsigned int jointId(std::string& _joint_name);

        void updateKinematics();

        void updateDynamics();

        void updateInverseInertia();

        void updateModel();

        int dof();

        int q_size();

        /**
         * @brief Gives joint gravity torque vector of the last update configuration using world
         *          model gravity.
         *@param g where gravity vector computed witll be written to.
         * 
        */
        void gravityVector(Eigen::VectorXd& g); 
        /**
         * @brief Gives joint gravity torque vector and World gravity vector computed with
         *          respect to base frame.
         * @param g joint gravity torque vector.
         * @param gravity 3d gravity vector of the world in base frame.
        */
        void gravityVector(Eigen::VectorXd& g, Eigen::VectorXd& gravity);

        void coriolisForces(Eigen::VectorXd& c);

        void coriolisPlusGravityForces(Eigen::VectorXd& h);

        void J_0(Eigen::MatrixXd& J, const std::string& link_name,
                    const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
        void J_0_World_Frame(Eigen::MatrixXd& J, const std::string& link_name,
                                const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
        void J_0_Local_Frame(Eigen::MatrixXd& J, const std::string& link_name, 
                                const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
        void J_v(Eigen::MatrixXd& Jv, const std::string& link_name,
                    const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
        void J_v_World_Frame(Eigen::MatrixXd& Jv, const std::string& link_name,
                    const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
        void J_w(Eigen::MatrixXd& Jw, const std::string& link_name,
                    const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
        void J_w_World_Frame(Eigen::MatrixXd& Jw, const std::string& link_name,
                    const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());


        void transformation(Eigen::Vector3d& _pos, Eigen::Matrix3d& _rot,
                        const std::string& link_name,
                        const Vector3d& pos_in_link = Eigen::Vector3d::Zero(),
                        const Matrix3d& rot_in_link = Eigen::Matrix3d::Identity(),
                        const std::string& base_frame = "");
        
        void position(Eigen::Vector3d& _pos, std::string& link_name,
                        const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());

        void positionInWorld(Eigen::Vector3d& _pos, std::string& link_name,
                        const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());

        void rotation(Eigen::Matrix3d& _rot, std::string& link_name,
                        const Eigen::Matrix3d& rot_in_link = Eigen::Matrix3d::Identity());
        void rotationInWorld(Eigen::Matrix3d& _rot, std::string& link_name,
                        const Eigen::Matrix3d& rot_in_link = Eigen::Matrix3d::Identity());

        void linearVelocity(Eigen::Vector3d& _vel, std::string& link_name,
                        const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
        void linearVelocityWorld(Eigen::Vector3d& _vel, std::string& link_name,
                        const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
        
        void angularVelocity(Eigen::Vector3d& _avel, std::string& link_name,
                const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
        void angularVelocityWorld(Eigen::Vector3d& _avel, std::string& link_name,
                const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
        
        void linearAcceleration(Eigen::Vector3d& _accel, std::string& link_name,
                    const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
        void linearAccelerationWorld(Eigen::Vector3d& _accel, std::string& link_name,
                    const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
        
        void angularAcceleration(Eigen::Vector3d& _aaccel, std::string& link_name,
                const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
        void angularAccelerationWorld(Eigen::Vector3d& _aaccel, std::string& link_name,
                const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());

        // class attributes below.
        Eigen::VectorXd _q;
        Eigen::VectorXd _dq;
        Eigen::VectorXd _ddq;
        Eigen::MatrixXd _M;
        Eigen::MatrixXd _M_inv;

        Eigen::Affine3d _T_world;
    };
} // namespace Dynamics


#endif