#include "ParallelRobot.h"


ParallelRobot::ParallelRobot(const std::string& robot_file, 
                const Eigen::Vector3d& bpose, const Eigen::Quaterniond& brot,
                bool floating_base, bool _verbose) :
DModel(robot_file,bpose, brot,floating_base,_verbose)     
{
    
    J_c.setZero();
    M_c.setZero();
    
    
    _nDof = dof();
    // reduced coordinate (eliminating two passive joints of five bar chain.)
    // elbow joint and support joint are passive joints.
    _nr = _nDof - 2; 
    // resize the reduced joint vector.
    _qr.resize(_nr);
    _dqr.resize(_nr);
    // resize the _q_prev, _dq_prev, _ddq_prev vectors.
    _q_prev.resize(_nDof);
    _dq_prev.resize(_nDof);
    _ddq_prev.resize(_nDof);

    M_c.resize(_nr, _nr);
    J_c.resize(_nr, _nr);

    // fill the initial C vector which maps reduced to full coordinates;
    C.resize(_nDof, _nr);
    C.setZero();
    int count = 0;
    for (int i = 0; i < _nDof; i++)
    {
        if ( (i == _nDof -1))
        {
            count -= 1;
            C(i,1) = 1 * (1);
            C(i,_nr -1) = -1* (1);
        }
        else if (i == 2)
        {
            // count -= 1;
            C(i,1) = -1 * (1);
            C(i,_nr - 1) = 1* (1);
        }
        else
        {
            C(i,count) = 1;
            count += 1;
        }
    }
    // computeMappingMatrix(C);
    std::cout << "C:\n" << C << std::endl;
    // resize the private variables for computation.
    _Jv.resize(3,_nr);
    _Jv.setZero();

    _Jv_tmp.resize(3, _nDof);
    _Jv_tmp.setZero();

    _J.resize(6,_nr);
    _J.setZero();

    _J_tmp.resize(6,_nDof);
    _J_tmp.setZero();

    _J_notSwapped.resize(6,_nDof);
    _J_notSwapped.setZero();

    Is.resize(6,6);
    Is.setZero();
}


void ParallelRobot::computeMappingMatrix(Eigen::MatrixXd& mat)
{
    if (mat.size() != _nDof * _nr)
    {
        mat.resize(_nDof,_nr);
    }
    mat.setZero();
    Eigen::MatrixXd tmp = Eigen::MatrixXd::Identity(_nDof, _nr);
    tmp.row(_nDof -2).swap(tmp.row(2));
    tmp(2,1) = 1;
    tmp(2,_nr - 1) = -1;

    tmp(_nDof-1, 1) = -1;
    tmp(_nDof-1, _nr - 1) = 1;
    // mat = tmp;
    std::cout << "tmp: \n" << tmp << std::endl;
}
// void ParallelRobot::getRc()      // Reduced coordinate.
// {
//     // joint positions
//     _qr(0) = _q(0);
//     _qr(1) = _q(1);
//     _qr(2) = _q(_nDof-2); // shoulder joint2
//     _qr(3) = _q(3);
//     _qr(4) = _q(4);
//     _qr(5) = _q(5);
//     // joint velocity.
//     _dqr(0) = _dq(0);
//     _dqr(1) = _dq(1);
//     _dqr(2) = _dq(_nDof-2); // the last joint
//     _dqr(3) = _dq(3);
//     _dqr(4) = _dq(4);
//     _dqr(5) = _dq(5);
// }

void ParallelRobot::setFc()      // Full Coordinate.
{
    // // joint positions
    // _q(0) = _qr(0);
    // _q(1) = _qr(1);
    // _q(_nDof - 2) = _qr(2); // shoulder joint2
    // _q(3) = _qr(3);
    // _q(4) = _qr(4);
    // _q(5) = _qr(5);
    // // joint velocity.
    // _dq(0) = _dqr(0);
    // _dq(1) = _dqr(1);
    // _dq(_nDof - 2) = _dqr(2); // the last joint
    // _dq(3) = _dqr(3);
    // _dq(4) = _dqr(4);
    // _dq(5) = _dqr(5);

    // joint positions
    _q(0) = _qr(0);
    _q(1) = _qr(1);
    _q(_nDof - 2) = _qr(_nr - 1); // shoulder joint2

    // _q(_nDof - 2) = _qr(2); // shoulder joint2 (Actual motor connection)

    if (_nDof > 5)
    {
        _q(3) = _qr(2);
        _q(4) = _qr(3);
        _q(5) = _qr(4);
    }
    
    // set current to prev.
    _dqr_prev = _dqr;  // required for computing acceleration.
    
    // joint velocity.
    _dq(0) = _dqr(0);
    _dq(1) = _dqr(1);
    _dq(_nDof - 2) = _dqr(_nr - 1); // the last joint
    if (_nDof > 5)
    {
        _dq(3) = _dqr(2);
        _dq(4) = _dqr(3);
        _dq(5) = _dqr(4);
    }
    
}


// void ParallelRobot::setFc()      // Full Coordinate.
// {
//     // the degree of freedom of the actual hand controller is 7.

//     // joint positions
//     _q(0) = _qr(0);
//     _q(1) = _qr(1);
//     _q(_nDof - 2) = _qr(2); // shoulder joint2 (Actual motor connection)

//     if (_nDof > 5)
//     {
//         _q(3) = _qr(3);
//         _q(4) = _qr(4);
//         _q(5) = _qr(5);
//         _q(5) = _qr(6);
//     }
//     // joint velocity.
//     _dq(0) = _dqr(0);
//     _dq(1) = _dqr(1);
//     _dq(_nDof - 2) = _dqr(2);   // shoulder joint2 (Actual motor connection)
//     if (_nDof > 5)
//     {
//         _dq(3) = _dqr(3);
//         _dq(4) = _dqr(4);
//         _dq(5) = _dqr(5);
//         _dq(6) = _dqr(6);
//     }
// }

void ParallelRobot::assignPassiveJoint()
{
    // joint position.
    _q(_nDof -1) = (M_PI + _q(1) - _q(_nDof - 2));
    _q(2) = _q(_nDof - 2) - _q(1); 

    // // joint velocity.
    // _dq(_nDof -1) = _dq(1);
    // _dq(2) = _dq(_nDof - 2) - _dq(1);
}

void ParallelRobot::getJacobian(Eigen::MatrixXd& J, const std::string& link_name, bool reduced,
                            const Eigen::Vector3d& pos_in_link)
{
    if (reduced)
    {
        assert(("Incorrect Linear Jacobian matrix size.",J.size() == 3 * _nr));
        // Eigen::MatrixXd J_tmp = Eigen::MatrixXd::Zero(3,_nDof);
        // J_tmp.setZero();
        // get bodyid
        int bid = linkId(link_name);
        CalcPointJacobian(*getRBDLModel(), _q,bid,pos_in_link,_Jv_tmp,false);
        J = _Jv_tmp * C;
    }
    else
    {
        assert(("Incorrect Linear Jacobian matrix size.",J.size() == 3 * _nDof));
        // Eigen::MatrixXd J_tmp = Eigen::MatrixXd::Zero(_nDof,_nDof);
        // get bodyid
        int bid = linkId(link_name);
        CalcPointJacobian(*getRBDLModel(), _q,bid,pos_in_link,J,false);
    }
    
}

void ParallelRobot::getJacobian6D(Eigen::MatrixXd& J, const std::string& link_name, bool reduced,
                            const Eigen::Vector3d& pos_in_link)
{
    if (reduced)
    {
        assert(("Incorrect 6D Jacobian matrix size.",J.size() == 6 * _nr));
        // Eigen::MatrixXd J_tmp = Eigen::MatrixXd::Zero(6,_nDof);
        // _J_tmp.setZero();
        // get bodyid
        int bid = linkId(link_name);
        CalcPointJacobian6D(*getRBDLModel(), _q,bid,pos_in_link,_J_notSwapped,false);
        _J_tmp << _J_notSwapped.block(3,0,3,_nDof), _J_notSwapped.block(0,0, 3, _nDof);
        J = _J_tmp * C;
    }
    else
    {
        assert(("Incorrect 6D Jacobian matrix size.",J.size() == 6 * _nDof));
        // Eigen::MatrixXd J_tmp = Eigen::MatrixXd::Zero(6,_nDof);
        // get bodyid
        int bid = linkId(link_name);
        // CalcPointJacobian(*getRBDLModel(), _q,bid,pos_in_link,J,false);
        J_0(J,link_name,pos_in_link);
    }
}

void ParallelRobot::getGravityVector(Eigen::VectorXd& _g)
{
    _g.setZero();

    // Eigen::MatrixXd Jv;
    // Jv.resize(3,_nr);
    // Jv.setZero();
    _Jv.setZero();

    // Eigen::MatrixXd J_tmp;
    // J_tmp.resize(3,_nDof);
    _Jv_tmp.setZero();

    int body_id = 0;
    for (auto it : getRBDLModel()->mBodies)
    {
        // J_tmp.setZero();
        double mass = it.mMass;
        CalcPointJacobian(*getRBDLModel(),_q,body_id,it.mCenterOfMass,_Jv_tmp,false);
        _Jv = _Jv_tmp * C;
        _g = _g + _Jv.transpose() * (- mass * getRBDLModel()->gravity);
        body_id++;
    }
}

void ParallelRobot::computeMassMatrix(Eigen::MatrixXd& M)
{
    assert(("Incorrect Mass matrix size.",M.size() == _nr * _nr));
    M.setZero();

    // Eigen::MatrixXd J, J_tmp, J_notSwapped, Is;
    // J.resize(6,_nr);
    // J.setZero();

    // J_tmp.resize(6,_nDof);
    // J_notSwapped.resize(6, _nDof);

    // Is.resize(6,6);
    // Is.setZero();

    int body_id = 0;
    for (auto it : getRBDLModel()->mBodies)
    {
        // J_tmp.setZero();
        // J_notSwapped.setZero();

        // compute the jacobian.
        double mass = it.mMass;
        CalcPointJacobian6D(*getRBDLModel(),_q,body_id,it.mCenterOfMass,_J_notSwapped,false);
        _J_tmp << _J_notSwapped.block(3,0,3,_nDof) , _J_notSwapped.block(0,0,3,_nDof);
        _J = _J_tmp *C ;
        // set up the spatial inertia matrix.
        Is.setZero();
        Is.block(0,0,3,3) = mass * Eigen::Matrix3d::Identity();
        Is.block(3,3,3,3) = it.mInertia;

        // Now compute the mass matrix.
        M = M + _J.transpose() * Is * _J;
        body_id++;
    }
}

void ParallelRobot::ComputeMassMatrix(Eigen::MatrixXd& M)
{
    assert(("Incorrect Mass matrix size.",M.size() == _nr * _nr));
    M.setZero();
    Eigen::MatrixXd _M;
    _M.resize(_nDof, _nDof);
    _M.setZero();
    // std::cout << "qsize: " << 
    CompositeRigidBodyAlgorithm(*getRBDLModel(), _q,_M,false);
    M = C.transpose() * _M * C;
}

void ParallelRobot::computeMassMatrixandGravityVector(Eigen::MatrixXd& M, Eigen::VectorXd& G)
{
    assert(("Incorrect Mass matrix size.",M.size() == _nr * _nr));
    assert(("Gravity vector size incorrect.", G.size() == _nr));
    M.setZero();
    G.setZero();

    // Eigen::MatrixXd J, J_tmp, J_notSwapped, Is;
    // _J.resize(6,_nr);
    // _J.setZero();

    // // for gravity torque computation.


    // _J_tmp.resize(6,_nDof);
    // _J_notSwapped.resize(6, _nDof);

    // Is.resize(6,6);
    // Is.setZero();

    int body_id = 0;
    for (auto it : getRBDLModel()->mBodies)
    {
        if (it.mMass != 0)
        {
            // _J_tmp.setZero();
            // _J_notSwapped.setZero();

            // compute the jacobian.
            double mass = it.mMass;
            CalcPointJacobian6D(*getRBDLModel(),_q,body_id,it.mCenterOfMass,_J_notSwapped,false);
            _J_tmp << _J_notSwapped.block(3,0,3,_nDof) , _J_notSwapped.block(0,0,3,_nDof);
            _J = _J_tmp * C;
            // set up the spatial inertia matrix.

            // Is.setZero();
            // Is.block(0,0,3,3) = mass * Eigen::Matrix3d::Identity();
            // Is.block(3,3,3,3) = it.mInertia;

            // Now compute the mass mat
            // M += _J.transpose() * Is * _J;
            M += mass * _J.block(0,0,3,_nr).transpose()* _J.block(0,0,3,_nr) + 
                        _J.block(3,0,3,_nr).transpose() * it.mInertia * _J.block(3,0,3,_nr);
            G += _J.block(0,0,3,_nr).transpose() * (-mass)* getRBDLModel()->gravity;
        }
        body_id++;
    }

}

void ParallelRobot::forward_step(Eigen::VectorXd _tor, double _tstep) // always call this after updating the model.
{
    assert(("torque vector size mismatch", _tor.size() == _nr));

    Eigen::VectorXd gT;
    gT.resize(_nr);
    gT.setZero();

    Eigen::VectorXd ddqr, dqr;

    ddqr.resize(_nr);
    ddqr.setZero();
    
    computeMassMatrix(M_c);
    getGravityVector(gT);

    // computeMassMatrixandGravityVector(M_c,gT);

    ddqr = M_c.inverse() * (_tor - gT);
    dqr = ddqr * _tstep;
    _qr = _qr + dqr * _tstep;

    UpdateParallelRobotKinematics();

}

void ParallelRobot::UpdateParallelRobotKinematics()
{
    // assign full joint angle from reduced coordinates
    setFc();
    // assign the passive joints.
    Eigen::VectorXd delq = _q - _q_prev;

    // _q(_nDof -1 ) = _q(_nDof -1) + (delq(1) - delq(_nDof - 2))*(1);
    // _q(2) = _q(2) + (delq(_nDof - 2) - delq(1))*(1);

    _q(2) = _q(2) + (delq(_nDof - 2) - delq(1));
    _q(_nDof -1 ) = _q(_nDof -1) + (delq(1) - delq(_nDof - 2));

    delq.setZero();

    updateKinematics();

    // assign current angles as prev.
    _q_prev = _q;
    _dq_prev = _dq;

    // compute acceleration
    if (_first_iteration)
    {
        t_curr = get_current_time();
        t_prev = t_curr;
        _ddqr.setZero();
    }
    else
    {
        t_curr = get_current_time();
        _ddqr = (_dqr - _dqr_prev)/(t_curr - t_prev);
        t_prev = t_curr;
    }
    
}

void ParallelRobot::UpdateParallelRobotModel()
{
    UpdateParallelRobotKinematics();
    computeMassMatrix(M_c);
}
