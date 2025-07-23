#include <simMultiBody.h>
#include <dynamics_model.h>
#include <signal.h>

std::string robot_file = "/home/merai/Files/resources/CR5_ROS/dobot_description/urdf/cr5_robot.urdf";
bool runloop = true;
void sighandler(int signum) {runloop = false;}

std::string control_link = "Link6";
Eigen::Vector3d control_pos_in_link_cam = Eigen::Vector3d(0,0, 0.62);
Eigen::Vector3d control_pos_in_link_inst = Eigen::Vector3d(0, -0.032, 0.25);

Eigen::Matrix3d R_cAr, R_ArL;
Eigen::Vector3d T_vec;

int main(int argc, char const *argv[])
{
    signal(SIGINT,sighandler);
    Eigen::Vector3d _bpose(0,0,0);
    Eigen::Quaterniond _brot;
    _brot.setIdentity();

    int nDof = 6;
    Eigen::VectorXd _qCam,_qInst;
    _qCam.resize(nDof);
    _qInst.resize(nDof);

    _qCam.setZero(nDof);
    _qInst.setZero(nDof);

    _qCam << 29.2, 26.5, 100.6, -92.9, -92.1, -28.7;
    _qInst << 89.2, 23.3, 77.8, -83.1, -81.8, 0.0;

    _qCam = _qCam * M_PI/180;
    _qInst = _qInst * M_PI/180;

    std::cout << "_qCam : " << _qCam.transpose() << std::endl;
    std::cout << "_qInst: " << _qInst.transpose() << std::endl;

    R_cAr <<  -0.305947491335088, 0.7653429363114411, 0.5662564104572657,
                0.9419396939566774, 0.1568936546132018, 0.296873700571327,
                    0.1383681520241092, 0.6242071539002852, -0.7689081112364069;

    T_vec = Eigen::Vector3d(0.00125272, -0.00820716, 0.124099);

    std::cout << R_cAr << std::endl;

    R_ArL = Eigen::AngleAxisd(-M_PI/2,Eigen::Vector3d::UnitX()).toRotationMatrix();


    Dynamics::DModel* robot_model = new Dynamics::DModel(robot_file,_bpose,_brot,
                                                          false,false);
    
    Eigen::Affine3d T_camr_ee, T_cam_ar, T_ar_ee, T_inr_ee, T;
    T_cam_ar.linear() = R_cAr;
    T_cam_ar.translation() = T_vec;

    T_ar_ee.linear() = R_ArL;
    T_ar_ee.translation() = Eigen::Vector3d(0,0,0);
    
    // get camera robot transformation.
    robot_model->_q = _qCam;
    robot_model->updateKinematics();

    Eigen::Vector3d pos;
    Eigen::Matrix3d rot;

    robot_model->position(pos,control_link,control_pos_in_link_cam);
    robot_model->rotation(rot,control_link);

    T_camr_ee.translation() = pos;
    T_camr_ee.linear() = rot;

    // get instrument robot transformation.
    robot_model->_q = _qInst;
    robot_model->updateKinematics();

    robot_model->position(pos,control_link,control_pos_in_link_inst);
    robot_model->rotation(rot,control_link, R_ArL);

    T_inr_ee.translation() = pos;
    T_inr_ee.linear() = rot;

    T = T_camr_ee * T_cam_ar.inverse() * T_inr_ee.inverse();

    std::cout << "Length : " << T.translation().norm() << std::endl;
    std::cout << "pos: " << T.translation().transpose() << std::endl;
    std::cout << "rot: \n" << T.linear() << std::endl;
    Eigen::AngleAxisd axis = Eigen::AngleAxisd(T.linear());
    std::cout << "rot angle: \n" << axis.angle() << "\n" << "rot axis: \n" << axis.axis() << std::endl;


    return 0;
}
