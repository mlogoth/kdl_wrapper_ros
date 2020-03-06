/*
* Kinematics and Dynamics Library
* Developed By NTUA Michalis Logothetis
*/
#define KINETIC false

#ifndef KDL_WRAPPER_HPP
#define KDL_WRAPPER_HPP

#include <cstddef>
#include <vector>
#include <string>
#include <fstream>
#include <stdio.h>
#include <math.h>

// ROS Library
#include <ros/ros.h>

// KDL Libraries
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#if KINETIC
#else
#include <kdl/chainjnttojacdotsolver.hpp>
#endif
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

// Eigen Libraries
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
// Boost LibrariesW
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#define assertm(exp, msg) assert(((void)msg, exp))

class KDLWrapper
{
public:
    // Constructors
    KDLWrapper(std::string robot_description, ros::NodeHandle nh);
    KDLWrapper(KDL::Tree kdl_tree, ros::NodeHandle nh);
    // Distructor
    virtual ~KDLWrapper();

    // costruct the chain and inialize the solvers
    bool init(const std::string &chain_root, const std::string &chain_tip, bool gravity);

    // returns true if the object was initialized correctly
    bool isInitialized() {return _isinitialized;};

    // returns the KDL chain
    KDL::Chain getKDLChain() {return _kdl_chain;};

    // Compute

    /*
    * Forward Kinematics :
    * @param Eigen::VectorXd or std::vector of doubles 
    *        that contains joints displacement
    * @return Eigen::VectorXd or Eigen::Matrix3d and Eigen::Vector3d 
    *         with pose of tip link  
    * */
    int ComputeFK(const Eigen::VectorXd &q_in, Eigen::Vector3d &position, Eigen::Matrix3d &orientation);
    int ComputeFK(const Eigen::VectorXd &q_in, Eigen::VectorXd &pose_pos);
    int ComputeFK(const std::vector<double> &q_in, Eigen::Vector3d &position, Eigen::Matrix3d &orientation);
    int ComputeFK(const Eigen::VectorXd &q_in, Eigen::Vector3d &position, Eigen::Quaterniond &orientation);
    int ComputeFK(const std::vector<double> &q_in, Eigen::VectorXd &pose_pos);

    /*
    * Forward Kinematics Velocity :
    * @param Eigen::VectorXd or std::vector of doubles 
    *        that contains q and q_dot
    * @return Eigen::VectorXd with the velocity of tip link  
    * */
    int ComputeFKVel(const Eigen::VectorXd &q_in, const Eigen::VectorXd &qdot_in, Eigen::VectorXd &v_out, Eigen::Vector3d &p_out, Eigen::Matrix3d &ori_out);
    int ComputeFKVel(const std::vector<double> &q_in, const std::vector<double> &qdot_in, Eigen::VectorXd &v_out, Eigen::Vector3d &p_out, Eigen::Matrix3d &ori_out);

    /*
    * Forward Kinematics Acceleration :
    * @param Eigen::VectorXd or std::vector of doubles 
    *        that contains q, q_dot and q_ddot
    * @return Eigen::VectorXd with the velocity of tip link  
    * */
    
    // TODO: ADD DEFINITION
    //int ComputeFKAcc(const Eigen::VectorXd &q_in, const Eigen::VectorXd &qdot_in, const Eigen::VectorXd &qddot_in, Eigen::VectorXd &acc_out);
    //int ComputeFKAcc(const std::vector<double> &q_in, const std::vector<double> &qdot_in, const std::vector<double> &qddot_in, Eigen::VectorXd &acc_out);

    /*
    * Inverse Kinematics :
    * @param Eigen::VectorXd of q and Eigen::VectorXd of pose   
    * @return Eigen::VectorXd with the q   
    * */
    int ComputeIK(const Eigen::VectorXd &q_in, const Eigen::VectorXd &pose_in, Eigen::VectorXd &q_out);
    int ComputeIK(const Eigen::VectorXd &q_in, const Eigen::Vector3d &position_in, const Eigen::Matrix3d &rotation_in, Eigen::VectorXd &q_out);
    int ComputeIK(const Eigen::VectorXd &q_in, const Eigen::Vector3d &position_in, const Eigen::Quaterniond &rotation_in, Eigen::VectorXd &q_out);

    /*
    * Inverse Kinematics Velocity:
    * @param Eigen::VectorXd of q and Eigen::VectorXd of tip velocity   
    * @return Eigen::VectorXd with the qdot   
    * */
    
    int ComputeIKVel(const Eigen::VectorXd &q_in, const Eigen::VectorXd &v_in, Eigen::VectorXd &qdot_out);
    int ComputeIKVel(const std::vector<double> &q_in, const std::vector<double> &v_in, Eigen::VectorXd &qdot_out);

    /*
    * Inverse Kinematics Acceleration:
    * @param Eigen::VectorXd of q and Eigen::VectorXd of tip velocity   
    * @return Eigen::VectorXd with the qdot   
    * */
    // TODO: ADD DEFINITION
    //int ComputeIKAcc(const Eigen::VectorXd &q_in, const Eigen::VectorXd &qdot_in, const Eigen::VectorXd &acc_in, Eigen::VectorXd &qddot_out);
    //int ComputeIKAcc(const std::vector<double> &q_in, std::vector<double> &qdot_in, const std::vector<double> &acc_in, Eigen::VectorXd &qddot_out);

    /*
    * Compute Jacobian:
    * @param Eigen::VectorXd of q    
    * @return Eigen::MatrixXd with the Jacobian   
    * */
    int ComputeJac(const Eigen::VectorXd &q_in, Eigen::MatrixXd &jacobian);
    int ComputeJac(const std::vector<double> &q_in, Eigen::MatrixXd &jacobian);


    /*
    * Compute Time Derivative of Jacobian:
    * @param Eigen::VectorXd of q and qdot    
    * @return Eigen::MatrixXd with the Jacobian   
    * */
    int ComputeJacDot(const Eigen::VectorXd &q_in, const Eigen::VectorXd &qdot_in, Eigen::MatrixXd &jacobian);
    int ComputeJacDot(const std::vector<double> &q_in, const std::vector<double> &qdot_in, Eigen::MatrixXd &jacobian);


    /*
    * Compute Mass Inertia Matrix:
    * @param Eigen::VectorXd of q    
    * @return Eigen::MatrixXd with the Mass Inertia MAtrix   
    * */
    int ComputeMassMatrix(const Eigen::VectorXd &q_in, Eigen::MatrixXd &Mass_mat);
    
    
    /*
    * Compute Coriolis Joint vector:
    * @param Eigen::VectorXd of q and qdot    
    * @return Eigen::VectorXd with the Coriolis Joint Vector   
    * */
    int ComputeCoriolisVector(const Eigen::VectorXd &q_in,const Eigen::VectorXd &qdot_in, Eigen::VectorXd &coriolis_vector);
    
    /*
    * Compute Gravity Vector
    * @param Eigen::VectorXd of q     
    * @return Eigen::VectorXd with gravity torques   
    * */
    int ComputeGravityVector(const Eigen::VectorXd &q_in, Eigen::VectorXd &gravity_vector);


    /*
    * Compute Inverse Dynamics:
    * @param Eigen::VectorXd of q, qdot, qddot, fext (external forces)    
    * @return Eigen::VectorXd with the torques   
    * */
    int ComputeInverseDynamics(const Eigen::VectorXd &q_in, const Eigen::VectorXd &qdot_in, const Eigen::VectorXd &qddot_in, const std::vector<Eigen::VectorXd> &fext, Eigen::VectorXd &torques);



     /*
    * Compute Inverse Dynamics:
    * @param Eigen::VectorXd of q, qdot, qddot,   
    * @return Eigen::VectorXd with the torques   
    * */
    int ComputeInverseDynamics(const Eigen::VectorXd &q_in, const Eigen::VectorXd &qdot_in, const Eigen::VectorXd &qddot_in, Eigen::VectorXd &torques);
    



    /*
    * Compute Cartesian Space Mass Inertia Matrix:
    * @param Eigen::VectorXd of q    
    * @return Eigen::MatrixXd Mc   
    * */
    int ComputeMassMatrixCartesian(const Eigen::VectorXd &q_in, Eigen::MatrixXd &Mass_mat);

    /*
    * Compute Coriolis Cartesian Space vector:
    * @param Eigen::VectorXd of q and qdot    
    * @return Eigen::VectorXd with the Coriolis Cartesian Vector   
    * */
    int ComputeCoriolisVectorCartesian(const Eigen::VectorXd &q_in,const Eigen::VectorXd &qdot_in, Eigen::VectorXd &coriolis_vector);

    /*
    * Compute Gravity Cartesian Vector:
    * @param Eigen::VectorXd of q     
    * @return Eigen::VectorXd with the Cartesian Gravity Vector   
    * */
    int ComputeGravityVectorCartesian(const Eigen::VectorXd &q_in, Eigen::VectorXd &gravity_vector);


    

    /*
    * Compute Pseudo Inverse:
    * @param Eigen::MatrixXd of matrix M    
    * @return Eigen::MatrixXd with the pinverse of M   
    * */
    int pinv(const Eigen::MatrixXd &M, Eigen::MatrixXd &Minv);



    /*
    * Compute Euler Transformation Matrix :
    * @param Eigen::Vector3d of euler vector, std::string with euler reprsentation     
    * @return Eigen::Matrix3d with the euler Tf   
    * */
    int eulerTf(const Eigen::Vector3d &eu, Eigen::Matrix3d &Tf, const std::string &representation = "ZYX");




    // Conversions
    /// Convert a KDL vector to an Eigen vector.
    Eigen::Vector3d KDLVec2EigenVec(KDL::Vector const &input)
    {
        return Eigen::Vector3d{input[0], input[1], input[2]};
    }

    /// Convert a KDL rotation to an Eigen quaternion.
    Eigen::Quaterniond KDLRot2EigenQuat(KDL::Rotation const &input)
    {
        Eigen::Quaterniond result;
        input.GetQuaternion(result.x(), result.y(), result.z(), result.w());
        return result;
    }

    /// Convert a KDL rotation to an Eigen rotation matrix.
    Eigen::Matrix3d KDLRot2EigenMat(KDL::Rotation const &input)
    {
        return (Eigen::Matrix3d{} << input(0, 0), input(0, 1), input(0, 2),
                input(1, 0), input(1, 1), input(1, 2),
                input(2, 0), input(2, 1), input(2, 2))
            .finished();
    }

    /// Convert an Eigen vector to a KDL vector.
    KDL::Vector toKdlVector(Eigen::Vector3d const &input)
    {
        return KDL::Vector{input.x(), input.y(), input.z()};
    }

    /// Convert an Eigen vector to a KDL vector.
    KDL::Vector toKdlVector(std::vector<double> const &input)
    {
        return KDL::Vector{input[0], input[1], input[2]};
    }

    /// Convert an Eigen quaternion to a KDL rotation.
    KDL::Rotation toKdlRotation(Eigen::Quaterniond const &input)
    {
        return KDL::Rotation::Quaternion(input.x(), input.y(), input.z(), input.w());
    }

    /// Convert an Eigen rotation matrix to a KDL rotation.
    KDL::Rotation toKdlRotation(Eigen::Matrix3d const &input)
    {
        KDL::Rotation result;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
            {
                result(i, j) = input(i, j);
            }
        return result;
    }

    
    /// Convert an Eigen isometry to a KDL frame.
    KDL::Frame toKdlFrame(const Eigen::Vector3d &position, const Eigen::Matrix3d &rotation)
    {
        return KDL::Frame(toKdlRotation(rotation), toKdlVector(position));
    }

    /// Convert an Eigen vector and Quaternion to a KDL frame.
    KDL::Frame toKdlFrame(const Eigen::Vector3d &position, const Eigen::Quaterniond &rotation)
    {
        return KDL::Frame(toKdlRotation(rotation), toKdlVector(position));
    }


    /// Convert an Eigen isometry to a KDL frame.
    KDL::FrameVel toKdlFrameVel(const Eigen::Vector3d &position, const Eigen::Matrix3d &rotation, const Eigen::Vector3d &vel_linear, const Eigen::Vector3d &vel_angular)
    {
        KDL::Frame frame(toKdlRotation(rotation),toKdlVector(position)); 
        KDL::Twist twist(toKdlVector(vel_linear),toKdlVector(vel_angular));
        return KDL::FrameVel(frame,twist);
    }

    /// Convert an Eigen vector and Quaternion to a KDL frame.
    KDL::FrameVel toKdlFrameVel(const Eigen::Vector3d &position, const Eigen::Quaterniond &rotation, const Eigen::Vector3d &vel_linear, const Eigen::Vector3d &vel_angular)
    {
        KDL::Frame frame(toKdlRotation(rotation),toKdlVector(position)); 
        KDL::Twist twist(toKdlVector(vel_linear),toKdlVector(vel_angular));
        return KDL::FrameVel(frame,twist);
    }

    /// Convert KDL Frame to Eigen Vector position and euler 
    void KDLFrame2EigenVec(const KDL::Frame &frame, Eigen::Vector3d &position, Eigen::Vector3d &euler);

    /// Convert KDL Frame to Eigen vector position and Matri3d rotation matrix
    void KDLFrame2EigenVec(const KDL::Frame &frame, Eigen::Vector3d &position, Eigen::Matrix3d &orientation);

    /// Convert KDL Frame to Eigen vector position and Quaterniond
    void KDLFrame2EigenVec(const KDL::Frame &frame, Eigen::Vector3d &position, Eigen::Quaterniond &orientation);

    /// Convert KDL Frame to Eigen vector position and Matrix
    void KDLFrame2EigenVec(const KDL::Frame &frame, Eigen::MatrixXd &mat);

    /// Convert Eigen Vector to KDL JntArray
    int EigenVect2KDLJnt(const Eigen::VectorXd& q, KDL::JntArray& q_kdl);

    /// KDL JntArray to Convert Eigen Vector
    int KDLJnt2EigenVec(const KDL::JntArray& q_kdl, Eigen::VectorXd& q);

    /// Convert Eigen Vector to KDL JntArrayVel 
    int EigenVect2KDLJntVel(const Eigen::VectorXd& q, const Eigen::VectorXd& q_dot, KDL::JntArrayVel& q_kdl);

    /// Convert KDL JntArrayVel to Eigen Vector
    int KDLJntVel2EigenVec(const KDL::JntArrayVel& q_kdl, Eigen::VectorXd& q, Eigen::VectorXd& q_dot);

    /// Convert Eigen Vector to KDL JntArrayAcc 
    int EigenVect2KDLJntAcc(const Eigen::VectorXd& q, const Eigen::VectorXd& q_dot, const Eigen::VectorXd& q_ddot, KDL::JntArrayAcc& q_kdl);

    /// Eigen Vector Euler to Eigen Matrix
    Eigen::Matrix3d EigenVec2Mat(const Eigen::Vector3d euler);

    // Get
    int getNdof() { return _ndof; }
    int getNlinks() { return _nlinks; }

    KDL::Chain getChain() { return _kdl_chain; }

    std::string getJointName(const int i)
    {
        assertm(i >= 0 && i < _ndof, "Not valid i-th joint");
        return _jnt_names[i];
    }

    std::string getLinkName(const int i)
    {
        assertm(i >= 0 && i < _nlinks, "Not valid i-th link");
        return _link_names[i];
    }
    std::vector<std::string> getJointNames() { return _jnt_names; }
    std::vector<std::string> getLinkNames() { return _link_names; }

private:
    bool _isinitialized;
    KDL::Chain m_chain, _kdl_chain;
    std::string _robot_description, _root_link, _tip_link;
    int _ndof, _nlinks;
    std::vector<std::string> _jnt_names, _link_names;
    KDL::Tree *_tree_ptr;

    bool getTreeFromURDF(KDL::Tree &tree);

    ros::NodeHandle _n;

    // Create Boost pointers for kdl solvers
    // forward kinematics related
    boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fksolver_ptr;
    boost::shared_ptr<KDL::ChainFkSolverVel_recursive> fksolvervel_ptr;
    boost::shared_ptr<KDL::ChainFkSolverAcc> fksolveracc_ptr;
    // Inverse Kinematics
    boost::shared_ptr<KDL::ChainIkSolverPos_NR> iksolver_ptr;
    boost::shared_ptr<KDL::ChainIkSolverVel> iksolvervel_ptr;
    boost::shared_ptr<KDL::ChainIkSolverVel_pinv> iksolvervelpinv_ptr;
    boost::shared_ptr<KDL::ChainIkSolverAcc> iksolveracc_ptr;
    // Jacobian related
    boost::shared_ptr<KDL::ChainJntToJacSolver> jacslover_ptr;
    boost::shared_ptr<KDL::ChainJntToJacDotSolver> jacdotslover_ptr;
    // Dynamics related
    boost::shared_ptr<KDL::ChainIdSolver_RNE> idsolver_ptr;
    boost::shared_ptr<KDL::ChainDynParam> dynparam_ptr;
};

#endif /* KDL_WRAPPER_HPP */