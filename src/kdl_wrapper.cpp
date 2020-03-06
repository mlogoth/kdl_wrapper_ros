#include <kdl_wrapper_ros/kdl_wrapper.hpp>


KDLWrapper::KDLWrapper(std::string robot_description, ros::NodeHandle nh) :
_robot_description(robot_description), _n(nh)
{
    /// Create a KDL Pointer
	_tree_ptr = new KDL::Tree();
	std::cout<< "Robot Description: "<<_robot_description<<std::endl;
	if (!kdl_parser::treeFromParam(_robot_description, *_tree_ptr)){
		std::string err("Exception catched during kdl_kinematics initialization: impossible to retrain the robot description and to create the kdl tree");
		throw std::runtime_error(err);
	}

    std::cout<<"Tree Pointer from URDF is initially constructed!"<<std::endl;
	
}


KDLWrapper::KDLWrapper(KDL::Tree kdl_tree, ros::NodeHandle nh) :
 _n(nh)
{
    /// Create a KDL Pointer
	_tree_ptr = new KDL::Tree();
	*_tree_ptr = kdl_tree;
	
}

/// Distructor
KDLWrapper::~KDLWrapper()
{
    _isinitialized = false;
    fksolver_ptr.reset();
    fksolvervel_ptr.reset();
    fksolveracc_ptr.reset();

    iksolver_ptr.reset();
    iksolvervel_ptr.reset();
    iksolvervelpinv_ptr.reset();
    iksolveracc_ptr.reset();

    jacslover_ptr.reset();
    jacdotslover_ptr.reset();

    idsolver_ptr.reset();
    dynparam_ptr.reset();



}


// costruct the chain and inialize the solvers
bool KDLWrapper::init(const std::string &chain_root, const std::string &chain_tip, bool gravity = true)
{
    
    
    // if (_isinitialized){
	// 	std::string err("Exception catched during kdl_kinematics initialization: Is already initialized!");
	// 	throw std::runtime_error(err);
    //     return false;
	// }

    std::cout<<"Chain from: "<<chain_root<<" to: "<<chain_tip<<std::endl;
    _root_link = chain_root;
    _tip_link = chain_tip;

    // Get Chain
    if(!_tree_ptr->getChain(_root_link,_tip_link,_kdl_chain)){
		std::string err("Exception catched during kdl_kinematics initialization: impossible to get the kdl chain from " + chain_root +" to " + chain_tip);
		throw std::runtime_error(err);
        return false;
	}

    std::cout<<"Chain is constructed!"<<std::endl;
    // Delete Pointer since it is not needed
    delete _tree_ptr;
    std::cout<<"Delete Tree Pointer!"<<std::endl;

    // Number of Joints
    _ndof = _kdl_chain.getNrOfJoints();
    std::cout<<"No of Joints: "<<_ndof<<std::endl;
    // Number of Links
    _nlinks = _kdl_chain.getNrOfSegments()-1;
    std::cout<<"No of links: "<<_nlinks<<std::endl;

    // Resize Joint Names
    _jnt_names.resize(_ndof);
    _link_names.resize(_nlinks);

    // Get Joint and Link  Names
    for(unsigned int i=1;i<_nlinks-1;i++) //NOTE Remove the origin and the last segments
	{
	  
      //std::cout<<"i: "<<i<<std::endl;
      _jnt_names[i-1] = _kdl_chain.getSegment(i+1).getJoint().getName();
      _link_names[i-1] = _kdl_chain.getSegment(i).getName();
      //std::cout<<"Joint: "<<_jnt_names[i-1]<< " | Link: "<<_link_names[i-1]<<std::endl;
      
	}

    _link_names[_nlinks-2] =_kdl_chain.getSegment(_nlinks-1).getName();
    _link_names[_nlinks-1] =_kdl_chain.getSegment(_nlinks).getName();
    //std::cout<<"Link: "<<_link_names[_nlinks-2]<<std::endl;
    //std::cout<<"Link: "<<_link_names[_nlinks-1]<<std::endl;


    // solvers
    fksolver_ptr = boost::make_shared<KDL::ChainFkSolverPos_recursive> (_kdl_chain);
    fksolvervel_ptr = boost::make_shared<KDL::ChainFkSolverVel_recursive> (_kdl_chain);
    //fksolveracc_ptr = boost::make_shared<KDL::ChainFkSolverAcc> (_kdl_chain);
    
    iksolvervelpinv_ptr = boost::make_shared<KDL::ChainIkSolverVel_pinv> (_kdl_chain);
    //iksolveracc_ptr = boost::make_shared<KDL::ChainIkSolverAcc> (_kdl_chain);

    iksolver_ptr = boost::make_shared<KDL::ChainIkSolverPos_NR> (_kdl_chain,*fksolver_ptr,*iksolvervelpinv_ptr,150,1e-5);

    jacslover_ptr = boost::make_shared<KDL::ChainJntToJacSolver> (_kdl_chain);
    jacdotslover_ptr = boost::make_shared<KDL::ChainJntToJacDotSolver> (_kdl_chain);
    // Dynamics

    KDL::Vector _gravity;
    _gravity = KDL::Vector(0.0,0.0,0.0);
    if (gravity) {
        _gravity(2)= -9.81;
    }
    
    idsolver_ptr = boost::make_shared<KDL::ChainIdSolver_RNE> (_kdl_chain,_gravity);
    dynparam_ptr = boost::make_shared<KDL::ChainDynParam> (_kdl_chain,_gravity);

    // Set init true
    _isinitialized = true;

    return true;
}



/*
* Forward Kinematics 
*/

int  KDLWrapper::ComputeFK(const Eigen::VectorXd &q_in, Eigen::Vector3d &position, Eigen::Matrix3d &orientation)
{
    assertm(q_in.size()>=_ndof,"Compute Forward Kinematics Failed");
    KDL::Frame out;
    KDL::JntArray q_kdl(_ndof);
    // Eigen to KDL
    KDLWrapper::EigenVect2KDLJnt(q_in,q_kdl);
    // Compute FK
    fksolver_ptr->JntToCart(q_kdl,out);
    // KDL to Eigen
    position = KDLWrapper::KDLVec2EigenVec(out.p);
    orientation = KDLWrapper::KDLRot2EigenMat(out.M);
    return true;
}


int  KDLWrapper::ComputeFK(const Eigen::VectorXd &q_in, Eigen::VectorXd &pose_pos)
{
    Eigen::Vector3d _position;
    Eigen::Vector3d _eulerXYZ;
    Eigen::Quaterniond _orientation;
    KDLWrapper::ComputeFK(q_in,_position,_orientation);
    _eulerXYZ = _orientation.toRotationMatrix().eulerAngles(2,1,0);
    pose_pos.resize(6);
    pose_pos << _position, _eulerXYZ[2],_eulerXYZ[1],_eulerXYZ[0];
    return true;
}


int  KDLWrapper::ComputeFK(const std::vector<double> &q_in, Eigen::Vector3d &position, Eigen::Matrix3d &orientation)
{
    assertm(q_in.size()>=_ndof,"Compute Forward Kinematics Failed");
    Eigen::VectorXd qq = Eigen::VectorXd::Map(q_in.data(),q_in.size());
    KDLWrapper::ComputeFK(qq,position,orientation);
    return true;
}

int  KDLWrapper::ComputeFK(const Eigen::VectorXd &q_in, Eigen::Vector3d &position, Eigen::Quaterniond &orientation)
{
    Eigen::Matrix3d mat;
    KDLWrapper::ComputeFK(q_in,position,mat);
    Eigen::Quaterniond quat(mat);
    orientation = quat;
    return true;
}


int  KDLWrapper::ComputeFK(const std::vector<double> &q_in,  Eigen::VectorXd &pose_pos)
{
    assertm(q_in.size()>=_ndof,"Compute Forward Kinematics Failed");
    Eigen::VectorXd qq = Eigen::VectorXd::Map(q_in.data(),q_in.size());
    KDLWrapper::ComputeFK(qq,pose_pos);
    return true;
}

/*
* Forward Kinematics Velocity
*/

int  KDLWrapper::ComputeFKVel(const Eigen::VectorXd &q_in, const Eigen::VectorXd &qdot_in, Eigen::VectorXd &v_out, Eigen::Vector3d &p_out, Eigen::Matrix3d &ori_out)
{
    assertm(q_in.size()>=_ndof,"Compute Forward Kinematics Velocity Failed");
    assertm(qdot_in.size()>=_ndof,"Compute Forward Kinematics Velocity Failed");
    KDL::JntArrayVel qq(_ndof);
    KDL::FrameVel out;
    KDL::Frame out_p;
    // convert to kdl
    KDLWrapper::EigenVect2KDLJntVel(q_in,qdot_in,qq);
    //compute
    fksolvervel_ptr->JntToCart(qq,out);
    //convert to eigen
    p_out = KDLWrapper::KDLVec2EigenVec(out.GetFrame().p);
    ori_out = KDLWrapper::KDLRot2EigenMat(out.GetFrame().M);
    v_out.resize(6);
    v_out<< KDLWrapper::KDLVec2EigenVec(out.GetTwist().vel), KDLWrapper::KDLVec2EigenVec(out.GetTwist().rot);
    return true;
}


int  KDLWrapper::ComputeFKVel(const std::vector<double> &q_in, const std::vector<double> &qdot_in, Eigen::VectorXd &v_out, Eigen::Vector3d &p_out, Eigen::Matrix3d &ori_out)
{
    assertm(q_in.size()>=_ndof,"Compute Forward Kinematics Velocity Failed");
    assertm(qdot_in.size()>=_ndof,"Compute Forward Kinematics Velocity Failed");
    Eigen::VectorXd qq = Eigen::VectorXd::Map(q_in.data(),q_in.size());
    Eigen::VectorXd qqdot = Eigen::VectorXd::Map(qdot_in.data(),qdot_in.size());
    KDLWrapper::ComputeFKVel(qq,qqdot,v_out,p_out,ori_out);
    return true;
}


/*
* Inverse Kinematics
*/

int KDLWrapper::ComputeIK(const Eigen::VectorXd &q_in, const Eigen::VectorXd &pose_in, Eigen::VectorXd &q_out)
{
    assertm(q_in.size()>=_ndof,"Compute Inverse Kinematics Failed");
    assertm(pose_in.size()>=6,"Compute Inverse Kinematics Failed-Euler");
    KDL::JntArray qq,qq_out;
    // Convert to KDL
    KDLWrapper::EigenVect2KDLJnt(q_in,qq);
    KDL::Frame frame = KDLWrapper::toKdlFrame(Eigen::Vector3d(pose_in[0],pose_in[1],pose_in[2]),KDLWrapper::EigenVec2Mat(Eigen::Vector3d(pose_in[3],pose_in[4],pose_in[5])));
    //solver
    iksolver_ptr->CartToJnt(qq,frame,qq_out);
    // Convert to Eigen
    KDLWrapper::KDLJnt2EigenVec(qq_out,q_out);
    return true;
}


int KDLWrapper::ComputeIK(const Eigen::VectorXd &q_in, const Eigen::Vector3d &position_in, const Eigen::Matrix3d &rotation_in, Eigen::VectorXd &q_out)
{
    assertm(q_in.size()>=_ndof,"Compute Inverse Kinematics  Failed");
    KDL::JntArray qq,qq_out;
    // Convert to KDL
    KDLWrapper::EigenVect2KDLJnt(q_in,qq);
    KDL::Frame frame = KDLWrapper::toKdlFrame(position_in,rotation_in);
    //solver
    iksolver_ptr->CartToJnt(qq,frame,qq_out);
    // Convert to Eigen
    KDLWrapper::KDLJnt2EigenVec(qq_out,q_out);
    return true;
}


int KDLWrapper::ComputeIK(const Eigen::VectorXd &q_in, const Eigen::Vector3d &position_in, const Eigen::Quaterniond &rotation_in, Eigen::VectorXd &q_out)
{
    assertm(q_in.size()>=_ndof,"Compute Inverse Kinematics  Failed");
    KDL::JntArray qq,qq_out;
    // Convert to KDL
    KDLWrapper::EigenVect2KDLJnt(q_in,qq);
    KDL::Frame frame = KDLWrapper::toKdlFrame(position_in,rotation_in);
    //solver
    iksolver_ptr->CartToJnt(qq,frame,qq_out);
    // Convert to Eigen
    KDLWrapper::KDLJnt2EigenVec(qq_out,q_out);
    return true;
}


/*
* Inverse Kinematics Vel
*/

int KDLWrapper::ComputeIKVel(const Eigen::VectorXd &q_in, const Eigen::VectorXd &v_in, Eigen::VectorXd &qdot_out)
{
    
    assertm(q_in.size()>=_ndof,"Compute Inverse Kinematics Velocity Failed");
    assertm(v_in.size()>=6,"Compute Inverse Kinematics Velocity Failed");
    // 
    KDL::JntArray qq(q_in.size());
    KDL::JntArrayVel qdot_kdl(q_in.size());
    KDLWrapper::EigenVect2KDLJnt(q_in,qq);
    Eigen::Matrix3d orientation;
    Eigen::Vector3d position;
    KDLWrapper::ComputeFK(q_in,position,orientation);
    KDL::FrameVel frame_vel = toKdlFrameVel(position,orientation,Eigen::Vector3d(v_in[0],v_in[1],v_in[2]),Eigen::Vector3d(v_in[3],v_in[4],v_in[5]));

    iksolvervelpinv_ptr->CartToJnt(qq,frame_vel,qdot_kdl);

    Eigen::VectorXd qout;
    KDLWrapper::KDLJntVel2EigenVec(qdot_kdl,qout,qdot_out);
    return true;
}
int KDLWrapper::ComputeIKVel(const std::vector<double> &q_in, const std::vector<double> &v_in, Eigen::VectorXd &qdot_out)
{
    assertm(q_in.size()>=_ndof,"Compute Inverse Kinematics Velocity Failed");
    assertm(v_in.size()>=6,"Compute Inverse Kinematics Velocity Failed");
    Eigen::VectorXd qq = Eigen::VectorXd::Map(q_in.data(),q_in.size());
    Eigen::VectorXd vel = Eigen::VectorXd::Map(v_in.data(),v_in.size());
    KDLWrapper::ComputeIKVel(qq,vel,qdot_out);
    return true;
}


/*
* Jacobian
*/

int KDLWrapper::ComputeJac(const Eigen::VectorXd &q_in, Eigen::MatrixXd &jacobian)
{
    assertm(q_in.size()>=_ndof,"Compute Jacobian Failed");
    KDL::JntArray q_kdl(q_in.size());
    KDL::Jacobian Jac(q_in.size());
    KDLWrapper::EigenVect2KDLJnt(q_in,q_kdl);
    jacslover_ptr->JntToJac(q_kdl,Jac);
    jacobian = Jac.data;
    return true;
}

int KDLWrapper::ComputeJac(const std::vector<double> &q_in, Eigen::MatrixXd &jacobian)
{
    Eigen::VectorXd qq = Eigen::VectorXd::Map(q_in.data(),q_in.size());
    KDLWrapper::ComputeJac(qq,jacobian);
    return true;
}


/*
* Time Derivative of Jacobian
*/

int KDLWrapper::ComputeJacDot(const Eigen::VectorXd &q_in, const Eigen::VectorXd &qdot_in, Eigen::MatrixXd &jacobian)
{
    
    KDL::JntArrayVel qkdl_vel_in(qdot_in.size());
    KDL::Jacobian Jac(q_in.size());
    
    KDLWrapper::EigenVect2KDLJntVel(q_in,qdot_in,qkdl_vel_in);
    jacdotslover_ptr->JntToJacDot(qkdl_vel_in,Jac);
    jacobian = Jac.data;

    return true;
}

int KDLWrapper::ComputeJacDot(const std::vector<double> &q_in, const std::vector<double> &qdot_in, Eigen::MatrixXd &jacobian)
{
    Eigen::VectorXd qq = Eigen::VectorXd::Map(q_in.data(),q_in.size());
    Eigen::VectorXd qqdot = Eigen::VectorXd::Map(qdot_in.data(),qdot_in.size());
    KDLWrapper::ComputeJacDot(qq,qqdot,jacobian);
    return true;
}



/* 
* Rigid Body Dynamics Parameters
*/

int KDLWrapper::ComputeMassMatrix(const Eigen::VectorXd &q_in, Eigen::MatrixXd &Mass_mat)
{
    KDL::JntArray q(q_in.size());
    KDL::JntSpaceInertiaMatrix Hmap(q_in.size());
    KDLWrapper::EigenVect2KDLJnt(q_in,q);
    dynparam_ptr->JntToMass(q,Hmap);
    Mass_mat = Hmap.data;
    return true;
}


int KDLWrapper::ComputeCoriolisVector(const Eigen::VectorXd &q_in,const Eigen::VectorXd &qdot_in, Eigen::VectorXd &coriolis_vector)
{
    
    KDL::JntArray cvec(q_in.size()),q(q_in.size()),q_dot(qdot_in.size());
    KDLWrapper::EigenVect2KDLJnt(q_in,q);
    KDLWrapper::EigenVect2KDLJnt(qdot_in,q_dot);
    dynparam_ptr->JntToCoriolis(q,q_dot,cvec);
    KDLWrapper::KDLJnt2EigenVec(cvec,coriolis_vector);
    return true;
}

int KDLWrapper::ComputeGravityVector(const Eigen::VectorXd &q_in, Eigen::VectorXd &gravity_vector)
{
    
    KDL::JntArray grav(q_in.size()),q(q_in.size());
    KDLWrapper::EigenVect2KDLJnt(q_in,q);
    dynparam_ptr->JntToGravity(q,grav);
    KDLWrapper::KDLJnt2EigenVec(grav,gravity_vector);
    return true;
}

int KDLWrapper::ComputeInverseDynamics(const Eigen::VectorXd &q_in, const Eigen::VectorXd &qdot_in, const Eigen::VectorXd &qddot_in, const  std::vector<Eigen::VectorXd> &fext, Eigen::VectorXd &torques)
{
    
    assertm(fext.size()>=_nlinks,"External forces acting on the links");
    KDL::Wrenches wrenches(fext.size());
    for (unsigned int i=0;i<fext.size();i++)
    {
        Eigen::VectorXd fi = fext[i];
        wrenches[i].force = KDLWrapper::toKdlVector(Eigen::Vector3d(fi[0],fi[1],fi[2]));
        wrenches[i].torque = KDLWrapper::toKdlVector(Eigen::Vector3d(fi[3],fi[4],fi[5]));
    }
    KDL::JntArray q(q_in.size()),qdot(qdot_in.size()),qddot(qddot_in.size()),trq(q_in.size());
    KDLWrapper::EigenVect2KDLJnt(q_in,q);
    KDLWrapper::EigenVect2KDLJnt(qdot_in,qdot);
    KDLWrapper::EigenVect2KDLJnt(qddot_in,qddot);
    
    idsolver_ptr->CartToJnt(q,qdot,qddot, wrenches, trq);

    KDLWrapper::KDLJnt2EigenVec(trq, torques);

    return true;

}


int KDLWrapper::ComputeInverseDynamics(const Eigen::VectorXd &q_in, const Eigen::VectorXd &qdot_in, const Eigen::VectorXd &qddot_in, Eigen::VectorXd &torques)
{
    
    std::vector<Eigen::VectorXd> fext;
    fext.resize(_nlinks);
    for (unsigned int i=0;i<_nlinks;i++)
    {
        fext[i].resize(6);
        fext[i]<< 0.0,0.0, 0.0,0.0, 0.0,0.0;
    }
    KDLWrapper::ComputeInverseDynamics(q_in,qdot_in,qddot_in,fext,torques);

    return true;

} 

/// Cartesian Space Dynamics

int KDLWrapper::ComputeMassMatrixCartesian(const Eigen::VectorXd &q_in, Eigen::MatrixXd &Mass_mat)
{
    Eigen::MatrixXd Mq(_ndof,_ndof), Jac(6,_ndof), Jac_inv(_ndof,6);
    KDLWrapper::ComputeMassMatrix(q_in,Mq);
    KDLWrapper::ComputeJac(q_in,Jac);
    KDLWrapper::pinv(Jac,Jac_inv);
    //J^+T * M * J^+
    Mass_mat = Jac_inv.transpose()*Mq*Jac_inv;
    return true;
    
}

int KDLWrapper::ComputeCoriolisVectorCartesian(const Eigen::VectorXd &q_in,const Eigen::VectorXd &qdot_in, Eigen::VectorXd &coriolis_vector)
{
    Eigen::MatrixXd Mq(_ndof,_ndof), Jac(6,_ndof), Jacdot(6,_ndof),Jac_inv(_ndof,6);
    Eigen::VectorXd cq(_ndof);
    KDLWrapper::ComputeMassMatrix(q_in,Mq);
    KDLWrapper::ComputeCoriolisVector(q_in,qdot_in,cq);
    KDLWrapper::ComputeJac(q_in,Jac);
    KDLWrapper::ComputeJacDot(q_in,qdot_in,Jacdot);
    KDLWrapper::pinv(Jac,Jac_inv);
    //J^+T * Cq - J^+T*Mq*J^+*Jdot*qdot
    coriolis_vector = Jac_inv.transpose()*cq - Jac_inv.transpose()*Mq*Jac_inv*Jacdot*qdot_in;
    return true;
    
}

int KDLWrapper::ComputeGravityVectorCartesian(const Eigen::VectorXd &q_in, Eigen::VectorXd &gravity_vector)
{
    Eigen::MatrixXd Jac(6,_ndof), Jac_inv(_ndof,6);
    Eigen::VectorXd gq(_ndof);
    KDLWrapper::ComputeJac(q_in,Jac);
    KDLWrapper::ComputeGravityVector(q_in,gq);
    KDLWrapper::pinv(Jac,Jac_inv);
    // J^+T * gq
    gravity_vector = Jac_inv.transpose()*gq;
    return true;

}


/*
* Mathematical Tools
*/

int KDLWrapper::pinv(const Eigen::MatrixXd &M, Eigen::MatrixXd &Minv)
{
    assertm(M.cols()>=M.rows(),"Compute Pseudo Inverse: M columns must be greater than its rows");
    double tolerance = 1e-4;
    auto svd = M.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::MatrixXd singularValuesInv(M.cols(), M.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = Eigen::MatrixXd::Scalar{1} / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = Eigen::MatrixXd::Scalar{0};
        }
    }
    Minv =  svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
    return true;
}

int KDLWrapper::eulerTf(const Eigen::Vector3d &eu, Eigen::Matrix3d &Tf, const std::string &representation)
{
    if (!representation.compare("ZYX")) {
        Tf.row(0) << 0, -sin(eu[2]), cos(eu[1])*cos(eu[2]);
        Tf.row(1) << 0,  cos(eu[2]), cos(eu[1])*sin(eu[2]);
        Tf.row(2) << 1,           0,           -sin(eu[2]);
        return 1;
    }

    else if (!representation.compare("XYZ")) {
        Tf.row(0) << 1,           0,            sin(eu[1]);
        Tf.row(1) << 0,  cos(eu[0]), -sin(eu[0])*cos(eu[1]);
        Tf.row(2) << 0,  sin(eu[0]),  cos(eu[0])*cos(eu[1]);
        return 1;
    }

    else{
        std::cerr << "KDLWrapper: Uknown representation of euler angles\n";
        std::terminate();
    }
}

/// Conversions
int KDLWrapper::EigenVect2KDLJnt(const Eigen::VectorXd& q, KDL::JntArray& q_kdl)
{
    
    assertm(q.size() >=_ndof,"Eigen Vector to KDL Joint Array Conversion Failed");
    //assertm(q_kdl.rows() >=_ndof,"Eigen Vector to KDL Joint Array Conversion Failed");
    q_kdl.resize(q.size());
    for(int i = 0; i<_ndof; i++)
	    q_kdl(i) = q[i];
    return true;
}

int KDLWrapper::KDLJnt2EigenVec(const KDL::JntArray& q_kdl, Eigen::VectorXd& q)
{
    
    assertm(q_kdl.rows() >=_ndof,"Eigen Vector to KDL Joint Array Conversion Failed");
    q.resize(_ndof);
    for(int i = 0; i<_ndof; i++)
	    q[i] =  q_kdl(i) ;
    return true;
}


int KDLWrapper::EigenVect2KDLJntVel(const Eigen::VectorXd& q, const Eigen::VectorXd& q_dot, KDL::JntArrayVel& q_kdl)
{
    
    assertm(q.size() >=_ndof,"Eigen Vector to KDL Joint Array Conversion Failed");
    assertm(q_dot.size() >=_ndof,"Eigen Vector to KDL Joint Array Conversion Failed");
    assertm(q_kdl.value().rows() >=_ndof,"Eigen Vector to KDL Joint Array Conversion Failed");
    
    KDL::JntArray _qkdl(q.size());
    KDL::JntArray _qdotkdl(q_dot.size());
    
    KDLWrapper::EigenVect2KDLJnt(q,_qkdl);
    KDLWrapper::EigenVect2KDLJnt(q_dot,_qdotkdl);

    KDL::JntArrayVel out(_qkdl,_qdotkdl);

    q_kdl = out;

    return true;

}


int KDLWrapper::KDLJntVel2EigenVec(const KDL::JntArrayVel& q_kdl, Eigen::VectorXd& q, Eigen::VectorXd& q_dot)
{
    
    assertm(q.size() >=_ndof,"Eigen Vector to KDL Joint Array Conversion Failed");
    assertm(q_dot.size() >=_ndof,"Eigen Vector to KDL Joint Array Conversion Failed");
    assertm(q_kdl.value().rows() >=_ndof,"Eigen Vector to KDL Joint Array Conversion Failed");
    
    q.resize(q_kdl.value().rows());
    q_dot.resize(q_kdl.deriv().rows());
    
    KDLWrapper::KDLJnt2EigenVec(q_kdl.value(),q);
    KDLWrapper::KDLJnt2EigenVec(q_kdl.deriv(),q_dot);

    return true;

}

int KDLWrapper::EigenVect2KDLJntAcc(const Eigen::VectorXd& q, const Eigen::VectorXd& q_dot, const Eigen::VectorXd& q_ddot, KDL::JntArrayAcc& q_kdl)
{
    
    assertm(q.size() >=_ndof,"Eigen Vector to KDL Joint Array Conversion Failed");
    assertm(q_dot.size() >=_ndof,"Eigen Vector to KDL Joint Array Conversion Failed");
    assertm(q_ddot.size() >=_ndof,"Eigen Vector to KDL Joint Array Conversion Failed");
    assertm(q_kdl.value().rows() >=_ndof,"Eigen Vector to KDL Joint Array Conversion Failed");
    
    KDL::JntArray _qkdl(q.size());
    KDL::JntArray _qdotkdl(q_dot.size());
    KDL::JntArray _qddotkdl(q_ddot.size());
    
    KDLWrapper::EigenVect2KDLJnt(q,_qkdl);
    KDLWrapper::EigenVect2KDLJnt(q_dot,_qdotkdl);
    KDLWrapper::EigenVect2KDLJnt(q_ddot,_qddotkdl);

    KDL::JntArrayAcc out(_qkdl,_qdotkdl,_qddotkdl);

    q_kdl = out;

    return true;

}

Eigen::Matrix3d KDLWrapper::EigenVec2Mat(const Eigen::Vector3d euler)
{
    Eigen::Matrix3d m;
    m =  Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) 
                        * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
    return m; 
}
