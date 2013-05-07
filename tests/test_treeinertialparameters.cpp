/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 

#include <chainidsolver_recursive_newton_euler_floating_base.hpp>
#include <treeidsolver_recursive_newton_euler.hpp>

#include <chaininertialparameters.hpp>
#include <treeinertialparameters.hpp>

#include <kdl/kinfam_io.hpp>
#include <kdl/frames_io.hpp>

#include <cstdlib>

#include <iostream>

using namespace KDL;
using namespace Eigen;
using namespace std;
using namespace RBIPL;

double random_double()
{
    return (double)rand()/(double)RAND_MAX;
}

bool MultiplyRegressor(const MatrixXd & regressor, VectorXd & inertial_parameters, JntArray & torques, Wrench & base_force)
{
    Vector f,t;
    if( regressor.cols() != inertial_parameters.rows() ) return false;
    
    VectorXd result = regressor*inertial_parameters;
    
    Map<Vector3d>(t.data) = result.segment(0,3);
    Map<Vector3d>(f.data) = result.segment(3,3);
    base_force = Wrench(f,t);
    
    torques.data = result.segment(6,result.rows()-6);

    return true;
}

Chain Puma560_floating(){
    Chain puma560;
    
   
    puma560.addSegment(Segment("seg1",Joint("jnt1",Joint::None),
                               Frame::DH(0.0203,-M_PI_2,0.15005,0.0),
                                RigidBodyInertia(10,Vector(1,1,1),RotationalInertia(0,7,7,4,7,0))));


    puma560.addSegment(Segment("seg2",Joint("jtn2",Joint::RotZ),
                               Frame::DH(0.4318,0.0,0.0,0.0),
                               RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
    puma560.addSegment(Segment("seg3",Joint("jnt3")));
    puma560.addSegment(Segment("seg4",Joint("jnt4",Joint::RotZ),
                               Frame::DH(0.0203,-M_PI_2,0.15005,0.0),
                               RigidBodyInertia(4.8,Vector(-.0203,-.0141,.070),RotationalInertia(0.066,0.086,0.0125,0,0,0))));
    puma560.addSegment(Segment("seg5",Joint("jnt5",Joint::RotZ),
                               Frame::DH(0.0,M_PI_2,0.4318,0.0),
                               RigidBodyInertia(0.82,Vector(0,.019,0),RotationalInertia(1.8e-3,1.3e-3,1.8e-3,0,0,0))));
    puma560.addSegment(Segment("seg6",Joint("jnt6")));
    puma560.addSegment(Segment("seg7",Joint("jnt7")));
    puma560.addSegment(Segment("seg8",Joint("jnt8",Joint::RotZ),
                               Frame::DH(0.0,-M_PI_2,0.0,0.0),
                               RigidBodyInertia(0.34,Vector::Zero(),RotationalInertia(.3e-3,.4e-3,.3e-3,0,0,0))));
    puma560.addSegment(Segment("seg9",Joint("jnt9",Joint::RotZ),
                               Frame::DH(0.0,0.0,0.0,0.0),
                               RigidBodyInertia(0.09,Vector(0,0,.032),RotationalInertia(.15e-3,0.15e-3,.04e-3,0,0,0))));
    puma560.addSegment(Segment("seg10",Joint("jnt10")));
    return puma560;
}

Chain Puma560(){
    Chain puma560;
    
    puma560.addSegment(Segment("seg2",Joint("jtn2",Joint::RotZ),
                               Frame::DH(0.4318,0.0,0.0,0.0),
                               RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
    puma560.addSegment(Segment("seg3",Joint("jnt3")));
    puma560.addSegment(Segment("seg4",Joint("jnt4",Joint::RotZ),
                               Frame::DH(0.0203,-M_PI_2,0.15005,0.0),
                               RigidBodyInertia(4.8,Vector(-.0203,-.0141,.070),RotationalInertia(0.066,0.086,0.0125,0,0,0))));
    puma560.addSegment(Segment("seg5",Joint("jnt5",Joint::RotZ),
                               Frame::DH(0.0,M_PI_2,0.4318,0.0),
                               RigidBodyInertia(0.82,Vector(0,.019,0),RotationalInertia(1.8e-3,1.3e-3,1.8e-3,0,0,0))));
    puma560.addSegment(Segment("seg6",Joint("jnt6")));
    puma560.addSegment(Segment("seg7",Joint("jnt7")));
    puma560.addSegment(Segment("seg8",Joint("jnt8",Joint::RotZ),
                               Frame::DH(0.0,-M_PI_2,0.0,0.0),
                               RigidBodyInertia(0.34,Vector::Zero(),RotationalInertia(.3e-3,.4e-3,.3e-3,0,0,0))));
    puma560.addSegment(Segment("seg9",Joint("jnt9",Joint::RotZ),
                               Frame::DH(0.0,0.0,0.0,0.0),
                               RigidBodyInertia(0.09,Vector(5,1,.032),RotationalInertia(.15,0.15,.04e-3,0,3,0))));
    puma560.addSegment(Segment("seg10",Joint("jnt10")));
    return puma560;
}


Chain OneLinkRobot() {
    Chain puma560;
 
    puma560.addSegment(Segment(Joint(Joint::None),
                               Frame::Identity(),
                                RigidBodyInertia(10,Vector(1,1,1),RotationalInertia(0,7,7,4,7,0))));

    
    
    puma560.addSegment(Segment(Joint(Joint::RotZ),
                               Frame::DH(0.4318,0.0,0.0,0.0),
                               RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
    
    return puma560;
}

void test_chain(Chain & puma) {
    Tree puma_tree("base_link");
    puma_tree.addChain(puma,"base_link");
    
    unsigned int nj = puma.getNrOfJoints();
    unsigned int ns = puma.getNrOfSegments();
    
    unsigned int i;
    
    //Creating random state
    JntArray q(nj), dq(nj), ddq(nj);
    Twist v_0, a_0; 
    std::vector<Wrench> f(ns);

    for(i = 0; i < ns; i++ ) {
        f[i] = Wrench::Zero();
    }
    
    //Creating results vectors
    std::vector<Wrench> f_0(4);
    std::vector<JntArray> tau(4);
    
    
    enum { CHAIN = 0, CHAIN_REGR = 1, TREE = 2, TREE_REGR = 3};
    
    tau[CHAIN] = JntArray(nj);
    tau[CHAIN_REGR] = JntArray(nj);
    tau[TREE] = JntArray(nj);
    tau[TREE_REGR] = JntArray(nj);

    
    v_0 = Twist(Vector(random_double(),random_double(),random_double()),
                Vector(random_double(),random_double(),random_double()));
                
    a_0 = Twist(Vector(random_double(),random_double(),random_double()),
                Vector(random_double(),random_double(),random_double()));
                
    //cout << "Base a, v" << endl;
    //cout << a_0 << endl;
    //cout << v_0 << endl;
    
    for(i = 0; i < nj; i++ ) {
        q(i) = random_double();
        dq(i) = random_double();
        ddq(i) = random_double();
    }
    
    //Instatiate objects
    ChainIdSolver_RNE_FB rne_solver(puma);
    ChainInertialParameters cip_solver(puma);
    
    TreeIdSolver_RNE trne_solver(puma_tree);
    TreeInertialParameters tip_solver(puma_tree);
    
    
    
    //Solve chain dynamics
    rne_solver.CartToJnt(q,dq,ddq,v_0,a_0,f,tau[CHAIN],f_0[CHAIN]);
    
    //Solve tree dynamics 
    trne_solver.CartToJnt(q,dq,ddq,v_0,a_0,f,tau[TREE],f_0[TREE]);
    
    //for(int i=0;i < f.size(); i++ ) { cout << "f_ext " << i << " " << f[i] << endl; }

    MatrixXd regressor(6+nj,10*ns);
    VectorXd extended_tau(nj);
    VectorXd inertial_param ;
    
    //Solve chain regressor dynamics
    inertial_param = cip_solver.getInertialParameters();
    
    //Get chain regressor
    cip_solver.dynamicsRegressor(q,dq,ddq,v_0,a_0,regressor);
    
    MultiplyRegressor(regressor,inertial_param,tau[CHAIN_REGR],f_0[CHAIN_REGR]);
    
    
    //Solve tree regressor dynamics
    inertial_param = tip_solver.getInertialParameters();
    
    //Get chain regressor
    tip_solver.dynamicsRegressor(q,dq,ddq,v_0,a_0,regressor);
    
    MultiplyRegressor(regressor,inertial_param,tau[TREE_REGR],f_0[TREE_REGR]);
    
    

    cout << "~~~~~~~~~~~~~Dynamics with chain RNE~~~~~~~~~~~~~~~~~~~~~~~" << endl;
    //cout << f_0.torque.Norm() << " " << f_0.force.Norm() << endl;
    cout << f_0[CHAIN] << endl;
    for(i = 0; i < nj; i++ ) { cout << tau[CHAIN](i) << endl; }
    
    cout << "~~~~~~~~~~~~~Dynamics with tree RNE~~~~~~~~~~~~~~~~~~~~~~~" << endl;
    //cout << f_0_tree.torque.Norm() << " " << f_0_tree.force.Norm() << endl;
    cout << f_0[TREE] << endl;
    for(i = 0; i < nj; i++ ) { cout << tau[TREE](i) << endl; }
    
    cout << "~~~~~~~~~~~~~Dynamics with chain regressor~~~~~~~~~~~~~~~~~~~~~~~" << endl;
    //cout << f_0.torque.Norm() << " " << f_0.force.Norm() << endl;
    cout << f_0[CHAIN_REGR] << endl;
    for(i = 0; i < nj; i++ ) { cout << tau[CHAIN_REGR](i) << endl; }
    
    cout << "~~~~~~~~~~~~~Dynamics with tree regressor~~~~~~~~~~~~~~~~~~~~~~~" << endl;
    //cout << f_0_tree.torque.Norm() << " " << f_0_tree.force.Norm() << endl;
    cout << f_0[TREE_REGR] << endl;
    for(i = 0; i < nj; i++ ) { cout << tau[TREE_REGR](i) << endl; }
    

}

 
int main() 
{

    //Create a KDL::Chain
    Chain puma = Puma560();
    Chain puma_fb = Puma560_floating();
    
    //Seed the random number generator
    cout << "fixed base joint" << endl;
    srand(5);
    test_chain(puma);
    
    cout << "Floating base joint" << endl;
    srand(5);
    test_chain(puma_fb);

    
    return 0;
    
}
