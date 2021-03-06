/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include <chainidsolver_recursive_newton_euler_floating_base.hpp>
#include <chaininertialparameters.hpp>

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

Chain Puma560_floating(){
    Chain puma560;
    
   
    puma560.addSegment(Segment(Joint(Joint::None),
                               Frame::DH(0.0203,-M_PI_2,0.15005,0.0),
                                RigidBodyInertia(10,Vector(1,1,1),RotationalInertia(0,7,7,4,7,0))));


    puma560.addSegment(Segment(Joint(Joint::RotZ),
                               Frame::DH(0.4318,0.0,0.0,0.0),
                               RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
    puma560.addSegment(Segment());
    puma560.addSegment(Segment(Joint(Joint::RotZ),
                               Frame::DH(0.0203,-M_PI_2,0.15005,0.0),
                               RigidBodyInertia(4.8,Vector(-.0203,-.0141,.070),RotationalInertia(0.066,0.086,0.0125,0,0,0))));
    puma560.addSegment(Segment(Joint(Joint::RotZ),
                               Frame::DH(0.0,M_PI_2,0.4318,0.0),
                               RigidBodyInertia(0.82,Vector(0,.019,0),RotationalInertia(1.8e-3,1.3e-3,1.8e-3,0,0,0))));
    puma560.addSegment(Segment());
    puma560.addSegment(Segment());
    puma560.addSegment(Segment(Joint(Joint::RotZ),
                               Frame::DH(0.0,-M_PI_2,0.0,0.0),
                               RigidBodyInertia(0.34,Vector::Zero(),RotationalInertia(.3e-3,.4e-3,.3e-3,0,0,0))));
    puma560.addSegment(Segment(Joint(Joint::RotZ),
                               Frame::DH(0.0,0.0,0.0,0.0),
                               RigidBodyInertia(0.09,Vector(0,0,.032),RotationalInertia(.15e-3,0.15e-3,.04e-3,0,0,0))));
    puma560.addSegment(Segment());
    return puma560;
}

Chain Puma560(){
    Chain puma560;
    

    puma560.addSegment(Segment(Joint(Joint::RotZ),
                               Frame::DH(0.4318,0.0,0.0,0.0),
                               RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
    puma560.addSegment(Segment());
    puma560.addSegment(Segment(Joint(Joint::RotZ),
                               Frame::DH(0.0203,-M_PI_2,0.15005,0.0),
                               RigidBodyInertia(4.8,Vector(-.0203,-.0141,.070),RotationalInertia(0.066,0.086,0.0125,0,0,0))));
    puma560.addSegment(Segment(Joint(Joint::RotZ),
                               Frame::DH(0.0,M_PI_2,0.4318,0.0),
                               RigidBodyInertia(0.82,Vector(0,.019,0),RotationalInertia(1.8e-3,1.3e-3,1.8e-3,0,0,0))));
    puma560.addSegment(Segment());
    puma560.addSegment(Segment());
    puma560.addSegment(Segment(Joint(Joint::RotZ),
                               Frame::DH(0.0,-M_PI_2,0.0,0.0),
                               RigidBodyInertia(0.34,Vector::Zero(),RotationalInertia(.3e-3,.4e-3,.3e-3,0,0,0))));
    puma560.addSegment(Segment(Joint(Joint::RotZ),
                               Frame::DH(0.0,0.0,0.0,0.0),
                               RigidBodyInertia(0.09,Vector(0,0,.032),RotationalInertia(.15e-3,0.15e-3,.04e-3,0,0,0))));
    puma560.addSegment(Segment());
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
    unsigned int nj = puma.getNrOfJoints();
    unsigned int ns = puma.getNrOfSegments();
    
    unsigned int i;
    
    //Creating random state
    JntArray q(nj), dq(nj), ddq(nj), tau(nj);
    Twist v_0, a_0; 
    Wrench f_0;
    std::vector<Wrench> f(ns);
    for(i = 0; i < ns; i++ ) {
        f[i] = Wrench::Zero();
    }
    
    
    v_0 = Twist(Vector(random_double(),random_double(),random_double()),
                Vector(random_double(),random_double(),random_double()));
                
    a_0 = Twist(Vector(random_double(),random_double(),random_double()),
                Vector(random_double(),random_double(),random_double()));
                
    cout << "Base a, v" << endl;
    cout << a_0 << endl;
    cout << v_0 << endl;
    
    for(i = 0; i < nj; i++ ) {
        q(i) = random_double();
        dq(i) = random_double();
        ddq(i) = random_double();
    }
    
    //Instatiate objects
    ChainIdSolver_RNE_FB rne_solver(puma);
    ChainInertialParameters cip_solver(puma);
    
    //Solve dynamics
    rne_solver.CartToJnt(q,dq,ddq,v_0,a_0,f,tau,f_0);
    
    VectorXd inertial_param = cip_solver.getInertialParameters();
    MatrixXd regressor(6+nj,10*ns);
    regressor.setZero();
    
    VectorXd extended_tau(nj);
    
    
    //Get regressor
    cip_solver.dynamicsRegressor(q,dq,ddq,v_0,a_0,regressor);

    
    extended_tau = regressor*inertial_param;

    cout << "Dynamics with RNE" << endl;
    cout << f_0.torque.Norm() << " " << f_0.force.Norm() << endl;
    cout << f_0 << endl;
    for(i = 0; i < nj; i++ ) { cout << tau(i) << endl; }
    
    cout << "Dynamics with regressor " << endl;
    cout << extended_tau.segment(0,3).norm() << " " << extended_tau.segment(3,3).norm() << endl;
    cout << extended_tau << endl;
}

 
int main() 
{

    //Create a KDL::Chain
    Chain puma = Puma560();
    Chain puma_fb = Puma560_floating();
    
    //Seed the random number generator
    cout << "Not fixed base joint" << endl;
    srand(5);
    test_chain(puma);
    
    cout << "Fixed base joint" << endl;
    srand(5);
    test_chain(puma_fb);

    
    return 0;
    
}
