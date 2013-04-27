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
 
    
    puma560.addSegment(Segment(Joint(Joint::RotZ),
                               Frame(Rotation::Identity(),Vector(1.0,0.0,0.0)),
                               RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
                               
    puma560.addSegment(Segment(Joint(Joint::RotZ),
                               Frame(Rotation::Identity(),Vector(1.0,0.0,0.0)),
                               RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
    
    return puma560;
}

void test_chain(Chain & puma) {
    unsigned int ns =  puma.getNrOfSegments();
    unsigned int i = 0;
    /*
    cout << "Angle 0" << endl;
    for( i=0; i < ns; i++ ) {
        Segment seg;
        seg = puma.getSegment(i);
        cout << "Segment " << i << endl;
        cout << seg.pose(0) << endl;
    }
    cout << "Angle 45" << endl;
    for( i=0; i < ns; i++ ) {
        Segment seg;
        seg = puma.getSegment(i);
        cout << "Segment " << i << endl;
        cout << seg.pose(M_PI/4) << endl;
    }*/
    Segment seg;
    puma.getSegment(0,seg);
    cout << "Segment " << 0 << endl;
    cout << seg.pose(0) << endl;
    puma.getSegment(1,seg);
    cout << "Segment " << 1 << endl;
    cout << seg.pose(M_PI/4) << endl;

}

 
int main() 
{

    //Create a KDL::Chain
    Chain puma = OneLinkRobot();
    
    test_chain(puma);
    
    return 0;
    
}
