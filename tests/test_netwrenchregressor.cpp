/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include <utils.hpp>

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

int main() 
{
    //Seed the random number generator
    srand(13);
    
    Twist a,v;
    
    RigidBodyInertia I = RigidBodyInertia(1,Vector(2,3,4),RotationalInertia(5,6,7,8,9,10));

    
    v = Twist(Vector(1,2,3),
                Vector(4,5,6));
                
    a = Twist(Vector(random_double(),random_double(),random_double()),
                Vector(random_double(),random_double(),random_double()));
    
    //Net wrench regressor test            
    Wrench f = I*a + v*(I*v);
    Matrix<double, 6, 1> f_regr = netWrenchRegressor(v,a)*Vectorize(I);
    
    cout << "NetWrenchRegressor test:" << endl;
    cout << f << endl;
    cout << f_regr << endl;
    
    //MomentumRegressor test 
    f = I*v;
    f_regr = momentumRegressor(v)*Vectorize(I);

    
    cout << "MomentumRegressor test:" << endl;
    cout << f << endl;
    cout << f_regr << endl;
    
    //rotatationalMomentumRegressor test
    RotationalInertia I_3D = I.getRotationalInertia();
    cout << "rotationalMomentumRegressor test" << endl;
    cout << I_3D*v.vel << endl;
    cout << rotationalMomentumRegressor(v.vel)*vech(I_3D) << endl;
    
    //Vectorize test
    Matrix<double, 10, 1> vecI;
    vecI << 1,2,3,4,5,6,7,8,9,10;
    cout << vecI << endl;
    cout << Vectorize(deVectorize(vecI)) << endl;
    
    cout << "spatialWrenchCrossProduct" << endl;
    cout << spatialCrossProductWrenchMatrix(v)*toEigen(f) << endl;
    cout << v*f << endl;
    
    cout << "spatialTwistCrossProduct" << endl;
    cout << spatialCrossProductTwistMatrix(v)*toEigen(a) << endl;
    cout << v*a << endl;
    
    cout << "a" << endl;
    cout << a << endl;
    cout << toEigen(a) << endl;
    
    cout << "f" << endl;
    cout << f << endl;
    cout << toEigen(f) << endl;
    
    cout << "*TransformationMatrix test" << endl;
    Frame X = Frame::DH(0.0203,0.354,0.15005,33.0);
    cout << X*f << endl;
    cout << WrenchTransformationMatrix(X)*toEigen(f) << endl;
    


    

    return 0;
    
}
