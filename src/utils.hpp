/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */ 
     
#ifndef RBIPL_UTILS_HPP
#define RBIPL_UTILS_HPP

#include <Eigen/Core>

#include <kdl/rotationalinertia.hpp>
#include <kdl/rigidbodyinertia.hpp>




namespace RBIPL {    
    
    Eigen::Matrix<double,10,1> Vectorize(const KDL::RigidBodyInertia & rbd_inertia);

    
    Eigen::Matrix<double,6,1> vech(const KDL::RotationalInertia & rot_inertia);
    
    KDL::RigidBodyInertia deVectorize(const Eigen::Matrix<double,10,1> & vec);
    KDL::RotationalInertia devech(const Eigen::Matrix<double,6,1> & vec);
    
    Eigen::Matrix3d crossProductMatrix(const KDL::Vector & v);
    
    Eigen::Matrix<double, 6, 6> spatialCrossProductTwistMatrix(const KDL::Twist & v);
    
    Eigen::Matrix<double, 6, 6> spatialCrossProductWrenchMatrix(const KDL::Twist & v);
    
    Eigen::Matrix<double, 6, 6> TwistTransformationMatrix(const KDL::Frame & frame);
    
    Eigen::Matrix<double, 6, 6> WrenchTransformationMatrix(const KDL::Frame & frame);


    Eigen::Matrix<double, 3, 6> rotationalMomentumRegressor(const KDL::Vector & w);


    /**
     * Get the momentum regressor for a given spatial twist.
     *
     * The momentum regressor is the 6x10 matrix such that:
     *      momentumRegressor(v)*Vectorize(I) == I*v
     * 
     * @
     */
    Eigen::Matrix<double, 6, 10> momentumRegressor(const KDL::Twist & v);
    
    Eigen::Matrix<double, 6, 10> netWrenchRegressor(const KDL::Twist & v, const KDL::Twist & a) ;    
    
    Eigen::Matrix<double, 6, 1> toEigen(const KDL::Twist & v);
    
    Eigen::Matrix<double, 6, 1> toEigen(const KDL::Wrench & v);


}
             
#endif
