/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */ 
 
#include "utils.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <kdl/rigidbodyinertia.hpp>


using namespace Eigen;
using namespace KDL;

namespace RBIPL {
    Matrix<double,6,1> vech(const RotationalInertia & rot_inertia)
    {
        Matrix<double,6,1> ret;
        
        //The rotational inertia is stored in inertial parameters as 
        //Ixx Ixy Ixz Iyy Iyz Izz
        ret <<  rot_inertia.data[0], 
                rot_inertia.data[1], 
                rot_inertia.data[2],
                rot_inertia.data[4],
                rot_inertia.data[5],
                rot_inertia.data[8];
                
        return ret;
    }
    
    Matrix<double,10,1> Vectorize(const RigidBodyInertia & rbd_inertia) 
    {
         Matrix<double,10,1> ret;
         
         Vector MCOG = rbd_inertia.getMass()*rbd_inertia.getCOG();
         
         //In RigidBodyInertia, the rotational inertia is store with 
         //respect to the link frame of refence, not the COG one
         
         ret << rbd_inertia.getMass(),
                Map<Vector3d>(MCOG.data),
                vech(rbd_inertia.getRotationalInertia());
         
         return ret;
    }
    
    RotationalInertia devech(const Eigen::Matrix<double,6,1> & vec)
    {
        //In the RotationalInertia initializer, the order is 
        //Ixx, Iyy, Izz, Ixy, Ixz, Iyz
        return RotationalInertia(vec[0],vec[3],vec[5],vec[1],vec[2],vec[4]);
    }
    
    
    
    RigidBodyInertia deVectorize(const Eigen::Matrix<double,10,1> & vec)
    {
        Vector COG;
        if( vec[0] != 0 ) {
            COG = Vector(vec[1],vec[2],vec[3]);
        
            COG = COG/vec[0];
        } else {
            COG = Vector(0.0,0.0,0.0);
        }
        
        Vector3d vCOG = Map<Vector3d>(COG.data);
        
        
        //In the constructor, the request rotational inertia is the one w.r.t. the COG
        RotationalInertia I_o = devech(vec.tail<6>());
        RotationalInertia I_c;
        Map<Matrix3d>(I_c.data) = Map<Matrix3d>(I_o.data)+
                                        vec[0]*(vCOG*vCOG.transpose()-vCOG.dot(vCOG)*Matrix3d::Identity());
        
        
        return RigidBodyInertia(vec[0],COG,I_c);
    }
    

    Matrix3d crossProductMatrix(const Vector & v)
    {
        Matrix3d ret;
        
        ret <<     0, -v[2],  v[1],
                v[2],     0, -v[0],
               -v[1],  v[0],     0;
        
        return ret;
    }
    
    Matrix<double, 6, 6>  spatialCrossProductTwistMatrix(const Twist & v)
    {
        Matrix<double, 6, 6>  ret;
        
        ret << crossProductMatrix(v.rot),          Matrix3d::Zero(),
               crossProductMatrix(v.vel), crossProductMatrix(v.rot);
        
        return ret;
    }
    
    Matrix<double, 6, 6> spatialCrossProductWrenchMatrix(const Twist & v)
    {
        Matrix<double, 6, 6> ret;
        
        ret << crossProductMatrix(v.rot), crossProductMatrix(v.vel),
                 Matrix3d::Zero(), crossProductMatrix(v.rot);
               
                     
        return ret;
    }
    
    Matrix<double, 3, 6> rotationalMomentumRegressor(const Vector & w)
    {
        Matrix<double, 3, 6> ret;
        
        ret << w[0], w[1], w[2],    0,    0,    0,
                  0, w[0],    0, w[1], w[2],    0,
                  0,    0, w[0],    0, w[1], w[2];
        
        return ret;
    }
    
    Matrix<double, 6, 10> momentumRegressor(const Twist & v) 
    {
        Matrix<double, 6, 10> ret;

        ret <<          Vector3d::Zero(), -crossProductMatrix(v.vel), rotationalMomentumRegressor(v.rot),
               Map<const Vector3d>(v.vel.data),  crossProductMatrix(v.rot),       Matrix<double, 3, 6>::Zero();                  
        
        return ret;
    }
    
    Matrix<double, 6, 10> netWrenchRegressor(const Twist & v, const Twist & a) 
    {
        return momentumRegressor(a)+spatialCrossProductWrenchMatrix(v)*momentumRegressor(v);
    }
    
    
    Matrix<double, 6, 6> TwistTransformationMatrix(const KDL::Frame & frame) 
    {
        Matrix<double, 6, 6> ret;
        
        ret <<                              Map<const Matrix<double,3,3,RowMajor> >(frame.M.data),                  Matrix3d::Zero(),
                crossProductMatrix(frame.p)*Map<const Matrix<double,3,3,RowMajor> >(frame.M.data), Map<const Matrix<double,3,3,RowMajor> >(frame.M.data);
        
        return ret;
        
    }
    
    Matrix<double, 6, 6> WrenchTransformationMatrix(const KDL::Frame & frame)
    {
        Matrix<double, 6, 6> ret;
        
        ret <<  Map<const Matrix<double,3,3,RowMajor> >(frame.M.data), crossProductMatrix(frame.p)*Map<const Matrix<double,3,3,RowMajor> >(frame.M.data),
                           Matrix3d::Zero(),                              Map<const Matrix<double,3,3,RowMajor> >(frame.M.data);
               
        return ret;
    }
    
    Matrix<double, 6, 1> toEigen(const KDL::Twist & v) 
    {
        Matrix<double, 6, 1> ret;
        
        ret << Map<const Vector3d>(v.rot.data), 
               Map<const Vector3d>(v.vel.data);
        
        return ret;
    }
    
    Matrix<double, 6, 1> toEigen(const KDL::Wrench & f) 
    {
        Matrix<double, 6, 1> ret;
        
        ret << Map<const Vector3d>(f.torque.data), 
               Map<const Vector3d>(f.force.data);
        
        return ret;
    }

             
}
