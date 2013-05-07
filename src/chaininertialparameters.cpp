/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */



#include "chaininertialparameters.hpp"



#include <Eigen/Core>
#include <Eigen/Dense>

#include <kdl/rigidbodyinertia.hpp>
#include <kdl/chain.hpp>




namespace RBIPL {
    
    void ChainInertialParameters::updateParams()
    {
        Segment seg;
        for(unsigned int i = 0; i < ref_chain.getNrOfSegments(); i++ ) {
            ref_chain.getSegment(i,seg);
            chain_param.segment(i*10,10) = Vectorize(seg.getInertia());
        }
    }
    
    //Reconsider their location !!
    Eigen::VectorXd ChainInertialParameters::getInertialParameters()
    {
        if( chain_param.rows() != 10*ref_chain.getNrOfSegments() ) updateParams();
        return chain_param;
    }
    
    //Reconsider their location !! (not important that they are real time safe)
    bool ChainInertialParameters::changeInertialParameters(const Eigen::VectorXd & new_chain_param,  Chain& new_chain)
    {
        new_chain = Chain();
        
        //assert( new_chain_param.size() == chain_param.size() )
        if( chain_param.rows() != 10*ref_chain.getNrOfSegments() ) updateParams();

        Segment seg;
        for(unsigned int i = 0; i < ref_chain.getNrOfSegments(); i++ ) {
            ref_chain.getSegment(i,seg);
            seg.setInertia(deVectorize(chain_param.segment(i*10,10)));
            new_chain.addSegment(seg);
        }
        return true;
    }
    
    ChainInertialParameters::ChainInertialParameters(Chain & chain) : ref_chain(chain), 
                                                                      nj(ref_chain.getNrOfJoints()),
                                                                      ns(ref_chain.getNrOfSegments()),
                                                                      X(ns),
                                                                      S(ns),
                                                                      v(ns),
                                                                      a(ns),
                                                                      f(ns),
                                                                      has_joint(ns)
    {
        chain_param = Eigen::VectorXd(10*ns);
        updateParams();
    }
    

    int ChainInertialParameters::dynamicsRegressor(  const JntArray &q, 
                            const JntArray &q_dot,
                            const JntArray &q_dotdot,  
                            const Twist& base_velocity, 
                            const Twist& base_acceleration,
                            Eigen::MatrixXd & dynamics_regressor)
    {
        if(q.rows()!=nj || q_dot.rows()!=nj || q_dotdot.rows()!=nj || dynamics_regressor.cols()!=10*ns || dynamics_regressor.rows()!=(6+nj))
            return -1;
        
        unsigned int i;
        unsigned int j=0;
        
        //Kinematic propagation copied by RNE         
        //Sweep from root to leaf
        for(i=0;i<ns;i++){
            double q_,qdot_,qdotdot_;
            Segment segm;
            ref_chain.getSegment(i,segm);
            if(segm.getJoint().getType()!=Joint::None){
                has_joint[i] = true;
                q_=q(j);
                qdot_=q_dot(j);
                qdotdot_=q_dotdot(j);
                j++;
            } else {
                has_joint[i] = false;
                q_=qdot_=qdotdot_=0.0;
            }
            //Calculate segment properties: X,S,vj,cj
            X[i]=segm.pose(q_);//Remark this is the inverse of the
                                                //frame for transformations from 
                                                //the parent to the current coord frame
            //Transform velocity and unit velocity to segment frame
            Twist vj=X[i].M.Inverse(segm.twist(q_,qdot_));
            S[i]=X[i].M.Inverse(segm.twist(q_,1.0));
            //We can take cj=0, see remark section 3.5, page 55 since the unit velocity vector S of our joints is always time constant
            //calculate velocity and acceleration of the segment (in segment coordinates)
            if(i==0){
                v[i]=X[i].Inverse(base_velocity)+vj;
                a[i]=X[i].Inverse(base_acceleration)+S[i]*qdotdot_+v[i]*vj;
            }else{
                v[i]=X[i].Inverse(v[i-1])+vj;
                a[i]=X[i].Inverse(a[i-1])+S[i]*qdotdot_+v[i]*vj;
            }
        }
        
        //just for design, then the loop can be put together
        Frame X_b_i = Frame::Identity();
        Eigen::Matrix<double, 6, 10> netWrenchRegressor_i;

        for(i=0;i<ns;i++) {
            //Updating frame
            if( i != 0 ) X_b_i = X_b_i*X[i];
            
            netWrenchRegressor_i = netWrenchRegressor(v[i],a[i]);
            
            dynamics_regressor.block(0,(int)(10*i),6,10) = WrenchTransformationMatrix(X_b_i)*netWrenchRegressor_i;

            Frame X_j_i;
            unsigned int l = 0;
            for(j=0;j<=i;j++) {
                if( j == 0 )
                    X_j_i = X_b_i;
                else
                    X_j_i = X[j].Inverse()*X_j_i;
                
                if( has_joint[j] ) {
                    dynamics_regressor.block(6+l,10*i,1,10) = 
                        toEigen(S[j]).transpose()*WrenchTransformationMatrix(X_j_i)*netWrenchRegressor_i;
                    l++;
                }
            }
            
            for(j=i+1;j<ns;j++) {
                if( has_joint[j] ) {
                    dynamics_regressor.block((int)(6+l),(int)(10*i),1,10).setZero();
                    l++;
                }
            }
        
            
        }

        return 0;
        
    }
    
}//namespace

