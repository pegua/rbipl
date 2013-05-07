/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */


#include "treeinertialparameters.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <kdl/rigidbodyinertia.hpp>
#include <kdl/tree.hpp>



namespace RBIPL {

    void TreeInertialParameters::updateParams()
    {
        Segment seg;
        for(unsigned int i = 0; i < ns; i++ ) {
            seg = seg_vector[i]->second.segment;
            tree_param.segment(i*10,10) = Vectorize(seg.getInertia());
        }
    }
    
    Eigen::VectorXd TreeInertialParameters::getInertialParameters()
    {
        if( tree_param.rows() != 10*ns ) updateParams();
        return tree_param;
    }
    
    bool TreeInertialParameters::changeInertialParameters(const Eigen::VectorXd & new_chain_param,  Tree& new_tree)
    {
        SegmentMap::const_iterator root;
        ref_tree.getRootSegment(root);
        new_tree = Tree(root->first);
        return changeInertialParametersRecursive(new_chain_param,new_tree,root,root->first);
    }
    
    //code modified from bool Tree::addTreeRecursive(..)
    bool TreeInertialParameters::changeInertialParametersRecursive(const Eigen::VectorXd & new_chain_param, Tree & new_tree, SegmentMap::const_iterator root, const std::string& hook_name) 
    {
        //Working segment object
        Segment seg;
        //get iterator for root-segment
        SegmentMap::const_iterator child;
        //try to add all of root's children
        for (unsigned int i = 0; i < root->second.children.size(); i++) {
            child = root->second.children[i];
            //Modify segment
            seg = child->second.segment;
            seg.setInertia(deVectorize(tree_param.segment(serial.getLinkId(child->first)*10,10)));
            
            //Try to add the modified child
            if (new_tree.addSegment(seg, hook_name)) {
                //if child is added, add all the child's children
                if (!(this->changeInertialParametersRecursive(new_chain_param,new_tree,child, child->first)))
                    //if it didn't work, return false
                    return false;
            } else
                //If the child could not be added, return false
                return false;
        }
        return true;
    }
    
    TreeInertialParameters::TreeInertialParameters(Tree & tree,Vector grav,const TreeSerialization & serialization) : ref_tree(tree), 
                                                                      nj(ref_tree.getNrOfJoints()),
                                                                      ns(ref_tree.getNrOfSegments()),
                                                                      X(ns),
                                                                      S(ns),
                                                                      v(ns),
                                                                      a(ns),
                                                                      f(ns),
                                                                      X_b(ns),
                                                                      serial(serialization)
    {
        tree_param = Eigen::VectorXd(10*ns);
        updateParams();
        
        //should compile only in debug mode
        if(!serial.is_consistent(ref_tree)) {
            serial = TreeSerialization(ref_tree);
        }
        
        //Get root name
		root_name = tree.getRootSegment()->first;
		
        //Initializing gravitational acceleration (if any)
        ag=-Twist(grav,Vector::Zero());
        
        //the deprecated method is more efficient
        const SegmentMap& sm = tree.getSegments();
        
        mu_root.resize(0);
        mu.resize(ns,std::vector<unsigned int>(0));
        lambda.resize(ns);
        
        link2joint.resize(tree.getNrOfSegments(),tree.getNrOfSegments());
                
        seg_vector.resize(tree.getNrOfSegments());
        
        
        //create necessary vectors
        SegmentMap::const_iterator root, i;
        
        tree.getRootSegment(root);
        for( unsigned int j=0; j < root->second.children.size(); j++ ) {
            mu_root.push_back(serial.getLinkId(root->second.children[j]->first));
        }
        
        for( SegmentMap::const_iterator i=sm.begin(); i!=sm.end(); ++i ) {
            if( i != root ) {
                unsigned int i_index = serial.getLinkId(i->first);
                seg_vector[i_index] = i;
                
                for( unsigned int j=0; j < i->second.children.size(); j++ ) {
                    mu[i_index].push_back(serial.getLinkId(i->second.children[j]->first));
                }
                
                if( i->second.segment.getJoint().getType() != Joint::None ) {
                    link2joint[i_index] = serial.getJointId(i->first);
                }
                
                if( i->second.parent == root ) {
                    lambda[i_index] = -1;
                } else {
                    lambda[i_index] = serial.getLinkId(i->second.parent->first);
                }
                
            }
		}
        
        //As the order of the recursion is the same, it is calculated only at configuration
        std::vector<unsigned int> index_stack;
        
        index_stack.reserve(tree.getNrOfSegments());
        recursion_order.reserve(tree.getNrOfSegments());
        
        index_stack.clear();
        recursion_order.clear();
        
        for( unsigned int j=0; j < mu_root.size(); j++ ) {
            index_stack.push_back(mu_root[j]);
        }
        
        while( !index_stack.empty() ) {
            
            unsigned int curr_index = index_stack.back();
            index_stack.pop_back();
            
            recursion_order.push_back(curr_index);
            
            //Doing the recursion on the children
            for( unsigned int j=0; j < mu[curr_index].size(); j++ ) {
                index_stack.push_back(mu[curr_index][j]);
            }
        }
        
        assert(recursion_order.size() == tree.getNrOfSegments());
        
        //Compiling indicator function;
        for(unsigned int j=0; j < ns; j++ ) {
            indicator_function[j].resize(nj,false);
        }
        
        for(unsigned int j=0; j < ns; j++ ) {
            unsigned int jnt_index = link2joint[j];
            if( seg_vector[j]->second.segment.getJoint().getType() != Joint::None ) {
                
                index_stack.clear();
                index_stack.push_back(j);
                while( !index_stack.empty() ) {
                    unsigned int curr_index = index_stack.back();
                    index_stack.pop_back();
                    
                    indicator_function[curr_index][jnt_index] = true;
                
                    for( unsigned int k=0; k < mu[curr_index].size(); k++ ) {
                        index_stack.push_back(mu[curr_index][k]);
                    }
                }
            }
        }
        
        
        for( unsigned int j=0; j < mu_root.size(); j++ ) {
            index_stack.push_back(mu_root[j]);
        }
        
        while( !index_stack.empty() ) {
            
            unsigned int curr_index = index_stack.back();
            index_stack.pop_back();
            
            recursion_order.push_back(curr_index);
            
            //Doing the recursion on the children
            for( unsigned int j=0; j < mu[curr_index].size(); j++ ) {
                index_stack.push_back(mu[curr_index][j]);
            }
        }

        
        
        
    }
    

    int TreeInertialParameters::dynamicsRegressor( const JntArray &q, 
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
        for(unsigned int l = 0; l < recursion_order.size(); l++ ) {
            
            unsigned int curr_index = recursion_order[l];
        
			const Segment& seg = seg_vector[curr_index]->second.segment;
			const Joint& jnt = seg.getJoint();
			
            double q_,qdot_,qdotdot_;
            if(jnt.getType() !=Joint::None){
				int idx = link2joint[curr_index];
                q_=q(idx);
                qdot_=q_dot(idx);
                qdotdot_=q_dotdot(idx);
            }else
                q_=qdot_=qdotdot_=0.0;
                
            Frame& eX  = X[curr_index];
            Twist& eS  = S[curr_index];
            Twist& ev  = v[curr_index];
            Twist& ea  = a[curr_index];
            Wrench& ef = f[curr_index];
            Frame& eX_b = X_b[curr_index];

            //Calculate segment properties: X,S,vj,cj
            eX=seg.pose(q_);//Remark this is the inverse of the 
                            //frame for transformations from 
                            //the parent to the current coord frame
            //Transform velocity and unit velocity to segment frame
            Twist vj=eX.M.Inverse(seg.twist(q_,qdot_));
            eS=eX.M.Inverse(seg.twist(q_,1.0));
            //We can take cj=0, see remark section 3.5, page 55 since the unit velocity vector S of our joints is always time constant
            //calculate velocity and acceleration of the segment (in segment coordinates)
            
            int parent_index = lambda[curr_index];
            
            if( parent_index == -1 ) {
                eX_b = eX;
            } else {
                eX_b = X_b[parent_index]*eX;
            }
            
            
  
            Twist parent_a, parent_v;
            
            if( parent_index == -1 ) {
                parent_a = base_acceleration;
                parent_v = base_velocity;
            } else {
                parent_a = a[parent_index];
                parent_v = v[parent_index];
            }
            
            ev=eX.Inverse(parent_v)+vj;
            ea=eX.Inverse(parent_a)+eS*qdotdot_+ev*vj;
            
        }      
        
        dynamics_regressor.setZero();
        
        //just for design, then the loop can be put together
        Eigen::Matrix<double, 6, 10> netWrenchRegressor_i;

        for(i=0;i<ns;i++) {
                        
            netWrenchRegressor_i = netWrenchRegressor(v[i],a[i]);
            
            dynamics_regressor.block(0,(int)(10*i),6,10) = WrenchTransformationMatrix(X_b[i])*netWrenchRegressor_i;

            Frame X_j_i;
            for(j=0;j<ns;j++) {
                X_j_i = X_b[j].Inverse()*X_b[i];
                
                if( seg_vector[j]->second.segment.getJoint().getType() != Joint::None ) {
                    if( indicator_function[i][link2joint[j]] ) {
                        dynamics_regressor.block(6+link2joint[j],10*i,1,10) = 
                            toEigen(S[j]).transpose()*WrenchTransformationMatrix(X_j_i)*netWrenchRegressor_i;
                    }
                }
            }
        
            
        }

        return 0;
        
    }
    
}//namespace

