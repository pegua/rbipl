/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#ifndef KDL_CHAIN_INERTIALPARAMETERS_HPP
#define KDL_CHAIN_INERTIALPARAMETERS_HPP

#include "utils.hpp"
#include "treeserialization.hpp"

#include <Eigen/Core>

#include <kdl/rotationalinertia.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

using namespace KDL;



namespace RBIPL {
    

    
    /**
     * \brief class encapsulating methods relative to inertial parameters
     * \todo What if Chain changes?
     */
     class TreeInertialParameters {
         private:
            void updateParams();
            bool changeInertialParametersRecursive(const Eigen::VectorXd & new_chain_param, Tree & new_tree, SegmentMap::const_iterator root, const std::string& hook_name) ;
         
            const Tree & ref_tree;
            std::string root_name;
            
            //Frame of link i with respect to the frame of link lambda[i]
            std::vector<Frame> X;
            std::vector<Twist> S;
            std::vector<Twist> v;
            std::vector<Twist> a;
            std::vector<Wrench> f;
            
            //Frame of link i with respect to the base
            std::vector<Frame> X_b;
            
            Eigen::VectorXd tree_param;
            
            unsigned int nj;
            unsigned int ns;
            
            Twist ag;
            
            TreeSerialization serial;
            
            
            std::vector<unsigned int> mu_root; //set of childrens of root
            std::vector< std::vector<unsigned int> > mu; //set of childrens of each segment
            std::vector< int > lambda; //set of parent of each segment
            std::vector<unsigned int> link2joint;
            
            std::vector< unsigned int > recursion_order;
            
            std::vector<SegmentMap::const_iterator> seg_vector;
            
            //Indicator function
            std::vector< std::vector<bool> > indicator_function;
            
        public:
            /**
             * Constructor, it will allocate all the necessary memory.
             *
             * @param tree the used tree, a reference to this tree is stored.
             */
            TreeInertialParameters(Tree& tree, Vector grav=Vector::Zero(),const TreeSerialization & serialization=TreeSerialization());
            ~TreeInertialParameters(){};
            
            Eigen::VectorXd getInertialParameters();
             
             /**
             * Get a copy of the current KDL::Tree, with modified inertial parameters.
             * 
             * @param new_chain_param the inertial parameters vector
             * @param new_chain a reference to the output chain
             * @return false in case of some error, true otherwise
             */
            bool changeInertialParameters(const Eigen::VectorXd & new_chain_param, Tree& new_tree);
            
            
             /**
             * Return the regressor for fixed base dynamics 
             * 
             * It replicates the TreeIdSolver_RNE::CartToJnt, the only difference 
             * it is that it outputs the regressor matrix instead of result. 
             * 
             * @param dynamics_regressor a (6+nj)x(10*ns) output matrix
             *
             */
            int dynamicsRegressor(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, Eigen::MatrixXd & dynamics_regressor);

            
            /**
             * Return the regressor for floating base dynamics 
             * 
             * It replicates the TreeIdSolver_RNE::CartToJnt, the only difference 
             * it is that it outputs the regressor matrix instead of result. 
             * 
             * @param dynamics_regressor a (6+nj)x(10*ns) output matrix
             *
             */
            int dynamicsRegressor(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot,  const Twist& base_velocity, const Twist& base_acceleration, Eigen::MatrixXd & dynamics_regressor);

        };
}


#endif 
