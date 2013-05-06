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
         
            struct Entry{
                Frame X;
                Twist S;
                Twist v;
                Twist a;
                Wrench f;
                Wrench f_ext;
            };
            const Tree & ref_tree;
            std::string root_name;
            std::vector<Entry> db;	///indexed by segment id
            
            Eigen::VectorXd tree_param;
            
            unsigned int nj;
            unsigned int ns;
            
            Twist ag;
            
        public:
            /**
             * Constructor, it will allocate all the necessary memory.
             *
             * @param chain the used chain, a reference of this chain is stored.
             */
            TreeInertialParameters(Tree& tree, Vector grav=Vector::Zero(),const TreeSerialization & serialization=TreeSerialization(tree));
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
