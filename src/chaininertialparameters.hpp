/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#ifndef KDL_CHAIN_INERTIALPARAMETERS_HPP
#define KDL_CHAIN_INERTIALPARAMETERS_HPP

#include "utils.hpp"

#include <Eigen/Core>

#include <kdl/rotationalinertia.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

using namespace KDL;



namespace RBIPL {
    

    
    /**
     * \brief class encapsulating methods relative to inertial parameters
     * \todo What if Chain changes?
     */
     class ChainInertialParameters {
         private:
            const Chain & ref_chain;
            Eigen::VectorXd chain_param;
            unsigned int nj;
            unsigned int ns;
            std::vector<Frame> X;
            std::vector<Twist> S;
            std::vector<Twist> v;
            std::vector<Twist> a;
            std::vector<Wrench> f;
            std::vector<bool> has_joint;
            Twist a0;
            Twist v0;
            
        public:
            /**
             * Constructor, it will allocate all the necessary memory.
             *
             * @param chain the used chain, a reference of this chain is stored.
             */
            ChainInertialParameters(Chain& chain);
            ~ChainInertialParameters(){};
            
            
            Eigen::VectorXd getInertialParameters();
             
             /**
             * Get a copy of the current KDL::Chain, with modified inertial parameters.
             * 
             * @param new_chain_param the inertial parameters vector
             * @param new_chain a reference to the output chain
             * @return false in case of some error, true otherwise
             */
            bool changeInertialParameters(const Eigen::VectorXd & new_chain_param,  Chain& new_chain);
            
            /**
             * Return the regressor for floating base dynamics 
             * 
             * It replicates the ChainIdSolver_RNE::CartToJnt, the only difference 
             * is it output the regressor matrix instead of result. 
             * 
             * @param dynamics_regressor a (6+nj)x(10*ns) output matrix
             *
             */
            int dynamicsRegressor(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot,  const Twist& base_velocity, const Twist& base_acceleration, Eigen::MatrixXd & dynamics_regressor);

        };
}


#endif 
