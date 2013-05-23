/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */ 
     
#ifndef RBIPL_PARAMETERSREGRESSOR_HPP
#define RBIPL_PARAMETERSREGRESSOR_HPP

#include <Eigen/Core>

namespace RBIPL {
    /**
	 * \brief This class interface
	 *
	 */
     class ParametersRegressorSolver 
     {   
         public:
            //naming in consistency with KDL
            //fixed base
            virtual int CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, Eigen::MatrixXd & dynamics_regressor)=0;
            
            //floating base
            virtual int CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Twist& base_velocity, const Twist& base_acceleration, Eigen::MatrixXd & dynamics_regressor)=0;
            
            /**
             * Return the number of parameters
             * 
             */
            virtual int getNrOfParameters()=0;
            
            /**
             * Return the number of outputs
             * 
             */
             virtual int getNrOfOutputs()=0;
     }
     
     
}
             
#endif
