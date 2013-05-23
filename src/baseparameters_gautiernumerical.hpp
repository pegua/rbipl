/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */ 
     
#ifndef RBIPL_BASEPARAMETERS_GAUTIERNUMERICAL_HPP
#define RBIPL_BASEPARAMETERS_GAUTIERNUMERICAL_HPP

#include <Eigen/Core>

namespace RBIPL {
    /**
	 * \brief This class encapsulates the Gautier numerical algorithm 
     * ( Gautier Maxime  
     *   Numerical calculation of the base inertial parameters of robots.
     *   Journal of Robotic Systems 8 , 4 (1991), 485–506.)
     * to identify the base parameters subspace of a KDL::Tree
     * Note that the actual algorithm is a slightly modified version of the 
     * Gautier Algorithm, so that it requires an amount of memory indipentent
     * of the number of samples. 
     * 
     * \todo If necessary efficiency of the algorithm can be improved by
     * using QR or Eigendecomposition instead of SVD
     * 
	 *
	 */
     class BaseParametersNumerical : public BaseParameters
     {   
         private:
            ParametersRegressorSolver param_solver;
            
         public:
            BaseParametersNumerical(const ParametersRegressorSolver & param_solver_arg);
            ~BaseParametersNumerical();
            
            
     }
     
     
}
             
#endif
