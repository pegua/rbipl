/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */ 
     
#ifndef RBIPL_BASEPARAMETERS_HPP
#define RBIPL_BASEPARAMETERS_HPP

#include <Eigen/Core>

namespace RBIPL {    
    /**
	 * \brief This <strong>abstract</strong> class is an interface for 
     * algorithms to calculate subspace of "base parameters" (so called 
     *  identifiable subspace)  for a given robot model.
     * Currently implementing only Gautier numerical algorithm.
	 *
	 */
     class BaseParameters
     {
         public:
            typedef enum { FixedBase, FloatingBase } BaseParametersType;
     }
     
     
}
             
#endif
