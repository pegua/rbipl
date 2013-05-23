/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */ 

#include "baseparameters_gautinernumerical.hpp"

namespace RBIPL {
    
    BaseParametersNumerical::BaseParametersNumerical(const ParametersRegressorSolver & param_solver_arg): param_solver(param_solver_arg)
    {
    }
    
    BaseParametersNumerical::~BaseParametersNumerical()
    {
    }
    
    int BaseParametersNumerical::computeBaseParameters(MatrixXd & IdentifiableSpaceBasis, bool fixed_base, int n_samples, double eps )
    {
        if( n_samples < 0 ) return -1;
        int no = param_solver.getNrOfOutputs();
        int np = param_solver.getNrOfParameters();

        
        //sketch ! not working
        MatrixXd A(np,np); //working matrix
        MatrixXd regressor(no,np);
        JntArray q(nj),dq(nj),ddq(nj);
        Twist a,v;
        MatrixXd V(np,np);

        
        for(int i=0; i < n_samples; i++ ) {
            //RegressorSolver 
            //Excite with random data RegressorSolver (random q, dq, ddq (for energy, simply do not use ddq)
            q.data = M_PI*VectorXd::Random(nj);
            dq.data = M_PI*VectorXd::Random(nj);
            ddq.data = M_PI*VectorXd::Random(nj);
            
            if( fixed_base ) {
                //fixed base 
                param_solver.CartToJnt(q,dq,ddq,regressor);
                
            } else {
                //floating base
                Map<Vector3d>(a.vel.data) = M_PI*Vector3d::Random();
                Map<Vector3d>(a.rot.data) = M_PI*Vector3d::Random();
                Map<Vector3d>(v.vel.data) = M_PI*Vector3d::Random();
                Map<Vector3d>(v.rot.data) = M_PI*Vector3d::Random();
                
                param_solver.CartToJnt(q,dq,ddq,a,v,regressor);
            }
            
            if( i == 0 ) {
                A = regressor.transpose()*regressor;
            } else {
                A += regressor.transpose()*regressor;
            }
        }
        
        //Probably can be improved using a different decomposition
        //NOT REAL TIME!!
        JacobiSVD<MatrixXd> svd(A);
        
        if( eps >= 0.0 ) {
            svd.setTreshold(eps);
        }
        
        int rank = svd.rank();
        
        V = svd.matrixV();
        
        IdentifiableSpaceBasis = V.block(0,0,np,rank);
        
        //NonBaseSpaceBasis = V.block(0,rank+1,np,np-rank);
        
        return 0;
        
    }
}
