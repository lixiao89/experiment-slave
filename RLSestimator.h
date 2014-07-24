#ifndef _RLS_ESTIMATOR_H
#define _RLS_ESTIMATOR_H

#include<iostream>

class RLSestimator{

    public:

        // x[0] = mu is the cofficient of kinetic friction, x[1] = Fc is the estimated cutting force
        vctFixedSizeVector<double,2> x;  
        // P is the covariance of x        
        vctFixedSizeMatrix<double,2,2> P;
        // Fest is the estimated total tangential force 
        double Fest;

        // Fn is the measured normal force
        // double Fn;
        // Fe is the measured tangential force
        //double Fe;

        // indicate a cutting failure mode
        bool fail;
        // error threshold
        double threshold;

        //system matrix yk = Hk.transpose*x + v
        vctFixedSizeMatrix<double,2,2> Hk;
        // measurement error covariance
        vctFixedSizeMatrix<double,2,2> Rk;
        
        // yk is the measurement vector with first element measured tangential force and second element 0
        vctFixedSizeVector<double,2> yk;
        // constructor
        RLSestimator(vctFixedSizeVector<double,2>& xinit): 
        
         x(xinit),
         P(vct2x2::Eye()),
         Rk(vct2x2::Eye()),
         fail(false),
         threshold(10),
         Hk(vct2x2::Eye()),
         Fest(0),
         yk(0,0){
                
               Hk[0][0] = 0;
               Hk[0][1] = 0;
               Hk[1][0] = 1;
               Hk[1][1] = 0;
         }

        void GetEstimates(vctFixedSizeVector<double,2>& xesti, double& Festi){

            xesti = x;
            Festi = Fest; 
        }        

        bool Evaluate(double const &Fn, double const &Fe){

                Hk[0][0] = Fn;
                yk[0] = Fe;
                

                vctFixedSizeMatrix<double,2,2> K;
                vctFixedSizeMatrix<double,2,2> tempK;
                vctFixedSizeMatrix<double,2,2> invtempK;

                tempK = Hk.Transpose()*P*Hk + Rk;

                Inverse(tempK,invtempK);
                K = P*Hk*invtempK;

                //std::cout<<invtempK<<std::endl;
                //std::cout<<""<<std::endl;


                x = x + K*(yk - Hk.Transpose()*x);

               //std::cout<<x[0]<<", "<<x[1]<<std::endl;

                P = (vct2x2::Eye() - K*Hk.Transpose())*P;
                
                Fest = x[0]*Fn + x[1];

                if(fabs(Fest - Fe) > threshold){
                    
                    return true;
                }
                else{

                    return false;
                }

         }

      void Inverse(vctFixedSizeMatrix<double,2,2>& M, vctFixedSizeMatrix<double,2,2>& Minv){
            double detM;
            detM = M[0][0]*M[1][1] - M[0][1]*M[1][0];
            
            if(abs(detM) < 0.0001){
                detM = 0.0001;
            }

            Minv[0][0] = M[1][1];
            Minv[0][1] = -M[0][1];
            Minv[1][0] = -M[1][0];
            Minv[1][1] = M[0][0];

            Minv = (1/detM)*Minv;


        }
};




#endif
