// #ifndef POLYFIT_H
// #define POLYFIT_H

// ********************************************************************
// * Code PolyFit                                                     *
// * Written by Ianik Plante                                          *
// *                                                                  *
// * KBR                                                              *
// * 2400 NASA Parkway, Houston, TX 77058                             *
// * Ianik.Plante-1@nasa.gov                                          *
// *                                                                  *
// * This code is used to fit a series of n points with a polynomial  *
// * of degree k, and calculation of error bars on the coefficients.  *
// * If error is provided on the y values, it is possible to use a    *
// * weighted fit as an option. Another option provided is to fix the *
// * intercept value, i.e. the first parameter.                       *
// *                                                                  *
// * This code has been written partially using data from publicly    *
// * available sources.                                               *
// *                                                                  *  
// * The code works to the best of the author's knowledge, but some   *   
// * bugs may be present. This code is provided as-is, with no        *
// * warranty of any kind. By using this code you agree that the      * 
// * author, the company KBR or NASA are not responsible for possible *
// * problems related to the usage of this code.                      * 
// *                                                                  *   
// * The program has been reviewed and approved by export control for * 
// * public release. However some export restriction exists. Please   *    
// * respect applicable laws.                                         *
// *                                                                  *   
// ********************************************************************


#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <cmath>
#include <iomanip>
#include <stdexcept>

//opencv
#include <opencv2/opencv.hpp>   
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

#define MAXIT 100
#define EPS 3.0e-7
#define FPMIN 1.0e-30

/*
 * zlib License
 *
 * Regularized Incomplete Beta Function
 *
 * Copyright (c) 2016, 2017 Lewis Van Winkle
 * http://CodePlea.com
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgement in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#define STOP 1.0e-8
#define TINY 1.0e-30

namespace polyfit{
    
    struct complex { double r,i; };
    struct quadComplex { complex p1, p2; };

    quadComplex GetResults(double a, double b, double c);
    double incbeta(double a, double b, double x);
    double invincbeta(double y,double alpha, double beta);
    double CalculateTValueStudent(const double nu, const double alpha);
    double cdfStudent(const double nu, const double t);
    double cdfFisher(const double df1, const double df2, const double x);
    double **Make2DArray(const size_t rows, const size_t cols);
    double **MatTrans(double **array, const size_t rows, const size_t cols);
    double **MatMul(const size_t m1, const size_t m2, const size_t m3, double **A, double **B);
    void MatVectMul(const size_t m1, const size_t m2, double **A, double *v, double *Av);
    double determinant(double **a, const size_t k);
    void transpose(double **num, double **fac, double **inverse, const size_t r);
    void cofactor(double **num, double **inverse, const size_t f);
    void displayMat(double **A, const size_t n, const size_t m);
    double CalculateRSS(const double *x, const double *y, const double *a, double **Weights,
        const bool fixed, const size_t N, const size_t n);
    double CalculateTSS(const double *x, const double *y, const double *a, double **Weights, 
        const bool fixed, const size_t N, const size_t n);
    double CalculateR2COD(const double *x, const double *y, const double *a, double **Weights,
        const bool fixed, const size_t N, const size_t n);
    double CalculateR2Adj(const double *x, const double *y, const double *a, double **Weights,
        const bool fixed,const size_t N, const size_t n);
    void PolyFit(const double *x, double *y, const size_t n, const size_t k, const bool fixedinter,
        const double fixedinterval, double *beta, double **Weights, double **XTWXInv);
    double calculatePoly(const double x, const double *a, const size_t n);
    void CalculateWeights(const double *erry, double **Weights, const size_t n,
        const int type);
    void CalculateSERRBeta(const bool fixedinter, const double SE, size_t k, double *serbeta, double **XTWXInv);
    void DisplayPolynomial(const size_t k);
    void DisplayANOVA(const size_t nstar, const size_t k, const double TSS, const double RSS);
    void DisplayCoefs(const size_t k, const size_t nstar, const double tstudentval, const double *coefbeta, const double *serbeta);
    void DisplayStatistics(const size_t n, const size_t nstar, const size_t k, const double RSS, const double R2,
        const double R2Adj, const double SE);
    void DisplayCovCorrMatrix(const size_t k, const double sigma, const bool fixed, double **XTWXInv);
    void WriteCIBands(std::string filename, const double *x, const double *coefbeta, double **XTXInv, 
        const double tstudentval, const double SE, const size_t n, const size_t k);
    vector<Point> PolyFitCV(vector<Point> poi, const double *erry, const size_t k, const std::pair<size_t,size_t> & frame_bounds);
    std::pair<size_t,size_t> calculateXBoundsSecPoly(const std::pair<size_t,size_t> &, const double*);
}
// #endif


// // The main program
// // **************************************************************
// int main(int argc, char *argv[]) {

//     cout << "Polynomial fit!" << endl;
    
//     // Input values
//     // **************************************************************
//     size_t k = 2;                                    // Polynomial order
//     bool fixedinter = false;                         // Fixed the intercept (coefficient A0)
//     int wtype = 0;                                   // Weight: 0 = none (default), 1 = sigma, 2 = 1/sigma^2
//     double fixedinterval = 0.;                       // The fixed intercept value (if applicable)
//     double alphaval = 0.05;                          // Critical apha value

//     double x[] = {0., 0.5, 1.0, 2.0, 4.0, 6.0};
//     double y[] = {0., 0.21723, 0.43445, 0.99924, 2.43292, 4.77895};
//     double erry[] = {0.1, 0.3, 0.2, 0.4, 0.1, 0.3};       // Data points (err on y) (if applicable)

//     // Definition of other variables
//     // **************************************************************
//     size_t n = 0;                                    // Number of data points (adjusted later)
//     size_t nstar = 0;                                // equal to n (fixed intercept) or (n-1) not fixed
//     double coefbeta[k+1];                            // Coefficients of the polynomial
//     double serbeta[k+1];                             // Standard error on coefficients
//     double tstudentval = 0.;                         // Student t value
//     double SE = 0.;                                  // Standard error
    
//     double **XTWXInv;                                // Matrix XTWX Inverse [k+1,k+1]
//     double **Weights;                                // Matrix Weights [n,n]


//     // Initialize values
//     // **************************************************************
//     n = sizeof(x)/sizeof(double);
//     nstar = n-1;
//     if (fixedinter) nstar = n;
           
//     cout << "Number of points: " << n << endl;
//     cout << "Polynomial order: " << k << endl;
//     if (fixedinter) {
//         cout << "A0 is fixed!" << endl;
//     } else {
//         cout << "A0 is adjustable!" << endl;
//     }

//     if (k>nstar) {
//         cout << "The polynomial order is too high. Max should be " << n << " for adjustable A0 ";
//         cout << "and " << n-1 << " for fixed A0. ";  
//         cout << "Program stopped" << endl;
//         return -1;
//     }

//     if (k==nstar) {
//         cout << "The degree of freedom is equal to the number of points. ";
//         cout << "The fit will be exact." << endl;  
//     }

//     XTWXInv = Make2DArray(k+1,k+1);
//     Weights = Make2DArray(n,n);

//     // Build the weight matrix
//     // **************************************************************
//     CalculateWeights(erry, Weights, n, wtype);
    
//     cout << "Weights" << endl;
//     displayMat(Weights,n,n);

//     if (determinant(Weights,n)==0.) {
//         cout << "One or more points have 0 error. Review the errors on points or use no weighting. ";
//         cout << "Program stopped" << endl;
//         return -1;
//     } 

//     // Calculate the coefficients of the fit
//     // **************************************************************
//     PolyFit(x,y,n,k,fixedinter,fixedinterval,coefbeta,Weights,XTWXInv);


//     // Calculate related values
//     // **************************************************************
//     double RSS = CalculateRSS(x,y,coefbeta,Weights,fixed,n,k+1);
//     double TSS = CalculateTSS(x,y,coefbeta,Weights,fixedinter,n,k+1);
//     double R2 = CalculateR2COD(x,y,coefbeta,Weights,fixedinter,n,k+1);
//     double R2Adj = CalculateR2Adj(x,y,coefbeta,Weights,fixedinter,n,k+1);

//     if ((nstar-k)>0) {
//         SE = sqrt(RSS/(nstar-k)); 
//         tstudentval = fabs(CalculateTValueStudent(nstar-k, 1.-0.5*alphaval)); 
//     }
//     cout << "t-student value: " << tstudentval << endl << endl;

//     // Calculate the standard errors on the coefficients
//     // **************************************************************
//     CalculateSERRBeta(fixedinter,SE,k,serbeta,XTWXInv);

//     // Display polynomial
//     // **************************************************************
//     DisplayPolynomial(k);

//     // Display polynomial coefficients
//     // **************************************************************
//     DisplayCoefs(k, nstar, tstudentval, coefbeta, serbeta);

//     // Display statistics
//     // **************************************************************
//     DisplayStatistics(n,nstar,k,RSS,R2,R2Adj,SE);
  
//     // Display ANOVA table
//     // **************************************************************
//     DisplayANOVA(nstar, k, TSS, RSS);

//     // Write the prediction and confidence intervals
//     // **************************************************************
//     WriteCIBands("CIBands2.dat",x,coefbeta,XTWXInv,tstudentval,SE,n,k);

//     // Display the covariance and correlation matrix
//     // **************************************************************
//     DisplayCovCorrMatrix(k, SE, fixedinter, XTWXInv);

// }