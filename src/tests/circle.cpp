#include <math.h>
#include <queue>
#include <sstream>
#define PI 3.14159265358979323
// taken from https://en.wikipedia.org/wiki/Latitude#The_geometry_of_the_ellipsoid
#define A_EQRAD 6378137
#define B_POLRAD 6356752.31425
#define E_SQUARED 0.00669437999014
#define MA_SIZE 4 // moving average size
#include <iostream>
using namespace std;
#include <array>
#include <ios>
#include <streambuf>
#include <istream>
#include <ostream>
 



double RadToDeg(double radians){
    return radians*180.0/PI;
}

double SlopeCalculator(std::array<double,3> &x,std::array<double,3> &y ){
    // measure the total heading change of the queue (in radians)
    double averHCd, totHeadingChange = 0.0;
    int sz=x.size();
    double x0=x[sz-1 -2];
    double x1=x[sz-1 -1];
    double  x2=x[sz-1   ];
    std::cout << printf( "x0 %f x1 %f x2 %f ",x0,x1,x2 )<< std::endl;
    double y0=y[sz-1 -2];
    double y1=y[sz-1 -1];
    double y2=y[sz-1   ];
    
    std::cout << printf( "y0 %f y1 %f y2 %f ",y0,y1,y2 )<< std::endl;
    
    // phi_ab = atan2 (yb-ya,xb-xa)
    double phi_01=atan2( y1-y0 , x1-x0 ); //heading two steps back
    double phi_12=atan2( y2-y1 , x2-x1 ); //heading one step back
    

    double p = 0.5*( cos(phi_12)*(x0-x2) +  sin(phi_12)*(y0-y2) ) / ( sin(phi_01 - phi_12) );
    double s = 0.5*( cos(phi_01)*(x0-x2) +  sin(phi_01)*(y0-y2) ) / ( sin(phi_01 - phi_12) );
    // double p = 0.5*( cos(phi_12)*(x2+x0)+sin(phi_12)*(y2+y0) ) / ( sin(phi_01)*cos(phi_12)-cos(phi_01)*sin(phi_12) );
    double CenterX=0.5*(x0+x1)-p*sin(phi_01);
    double CenterY=0.5*(y0+y1)+p*cos(phi_01);

    double CenterX2=0.5*(x1+x2)-s*sin(phi_12);
    double CenterY2=0.5*(y1+y2)+s*cos(phi_12);

    double theta_hat, theta =0.0;

    std::cout << printf( "phi 0->1 %f, phi 1->2 %f, dif phi %f " , RadToDeg(phi_01) , RadToDeg(phi_12), RadToDeg(phi_01 - phi_12) )<< std::endl;
    std::cout << printf( "p~ %f " ,p)<< std::endl;
    std::cout << printf( "C_x: %f, C_y: %f " , CenterX, CenterY)<< std::endl;
    
    // std::cout << printf( "s~ %f " ,s)<< std::endl;
    // std::cout << printf( "C_x2: %f, C_y2: %f " , CenterX2, CenterY2)<< std::endl;
    // std::cout << printf( "total %f " , RadToDeg(totHeadingChange) )<< std::endl;
    theta_hat=atan2(y2-CenterY,x2-CenterX);
    std::cout << printf( "theta^^^: %f", RadToDeg(theta_hat) )<< std::endl;

    if (p>0) { 
        theta = theta_hat + PI/2;
        std::cout << printf( "CCW theta: %f", RadToDeg(theta) )<< std::endl;
     }
    else{
         theta = theta_hat - PI/2;
        std::cout << printf( "CW theta: %f", RadToDeg(theta) )<< std::endl;
    }


    std::cout << std::endl;
    return theta;
   
}


int main(int argc, char *argv[]) { 
    
    std::array<double,3> x = {1.0, 2.0, 2.0};
    std::array<double,3> y = {0.0, 1.0, 2.0};
    double z = SlopeCalculator(x,y);

    std::array<double,3> r = {1.0, 2.0, 3.0};
    std::array<double,3> s = {0.0, 1.0, 1.0};
    double t = SlopeCalculator(r,s);
    // std::cout << printf( "ret %f" , SlopeCalculator(x,y) )<< std::endl;;

    return 0;

 }