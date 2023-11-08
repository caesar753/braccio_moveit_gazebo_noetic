#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <tuple>

std::pair<double, double> cart2pol(double x, double y) {
  // helper, convert cartesian to polar coordinates
  double rho = std::sqrt(x * x + y * y);
  double phi = std::atan2(y, x);
//   return {rho, phi};
// std::cout << rho << "  " <<phi << std::endl;

return std::make_pair(rho, phi);

}

int main(){
    double s, alfa;
    // std::pair<double, double>(s, alfa);
    std::tie(s, alfa) = cart2pol(2.3334,3.114);
    std::cout << s << "  " << alfa << std::endl;

    return 0;
    
}