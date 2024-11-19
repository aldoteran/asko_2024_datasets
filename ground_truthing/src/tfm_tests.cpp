#include "utils/tfm_utils.h"

int main(){
  double a = 30;
  double b = 0;
  double c = -30;
  double d = 179;
  double e = -179;

  double a_enu =dockslam::NedHeadingInEnu(a, true);
  double b_enu =dockslam::NedHeadingInEnu(b, true);
  double c_enu =dockslam::NedHeadingInEnu(c, true);
  double d_enu =dockslam::NedHeadingInEnu(d, true);
  double e_enu =dockslam::NedHeadingInEnu(e, true);

  std::cout << "a ned: " << a << " ; a_enu: " << a_enu << "\n";
  std::cout << "b ned: " << b << " ; b_enu: " << b_enu << "\n";
  std::cout << "c ned: " << c << " ; c_enu: " << c_enu << "\n";
  std::cout << "d ned: " << d << " ; d_enu: " << d_enu << "\n";
  std::cout << "e ned: " << e << " ; e_enu: " << e_enu << "\n";

  std::cout << "sin(-30) = " << std::sin(-30*M_PI/180.0) << "\n";
  std::cout << "sin(330) = " << std::sin(330*M_PI/180.0) << "\n";
  std::cout << "cos(-30) = " << std::cos(-30*M_PI/180.0) << "\n";
  std::cout << "cos(330) = " << std::cos(330*M_PI/180.0) << "\n";

}
