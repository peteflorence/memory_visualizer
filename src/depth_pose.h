#include <iostream>

class DepthPose {
public:
  void printExists() { std::cout << "I'm alive" << std::endl; };
  void alsoPrint();

private:
  
  double x = 0.0;

};