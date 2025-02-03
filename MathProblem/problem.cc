#include <iostream>
using namespace std;

int main() {
   
    int red_balls = 5;
    int blue_balls = 3;

    
    int total_balls = red_balls + blue_balls;

   
    double probability_red = static_cast<double>(red_balls) / total_balls;

    
    cout << "The probability of drawing a red ball is: " << probability_red << endl;

    return 0;
}

