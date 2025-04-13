#include <iostream>
using namespace std;

int main() {
    int red_balls, blue_balls;

    cout << "Enter the number of red balls: ";
    cin >> red_balls;
    cout << "Enter the number of blue balls: ";
    cin >> blue_balls;


    int total_balls = red_balls + blue_balls;

  
    if (total_balls == 0) {
        cout << "Error: The total number of balls cannot be zero." << endl;
    } else {
        
        double probability_first_red = static_cast<double>(red_balls) / total_balls;

        
        int remaining_red_balls = red_balls - 1;
        int remaining_total_balls = total_balls - 1;

        
        double probability_second_red = static_cast<double>(remaining_red_balls) / remaining_total_balls;

        
        cout << "The probability of drawing a red ball on the first draw is: " << probability_first_red << endl;
        cout << "The probability of drawing a red ball on the second draw (after a red ball was drawn first) is: " << probability_second_red << endl;
    }

    return 0;
}

