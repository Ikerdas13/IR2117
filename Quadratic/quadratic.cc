#include <iostream>
#include <cmath> 

using namespace std;

int main() {
cout << "Versión 4: Discriminante igual a 0 (una solución)\n";
    double a, b, c;
    cout << "Introduce los coeficientes a, b y c: ";
    cin >> a >> b >> c;

    if (a == 0) {
        cout << "Esto no es una ecuación cuadrática.\n";
        return 0;
    }

    double discriminante = b * b - 4 * a * c;
    if (discriminante == 0) {
        double x = -b / (2 * a);
        cout << "Hay una única solución: " << x << "\n";
    } else {
        cout << "El discriminante no es igual a 0. Continuar con la siguiente versión.\n";
    }

return 0; 
}

