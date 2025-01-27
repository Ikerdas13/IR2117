#include <iostream>
#include <cmath> 

using namespace std;

int main() {
 cout << "Versión 3: Discriminante negativo (mensaje)\n";
    double a, b, c;
    cout << "Introduce los coeficientes a, b y c: ";
    cin >> a >> b >> c;

    if (a == 0) {
        cout << "Esto no es una ecuación cuadrática.\n";
        return 0;
    }

    double discriminante = b * b - 4 * a * c;
    if (discriminante < 0) {
        cout << "El discriminante es negativo. No hay soluciones reales.\n";
    } else {
        cout << "El discriminante no es negativo. Continuar con la siguiente versión.\n";
    }

return 0; 
}

