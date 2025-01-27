#include <iostream>
#include <cmath> 

using namespace std;

int main() {
    cout << "Versión 2: Resolver el caso general (discriminante >= 0)\n";

    double a, b, c;
    cout << "Introduce los coeficientes a, b y c: ";
    cin >> a >> b >> c;

    if (a == 0) {
        cout << "Esto no es una ecuación cuadrática.\n";
        return 1; // Devuelve un valor de error
    }

    double discriminante = b * b - 4 * a * c;
    if (discriminante >= 0) {
        double x1 = (-b + sqrt(discriminante)) / (2 * a);
        double x2 = (-b - sqrt(discriminante)) / (2 * a);
        cout << "Las soluciones son: " << x1 << " y " << x2 << "\n";
    } else {
        cout << "El discriminante es negativo. No hay soluciones reales.\n";
    }

    return 0; 
}

