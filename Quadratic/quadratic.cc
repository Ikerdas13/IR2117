#include <iostream>
#include <complex>
#include <cmath>

using namespace std;

void resolverSoluciones(double a, double b, double c) {
    complex<double> discriminante = b * b - 4 * a * c;
    complex<double> x1 = (-b + sqrt(discriminante)) / (2.0 * a);
    complex<double> x2 = (-b - sqrt(discriminante)) / (2.0 * a);

    if (discriminante.imag() == 0 && discriminante.real() > 0) {
        cout << "Las soluciones reales son:\n";
        cout << "x1 = " << x1.real() << "\n";
        cout << "x2 = " << x2.real() << "\n";
    } else if (discriminante.imag() == 0 && discriminante.real() == 0) {
        cout << "Discriminante == 0: Una solución real única\n";
        cout << "x = " << x1.real() << "\n";
    } else {
        cout << "Las soluciones complejas son:\n";
        cout << "x1 = " << x1 << "\n";
        cout << "x2 = " << x2 << "\n";
    }
}

int main() {
    cout << "Versión 5: Resolver ecuación cuadrática con discriminante < 0\n";

    double a, b, c;
    cout << "Introduce el coeficiente a: ";
    cin >> a;
    cout << "Introduce el coeficiente b: ";
    cin >> b;
    cout << "Introduce el coeficiente c: ";
    cin >> c;

    if (a == 0) {
        cout << "Esto no es una ecuación cuadrática.\n";
        return 0;
    }

    resolverSoluciones(a, b, c);

    return 0;
}

