#include<iostream>
#include<string>

int pedir_numero1(){
int numero1 = 0;
std::cout<<"Enter a number >=1:" << std::endl;
std::cin >> numero1;
return numero1;
}



int pedir_numero2(){
int numero2 = 0;
std::cout<<"Enter b number >=1:" << std::endl;
std::cin >> numero2;
return numero2;
}

void calcular_sum(int numero1, int numero2){
int rdo = 0;
for(int i=numero1; i<=numero2; i++){
rdo+=i;
}
std::cout<<"the sum from " << numero1 << " to " << numero2 << " is " << rdo << std::endl;
}


int main()
{
int numero1 = 0;
int numero2 = 0;
while(numero1<1 ){
numero1=pedir_numero1();
}

while(numero2<1 ){
numero2=pedir_numero2();
}

while(numero2<=numero1){
std::cout << "" << std::endl;
std::cout << "Number b<=a" << std::endl;
numero2=pedir_numero2();
}

calcular_sum(numero1, numero2);
return 0;
}
