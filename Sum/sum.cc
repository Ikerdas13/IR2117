#include<iostream>
#include<string>


void sum(){
int numero = 0;
int rdo= 0;
std::cout<<"Enter a number:" << std::endl;
std::cin >> numero;

if (numero>=1){
for(int i=1; i <= numero; i++) {
rdo += i;
}
std::cout<<"The sum from 1 to " << numero << " is " << rdo <<std::endl;
}
else{
std::cout<<"Invalid number" << std::endl;
sum();
}
}





int main()
{
sum();
}
