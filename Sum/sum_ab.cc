#include<iostream>
#include<string>




int main()
{
int numero1 = 0;
int numero2 = 0;
int rdo = 0;

std::cout<<"Enter a number >=1:" << std::endl;
std::cin >> numero1;

std::cout<<"Enter b number >=1:" << std::endl;
std::cin >> numero2;
if(numero1>=1 and numero2>=1){
for(int i=numero1; i<=numero2; i++){
rdo+=i;
}
std::cout<<"the sum from " << numero1 << " to " << numero2 << " is " << rdo << std::endl;
}
else if(numero1>=1){
std::cout<<"Enter b number >=1:" << std::endl;
std::cin >> numero2;

for(int i=numero1; i<=numero2; i++){
rdo+=i;
}
std::cout<<"the sum from " << numero1 << " to " << numero2 << " is " << rdo << std::endl;



}
else{
std::cout<<"Enter a number >=1:" << std::endl;
std::cin >> numero1;

std::cout<<"Enter b number >=1:" << std::endl;
std::cin >> numero2;

for(int i=numero1; i<=numero2; i++){
rdo+=i;
}
std::cout<<"the sum from " << numero1 << " to " << numero2 << " is " << rdo << std::endl;
}
}
