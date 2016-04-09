#include <iostream>
#include <fstream>

using namespace std;

int main()
{
ofstream outputFile;
outputFile.open("numeri.txt");

double num1, num2, num3, num4, num5;

cout << "Enter the first number: ";
cin >> num1;
outputFile << num1 << endl;
cout << "Enter the second number: ";
cin >> num2;
outputFile << num2 << endl;
cout << "Enter the third number: ";
cin >> num3;
outputFile << num3 << endl;
cout << "Enter the fourth number: ";
cin >> num4;
outputFile << num4 << endl;
cout << "Enter the fifth number: ";
cin >> num5;
outputFile << num5 << endl;

outputFile.close();
cout << "Done!\n";

cout<< "Reading...\n";

ifstream myReadFile;
myReadFile.open("numeri.txt");
char output[100];
 if (myReadFile.is_open()) 
 {
	while (!myReadFile.eof()) 
	{
		myReadFile >> output;
		cout<<output<<endl;
	}
 }
 myReadFile.close();
 cout<<"Done\n";




return 0;
}