#include<stdio.h>
int main()
{
	int num1, num2, num3;
	int largest;
	printf("Enter three numbers: ");
	scanf("%d %d %d", &num1, &num2, &num3);
	// Using ternary operator
   	largest = (num1 > num2) ? (num1 > num3 ? num1 : num3) : (num2 > num3 ? num2 : num3);
   	printf("Largest number using ternary operator: %d\n", largest);
   	// Using if-else-if ladder with logical operators
	if (num1 >= num2 && num1 >= num3)
	{
    	largest = num1;
	}
	else if (num2 >= num1 && num2 >= num3)
	{
      	largest = num2;
   	}
	else
	{
      	largest = num3;
   	}
   	printf("Largest number using if-else-if ladder with logical operators: %d\n", largest);
   	// Using nested if
   	if (num1 >= num2){
    	if (num1 >= num3){
         	largest = num1;
		}
		else
		{
         	largest = num3;
      	}
   	}
   	else
	{
      	if (num2 >= num3)
		{
         	largest = num2;
      	}
		else
		{
         	largest = num3;
      	}
   	}
   	printf("Largest number using nested if: %d\n", largest);
}

