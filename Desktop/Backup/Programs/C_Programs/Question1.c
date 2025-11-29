#include<stdio.h>
int main()
{
	int a,b;
	printf("Enter the values of a and b:- ");
	scanf("%d %d",&a,&b);
	printf("The Addition of %d and %d is = %d\n",a,b,a+b);
	printf("The Subtraction of %d and %d is = %d\n",a,b,a-b);
	printf("The Multiplication of %d and %d is = %d\n",a,b,a*b);
	printf("The Quotient of %d and %d is = %f\n",a,b,a/b);
	printf("The Remaindern of %d and %d is = %d\n",a,b,a%b);
}
