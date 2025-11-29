#include<stdio.h>
int main()
{
	/*	Name:- Aashraey Vaid
		Class:- 24AML-101-A
		UID:- 24BAI70187
	*/
	int a = 40;
	int b = 0;
	printf("\n Enter number of positions for the bits to shift to the left : ");
	scanf("%d", &b);
	printf(" The result of left shift operation is : ");
	printf("%d << %d = %d", a, b, a<<b);
	printf("\n The result of right shift operation is : ");
	printf("%d >> %d = %d", a, b, a>>b);
}

