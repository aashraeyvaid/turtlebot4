#include<stdio.h>
#include<math.h>
int main()
{
	/*	Name:- Aashraey Vaid
		Class:- 24AML-101-A
		UID:- 24BAI70187
	*/
	double num;
	int n;
	printf("Enter the number:- ");
	scanf("%lf\n",&num);
	printf("Enter the value of n:- ");
	scanf("%d\n",&n);
	printf("The %dth root of %lf is %lf.",n,num,pow(num,1.0/n));
}
