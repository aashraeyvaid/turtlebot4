#include<stdio.h>
#include<math.h>
int main()
{
	int a,b,c,p;
	float s,area;
	printf("Enter the sides of triangle:- ");
	scanf("%d %d %d",&a,&b,&c);
	p=a+b+c;
	s=p/2;
	area=sqrt(s*(s-a)*(s-b)*(s-c));
	printf("The area is%f",area);
	return 0;
}
