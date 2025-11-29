#include<stdio.h>
int main()
{
	int i=1,sum=0;
	pass:
		sum=3+i;
		i++;
	if(i<=5)
	{
		goto pass;
	}
	printf("%d",sum);
}
