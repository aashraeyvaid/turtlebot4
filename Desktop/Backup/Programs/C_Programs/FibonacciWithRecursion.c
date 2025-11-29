#include<stdio.h>
int fib(int i)
{
    if(i==0)
    return 0;
    else if(i==1)
    return 1;
    else
    {
        return fib(i-1)+fib(i-2);
    }
}
int main()
{
    int a;
    printf("Enter the number of terms:- ");
    scanf("%d",&a);
    for(int i=0;i<a;i++)
    {
        printf("%d\n",fib(i));
    }
}