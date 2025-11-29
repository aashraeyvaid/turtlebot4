#include<stdio.h>
int main()
{
    int fibonacci(int a=0, int b=1,int sum=0,n);
    printf("Enter the number of terms in Fibonacci Sequence:- ");
    scanf("%d",&n);
    printf("%d\n",a);
    printf("%d\n",b);
    for(int i=1;i<=n;i++)
    {
        sum=a+b;
        a=b;
        b=sum;
        printf("%d\n",sum);
    }
}