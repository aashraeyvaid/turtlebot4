#include<stdio.h>
int sum(int n)
{
    if(n==1) return 1;
    int add=0;
    return add=n+sum(n-1);
}
int main()
{
    printf("%d",sum(10));
}