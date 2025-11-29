#include<stdio.h>
int fnx(int n)
{
    if(n<=50)
    {
        printf("%d\n",n);
        fnx(n+1);
    }
}
int main()
{
    fnx(1);
}