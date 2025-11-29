#include<stdio.h>
int digcnt(int n)
{
    if(n/10==0)
    {
        return 1;
    }
    int count=0;
    return count=1+digcnt(n/10);
}
int main()
{
    printf("%d",digcnt(222));
}