#include<stdio.h>
int main()
{
    int a;
    int f=0;
    printf("Enter:- ");
    scanf("%d",&a);
    for(int i=2;i<a;i++)
    {
        if(a%i==0)
        {
            f=1;
        }
        else
        {
            f=2;
        }
    }
    if(f==1)
    {
        printf("Not a Prime Number.");
    }
    else if(f==2)
    {
        printf("Not a Prime Number.");
    }
}