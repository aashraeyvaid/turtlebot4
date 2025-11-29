#include<stdio.h>
int main()
{
    int a,rem,q;
    printf("Enter a 3 digit number:- ");
    scanf("%d",&a);
    q=a;
    while(q!=0)
    {
        rem=q%10;
        q=q/10;
        printf("%d\n",rem);
    }
}