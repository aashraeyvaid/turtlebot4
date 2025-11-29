#include<stdio.h>
int main()
{
    int C,D,pro;
    printf("Enter 2 numbers:- ");
    scanf("%d %d",&C,&D);
    pro=C*D;
    C=pro/C;
    D=pro/C;
    printf("%d %d",C,D);
}