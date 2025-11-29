#include<stdio.h>
#include<string.h>
int main()
{
    char a[1];
    printf("Enter the charectar:- ");
    scanf("%s",&a);
    printf("The lower case is %s.",strlwr(a));
}