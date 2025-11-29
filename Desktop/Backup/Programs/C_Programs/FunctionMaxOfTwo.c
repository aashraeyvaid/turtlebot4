#include<stdio.h>
void *max(int *a,int *b)
{
    if(*a>*b)
    {
        printf("The maximum is %d",*a);
    }
    else
    {
        printf("The maximum is %d",*b);
    }
}
int main()
{
    int x,y;
    printf("Enter x ");
    scanf("%d",&x);
    printf("Enter y ");
    scanf("%d",&y);
    max(&x,&y);
    printf("\n%d\n",max);
    printf("\n%d\n",&x);
    printf("\n%d\n",&y);
}