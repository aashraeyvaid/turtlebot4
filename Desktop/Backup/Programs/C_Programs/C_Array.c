#include<stdio.h>
int main()
{
    int val[5];
    printf("Enter:- ");
    for(int i=0;i<5;i++);
    {
        scanf("%d",&val[i]);
    }
    printf("\n");
    int sum=0;
    for(int i=1;i<=4;i++)
    {
        printf("%d",val[i]);
        sum+=val[i];
    }
    int avg;
    avg=sum/5;
    printf("%d",avg);
}