#include<stdio.h>
int main()
{
    int a[100],count=0,sum=0;
    char y;
    for(int i=0;i==i;i++)
    {
        printf("Enter the value:- ");
        scanf("%d",&a[i]);
        count+=1;
        printf("Do you wanna enter more?? ");
        scanf(" %c",&y);
        if(y=='n')
        {
            break;
        }
    }
    for(int i=0;i<count;i++)
    {
        sum+=a[i];
    }
    printf("%d",sum);
}