#include<stdio.h>
int main()
{
    int a[100],count=0;
    int x,y;
    char z;
    for(int i=0;i==i;i++)
    {
        printf("Enter the value:- ");
        scanf("%d",&a[i]);
        count+=1;
        printf("Do you wanna enter more?? ");
        scanf(" %c",&z);
        if(z=='n')
        {
            break;
        }
    }
    for(int i=0;i<count;i++)
    {
        printf("%d ",a[i]);
    }
    printf("\n");
    printf("Enter the number you wanna insert:- ");
    scanf("%d",&x);
    printf("Enter the position you wanna insert in:- ");
    scanf("%d",&y);
    for(int i=count-1;i>=y-1;i--)
    {
        a[i+1]=a[i];
    }
    a[y-1]=x;
    for(int i=0;i<=count;i++)
    {
        printf("%d ",a[i]);
    }
}