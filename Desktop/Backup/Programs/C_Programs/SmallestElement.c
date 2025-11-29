#include<stdio.h>
int main()
{
    int a[20],min;
    int* t;
    char z;
    for(int i=0;i==i;i++)
    {
        printf("Enter the value:- ");
        scanf("%d",&a[i]);
        t=&a[i];
        printf("Do you wanna enter more?? ");
        scanf(" %c",&z);
        if(z=='n')
        {
            break;
        }
    }
    int i=0;
    min=a[0];
    while(a[i]!=*t)
    {
        if(a[i]<min)
        {
            min=a[i];
        }  
        i++; 
    }
    printf("%d",min);
} 