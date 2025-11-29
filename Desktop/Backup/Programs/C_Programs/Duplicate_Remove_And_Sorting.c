#include<stdio.h>
int main()
{
    int a[200],count=0;
    int i=0,min;
    char x;
    while(1)
    {
        printf("Enter the number you wanna enter:- ");
        scanf("%d",&a[i]);
        count++;
        printf("\n");
        printf("Do you wanna enter more?? ");
        scanf(" %c",&x);
        if(x=='n')
        {
            break;
        }
        i++;
    }
    for(int i=0;i<count;i++)
    {
        for(int j=i+1;j<count;j++)
        {
            if(a[j]<a[i])
            {
                a[j]=a[j]+a[i];
                a[i]=a[j]-a[i];
                a[j]=a[j]-a[i];
            }
        }
    }
    for(int i=0;i<count;i++)
    {
        for(int j=i+1;j<count;j++)
        {
            if(a[j]==a[i])
            {
                for(int k=j;k<count;k++)
                {
                    a[k]=a[k+1];
                }
                count--;
            }
        }
    }
    for(int i=0;i<count;i++)
    {
        printf("%d",a[i]);
    }
}