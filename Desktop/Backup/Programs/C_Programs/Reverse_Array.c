#include<stdio.h>
#include<string.h>
int main()
{
    int a[100],i=0,count=1;
    int* p;
    char x;
    while(1)
    {
        printf("Enter the Value:- ");
        scanf("%d",&a[i]);
        printf("Do you wanna enter more?? ");
        scanf(" %c",&x);
        if(x=='n')
        {
            break;
        }
        i++;
        count++;
    }
    for(int j=0;j<count/2;j++)
    {
        a[i]=a[i]+a[j];
        a[j]=a[i]-a[j];
        a[i]=a[i]-a[j];
        i--;
    }
    for(int i=0;i<count;i++)
    {
        printf("%d\t",a[i]);
        p=&a[i];
        printf("%p\n",p);
    }
}