#include<stdio.h>
int main()
{
    int a[100],i=0,count=0,x,y;
    char c;
    while(1)
    {
        printf("Enter the value:- ");
        scanf("%d",&a[i]);
        count+=1;
        getchar();
        printf("Do you want to enter more?? ");
        scanf("%c",&c);
        if(c=='n')
        {
            break;
        }
        i++;
    }
    for(int i=0;i<count;i++)
    {
        printf("%d",a[i]);
    }
    printf("\n");
    printf("Enter the number you wanna delete:- ");
    scanf("%d",&x);
    for(int i=0;i<count;i++)
    {
        if(a[i]==x)
        {
            y=i;
            break;
        }
    }
    printf("\n");
    for(int i=y;i<count;i++)
    {
        a[i]=a[i+1];
    }
    for(int i=0;i<count-1;i++)
    {
        printf("%d",a[i]);
    }
}