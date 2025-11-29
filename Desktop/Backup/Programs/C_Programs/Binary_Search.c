#include<stdio.h>
int main()
{
    int a[100],count=0,i=0,n;
    char c;
    while(1)
    {
        printf("Enter the vadlue:- ");
        scanf("%d",&a[i]);
        count++;
        getchar();
        printf("Do you wanna enter more?? ");
        scanf("%c",&c);
        if(c=='n')
        break;
        i++;
    }
    printf("The array is:- ");
    for(int i=0;i<count;i++)
    {
        printf("%d ",a[i]);
    }
    printf("\n");
    printf("The sorted array is:- ");
    for(int i=0;i<count;i++)
    {
        for(int j=i;j<count;j++)
        {
            if(a[i]>a[j])
            {
                a[i]=a[i]*a[j];
                a[j]=a[i]/a[j];
                a[i]=a[i]/a[j];
            }
        }
    }
    for(int i=0;i<count;i++)
    {
        printf("%d ",a[i]);
    }
    printf("\n");
    printf("Enter the number you wnna seardch:- ");
    scanf("%d",&n);
    if(count%2==0)
    {
        if(a[(n+1)/2]>n)
        {
            for(int i=0;i<(n+1)/2;i++)
            {
                if(a[i]==n)
                {
                    printf("The poisiton is %d",i+1);
                    break;
                }
            }
        }
        else
        {
            for(int i=(n+1)/2;i<count;i++)
            {
                if(a[i]==n)
                {
                    printf("The position is %d",i+1);
                    break;
                }
            }
        }
    }
}