#include<stdio.h>
int main()
{
    int x,y,j=0,count1=0,count2=0,count3=0,cfact[100];
    int a[200],b[200],max;
    printf("Enter the first number:- ");
    scanf("%d",&x);
    printf("Enter the second number:- ");
    scanf("%d",&y);
    for(int i=1;i<=x;i++)
    {
        if(x%i==0)
        {
            a[j]=i;
        }
        j++;
        count1++;
    }
    j=0;
    for(int i=1;i<=y;i++)
    {
        if(y%i==0)
        {
            b[j]=i;
        }
        j++;
        count2++;
    }
    int k=0;
    for(int i=0;i<count1;i++)
    {
        for(int j=0;j<count2;j++)
        {
            if(a[i]==b[j])
            {
                cfact[k]=a[i];
                max=cfact[k];
                count3++;
                k++;
            }
        }
    }
    
    for(int i=0;i<count3;i++)
    {
        if(cfact[i]>max)
        {
            max=cfact[i];
        }
    }
    printf("The HCF is %d",max);
}