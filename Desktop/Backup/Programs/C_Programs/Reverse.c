#include<stdio.h>
int main()
{
    int a[5]={5,9,7,4,2};
    int* ptr;
    ptr=&a[0];
    for(int i=0;i<=4;i++,ptr++)
    {
        printf("%p\n",ptr);
    }
    for(int i=4;i>=0;i--)
    {
        printf("%d",a[i]);
    }
}