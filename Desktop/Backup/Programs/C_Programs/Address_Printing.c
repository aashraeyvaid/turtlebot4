#include<stdio.h>
int main()
{
    float a=30.5;
    printf("%d\n",a);
    printf("%p\n",&a);
    float* ptr;
    ptr=&a;
    printf("%p\n",ptr);
    printf("%d",*ptr);
}