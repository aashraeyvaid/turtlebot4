#include<stdio.h>
int swap1(int a,int b);
int swap2(int* a, int* b);       // function declaration
int main()
{
    int i, j, result;
    printf("Enter 2 numbers that you want to compare:- ");
    scanf("%d %d", &i, &j);
    swap1(i,j);
    printf("Call by Value Swap:- a=%d b=%d\n",i,j);
    swap2(&i,&j);
    printf("Call by Reference Swap:- a=%d b=%d",i,j);
    return 0;
}
int swap1(int x,int y)
{
    int temp;
    temp=x;
    x=y;
    y=temp;
}
int swap2(int* x,int* y)
{
    int temp;
    temp=*x;
    *x=*y;
    *y=temp;
}