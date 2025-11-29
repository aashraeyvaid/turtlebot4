#include<stdio.h>
#include<math.h>
int fnx(int a,int b);
void fnx1();
int fnx3();
int main()
{
        int x,y;
        printf("Enter ");
        scanf("%d %d",&x,&y);
        printf("sum is %d",fnx(x,y));
        fnx1();
        printf("%d",fnx3());
}
int fnx(int a,int b)
{
        return a+b;
}
void fnx1()
{
        printf("Hello");
        return;
}
int fnx3()
{
        int c,d;
        printf("Enter ");
        scanf("%d %d",&c,&d);
        printf("%d",c+d);
        return c+d;
}