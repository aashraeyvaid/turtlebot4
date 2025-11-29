#include<stdio.h>
int main()
{
    int sum=0,count=0,avg;
    for(int i=1;i<300;i+=2)
    {
        if(i%7==0)
        {
            sum+=i;
            count+=1;
        }
    }
    avg=sum/count;
    printf("%d %d",sum,avg);
}