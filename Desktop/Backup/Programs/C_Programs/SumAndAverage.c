#include<stdio.h>
#include<stdlib.h>
int main()
{
    int sum;
    float avg;
    char a[10];
    while (1<2)
    {
        fgets(a,sizeof(a),stdin);
        if(a[0]=='q')
        {
            break;
        }
        else
        {
            sum=(a[0]-'0')+ (a[2]-'0') +(a[4]-'0');
            avg=sum/3;
            printf("%d ",sum);
            printf("%.2f\n",avg);
        }
    }
}