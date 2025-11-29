#include<stdio.h>
int main()
{
    int a[3][3]={{1,2,3},
                 {4,5,6},
                 {7,8,9}};
    int b[3][3]={{1,7,6},
                 {6,5,9},
                 {2,4,0}};
    int c[3][3]={{0,0,0},
                 {0,0,0},
                 {0,0,0}};
    for(int i=0;i<3;i++)
    {
        for(int k=0;k<3;k++)
        {
        for(int j=0;j<3;j++)
        {
            c[i][k]+=a[i][j]*b[j][k];
        }
        }
    }
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            printf("%d ",a[i][j]);
        }
        printf("\n");
    }
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            printf("%d ",b[i][j]);
        }
        printf("\n");
    }
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            printf("%d ",c[i][j]);
        }
        printf("\n");
    }
}