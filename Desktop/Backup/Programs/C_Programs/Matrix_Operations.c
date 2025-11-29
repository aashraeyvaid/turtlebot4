#include<stdio.h>
#include<stdlib.h>
int main()
{
    int d,m1,n1,m2,n2;
    int a[100][100];
    int b[100][100];
    int c[100][100];
    printf("1- Addition\n2- Subtraction\n3-Multiplication\n4-Transpose\n5-Exit\n");
    printf("Enter your choice:- ");
    scanf("%d",&d);
    
    switch(d)
    {
        case 1:
            printf("Enter the order of 1st matrix:- ");
            scanf("%d %d",&m1,&n1);
            printf("Enter the order of 2nd matrix:- ");
            scanf("%d %d",&m2,&n2);
            if(m1==m2 && n1==n2)
            {
                for(int i=0;i<m1;i++)
                {
                    for(int j=0;j<n1;j++)
                    {
                        printf("Enter a[%d][%d]",i,j);
                        scanf("%d",&a[i][j]);
                    }
                }
                for(int i=0;i<m2;i++)
                {
                    for(int j=0;j<n2;j++)
                    {
                        printf("Enter b[%d][%d]",i,j);
                        scanf("%d",&b[i][j]);
                    }
                }
            }
            else
            {
                printf("Order is not correct for the operation.");
            }
            for(int i=0;i<m1;i++)
            {
                for(int j=0;j<n1;j++)
                {
                    c[i][j]=a[i][j]+b[i][j];
                }
            }
            printf("The added matrix is:- \n");
            for(int i=0;i<m1;i++)
            {
                for(int j=0;j<n1;j++)
                {
                    printf("%d ",c[i][j]);
                }
                printf("\n");
            }
            break;
        case 2:
            printf("Enter the order of 1st matrix:- ");
            scanf("%d %d",&m1,&n1);
            printf("Enter the order of 2nd matrix:- ");
            scanf("%d %d",&m2,&n2);
            if(m1==m2 && n1==n2)
            {
                for(int i=0;i<m1;i++)
                {
                    for(int j=0;j<n1;j++)
                    {
                        printf("Enter a[%d][%d]",i,j);
                        scanf("%d",&a[i][j]);
                    }
                }
                for(int i=0;i<m2;i++)
                {
                    for(int j=0;j<n2;j++)
                    {
                        printf("Enter b[%d][%d]",i,j);
                        scanf("%d",&b[i][j]);
                    }
                }
            }
            else
            {
                printf("Order is not correct for the operation.");
            }
            for(int i=0;i<m1;i++)
            {
                for(int j=0;j<n1;j++)
                {
                    c[i][j]=a[i][j]-b[i][j];
                }
            }
            printf("The subtracted matrix is:-\n");
            for(int i=0;i<m1;i++)
            {
                for(int j=0;j<n1;j++)
                {
                    printf("%d ",c[i][j]);
                }
                printf("\n");
            }
            break;
        case 3:
            printf("Enter the order of 1st matrix:- ");
            scanf("%d %d",&m1,&n1);
            printf("Enter the order of 2nd matrix:- ");
            scanf("%d %d",&m2,&n2);
            if(n1==m2)
            {
                for(int i=0;i<m1;i++)
                {
                    for(int j=0;j<n1;j++)
                    {
                        printf("Enter a[%d][%d]",i,j);
                        scanf("%d",&a[i][j]);
                    }
                }
                for(int i=0;i<m2;i++)
                {
                    for(int j=0;j<n2;j++)
                    {
                        printf("Enter b[%d][%d]",i,j);
                        scanf("%d",&b[i][j]);
                    }
                }
            }
            else
            {
                printf("Order is not correct for the operation.");
            }
            for(int i=0;i<n1;i++)
            {
                for(int j=0;j<m2;j++)
                {
                    c[i][j]=0;
                }
            }
            int sum=0;
            for(int i=0;i<n1;i++)
            {
                for(int k=0;k<m2;k++)
                {
                    for(int j=0;j<m2;j++)
                    {
                        c[i][k]+=a[i][j]*b[j][k];
                    }
                }
            }
            printf("The multiplied  matrix is:-\n");
            for(int i=0;i<m1;i++)
            {
                for(int j=0;j<n1;j++)
                {
                    printf("%d ",c[i][j]);
                }
                printf("\n");
            }
            break;
        case 4:
            printf("Enter the order of 1st matrix:- ");
            scanf("%d %d",&m1,&n1);
            printf("Enter the order of 2nd matrix:- ");
            scanf("%d %d",&m2,&n2);
            {
                for(int i=0;i<m1;i++)
                {
                    for(int j=0;j<n1;j++)
                    {
                        printf("Enter a[%d][%d]",i,j);
                        scanf("%d",&a[i][j]);
                    }
                } 
            }
            for(int i=0;i<m1;i++)
            {
                for(int j=0;j<n1;j++)
                {
                    c[i][j]=a[j][i];
                }
            }
            printf("The transposed matrix is:-\n");
            for(int i=0;i<m1;i++)
            {
                for(int j=0;j<n1;j++)
                {
                    printf("%d ",c[i][j]);
                }
                printf("\n");
            }
            break;
        case 5:
            exit(0);
            break;
        default:
            printf("Wrong Choice!!!");
            break;
    }
}