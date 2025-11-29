#include<stdio.h>
#include<stdlib.h>
int main()
{
    int *jkl;
    int n,a,f=0;
    printf("Define the length of the array:- ");
    scanf("%d",&n);
    jkl=(int*)malloc(n*sizeof(char));
    for(int i=0;i<n;i++)
    {
        printf("Enter the element:- ");
        scanf("%d",&jkl[i]);
    }
    printf("Enter the value to be searched:- ");
    scanf("%d",&a);
    for(int i=0;i<n;i++)
    {
        if(jkl[i]==a)
        {
            printf("Found!!!");
            f=0;
            break;
        }
        else
        f=1;
    }
    if(f==1)
    printf("Not Found!!!");
    free(jkl);
}