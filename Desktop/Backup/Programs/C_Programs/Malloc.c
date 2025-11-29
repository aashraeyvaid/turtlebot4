#include<stdio.h>
#include<stdlib.h>
int main()
{
    char *str;
    int cols;
    printf("Define the length of the string:- ");
    scanf("%d",&cols);
    getchar();
    str=(char*)malloc(cols*sizeof(char));
    printf("Enter the string:- ");
    fgets(str,cols,stdin);
    printf("\nThe string is %s\n",str);
    free(str);
    printf("\n\nDefine the length of the new string:- ");
    scanf("%d",&cols);
    getchar();
    str=(char*)malloc(cols*sizeof(char));
    printf("Enter the string:- ");
    fgets(str,cols,stdin);
    printf("\nThe new string is %s\n",str);
    free(str);
}