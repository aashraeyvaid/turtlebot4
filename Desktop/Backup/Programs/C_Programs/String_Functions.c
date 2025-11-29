#include<stdio.h>
#include<string.h>
int main()
{
    char a[100],b[100],c[100];
    printf("Enter the string:- ");
    fgets(a,sizeof(a),stdin);
    printf("Enter the string:- ");
    fgets(b,sizeof(b),stdin);
    printf("The comparison is %d\n",strcmp(a,b));
    printf("The length of the string is %d\n",strlen(b));
    printf("The copied string is %s",strcpy(c,a));
    printf("The added string is:-\n%s\n",strcat(a,b));
}