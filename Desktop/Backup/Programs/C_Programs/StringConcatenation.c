#include<stdio.h>
int main()
{
    char a[100],b[100],c[250];
    printf("Enter the frist string:- ");
    fgets(a,sizeof(a),stdin);
    printf("Enter the second string:- ");
    fgets(b,sizeof(b),stdin);
    int i=0;
    while(a[i]!='\n')
    {
        c[i]=a[i];
        i++;
    }
    int j=0;
    while(b[j]!='\0')
    {
        c[i]=b[j];
        i++;
        j++;
    }
    int k=0;
    puts(c);
}