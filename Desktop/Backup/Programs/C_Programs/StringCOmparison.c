#include<stdio.h>
#include<string.h>
int main()
{
    char a[100],b[100];
    printf("Enter");
    fgets(a,sizeof(a),stdin);

    printf("Enter");
    fgets(b,sizeof(b),stdin);

    if (strcmp(a,b)==0)
    {
        printf("Equal");
    }
    else{
        "Not equal";
    }
    //printf("%s",strcat(a,b));
    //printf("%s",strcmpi(a,b));
    printf("%s",strlwr(a));
} 