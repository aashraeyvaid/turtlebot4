#include<stdio.h>
#include<string.h>
int main()
{
    char a[100],b[100];
    printf("Enter the string:- ");
    fgets(a,sizeof(a),stdin);
    int count=0;
    strcpy(b,a);
    strlwr(a);
    for(int i=0;i<strlen(a);i++)
    {
        if(a[i]=='a' || a[i]=='e' || a[i]=='i' || a[i]=='o' || a[i]=='u')
        {
            count+=1;
        }
    }
    printf("The number of vowels in the entered string are:- %d",count);
}