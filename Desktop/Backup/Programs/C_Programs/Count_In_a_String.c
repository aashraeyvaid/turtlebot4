#include<stdio.h>
int main()
{
    char a[100];
    printf("Enter the string:- ");
    fgets(a,sizeof(a),stdin);
    printf("\n");
    puts(a);
    printf("\n");
    int countchar=0,countword=0,countline=0;
    int i=0;
    while(a[i]!='\0')
    {
        countchar++;
        if(a[i]==' ')
        {
            countword+=1;
        }
        else if(a[i]=='\n')
        {
            countword+=1;
            countline+=1;
        }
        i++;
    }
    printf("The number of charectars are %d.\n",countchar);
    printf("The number of words are %d.\n",countword);
    printf("The number of lines are %d.",countline);
}