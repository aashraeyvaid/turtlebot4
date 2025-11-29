#include<stdio.h>
#include<string.h>
int main()
{
    struct book{
        char name[50];
        char genre[50];
        int no;
    };
    struct book book1;
    strcpy(book1.genre,"Horror");
    strcpy(book1.name,"Bhoot");
    book1.no=100;
    printf("%s\n",book1.name);
    printf("%s\n",book1.genre);
    printf("%d\n",book1.no);
}