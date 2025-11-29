#include<stdio.h>
int main()
{
    struct personal
    {
        char name[1][30];
        char date[1][30];
        float salary;
    };
    struct personal emp1;
    printf("Enter the name:- ");
    fgets(emp1.name[0],sizeof(emp1.name),stdin);
    printf("Enter the Date of Joining:- ");
    fgets(emp1.date[0],sizeof(emp1.date),stdin);
    printf("Enter the Salary:- ");
    scanf("%f",&emp1.salary);
    printf("\n\n");
    printf("The name is %s",emp1.name[0]);
    printf("The Date of Joining is %s",emp1.date[0]);
    printf("The Salary is %f",emp1.salary);
}