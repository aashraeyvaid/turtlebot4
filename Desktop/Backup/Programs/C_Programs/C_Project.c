#include<stdio.h>
void DayPrint()
{
    char day[7]={'M','T','W','T','F','S','S'};
    for(int i=0;i<=6;i++)
    {
        printf("%c\t",day[i]);
    }
    printf("\n");
}
void assign(int i)
{
    int date[31];
    date[i-1]=i;
    printf("%d\t",date[i-1]);
}
int main()
{
    char month[12][9]={"January","February","March","April","May","June","July","August","September","October","November","December"};
    int n;
    printf("Months:- \n\n");
    for(int i=0;i<=11;i++)
    {
        printf("%d- %s\n",i+1,month[i]);
    }
    printf("\nSelect the Month you wanna view:- ");
    scanf("%d",&n);
    DayPrint();
    printf("\n");
    switch(n)
    {
        case 1:
        {
            int j=0;
            for(int i=1;i<=31;i++)
            {
                assign(i);
                if(i==(7+(7*j)))
                {
                    printf("\n");
                    j++;
                }
            }
            printf("\n");
            break;
        }
        case 2:
        {
            int j=0;
            printf("\t\t\t");
            for(int i=1;i<=29;i++)
            {
                assign(i);
                if(i==4+(7*j))
                {
                    printf("\n");
                    j++;
                }
                
            }
            printf("\n");
            break;
        }
        case 3:
        {
            int j=0;
            printf("\t\t\t\t");
            for(int i=1;i<=31;i++)
            {
                assign(i);
                if(i==3+(7*j))
                {
                    printf("\n");
                    j++;
                }
            }
            printf("\n");
            break;
        }
        case 4:
        {
            int j=0;
            for(int i=1;i<=30;i++)
            {
                assign(i);
                if(i==7+(7*j))
                {
                    printf("\n");
                    j++;
                }
            }
            printf("\n");
            break;
        }
        case 5:
        {
            int j=0;
            printf("\t\t");
            for(int i=1;i<=31;i++)
            {
                assign(i);
                if(i==5+(7*j))
                {
                    printf("\n");
                    j++;
                }
            }
            printf("\n");
            break;
        }
        case 6:
        {
            int j=0;
            printf("\t\t\t\t\t");
            for(int i=1;i<=30;i++)
            {
                assign(i);
                if(i==2+(7*j))
                {
                    printf("\n");
                    j++;
                }
            }
            printf("\n");
            break;
        }
        case 7:
        {
            int j=0;
            for(int i=1;i<=31;i++)
            {
                assign(i);
                if(i==7+(7*j))
                {
                    printf("\n");
                    j++;
                }
            }
            printf("\n");
            break;
        }
        case 8:
        {
            int j=0;
            printf("\t\t\t");
            for(int i=1;i<=31;i++)
            {
                assign(i);
                if(i==4+(7*j))
                {
                    printf("\n");
                    j++;
                }
            }
            printf("\n");
            break;
        }
        case 9:
        {
            int j=0;
            printf("\t\t\t\t\t\t");
            for(int i=1;i<=30;i++)
            {
                assign(i);
                if(i==1+(7*j))
                {
                    printf("\n");
                    j++;
                }
            }
            printf("\n");
            break;
        }
        case 10:
        {
            int j=0;
            printf("\t");
            for(int i=1;i<=31;i++)
            {
                assign(i);
                if(i==6+(7*j))
                {
                    printf("\n");
                    j++;
                }
            }
            printf("\n");
            break;
        }
        case 11:
        {
            int j=0;
            printf("\t\t\t\t");
            for(int i=1;i<=30;i++)
            {
                assign(i);
                if(i==3+(7*j))
                {
                    printf("\n");
                    j++;
                }
            }
            printf("\n");
            break;
        }
        case 12:
        {
            int j=0;
            printf("\t\t\t\t\t\t");
            for(int i=1;i<=31;i++)
            {
                assign(i);
                if(i==1+(7*j))
                {
                    printf("\n");
                    j++;
                }
            }
            printf("\n");
            break;
        }
    }
}