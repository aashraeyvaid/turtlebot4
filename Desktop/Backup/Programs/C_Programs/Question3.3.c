#include <stdio.h>
#include <math.h>
int main(){
	int choice;
    float num;
    while (1){
    	printf("Enter a number: ");
    	scanf("%f", &num);
        printf("\nMenu:\n");
        printf("1. Check if the number is even or odd\n");
        printf("2. Check if the number is positive or negative\n");
        printf("3. Print the square of the number\n");
        printf("4. Print the square root of the number\n");
        printf("5. Exit\n");
        printf("Enter your choice: ");
        scanf("%d", &choice);
        switch (choice){
            case 1:
                if ((int)num % 2 == 0)
                    printf("%.2f is an even number.\n", num);
                else
                    printf("%.2f is an odd number.\n", num);
                break;
            case 2:
                if (num > 0)
                    printf("%.2f is a positive number.\n", num);
                else if (num < 0)
                    printf("%.2f is a negative number.\n", num);
                else
                    printf("The number is zero.\n");
                break;
            case 3:
                printf("The square of %.2f is %.2f.\n", num, num * num);
                break;
            case 4:
                if (num >= 0)
                    printf("The square root of %.2f is %.2f.\n", num, sqrt(num));
                else
                    printf("Invalid input. Cannot calculate square root of a negative number.\n");
                break;
            case 5:
                printf("Exiting the program.\n");
                goto end;
            default:
                printf("Invalid choice. Please enter a valid option.\n");}
        printf("\n");}
    end:
    return 0;
}
