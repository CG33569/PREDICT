#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
int main()
{
   int num;
   FILE *fptr;
   char gps_data;
   //------ to filter data ------------------
   char str[500];
   int str_i[500];
   char str_c[500];
   char cRMC[6] = "$GNRMC";
   char cVTG[6] = "$GNVTG";
   char cGGA[6] = "$GNGGA";
   char cGGL[6] = "$GNGGL";
   char cGSA[6] = "$GNGSA";
   int str_temp[6];
   char str_tempc[6];
   //----------------------------------------
   fptr = fopen("sample_to_process.txt","r");

   if(fptr == NULL)
   {
      printf("Error!");
      exit(1);
   }

   // Read contents from file
   int start_temp = 0;
   int end_temp = 6;
   bool b_temp = false;
   bool printed = false;
   bool equal_RMC = false;
   bool equal_VTG = false;

   int start_c = 6;
   int end_c = 300;

   int loop_counter;

   do{
       loop_counter++;
       gps_data = fgetc(fptr);
       printf ("%c", gps_data);

       if(gps_data == 36){
         b_temp = true;
       }
       if(b_temp && (start_temp<end_temp)&&!equal_RMC&&!equal_VTG){
         str_temp[start_temp]=gps_data;
         str_tempc[start_temp]=(char)gps_data;
         start_temp++;
       }

       if(equal_RMC&&(start_c<end_c)){
         str_c[start_c] = (char) gps_data;
         start_c++;
       }
       //find the RMC string
       if((start_temp==end_temp)&&!equal_RMC){//&&!printed){
         //printed = true;
         b_temp = false;
         int comp = 0;
         for(int j=0;j<6;j++){
            comp = comp + str_tempc[j] - cRMC[j];
            str_c[j] = str_tempc[j];
         }
         if(comp == 0){
           equal_RMC = true;
           printf("\n Found RMC \n");
         }
         start_temp = 0; // Try again?

        // printf("Found?: %d\n", equal_RMC);
       }

       //find the VTG string
       if((start_temp==end_temp)&&equal_RMC&&!equal_VTG){//&&!printed){
         //printed = true;
         b_temp = false;
         int comp = 0;
         for(int j=0;j<6;j++){
            comp = comp + str_tempc[j] - cVTG[j];
            //str_c[j] = str_tempc[j];
         }
         if(comp == 0){
           equal_VTG = true;
           printf("\n\n");
         }else{
           equal_RMC = false; //If the next string is not VTG, it must go back false
           start_c = 6;
         }
         start_temp = 0; // Try again?
       }
   }while (gps_data != EOF);

   printf("\n========\n");
   printf("str_temp : ");
   for(int i=0;i<6;i++){
     printf("%d ", str_temp[i]);
   }
   printf("\nstr_tempc = ");
   for(int i=0;i<6;i++){
     printf("%c", str_tempc[i]);
   }
   printf("\nstr_c =  \n");

   for(int i=0;i<300;i++){
     printf("%c", str_c[i]);
   }
   printf("\n");

   fclose(fptr);
   return 0;

   return 0;
}
