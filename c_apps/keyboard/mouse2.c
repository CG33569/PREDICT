#include<graphics.h>
#include<conio.h>
#include<stdio.h>
#include<dos.h>

int initmouse();
void showmouseptr();
void hidemouseptr();
void getmousepos(int*,int*,int*);

union REGS i, o;

int main()
{
   int gd = DETECT, gm, status, button, x, y, tempx, tempy;
   char array[50];

   initgraph(&gd,&gm,"C:\\TC\\BGI");
   settextstyle(DEFAULT_FONT,0,2);

   status = initmouse();

   if ( status == 0 )
      printf("Mouse support not available.\n");
   else
   {
      showmouseptr();

      getmousepos(&button,&x,&y);

      tempx = x;
      tempy = y;

      while(!kbhit())
      {
         getmousepos(&button,&x,&y);

         if( x == tempx && y == tempy )
         {}
         else
         {
            cleardevice();
            sprintf(array,"X = %d, Y = %d",x,y);
            outtext(array);
            tempx = x;
            tempy = y;
         }
      }
   }

   getch();
   return 0;
}

int initmouse()
{
   i.x.ax = 0;
   int86(0X33, &i, &o);
   return ( o.x.ax );
}

void showmouseptr()
{
   i.x.ax = 1;
   int86(0X33, &i, &o);
}

void getmousepos(int *button, int *x, int *y)
{
   i.x.ax = 3;
   int86(0X33, &i, &o);

   *button = o.x.bx;
   *x = o.x.cx;
   *y = o.x.dx;
}
