/* WAMDiscover                                                              *
 *                                                                          *
 * Author: Christopher Dellin                                               *
 *   Date: 2009-01-14                                                       * 
 *                                                                          *
 * Description.                                                             *
 *                                                                          */ 

/* Standard Windows header file */
#include <windows.h>
#include <winsock2.h>
#include <stdio.h> /* For sprintf */

#include "windows-resources.h"
#include "common.h"

#define ID_BUTTON_FIND 1
#define ID_LISTBOX 2
#define ID_BUTTON_CONNECT 3

/* Window Callback */
LRESULT CALLBACK WndProc(HWND hwnd, UINT iMsg, WPARAM wParam, LPARAM lParam);

HWND w; /* Window Instance Handle*/
HWND w_label;
HWND w_button_find;
HWND w_listbox;
HWND w_button_connect;

int call_putty()
{
   BOOL success;
   int i;
   int listbox_i;
   char systembuf[100];
   STARTUPINFO si;
   PROCESS_INFORMATION pi;
   
   if (!list_get_size())
   {
      MessageBox(w,"No WAMs discovered. Hit \"Refresh\" to try again.","Error",MB_OK|MB_ICONERROR);
      return 0;
   }
   
   listbox_i = (int) SendMessage(w_listbox, LB_GETCURSEL, 0, 0 );
   if (listbox_i == LB_ERR)
   {
      MessageBox(w,"No WAM selected.","Error",MB_OK|MB_ICONERROR);
      return 0;
   }
   
   i = (int) SendMessage(w_listbox, LB_GETITEMDATA, listbox_i, 0 );
   
   ZeroMemory( &si, sizeof(si) );
   si.cb = sizeof(si);
   ZeroMemory( &pi, sizeof(pi) );
   
   sprintf(systembuf,"putty %s",list_get_ipstr(i));
   success = CreateProcess(NULL, systembuf, NULL, NULL, FALSE, 0, NULL, NULL, &si, &pi);
   if (!success)
      MessageBox(w,"Could not find Putty. Ensure putty.exe is located in your PATH or alongside wamdiscover.exe.","Error",MB_OK|MB_ICONERROR);
   
   return 0;
}

/*  Program Entry Point */
int WINAPI WinMain(HINSTANCE hInstance,     /* Handle to current instance */
                   HINSTANCE hPrevInstance, /* Handle to previous instance */
		             LPSTR szCmdLine,         /* Command line arguments */
                   int iCmdShow)            /* Show state of window
                                               (max, min, etc) */
{
   int err;
   WSADATA wsaData;
   int optval;
   WNDCLASSEX w_class;  /* Window Class */
   RECT rcl;
   MSG msg;
   
   if (init_socket())
   {
      MessageBox(w,"Couldn't enable broadcast transmission.","Error",MB_OK|MB_ICONERROR);
      return 1;
   }
   
   /* Fill in Window Class */
   ZeroMemory(&w_class,sizeof(w_class));
   w_class.cbSize        = sizeof(w_class);
   w_class.style         = CS_HREDRAW | CS_VREDRAW;
   w_class.lpfnWndProc   = WndProc; /* Window Callback */
   w_class.cbClsExtra    = 0;
   w_class.cbWndExtra    = 0;
   w_class.hInstance     = hInstance; /* Make this window in my instance. */
   w_class.hIcon         = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_LOGO32));
   w_class.hIconSm       = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_LOGO32));
   w_class.hCursor       = LoadCursor(NULL, IDC_ARROW);
   /*w_class.hbrBackground = (HBRUSH) GetStockObject(WHITE_BRUSH);*/
   w_class.hbrBackground = (HBRUSH)(COLOR_BACKGROUND);
   w_class.lpszClassName = "WAMDiscoverWindowClass";
   w_class.lpszMenuName  = NULL;

   /* Register this Window Class with Windows  */
   RegisterClassEx(&w_class);

   /*  Create a window based on our new class  */
   w = CreateWindow("WAMDiscoverWindowClass",
                    "WAM Discovery Gateway", /* Window Title */
                    /* WS_CAPTION | WS_BORDER | WS_SYSMENU,*/ /* Window Style */
                    WS_OVERLAPPEDWINDOW,
                    CW_USEDEFAULT, CW_USEDEFAULT, /* (x,y) position */
                    /*CW_USEDEFAULT, CW_USEDEFAULT,*/ /* width, height */
                    400, 300, /* width, height */
                    NULL, /* Parent */
                    NULL, hInstance, NULL);
   
   GetClientRect(w, &rcl); 
   
   /* Create a label */
   w_label = CreateWindow(TEXT("STATIC"),
                          TEXT("WAM Discovery Gateway"),
                          WS_CHILD | WS_VISIBLE | SS_CENTER | SS_ENDELLIPSIS,
                          5, 5,
                          200, 25,
                          w,
                          NULL, hInstance, NULL);
   
   /* Create a find button */
   w_button_find = CreateWindow(TEXT("BUTTON"),
                               TEXT("Refresh List"),
                               WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                               5+200+5, 5,
                               150, 25,
                               w,
                               (HMENU)ID_BUTTON_FIND,
                               hInstance, NULL);
   
   /* Create a listbox */
   w_listbox = CreateWindow(TEXT("LISTBOX"),
                            TEXT("Listbox Test"),
                            WS_CHILD | WS_VISIBLE | LBS_STANDARD | LBS_DISABLENOSCROLL | LBS_NOINTEGRALHEIGHT,
                            0, 35,
                            rcl.right - rcl.left, 100,
                            w,
                            (HMENU)ID_LISTBOX,
                            hInstance, NULL);
   SendMessage( w_listbox, LB_SETITEMHEIGHT, 0, 30 );
   
   /* Create a connect button */
   w_button_connect = CreateWindow(TEXT("button"),
                                   TEXT("Connect"),
                                   WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
                                   5+200+5, 150,
                                   150, 25,
                                   w,
                                   (HMENU)ID_BUTTON_CONNECT,
                                   hInstance, NULL);

   /*  Show and update our window  */
   ShowWindow(w, iCmdShow);
   UpdateWindow(w);
   
   /* Forward socket events to the message-based notification system */
   WSAAsyncSelect( get_socket(), w, WM_USER+1, FD_READ );

   /*  Retrieve and process messages until we get WM_QUIT  */
   while ( GetMessage(&msg, NULL, 0, 0) )
   {
      TranslateMessage(&msg);    /*  for certain keyboard messages  */
      DispatchMessage(&msg);     /*  send message to WndProc        */
   } 
   
   close_socket();
   
   /*  Exit with status specified in WM_QUIT message  */
   return msg.wParam;
}

/* Window Callback */
LRESULT CALLBACK WndProc(HWND hwnd, UINT iMsg, WPARAM wParam, LPARAM lParam)
{
   PAINTSTRUCT ps;
   HDC         hdc;
   
   /*  Switch according to what type of message we have received  */
   switch ( iMsg )
   {
      case WM_CREATE:
      {
         if (send_broadcast() == -1)
            MessageBox(w,"Could not send broadcast packet. Check local firewall settings.","Error",MB_OK|MB_ICONERROR);
         return 0;
      }
      case WM_SIZE:
      {
         RECT rcl;
         int fw; /* full-width */
         int hw; /* half-width */
         int right_offset; /* right-column offset */
         int lb_height;
         int bottom_offset;
         
         GetClientRect(w, &rcl);
         
         /* Calculate horizontals */
         fw = (rcl.right-rcl.left) - 10;
         hw = ((rcl.right-rcl.left) - 15) / 2;
         right_offset = 10 + hw;
         
         /* Calculate verticals */
         lb_height = (rcl.bottom-rcl.top) - 40 - 40;
         bottom_offset = 40 + lb_height + 5;
         
         MoveWindow(w_label,                  5, 10, hw,  20, TRUE);
         MoveWindow(w_button_find, right_offset,  5, hw,  30, TRUE);
         MoveWindow(w_listbox,                5, 40, fw, lb_height, TRUE);
         
         MoveWindow(w_button_connect, right_offset, bottom_offset, hw, 30, TRUE );
         
         return 0;
      }
      case WM_USER+1:
      {
         int i;
         int listbox_i;
         int num;
         
         #define ADDBUF_SIZE 100
         char addbuf[ADDBUF_SIZE];
         
         i = receive_packet();
         if (i < 0)
         {
            MessageBox(w,"Error receiving packet","Error",MB_OK|MB_ICONERROR);
            return 0;
         }
         
         /* Put the entry in the list box, with its index as item data */
         strncpy(addbuf, list_get_display_str(i), ADDBUF_SIZE);
         listbox_i = (int) SendMessage(w_listbox, LB_ADDSTRING, 0, (LPARAM) TEXT(addbuf) );
         SendMessage(w_listbox, LB_SETITEMDATA, listbox_i, (LPARAM)i);
         
         if (i == 1)
            SendMessage(w_listbox,LB_SETSEL,TRUE,0);
         
         return 0;
      }
      
      case WM_COMMAND:
         switch (LOWORD(wParam))
         {
            case ID_BUTTON_FIND:
            {
               /* First, reset the list box */
               SendMessage(w_listbox, LB_RESETCONTENT, 0, 0 );
               
               if ( send_broadcast() == -1 )
                  MessageBox(w,"Could not send broadcast packet. Check local firewall settings.","Error",MB_OK|MB_ICONERROR);
               
               return 0;
            }
            case ID_BUTTON_CONNECT:
               call_putty();
               return 0;
            case ID_LISTBOX:
            {
               switch (HIWORD(wParam))
               {
                  case LBN_DBLCLK:
                     call_putty();
                     return 0;
               }

               return 0;
            }
         }
         return 0;
            

      case WM_DESTROY:
         /* Window has been destroyed, so exit cleanly */
         PostQuitMessage(0);
	      return 0;
   }
   
   /*  Send any messages we don't handle to default window procedure  */
   return DefWindowProc(hwnd, iMsg, wParam, lParam);
}


