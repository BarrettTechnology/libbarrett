/* ======================================================================== *
 *  Module ............. bt-wam-list
 *  File ............... bt-wam-list.c
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... 14 Oct 2005
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  NOTES:
 *    This is the primary WAM demo application. It shows off many
 *    features of the WAM library.
 *    This application is also used for diagnostics and testing.
 *
 *  REVISION HISTORY:
 *    2008 Sept 16 - CD
 *      Ported from btsystem to libbt, copied from btdiag to demo
 *                                                                          *
 * ======================================================================== */

/** \file bt-wam-list.c
 */

/* Include the high-level WAM header file */
#include <libbt/wam.h>

/* Program entry point */
int main(int argc, char ** argv)
{
   struct bt_wam_list * list;
   char buf1[100];
   char buf2[100];
   int i;
   
   list = bt_wam_list_create(0);
   if (!list)
   {
      printf("Could not get wam list.\n");
      return -1;
   }
   
   for (i=0; i<bt_wam_list_get_num(list); i++)
   {
      printf("Entry %d: %s, status %d, id %d, name %s\n",
             i,
             bt_wam_list_get_name(list,i,buf1),
             bt_wam_list_get_status(list,i),
             bt_wam_list_get_pid(list,i),
             bt_wam_list_get_programname(list,i,buf2));
   }
   
   bt_wam_list_destroy(list);
   
   return 0;
}
