
/* This quasi-header file defines the WAM database
 * to be included in the wamdiscover client program.
 * It defines a table of WAM MAC addresses and numbers. */

int init_socket();

#ifdef WINDOWS
SOCKET get_socket();
#else
int get_socket();
#endif

int close_socket();

int send_broadcast();
int receive_packet();

int list_get_size();
/*int list_get_num(int idx);*/
char * list_get_display_str(int idx);
char * list_get_ipstr(int idx);
/*int list_idx_from_num(int num);*/

