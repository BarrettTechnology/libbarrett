/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... file.h
 *  Author ............. Sam Clanton
 *                       Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... 2004 Q3
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2004-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *
 * ======================================================================== */

/* This provides an asynchronous temporary buffer file;
 * For now, it is designed to be created, written once,
 * read once, and destroyed.
 * Null characters are not allowed.
 * The internal representation uses \n for line breaks. */

enum bt_file_seek {
   BT_FILE_SEEK_SET,
   BT_FILE_SEEK_CUR,
   BT_FILE_SEEK_END
};

struct bt_file;

struct bt_file * bt_file_create(char * fileloc);
int bt_file_destroy(struct bt_file * file);

int bt_file_fputs(struct bt_file * file, char * line);
int bt_file_fseek(struct bt_file * file, int offset, enum bt_file_seek seek);
int bt_file_getline(struct bt_file * file, char ** lineptr, int * linelen);
