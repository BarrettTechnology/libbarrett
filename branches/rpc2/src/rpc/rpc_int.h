
/* Here we have definitions of the rpc_interface */

/* C type names */
#define STR_CHAR char
#define STR_INT int
#define STR_DOUBLE double


enum rpc_valrtype {
   RPC_TR_VALUE = 0,
   /* Static array types (in and/or out): */
   RPC_TR_CONST,    /* size gives exact size */
   RPC_TR_NULLTERM, /* size gives max size */
   RPC_TR_EXPSIZE,  /* size gives arg location, zero-indexed */
   /* Dynamic array types: */
   RPC_TR_GETLINE   /* size gives arg location, zero-indexed */
};
enum rpc_valrdir {
   RPC_TD_IN = 0,
   RPC_TD_OUT,
   RPC_TD_INOUT
};
enum rpc_valtypebase {
   RPC_T_NULL = 0, /* Used to terminate list of types */
   /* Basic types */
   RPC_T_CHAR,
   RPC_T_INT,
   RPC_T_DOUBLE,
   /* Substituted handle types */
   RPC_T_HANDLE
};


/* Here is the function table;
 * holds types, names, etc */
struct f_tab_entry_type
{
   enum rpc_valtypebase base;
   enum rpc_valrtype type;
   enum rpc_valrdir dir;
   int maxsize; /* or start for getline type */
   int numarg; /* location of the size, 0-indexed, used only for expsize */
}
struct f_tab_entry
{
   char name[50]; /* custom-size? */
   int (*calling)(); /* calling function (not for async-only) */
   struct f_tab_entry_type * params;
   struct f_tab_entry_type rettype;
};

/* This is the interface;
 * it consists of a function table. */
struct bt_rpc_interface
{
   struct f_tab_entry * f_tab[];
}

