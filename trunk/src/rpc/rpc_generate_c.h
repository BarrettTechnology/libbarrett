/* Note: The following enums should go in rpc.h eventually ... */

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


/* Parameter-expanding stuff */
#define A_P1(a)     A(a,0)
#define A_P2(a,b)   A(a,0) A(b,1)
#define A_P3(a,b,c) A(a,0) A(b,1) A(c,2)
#define B_P1(a)     B(a,0)
#define B_P2(a,b)   B(a,0) B(b,1)
#define B_P3(a,b,c) B(a,0) B(b,1) B(c,2)
#define C_P1(a)     C(a,0)
#define C_P2(a,b)   C(a,0), C(b,1)
#define C_P3(a,b,c) C(a,0), C(b,1), C(c,2)

/* C type names */
#define STR_CHAR char
#define STR_INT int
#define STR_DOUBLE double

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

/* ======================================================================== */
/* Define params lists */
#define CFUNC( n, ptr, ht, ps )
#define FUNC( n, ptr, ret, ps ) \
static struct f_tab_entry_type params_ ## n [] =                            \
{                                                                           \
   A_ ## ps                                                                 \
   {0,0,0,0,0}                                                              \
};                                                                     XXXX
#define A(t,n) A_ ## t ,
#define A_T(t)                  { RPC_T_ ## t, RPC_TR_VALUE, 0, 0, 0 }
#define A_R(t,tr,td,max,argnum) { RPC_T_ ## t,  RPC_TR_ ## tr, RPC_TD_ ## td, max, argnum }
#define A_G(t,td,start,argnum)  { RPC_T_ ## t,  RPC_TR_GETLINE, RPC_TD_ ## td, start, argnum }
#define A_H(t)                  { RPC_T_HANDLE, RPC_TR_VALUE, 0, 0, 0 }

RPC_FUNCS

#undef CFUNC
#undef FUNC
#undef A
#undef A_T
#undef A_R
#undef A_G
#undef A_H

/* ======================================================================== */
/* Define calling functions (not for async-only!) */

#define CFUNC( n, ptr, ht, ps )
#define FUNC( n, ptr, ret, ps )                                             \
static int calling_ ## n ( void * callhandle )                         XXXX \
{                                                                      XXXX \
   RET_ ## ret r;                                                           \
   A_ ## ps                                                            XXXX \
   bt_rpc_calling_pre( callhandle, B_ ## ps &r );                      XXXX \
   r = ptr ( C_ ## ps );                                               XXXX \
   bt_rpc_calling_post( callhandle, B_ ## ps &r );                     XXXX \
   return 0;                                                           XXXX \
}                                                                 XXXX XXXX

#define RET_T(t) STR_ ## t
#define RET_R(t,tr,td,max,argnum) STR_ ## t *
#define A(x,n) A_PRE_ ## x arg ## n A_PST_ ## x;
#define A_PRE_T(t) STR_ ## t
#define A_PST_T(t)
#define A_PRE_R(t,tr,td,max,argnum) STR_ ## t
#define A_PST_R(t,tr,td,max,argnum) [max]
#define A_PRE_G(t,td,start,argnum) STR_ ## t
#define A_PST_G(t,td,start,argnum) **
#define A_PRE_H(t) t *
#define A_PST_H(t)
#define B(x,n) & arg ## n,
#define C(x,n) arg ## n

RPC_FUNCS

#undef CFUNC
#undef FUNC
#undef RET_T
#undef RET_R
#undef A
#undef A_PRE_T
#undef A_POS_T
#undef A_PRE_R
#undef A_POS_R
#undef A_PRE_H
#undef A_POS_H
#undef B
#undef C
 
/* ======================================================================== */
/* Make individual function table entries (with ptrs to above) */
#define CFUNC( n, ptr, ht, ps )
#define FUNC( n, ptr, ret, ps )                                             \
static struct f_tab_entry entry_ ## n =                                     \
{                                                                           \
   #n,                                                                      \
   & calling_ ## n,                                                         \
   params_ ## n                                                             \
};                                                                     XXXX

RPC_FUNCS

#undef CFUNC
#undef FUNC

/* ======================================================================== */
/* Make function table from entries (for callee lookup) */
#define CFUNC( n, ptr, ht, ps )
#define FUNC( n, ptr, ret, ps ) & entry_ ## n , XXXX
struct f_tab_entry * f_tab[] =
{
   RPC_FUNCS
   0
}
#undef CFUNC
#undef FUNC

/* ======================================================================== */
/* Define callable functions (with ptr to structures) */
/* Creation functions */
#define CFUNC( n, ptr, ht, ps )                                             \
ht * n ( C_ ## ps ) {                                                       \
   return (ht *) bt_rpc_caller_create( & entry_ ## n, d ## ps );            \
} XXXX

/* Non-creation functions */
#define FUNC( n, ptr, ret, ps )                                             \
RET_ ## ret n( C_ ## ps )                                                   \
{                                                                           \
   RET_ ## ret r;                                                           \
   bt_rpc_caller_handle( & entry_ ## n, A_ ## ps &r )                       \
   return r;                                                                \
} XXXX

#define RET_T(t) STR_ ## t
#define RET_R(t,tr,td,max,argnum) STR_ ## t *
#define C(x,n) C_ ## x arg ## n
#define C_T(t) STR_ ## t
#define C_R(t,tr,td,max,argnum) STR_ ## t *
#define C_G(t,td,start,argnum) STR_ ## t **
#define C_H(t) t *
#define A(x,n) arg ## n,

RPC_FUNCS

#undef CFUNC
#undef FUNC
#undef RET_T
#undef RET_R
#undef C
#undef C_T
#undef C_R
#undef C_G
#undef C_H
#undef A


