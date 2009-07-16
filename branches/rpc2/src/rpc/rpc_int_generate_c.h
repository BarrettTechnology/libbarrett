/* Note: The following enums should go in rpc.h eventually ... */

#if 0
/* An example (like from wam.c) */
#define RPC_FUNCS \
   CFUNC( bt_wam_create,   bt_wam_local_create,   struct bt_wam, P2(T(CHAR),T(INT)) ) \
    FUNC( bt_wam_destroy,  bt_wam_local_destroy,  T(INT),    P1(H(struct bt_wam))         ) \
    FUNC( bt_wam_do_thing, bt_wam_local_do_thing, T(INT),    P2(H(struct bt_wam), T(INT))  ) \
    FUNC( bt_wam_use,      bt_wam_local_use,      T(INT),    P2(H(struct bt_wam), H(struct bt_refgen)) ) \
    FUNC( bt_wam_goto,     bt_wam_local_goto,     T(INT),    P2(H(struct bt_wam), R(DOUBLE,CONST,IN,2,0)) ) \
    FUNC( bt_wam_getline,  bt_wam_local_getline,  T(INT),    P3(H(struct bt_wam), G(CHAR,OUT,100,2), R(INT,CONST,OUT,1,0)) ) \
    FUNC( bt_wam_getstr,   bt_wam_local_getstr,   R(DOUBLE,CONST,OUT,2,0), P1(H(struct bt_wam)) )
#endif

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
static struct f_tab_entry * f_tab[] =
{
   RPC_FUNCS
   0
}
#undef CFUNC
#undef FUNC

/* ======================================================================== */
/* Make the bt_rpc_interface itself */
static const struct bt_rpc_interface bt_rpc_interface_instance = {
   f_tab
};
const struct bt_rpc_interface * custom_name = &bt_rpc_interface_instance;

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


