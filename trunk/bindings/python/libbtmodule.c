#include <Python.h>
#include "structmember.h"

#include "wam.h"

/* Prototypes of types that we support */
static PyTypeObject WamType;
static PyTypeObject WamListType;

/* The module ------------------------------------------------------------- */

/* Function prototypes */
static PyObject * libbt_version(PyObject * self, PyObject * args);

/* Function table */
static PyMethodDef LibbtMethods[] =
{
   {"version", libbt_version, METH_VARARGS, "Get the version of libbt."},
   {0, 0, 0, 0}
};

/* Module initialization */
#ifndef PyMODINIT_FUNC	/* declarations for DLL import/export */
#define PyMODINIT_FUNC void
#endif
PyMODINIT_FUNC
initlibbt(void)
{
   PyObject * m;
   
   if (PyType_Ready(&WamType) < 0) return;
   if (PyType_Ready(&WamListType) < 0) return;
   
   m = Py_InitModule("libbt", LibbtMethods);
   if (!m)
      return;
   
   /* Add the wam type */
   Py_INCREF(&WamType);
   PyModule_AddObject(m, "Wam",     (PyObject *)&WamType);
   PyModule_AddObject(m, "WamList", (PyObject *)&WamListType);
}

/* Get version */
static PyObject * libbt_version(PyObject * self, PyObject * args)
{
   return Py_BuildValue("i", 1337);
}

/* The wam class ---------------------------------------------------------- */

/* A WAM instance */
typedef struct
{
   PyObject_HEAD
   struct bt_wam * wam;
} Wam;

/* Wam members table */
static PyMemberDef Wam_members[] =
{
   {0,0,0,0,0}
};

/* Creation function */
static PyObject * Wam_new(PyTypeObject * type, PyObject * args, PyObject * kwds)
{
   Wam * self;
   
   /* Create */
   self = (Wam *) type->tp_alloc(type,0);
   if (!self)
      return 0;
   
   /* Initialize */
   self->wam = 0;
   
   return (PyObject *)self;
}

/* Init function, expects string location */
static int Wam_init(Wam * self, PyObject * args, PyObject * kwds)
{
   char * wamname;
   
   /* Grab the string */
   if (!PyArg_ParseTuple(args,"s",&wamname))
      return -1;
   
   /* Attempt to initialize the wam */
   self->wam = bt_wam_create(wamname);
   if (!self->wam)
      return -1;
   
   return 0;
}

/* Deallocate function */
static void Wam_dealloc(Wam * self)
{
   /* If we have a wam, destroy it */
   if (self->wam)
      bt_wam_destroy(self->wam);
   
   /* Free myself */
   self->ob_type->tp_free((PyObject *)self);
}

static PyObject * Wam_str_jposition(Wam * self);
static PyObject * Wam_str_jvelocity(Wam * self);
static PyObject * Wam_str_jtorque(Wam * self);
static PyObject * Wam_str_cposition(Wam * self);
static PyObject * Wam_str_crotation_r1(Wam * self);
static PyObject * Wam_str_crotation_r2(Wam * self);
static PyObject * Wam_str_crotation_r3(Wam * self);
static PyObject * Wam_isgcomp(Wam * self);
static PyObject * Wam_setgcomp(Wam * self, PyObject * args);
static PyObject * Wam_get_current_controller_name(Wam * self);
static PyObject * Wam_controller_toggle(Wam * self);
static PyObject * Wam_idle(Wam * self);
static PyObject * Wam_hold(Wam * self);
static PyObject * Wam_is_holding(Wam * self);
static PyObject * Wam_get_current_refgen_name(Wam * self);
static PyObject * Wam_movehome(Wam * self);
static PyObject * Wam_moveisdone(Wam * self);
static PyObject * Wam_is_teaching(Wam * self);
static PyObject * Wam_teach_start(Wam * self);
static PyObject * Wam_teach_end(Wam * self);
static PyObject * Wam_playback(Wam * self);

/* Wam methods table */
static PyMethodDef Wam_methods[] =
{
   {"str_jposition",     (PyCFunction)Wam_str_jposition,     METH_NOARGS,  "Return the number"},
   {"str_jvelocity",     (PyCFunction)Wam_str_jvelocity,     METH_NOARGS,  "Return the number"},
   {"str_jtorque",       (PyCFunction)Wam_str_jtorque,       METH_NOARGS,  "Return the number"},
   {"str_cposition",     (PyCFunction)Wam_str_cposition,     METH_NOARGS,  "Return the number"},
   {"str_crotation_r1",  (PyCFunction)Wam_str_crotation_r1,  METH_NOARGS,  "Return the number"},
   {"str_crotation_r2",  (PyCFunction)Wam_str_crotation_r2,  METH_NOARGS,  "Return the number"},
   {"str_crotation_r3",  (PyCFunction)Wam_str_crotation_r3,  METH_NOARGS,  "Return the number"},
   {"isgcomp",           (PyCFunction)Wam_isgcomp,           METH_NOARGS,  "Return the number"},
   {"setgcomp",          (PyCFunction)Wam_setgcomp,          METH_VARARGS, "Return the number"},
   {"get_current_controller_name",
                         (PyCFunction)Wam_get_current_controller_name,
                                                             METH_NOARGS,  "Return the number"},
   {"controller_toggle", (PyCFunction)Wam_controller_toggle, METH_NOARGS,  "Return the number"},
   {"idle",              (PyCFunction)Wam_idle,              METH_NOARGS,  "Return the number"},
   {"hold",              (PyCFunction)Wam_hold,              METH_NOARGS,  "Return the number"},
   {"is_holding",        (PyCFunction)Wam_is_holding,        METH_NOARGS,  "Return the number"},
   {"get_current_refgen_name",
                         (PyCFunction)Wam_get_current_refgen_name,
                                                             METH_NOARGS,  "Return the number"},
   {"movehome",          (PyCFunction)Wam_movehome,          METH_NOARGS,  "Return the number"},
   {"moveisdone",        (PyCFunction)Wam_moveisdone,        METH_NOARGS,  "Return the number"},
   {"is_teaching",       (PyCFunction)Wam_is_teaching,       METH_NOARGS,  "Return the number"},
   {"teach_start",       (PyCFunction)Wam_teach_start,       METH_NOARGS,  "Return the number"},
   {"teach_end",         (PyCFunction)Wam_teach_end,         METH_NOARGS,  "Return the number"},
   {"playback",          (PyCFunction)Wam_playback,          METH_NOARGS,  "Return the number"},
   {0, 0, 0, 0}
};

static PyObject * Wam_str_jposition(Wam * self)
{
   char buf[300];
   bt_wam_str_jposition(self->wam, buf);
   return Py_BuildValue("s",buf);
}

static PyObject * Wam_str_jvelocity(Wam * self)
{
   char buf[300];
   bt_wam_str_jvelocity(self->wam, buf);
   return Py_BuildValue("s",buf);
}

static PyObject * Wam_str_jtorque(Wam * self)
{
   char buf[300];
   bt_wam_str_jtorque(self->wam, buf);
   return Py_BuildValue("s",buf);
}

static PyObject * Wam_str_cposition(Wam * self)
{
   char buf[300];
   bt_wam_str_cposition(self->wam, buf);
   return Py_BuildValue("s",buf);
}

static PyObject * Wam_str_crotation_r1(Wam * self)
{
   char buf[300];
   bt_wam_str_crotation_r1(self->wam, buf);
   return Py_BuildValue("s",buf);
}

static PyObject * Wam_str_crotation_r2(Wam * self)
{
   char buf[300];
   bt_wam_str_crotation_r2(self->wam, buf);
   return Py_BuildValue("s",buf);
}

static PyObject * Wam_str_crotation_r3(Wam * self)
{
   char buf[300];
   bt_wam_str_crotation_r3(self->wam, buf);
   return Py_BuildValue("s",buf);
}

static PyObject * Wam_isgcomp(Wam * self)
{
   return Py_BuildValue("i",bt_wam_isgcomp(self->wam));
}

static PyObject * Wam_setgcomp(Wam * self, PyObject * args)
{
   int myint;
   /* Grab the int */
   if (!PyArg_ParseTuple(args,"i",&myint))
      return 0;
   return Py_BuildValue("i",bt_wam_setgcomp(self->wam,myint));
}

static PyObject * Wam_get_current_controller_name(Wam * self)
{
   char buf[300];
   bt_wam_get_current_controller_name(self->wam, buf);
   return Py_BuildValue("s",buf);
}

static PyObject * Wam_controller_toggle(Wam * self)
{
   return Py_BuildValue("i",bt_wam_controller_toggle(self->wam));
}

static PyObject * Wam_idle(Wam * self)
{
   return Py_BuildValue("i",bt_wam_idle(self->wam));
}

static PyObject * Wam_hold(Wam * self)
{
   return Py_BuildValue("i",bt_wam_hold(self->wam));
}

static PyObject * Wam_is_holding(Wam * self)
{
   return Py_BuildValue("i",bt_wam_is_holding(self->wam));
}

static PyObject * Wam_get_current_refgen_name(Wam * self)
{
   char buf[300];
   bt_wam_get_current_controller_name(self->wam, buf);
   return Py_BuildValue("s",buf);
}

static PyObject * Wam_movehome(Wam * self)
{
   return Py_BuildValue("i",bt_wam_movehome(self->wam));
}

static PyObject * Wam_moveisdone(Wam * self)
{
   return Py_BuildValue("i",bt_wam_moveisdone(self->wam));
}

static PyObject * Wam_is_teaching(Wam * self)
{
   return Py_BuildValue("i",bt_wam_is_teaching(self->wam));
}

static PyObject * Wam_teach_start(Wam * self)
{
   return Py_BuildValue("i",bt_wam_teach_start(self->wam));
}

static PyObject * Wam_teach_end(Wam * self)
{
   return Py_BuildValue("i",bt_wam_teach_end(self->wam));
}

static PyObject * Wam_playback(Wam * self)
{
   return Py_BuildValue("i",bt_wam_playback(self->wam));
}

/* Define the type */
static PyTypeObject WamType =
{
   PyObject_HEAD_INIT(0)
   0,                         /*ob_size*/
   "libbt.Wam",               /*tp_name*/
   sizeof(Wam),               /*tp_basicsize*/
   0,                         /*tp_itemsize*/
   (destructor)Wam_dealloc,   /*tp_dealloc*/
   0,                         /*tp_print*/
   0,                         /*tp_getattr*/
   0,                         /*tp_setattr*/
   0,                         /*tp_compare*/
   0,                         /*tp_repr*/
   0,                         /*tp_as_number*/
   0,                         /*tp_as_sequence*/
   0,                         /*tp_as_mapping*/
   0,                         /*tp_hash */
   0,                         /*tp_call*/
   0,                         /*tp_str*/
   0,                         /*tp_getattro*/
   0,                         /*tp_setattro*/
   0,                         /*tp_as_buffer*/
   Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, /*tp_flags*/
   "Wam objects",             /* tp_doc */
   0,		                     /* tp_traverse */
   0,		                     /* tp_clear */
   0,		                     /* tp_richcompare */
   0,		                     /* tp_weaklistoffset */
   0,		                     /* tp_iter */
   0,		                     /* tp_iternext */
   Wam_methods,               /* tp_methods */
   Wam_members,               /* tp_members */
   0,                         /* tp_getset */
   0,                         /* tp_base */
   0,                         /* tp_dict */
   0,                         /* tp_descr_get */
   0,                         /* tp_descr_set */
   0,                         /* tp_dictoffset */
   (initproc)Wam_init,        /* tp_init */
   0,                         /* tp_alloc */
   Wam_new,                   /* tp_new */
};


/* The wamlist class ------------------------------------------------------ */

/* A WAM instance */
typedef struct
{
   PyObject_HEAD
   struct bt_wam_list * list;
} WamList;

/* Wam members table */
static PyMemberDef WamList_members[] =
{
   {0,0,0,0,0}
};

/* Creation function */
static PyObject * WamList_new(PyTypeObject * type, PyObject * args, PyObject * kwds)
{
   WamList * self;
   
   /* Create */
   self = (WamList *) type->tp_alloc(type,0);
   if (!self)
      return 0;
   
   /* Initialize */
   self->list = 0;
   
   return (PyObject *)self;
}

/* Init function, expects string location */
static int WamList_init(WamList * self, PyObject * args, PyObject * kwds)
{
   char * prefixhost;
   
   /* Grab the string */
   if (!PyArg_ParseTuple(args,"s",&prefixhost))
      return -1;
   
   /* Attempt to initialize the wam_list */
   self->list = bt_wam_list_create(prefixhost);
   if (!self->list)
      return -1;
   
   return 0;
}

/* Deallocate function */
static void WamList_dealloc(WamList * self)
{
   /* If we have a wam, destroy it */
   if (self->list)
      bt_wam_list_destroy(self->list);
   
   /* Free myself */
   self->ob_type->tp_free((PyObject *)self);
}

static PyObject * WamList_get_num(WamList * self);
static PyObject * WamList_get_name(WamList * self, PyObject * args);
static PyObject * WamList_get_status(WamList * self, PyObject * args);
static PyObject * WamList_get_pid(WamList * self, PyObject * args);
static PyObject * WamList_get_programname(WamList * self, PyObject * args);

/* Wam methods table */
static PyMethodDef WamList_methods[] =
{
   {"get_num",         (PyCFunction)WamList_get_num,         METH_NOARGS,  "Get number of list entries"},
   {"get_name",        (PyCFunction)WamList_get_name,        METH_VARARGS, "Get an entry's name"},
   {"get_status",      (PyCFunction)WamList_get_status,      METH_VARARGS, "Get an entry's status"},
   {"get_pid",         (PyCFunction)WamList_get_pid,         METH_VARARGS, "Get an entry's PID"},
   {"get_programname", (PyCFunction)WamList_get_programname, METH_VARARGS, "Get an entry's program name"},
   {0, 0, 0, 0}
};

/* Wam method getnumber */
static PyObject * WamList_get_num(WamList * self)
{
   return Py_BuildValue("i",bt_wam_list_get_num(self->list));
}

static PyObject * WamList_get_name(WamList * self, PyObject * args)
{
   int myint;
   char buf[300];
   /* Grab the int */
   if (!PyArg_ParseTuple(args,"i",&myint))
      return 0;
   return Py_BuildValue("s",bt_wam_list_get_name(self->list,myint,buf));
}

static PyObject * WamList_get_status(WamList * self, PyObject * args)
{
   int myint;
   /* Grab the int */
   if (!PyArg_ParseTuple(args,"i",&myint))
      return 0;
   return Py_BuildValue("i",bt_wam_list_get_status(self->list,myint));
}

static PyObject * WamList_get_pid(WamList * self, PyObject * args)
{
   int myint;
   /* Grab the int */
   if (!PyArg_ParseTuple(args,"i",&myint))
      return 0;
   return Py_BuildValue("i",bt_wam_list_get_pid(self->list,myint));
}

static PyObject * WamList_get_programname(WamList * self, PyObject * args)
{
   int myint;
   char buf[300];
   /* Grab the int */
   if (!PyArg_ParseTuple(args,"i",&myint))
      return 0;
   return Py_BuildValue("s",bt_wam_list_get_programname(self->list,myint,buf));
}


/* Define the type */
static PyTypeObject WamListType =
{
   PyObject_HEAD_INIT(0)
   0,                         /*ob_size*/
   "libbt.WamList",           /*tp_name*/
   sizeof(WamList),           /*tp_basicsize*/
   0,                         /*tp_itemsize*/
   (destructor)WamList_dealloc, /*tp_dealloc*/
   0,                         /*tp_print*/
   0,                         /*tp_getattr*/
   0,                         /*tp_setattr*/
   0,                         /*tp_compare*/
   0,                         /*tp_repr*/
   0,                         /*tp_as_number*/
   0,                         /*tp_as_sequence*/
   0,                         /*tp_as_mapping*/
   0,                         /*tp_hash */
   0,                         /*tp_call*/
   0,                         /*tp_str*/
   0,                         /*tp_getattro*/
   0,                         /*tp_setattro*/
   0,                         /*tp_as_buffer*/
   Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, /*tp_flags*/
   "WamList objects",         /* tp_doc */
   0,		                     /* tp_traverse */
   0,		                     /* tp_clear */
   0,		                     /* tp_richcompare */
   0,		                     /* tp_weaklistoffset */
   0,		                     /* tp_iter */
   0,		                     /* tp_iternext */
   WamList_methods,           /* tp_methods */
   WamList_members,           /* tp_members */
   0,                         /* tp_getset */
   0,                         /* tp_base */
   0,                         /* tp_dict */
   0,                         /* tp_descr_get */
   0,                         /* tp_descr_set */
   0,                         /* tp_dictoffset */
   (initproc)WamList_init,    /* tp_init */
   0,                         /* tp_alloc */
   WamList_new,               /* tp_new */
};
