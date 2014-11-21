set(BARRETT_INCLUDE_DIRS /usr/xenomai/include;/usr/include/xenomai;/usr/local/include/eigen2;/usr/include/eigen2)
set(BARRETT_DEFINITIONS -D_GNU_SOURCE -D_REENTRANT -Wall -pipe -D__XENO__)
set(BARRETT_LIBRARIES libboost_thread-mt.so;pthread;libboost_python.so;-L/usr/lib;-lgsl;-lgslcblas;-lm;config;config++;/usr/xenomai/lib/libnative.so;/usr/xenomai/lib/libxenomai.so;/usr/xenomai/lib/librtdm.so;libpython2.7.so;barrett)
