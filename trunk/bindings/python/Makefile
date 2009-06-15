
all: libbt.so
libbt.so : libbarrett.c
	python setup.py build

install: libbt.so
	python setup.py install ${BT_PY_INSTALL_ARGS} --prefix=${DESTDIR}/usr

clean:
	python setup.py clean
	rm -rf build

distdir:
	cp Makefile setup.py libbarrett.c $(distdir)
