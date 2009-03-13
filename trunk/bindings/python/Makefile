
all: libbt.so
libbt.so : libbtmodule.c
	python setup.py build

install: libbt.so
	python setup.py install

clean:
	python setup.py clean
	rm -rf build
