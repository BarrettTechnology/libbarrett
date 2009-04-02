from distutils.core import setup, Extension

libbt = Extension('libbarrett',
                  include_dirs = ['/usr/local/include',
                                  '../../src/wam',
                                  '../../src/discover'],
                  libraries = ['barrett','json'],
                  library_dirs = ['/usr/local/lib','../../src/.libs'],
                  sources=['libbarrett.c'])

setup(name='libbarrett',
      version='0.1',
      description='libbarrett module',
      ext_modules=[libbt])
