from distutils.core import setup, Extension

libbt = Extension('libbt',
                  include_dirs = ['/usr/local/include',
                                  '../../src/wam',
                                  '../../src/discover'],
                  libraries = ['bt','json'],
                  library_dirs = ['/usr/local/lib','../../src/.libs'],
                  sources=['libbtmodule.c'])

setup(name='libbt',
      version='0.1',
      description='libbt module',
      ext_modules=[libbt])
