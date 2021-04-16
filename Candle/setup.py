from distutils.core import setup, Extension
 
module = Extension('Candle', sources = ['candle.cpp', 'uart.c'])
 
setup (name = 'CandlePkg',
        version = '1.0',
        description = 'This is a package for can communication',
        ext_modules = [module])