#!/bin/bash

rev=HEAD
prefix_dir=/usr/local

cd `dirname $0`
deploy_dir=`pwd`
root_dir=$deploy_dir/root

#echo $deploy_dir
#echo $root_dir

# pass an argument to the script to prevent cleaning $root_dir and rebuilding dependencies
if [ $# -eq 0 ]
then
    if [ -d $root_dir ]
    then
        rm -Rf $root_dir/*
    else
        if [ -f $root_dir ]
        then
            rm $root_dir
        fi
        
        mkdir $root_dir
    fi
    
    svn export DEBIAN $root_dir/DEBIAN
    
    cd $deploy_dir
    svn export -r $rev http://web.barrett.com/svn/libbarrett/dependencies/libconfig-1.3.2-PATCHED.tar.gz
    tar xzf libconfig-1.3.2-PATCHED.tar.gz
    cd libconfig-1.3.2
    ./configure --prefix=$prefix_dir && make && make DESTDIR=$root_dir install
    cd $deploy_dir
    rm -Rf libconfig-1.3.2-PATCHED.tar.gz libconfig-1.3.2

    cd $deploy_dir
    svn export -r $rev http://web.barrett.com/svn/libbarrett/dependencies/gsl-1.14.tar.gz
    tar xzf gsl-1.14.tar.gz
    cd gsl-1.14
    ./configure --prefix=$prefix_dir && make && make DESTDIR=$root_dir install
    cd $deploy_dir
    rm -Rf gsl-1.14.tar.gz gsl-1.14

    cd $deploy_dir
    svn export -r $rev http://web.barrett.com/svn/libbarrett/dependencies/eigen-2.0.12.tar.bz2
    tar xjf eigen-2.0.12.tar.bz2
    cd eigen
    cmake -D CMAKE_INSTALL_PREFIX=$prefix_dir . && make && make DESTDIR=$root_dir install
    cd $deploy_dir
    rm -Rf eigen-2.0.12.tar.bz2 eigen
fi




