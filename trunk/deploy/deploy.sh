#!/bin/bash

dep_rev=HEAD
repo_base=http://web.barrett.com/svn/libbarrett/trunk

prefix_dir=/usr/local

cd `dirname $0`
deploy_dir=`pwd`
root_dir=$deploy_dir/root
rp_dir=$root_dir$prefix_dir
build_dir=$deploy_dir/..

#echo $deploy_dir
#echo $root_dir


# clean the package root
if [ $# -eq 0  -o  $1 = "clean" ]
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
    
    mkdir $root_dir/etc
    mkdir $root_dir/etc/barrett
fi


# build source dependencies    
if [ $# -eq 0  -o  $1 = "deps" ]
then
    cd $deploy_dir
    svn export -r $dep_rev http://web.barrett.com/svn/libbarrett/dependencies/libconfig-1.3.2-PATCHED.tar.gz
    tar xzf libconfig-1.3.2-PATCHED.tar.gz
    cd libconfig-1.3.2
    ./configure --prefix=$prefix_dir && make && make DESTDIR=$root_dir install
    cd $deploy_dir
    rm -Rf libconfig-1.3.2-PATCHED.tar.gz libconfig-1.3.2

    cd $deploy_dir
    svn export -r $dep_rev http://web.barrett.com/svn/libbarrett/dependencies/gsl-1.14.tar.gz
    tar xzf gsl-1.14.tar.gz
    cd gsl-1.14
    ./configure --prefix=$prefix_dir && make && make DESTDIR=$root_dir install
    cd $deploy_dir
    rm -Rf gsl-1.14.tar.gz gsl-1.14

    cd $deploy_dir
    svn export -r $dep_rev http://web.barrett.com/svn/libbarrett/dependencies/eigen-2.0.12.tar.bz2
    tar xjf eigen-2.0.12.tar.bz2
    cd eigen
    cmake -D CMAKE_INSTALL_PREFIX=$prefix_dir . && make && make DESTDIR=$root_dir install
    cd $deploy_dir
    rm -Rf eigen-2.0.12.tar.bz2 eigen
fi


# copy libbarrett into the package root
# binaries should already be built
if [ $# -eq 0  -o  $1 = "lib" ]
then
    rm -Rf $rp_dir/include/barrett
    svn export $repo_base/library/src/barrett $rp_dir/include/barrett
    find $rp_dir/include/barrett/ -depth \( \! -iname "*.h" \) -and -type f -print -delete

    cp $build_dir/library/Debug/libbarrett.so $rp_dir/lib/
    rm -Rf $rp_dir/bin/bt-*
    cp $build_dir/programs/Debug/bt-* $rp_dir/bin/

    cp -R $build_dir/config/* $root_dir/etc/barrett/
fi


# build package
if [ $# -eq 0  -o  $1 = "package" ]
then
    rm -Rf $root_dir/DEBIAN
    svn export $repo_base/deploy/DEBIAN $root_dir/DEBIAN
    dpkg-deb -b root .
fi

