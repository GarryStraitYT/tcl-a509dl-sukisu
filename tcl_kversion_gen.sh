#!/bin/bash
#SPDX-License-Identifier

filename=tcl_kversion.h
out_path=include/linux
kernel_path=`pwd`

if test -e $out_path/$filename
then
	rm -f $out_path/$filename
fi

echo \#ifndef _TCL_KVERSION_H >  $out_path/$filename
echo \#define _TCL_KVERSION_H >>  $out_path/$filename
echo ""  >>  $out_path/$filename
#recored kernel latest commit id
current_git_branch_kernel_latest_short_id=`git rev-parse --short HEAD`
echo \#define KERNEL_COMMIT_ID $current_git_branch_kernel_latest_short_id\
	>>  $out_path/$filename

#recored tcl kernel latest commit id
cd tcl
current_git_branch_tclkernel_latest_short_id=`git rev-parse --short HEAD`
cd $kernel_path
echo \#define MSTAR2_COMMIT_ID $current_git_branch_tclkernel_latest_short_id\
	>>  $out_path/$filename

echo \#define TCL_KVERSION \"KL:$current_git_branch_kernel_latest_short_id\
	TCL_KERNEL:$current_git_branch_tclkernel_latest_short_id\
\" >> $out_path/$filename

echo ""  >>  $out_path/$filename
echo \#endif >>  $out_path/$filename


