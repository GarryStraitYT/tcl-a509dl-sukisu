#!/bin/bash


if [ $# -eq 1 ] && [ -d $1 ]
then
	K_LOC=`realpath $1`
else
	K_LOC=`pwd`
fi

function tcl_compile() {
	if [ -f $K_LOC"/.tcl_precompile.sh" ] && [ -f $K_LOC"/.tcl_postcompile.sh" ];then
		return
	else
		local res=$($K_LOC"/"tcl_precompile.sh $K_LOC)
		if [$res -z ];then
			return
		else
			cd $K_LOC
			sh tcl_postcompile.sh
			cd -
		fi
	fi
}

tcl_compile;
