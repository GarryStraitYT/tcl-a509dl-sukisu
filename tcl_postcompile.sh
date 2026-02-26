#!/bin/bash

if [ $# -eq 1 ] && [ -d $1 ]
then
	KERN_LOC=`realpath $1`
else
	KERN_LOC=`pwd`
fi

TCL_TMP_PATH=$KERN_LOC"/.tcl_tmp"
TCL_LOG_FILE=$KERN_LOC"/.tcl_compile.log"
TCL_FILE_LOCK=$KERN_LOC"/.tcl_compile.lock"
TCT_TMP_PATH=$(pwd)

function info_log() {
	echo "PID:$$ INFO: $1" >> $TCL_LOG_FILE;
}

function process_dir() {
	for file in `ls -a $1`
	do
		if [[ $file == "." ]] || [[ $file == ".." ]]; then
			continue;
		fi
		if [ -d $1"/"$file ]
		then
			process_dir $1"/"$file
		else
			if [ ! -d $KERN_LOC"/"$1"/"$file ]; then
				rm -f $KERN_LOC"/"$1"/"$file;
			fi
			mv $TCL_TMP_PATH"/"$1"/"$file $KERN_LOC"/"$1"/"$file
			#mv $1"/"$file $KERN_LOC"/"$1"/"$file
		fi
	done
}

TIME=`date`
info_log "CLEAN begin: $TIME"

info_log "kgl tcl_postcompile.sh begin"

while ! mkdir $TCL_FILE_LOCK 2>/dev/null;do
	info_log "wait in mutex: PID $$, TIME `date`"
	sleep 5;
done

if [ -f $KERN_LOC"/.tcl_filelist" ]; then
	for i in `cat $KERN_LOC/.tcl_filelist`
	do
		rm -rf $KERN_LOC"/"$i 2>/dev/null;
	done
	rm -f $KERN_LOC/.tcl_filelist;
fi

if [ -f $KERN_LOC"/.tcl_filelist_bak" ]; then
	for i in `cat $KERN_LOC/.tcl_filelist_bak`
	do
		rm -rf $KERN_LOC"/"$i 2>/dev/null;
	done
	rm -f $KERN_LOC/.tcl_filelist_bak;
fi

if [ -d $TCL_TMP_PATH ]; then
	cd $TCL_TMP_PATH;
	process_dir . ;
	cd $KERN_PATH
	rm -rf $KERN_LOC/.tcl_tmp;
fi

rm -rf $TCL_FILE_LOCK
TIME=`date`
info_log "CLEAN end: $TIME"
