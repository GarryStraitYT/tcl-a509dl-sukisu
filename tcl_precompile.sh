#!/bin/bash

if [ $# -eq 1 ] && [ -d $1 ]
then
	KERN_LOC=`realpath $1`
else
	KERN_LOC=`pwd`
fi

TCL_PATH=$KERN_LOC"/tcl"
TCL_TMP_PATH=$KERN_LOC"/.tcl_tmp"
TCL_LOG=$KERN_LOC"/.tcl_compile.log"
TCL_FILELIST=$KERN_LOC"/.tcl_filelist"
TCL_FILELIST_BAK=$KERN_LOC"/.tcl_filelist_bak"
TCL_FILELOCK=$KERN_LOC"/.tcl_compile.lock"
TCL_LASTFILES=""
TCL_LASTFILES_ERR=""
TCL_FLAGS="pass"
TCL_TMP_LOG=""

function err_exit() {
	echo "PID:$$ ERROR: $1" >> $TCL_LOG;
	TCL_TMP_LOG=$TCL_TMP_LOG"PID $$ ERROR: $1
"
	TCL_FLAGS="failed";
}

function info_log() {
	echo "PID:$$ INFO: $1" >> $TCL_LOG;
}

function inode_compare() {
	local INODE1=`ls -i $1|awk '{print $1}'`
	local INODE2=`ls -i $2|awk '{print $1}'`
	if [[ $INODE1 == $INODE2 ]]; then
		echo "yes"
	else
		echo "no"
	fi
}

function md5_compare() {
	local MD1=`md5sum $1|awk '{print $1}'`
	local MD2=`md5sum $2|awk '{print $1}'`
	if [[ $MD1 == $MD2 ]]; then
		echo "yes"
	else
		echo "no"
	fi
}

function is_sl(){
	if [[ `realpath $1 2>/dev/null` == `realpath $2 2>/dev/null` ]]; then
		echo "yes"
	else
		echo "no"
	fi
}

function is_special_fullpath(){
	if [[ $1 == *"/Makefile"* ]] || [[ $1 == *"/Kconfig"* ]] || [[ $1 == *"arch/"*"/configs"* ]] || [[ $1 == *"/.config"* ]]; then
		echo "yes"
	else
		echo "no"
	fi
}

function remove_deleted_file() {
	if [ -f $TCL_FILELIST ]; then
		info_log "filelist exits:$TCL_FILELIST, last merge fail"
		if [ -f $TCL_FILELIST_BAK ]; then
			if [[ `md5_compare $TCL_FILELIST $TCL_FILELIST_BAK` == "no" ]]; then
				TCL_LASTFILES_ERR=`cat $TCL_FILELIST`
				TCL_LASTFILES=`cat $TCL_FILELIST_BAK`
				for entry in $TCL_LASTFILES_ERR
				do
					if [[ $TCL_LASTFILES == *"$entry"* ]]; then
						echo $entry >> $TCL_FILELIST_BAK;
						info_log "recover: add new file $entry"
					fi
				done
			fi
			#sync;
			rm -f $TCL_FILELIST;
			info_log "recover the file list end"
		else
			mv $TCL_FILELIST $TCL_FILELIST_BAK;
		fi
	fi
	if [ -f $TCL_FILELIST_BAK ]; then
		TCL_LASTFILES=`cat $TCL_FILELIST_BAK`;
		for i in $TCL_LASTFILES
		do
			if [ ! -f $TCL_PATH"/"$i ] && [ ! -d $TCL_PATH"/"$i ]; then
				if [ -f $KERN_LOC"/"$i ] || [ -L $KERN_LOC"/"$i ]; then
					info_log "remove deleted files: $i"
					rm -rf $i 2>/dev/null;
					if [[ `is_special_fullpath $i` == "yes" ]] && [[ `is_exist $TCL_TMP_PATH"/"$i` == "yes" ]]; then
						info_log "recover cached special files: $TCL_TMP_PATH/$i to $i"
						mv $TCL_TMP_PATH"/"$i $i;
					fi
				fi
			fi
		done
		#sync;
		info_log "TCL FIELLIST:"
		info_log "$TCL_LASTFILES"
	fi
}

function is_old(){
	if [[ $TCL_LASTFILES == *"$1"* ]]; then
		echo "yes"
	fi
}

function is_special(){
	if [[ $1 == "Makefile"* ]] || [[ $1 == "Kconfig"* ]] || [[ $1 == ".config"* ]]; then
		echo "yes"
	elif [[ $2 == *"arch/"*"/configs"* ]]; then
		echo "yes"
	else
		echo "no"
	fi
}

function is_exist(){
	if [ -e $1 ] || [ -L $1 ]; then
		echo "yes"
	else
		echo "no"
	fi
}

function update_dir() {
	local TCL_VENDOR_FILE=""
	local KERNEL_FILE=""
	local TCL_TMP_FILE=""
	local TCL_TMP_FILE_PATH=""
	local TCL_CACHE_FILE=""
	for file in `ls -a $1`
	do
		if [[ $file == "." ]] || [[ $file == ".." ]] || [[ $file == ".git"* ]]; then
			continue;
		fi
		TCL_VENDOR_FILE=$1"/"$file;
		KERNEL_FILE=$KERN_LOC"/"$TCL_VENDOR_FILE
		KERNEL_FILE_PATH=$KERN_LOC"/"$1
		TCL_TMP_FILE_PATH=$TCL_TMP_PATH"/"$1;
		TCL_TMP_FILE=$TCL_TMP_FILE_PATH"/"$file;
		TCL_CACHE_FILE=$1"/.tmp_"$file
		info_log "process: $TCL_VENDOR_FILE"
		#softlink is not allowed in tcl directory
		if [ -L $TCL_VENDOR_FILE ]; then
			err_exit "invalid softlink file: $TCL_VENDOR_FILE";
			continue;
		fi
		if [ -d $TCL_VENDOR_FILE ]; then
			info_log "is dir: $TCL_VENDOR_FILE"
			info_log "kernel path: $KERNEL_FILE"
			# vendor_is_dir
			if [ ! -e $KERNEL_FILE ]; then
				# no_kernel_path
				if [ -L $KERNEL_FILE ]; then
					rm -f $KERNEL_FILE;
				fi
				echo $TCL_VENDOR_FILE >> $TCL_FILELIST;
				ln -s $TCL_PATH"/"$TCL_VENDOR_FILE $KERNEL_FILE;
				info_log "new softlink dir: $TCL_VENDOR_FILE"
				continue;
				#no_kernel_path end
			fi
			if [ -d $KERNEL_FILE ]; then
				# kernel_is_dir
				if [ -L $KERNEL_FILE ] && [[ `is_sl $KERNEL_FILE $TCL_VENDOR_FILE` == "yes" ]]; then
					# kernel_dir_is_softlink and their realpath is same
					echo $TCL_VENDOR_FILE >> $TCL_FILELIST;
					info_log "old softlink dir exists: $TCL_VENDOR_FILE"
				else
					# kernel_dir_is_not_softlink or not same realpath
					update_dir $TCL_VENDOR_FILE;
				fi
				# kernel_is_dir end
			else
				# kernel_is_file
				if [ ! -L $KERNEL_FILE ] && [[ `is_old $TCL_VENDOR_FILE` == "yes" ]]; then
					# kernel_file_not_softlink and inlist
					info_log "softlink dir instead of old hardlink file: $TCL_VENDOR_FILE"
					echo $TCL_VENDOR_FILE >> $TCL_FILELIST;
					rm -f $KERNEL_FILE;
					ln -s $TCL_PATH"/"$TCL_VENDOR_FILE $KERNEL_FILE;
					# kernel_file_not_softlink and inlist end
				else
					# kernel_file_is_softlink or not inlist
					err_exit "dir conflict with origin file: $TCL_VENDOR_FILE";
					continue;
					# kernel_file_is_softlink or not inlist end
				fi
				# kernel_is_file end
			fi
			# vendor_is_dir end
		else
			info_log "is file: $TCL_VENDOR_FILE"
			info_log "kernel path: $KERNEL_FILE"
			# vendor_is_file
			if [ ! -e $KERNEL_FILE ]; then
				# no_kernel
				if [ -L $KERNEL_FILE ]; then
					rm -f $KERNEL_FILE;
				fi
				info_log "new hardlink file: $TCL_VENDOR_FILE"
				echo $TCL_VENDOR_FILE >> $TCL_FILELIST;
				ln $TCL_VENDOR_FILE $KERNEL_FILE
				continue;
				#no_kernel end
			fi
			if [ -d $KERNEL_FILE ]; then
				# kernel_is_dir
				if [ -L $KERNEL_FILE ] && [[ `is_old $TCL_VENDOR_FILE` == "yes" ]]; then
					# kernel_dir_is_softlink and inlist
					rm -rf $KERNEL_FILE;
					info_log "hardlink file instead of old softlink: $TCL_VENDOR_FILE"
					echo $TCL_VENDOR_FILE >> $TCL_FILELIST;
					ln $TCL_VENDOR_FILE $KERNEL_FILE
					# kernel_dir_is_softlink and inlist end
				else
					# kernel_dir_is_not_softlink
					err_exit "file conflict with origin dir: $TCL_VENDOR_FILE";
					continue;
					# kernel_dir_is_not_softlink end
				fi
				# kernel_is_dir end
			else
				# kernel_is_file
				if [ -L $KERNEL_FILE ]; then
					# kernel_file_is_softlink
					if [[ `is_special $file $1` == "yes" ]]; then
						info_log "Should not edit softlink file: $TCL_VENDOR_FILE"
						info_log "Please edit the file: `realpath $KERNEL_FILE`"
					fi
					err_exit "file conflict with origin softlink: $TCL_VENDOR_FILE";
					continue;
					# kernel_file_is_softlink end
				else
					# kernel_file_is_not_softlink
					if [[ `is_old $TCL_VENDOR_FILE` == "yes" ]]; then
						# in filelist
						if [[ `inode_compare $KERNEL_FILE $TCL_VENDOR_FILE` == "yes" ]]; then
							info_log "old hard link exists: $TCL_VENDOR_FILE"
							echo $TCL_VENDOR_FILE >> $TCL_FILELIST;
							continue;
						else
							if [[ `is_special $file $1` == "yes" ]] && [[ `is_exist $TCL_TMP_FILE` == "yes" ]]; then
								if [ -L $TCL_TMP_FILE ]; then
									err_exit "tmp dir should has no softlink: $TCL_VENDOR_FILE";
									continue;
								elif [ -f $TCL_TMP_FILE ]; then
									cp $TCL_TMP_FILE $TCL_CACHE_FILE;
								else
									err_exit "tmp is dir: $TCL_VENDOR_FILE";
									continue;
								fi
								echo $TCL_VENDOR_FILE >> $TCL_FILELIST;
								echo "" >> $TCL_CACHE_FILE
								cat $TCL_VENDOR_FILE >> $TCL_CACHE_FILE;
								if [[ `md5_compare $TCL_CACHE_FILE $KERNEL_FILE` == "no" ]]; then
									rm -f $KERNEL_FILE;
									mv $TCL_CACHE_FILE $KERNEL_FILE;
									info_log "update patched special file: $TCL_VENDOR_FILE"
								else
									info_log "skip special file: $TCL_VENDOR_FILE"
									rm $TCL_CACHE_FILE
								fi
							else
								echo $TCL_VENDOR_FILE >> $TCL_FILELIST;
								if [[ `md5_compare $KERNEL_FILE $TCL_VENDOR_FILE` == "no" ]]; then
									rm -f $KERNEL_FILE;
									ln $TCL_VENDOR_FILE $KERNEL_FILE
									info_log "overwrite old hardlink file: $TCL_VENDOR_FILE"
								else
									info_log "totally same file exists: $TCL_VENDOR_FILE"
								fi
							fi
						fi
						# in filelist end
					else
						# not in filelist
						if [[ `is_special $file $1` == "yes" ]]; then
							echo $TCL_VENDOR_FILE >> $TCL_FILELIST;
							mkdir -p $TCL_TMP_FILE_PATH;
							cp $KERNEL_FILE $TCL_CACHE_FILE
							mv -f $KERNEL_FILE $TCL_TMP_FILE;
							echo "" >> $TCL_CACHE_FILE
							cat $TCL_VENDOR_FILE >> $TCL_CACHE_FILE
							mv -f $TCL_CACHE_FILE $KERNEL_FILE
							info_log "new patched special file: $TCL_VENDOR_FILE"
						else
							err_exit "file conflict with origin file: $TCL_VENDOR_FILE";
							continue;
						fi
						# not in filelist end
					fi
					# kernel_file_is_not softlink
				fi
				# kernel_is_file end
			fi
			# vendor_is_file end
		fi
	done
}

rm -f $TCL_LOG;

TIME=`date`
info_log "Merge start: $TIME";

while ! mkdir $TCL_FILELOCK 2>/dev/null;do
	info_log "wait in mutex: PID $$, TIME `date`"
	sleep 5;
done

remove_deleted_file;

if [ ! -d $TCL_PATH ]; then
	info_log "prebuild:can not find $TCL_PATH";
	rm -rf $TCL_FILELOCK;
	exit 1;
fi

cd $TCL_PATH;
update_dir . ;
cd $KERN_PATH;

mv $TCL_FILELIST $TCL_FILELIST_BAK;
#sync;

rm -rf $TCL_FILELOCK;
TIME=`date`
info_log "Merge end: $TIME";
if [[ $TCL_FLAGS == "failed" ]]; then
	echo "$TCL_TMP_LOG"
fi

true

