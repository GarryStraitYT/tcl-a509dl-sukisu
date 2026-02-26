
menu "Tcl Kernel Feature"

#
# example:
# if want to add config EXT4_TCL_TEST in fs/ext4
# source "fs/Kconfig.tcl"
# add Kconfig.tcl in tcl/fs
#     ---> menu "File Systems"
#     ---> source "fs/ext4/Kconfig.tcl"
#     ---> endmenu
#
# add Kconfig.tcl in tcl/fs/ext4
#     ---> menu "EXT4 Features"
#     ---> config EXT4_TCL_TEST
#     ---> bool "Ext4 test Support"
#     ---> default y
#     ---> depends on EXT4_FS
#     ---> help
#     --->  Enables test support for the ext4 filesystem.
#
#     --->  If you select Y here, then ext4 filesystem will do test
#     --->  background with the mount option: test
#
#     ---> endmenu

# source "fs/Kconfig.tcl"

endmenu
