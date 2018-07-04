# TRANS TEST

obj-$(CPTCFG_IWLTEST)   += iwltest.o
iwltest-y		+= tx.o
iwltest-y		+= rx.o
iwltest-y		+= fw.o
iwltest-y		+= tester_dev.o
iwltest-y		+= ops.o
iwltest-y		+= debugfs.o


ccflags-y += -D__CHECK_ENDIAN__ -I$(src)/../
