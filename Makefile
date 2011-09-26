

CPUBOARD=IP30

sinclude /var/sysgen/Makefile.kernloadio


all: vt6212.o

vt6212.o: vt6212.c
	$(CC) -c -D _KERNEL $(CFLAGS) $<


#	$(CC) -c -D _KERNEL -nostdinc -I//usr/include -O3 -OPT:Olimit=0  -OPT:IEEE_arithmetic=1 -OPT:roundoff=0  -TENV:X=1 -OPT:wrap_around_unsafe_opt=off  -DEBUG:optimize_space=on -OPT:space=on -CG:unique_exit=on  -OPT:unroll_times=0  -MDupdate Makedepend -woff 1685,515,608,658,799,803,852,1048,1233,1499 vt6212.c 

# CFLAGS=-non_shared -elf -64 -mips4  -G 8  -r -D_KERNEL  -D _IP26_SYNC_WAR -D _NO_UNCCHED_MEM_WAR -D R10000_SPECULATION_WAR -D USE_PCI_PIO

