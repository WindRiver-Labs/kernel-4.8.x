#!/bin/sh

# Generate system call probe description macros from syscall metadata dump file.
# example usage:
# lttng-syscalls-generate-headers.sh 3.0.4 x86-64-syscalls-3.0.4

INPUTDIR=$1
INPUTFILE=$2
INPUT=${INPUTDIR}/${INPUTFILE}
SRCFILE=gen.tmp.0
TMPFILE=gen.tmp.1

cp ${INPUT} ${SRCFILE}

#Cleanup
sed 's/^\[.*\] //g' ${SRCFILE} > ${TMPFILE}
mv ${TMPFILE} ${SRCFILE}

sed 's/^syscall sys_\([^ ]*\)/syscall \1/g' ${SRCFILE} > ${TMPFILE}
mv ${TMPFILE} ${SRCFILE}

#Filter

#select only syscalls we currently support
#move non-pointers with and without arguments to a integer-only file.
CLASS=integers
grep -v "\\*\|cap_user_header_t" ${SRCFILE} > ${TMPFILE}
mv ${TMPFILE} ${SRCFILE}

#TODO
# move all system calls using pointers to a separate file.
#CLASS=pointers
#grep "\\*\|cap_#user_header_t" ${SRCFILE} > ${TMPFILE}
#mv ${TMPFILE} ${SRCFILE}

HEADER=headers/${INPUTFILE}-${CLASS}.h

echo "/* THIS FILE IS AUTO-GENERATED. DO NOT EDIT */" > ${HEADER}

echo \
"#ifndef CREATE_SYSCALL_TABLE

#undef TRACE_SYSTEM
#define TRACE_SYSTEM syscalls

#if !defined(_TRACE_SYSCALLS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_SYSCALLS_H

#include <linux/tracepoint.h>
#include <linux/syscalls.h>
" >> ${HEADER}

NRARGS=0

echo \
'DECLARE_EVENT_CLASS_NOARGS(syscalls_noargs,\n'\
'	TP_STRUCT__entry(),\n'\
'	TP_fast_assign(),\n'\
'	TP_printk()\n'\
')'\
	>> ${HEADER}

grep "^syscall [^ ]* nr [^ ]* nbargs ${NRARGS} " ${SRCFILE} > ${TMPFILE}
sed 's/^syscall \([^ ]*\) nr \([^ ]*\) nbargs \([^ ]*\) '\
'types: (\([^)]*\)) '\
'args: (\([^)]*\))/'\
'DEFINE_EVENT_NOARGS(syscalls_noargs, sys_\1)'\
'/g'\
	${TMPFILE} >> ${HEADER}


# types: 4
# args   5

NRARGS=1
grep "^syscall [^ ]* nr [^ ]* nbargs ${NRARGS} " ${SRCFILE} > ${TMPFILE}
sed 's/^syscall \([^ ]*\) nr \([^ ]*\) nbargs \([^ ]*\) '\
'types: (\([^)]*\)) '\
'args: (\([^)]*\))/'\
'TRACE_EVENT(sys_\1,\n'\
'	TP_PROTO(\4 \5),\n'\
'	TP_ARGS(\5),\n'\
'	TP_STRUCT__entry(__field(\4, \5)),\n'\
'	TP_fast_assign(tp_assign(\5, \5)),\n'\
'	TP_printk()\n'\
')/g'\
	${TMPFILE} >> ${HEADER}

# types: 4 5
# args   6 7

NRARGS=2
grep "^syscall [^ ]* nr [^ ]* nbargs ${NRARGS} " ${SRCFILE} > ${TMPFILE}
sed 's/^syscall \([^ ]*\) nr \([^ ]*\) nbargs \([^ ]*\) '\
'types: (\([^,]*\), \([^)]*\)) '\
'args: (\([^,]*\), \([^)]*\))/'\
'TRACE_EVENT(sys_\1,\n'\
'	TP_PROTO(\4 \6, \5 \7),\n'\
'	TP_ARGS(\6, \7),\n'\
'	TP_STRUCT__entry(__field(\4, \6) __field(\5, \7)),\n'\
'	TP_fast_assign(tp_assign(\6, \6) tp_assign(\7, \7)),\n'\
'	TP_printk()\n'\
')/g'\
	${TMPFILE} >> ${HEADER}

# types: 4 5 6
# args   7 8 9

NRARGS=3
grep "^syscall [^ ]* nr [^ ]* nbargs ${NRARGS} " ${SRCFILE} > ${TMPFILE}
sed 's/^syscall \([^ ]*\) nr \([^ ]*\) nbargs \([^ ]*\) '\
'types: (\([^,]*\), \([^,]*\), \([^)]*\)) '\
'args: (\([^,]*\), \([^,]*\), \([^)]*\))/'\
'TRACE_EVENT(sys_\1,\n'\
'	TP_PROTO(\4 \7, \5 \8, \6 \9),\n'\
'	TP_ARGS(\7, \8, \9),\n'\
'	TP_STRUCT__entry(__field(\4, \7) __field(\5, \8) __field(\6, \9)),\n'\
'	TP_fast_assign(tp_assign(\7, \7) tp_assign(\8, \8) tp_assign(\9, \9)),\n'\
'	TP_printk()\n'\
')/g'\
	${TMPFILE} >> ${HEADER}


# types: 4 5  6  7
# args   8 9 10 11

NRARGS=4
grep "^syscall [^ ]* nr [^ ]* nbargs ${NRARGS} " ${SRCFILE} > ${TMPFILE}
sed 's/^syscall \([^ ]*\) nr \([^ ]*\) nbargs \([^ ]*\) '\
'types: (\([^,]*\), \([^,]*\), \([^,]*\), \([^)]*\)) '\
'args: (\([^,]*\), \([^,]*\), \([^,]*\), \([^)]*\))/'\
'TRACE_EVENT(sys_\1,\n'\
'	TP_PROTO(\4 \8, \5 \9, \6 \10, \7 \11),\n'\
'	TP_ARGS(\8, \9, \10, \11),\n'\
'	TP_STRUCT__entry(__field(\4, \8) __field(\5, \9) __field(\6, \10) __field(\7, \11)),\n'\
'	TP_fast_assign(tp_assign(\8, \8) tp_assign(\9, \9) tp_assign(\10, \10) tp_assign(\11, \11)),\n'\
'	TP_printk()\n'\
')/g'\
	${TMPFILE} >> ${HEADER}

# types: 4  5  6  7  8
# args   9 10 11 12 13

NRARGS=5
grep "^syscall [^ ]* nr [^ ]* nbargs ${NRARGS} " ${SRCFILE} > ${TMPFILE}
sed 's/^syscall \([^ ]*\) nr \([^ ]*\) nbargs \([^ ]*\) '\
'types: (\([^,]*\), \([^,]*\), \([^,]*\), \([^,]*\), \([^)]*\)) '\
'args: (\([^,]*\), \([^,]*\), \([^,]*\), \([^,]*\), \([^)]*\))/'\
'TRACE_EVENT(sys_\1,\n'\
'	TP_PROTO(\4 \9, \5 \10, \6 \11, \7 \12, \8 \13),\n'\
'	TP_ARGS(\9, \10, \11, \12, \13),\n'\
'	TP_STRUCT__entry(__field(\4, \9) __field(\5, \10) __field(\6, \11) __field(\7, \12) __field(\8, \13)),\n'\
'	TP_fast_assign(tp_assign(\9, \9) tp_assign(\10, \10) tp_assign(\11, \11) tp_assign(\12, \12) tp_assign(\13, \13)),\n'\
'	TP_printk()\n'\
')/g'\
	${TMPFILE} >> ${HEADER}


# types: 4   5  6  7  8  9
# args   10 11 12 13 14 15

NRARGS=6
grep "^syscall [^ ]* nr [^ ]* nbargs ${NRARGS} " ${SRCFILE} > ${TMPFILE}
sed 's/^syscall \([^ ]*\) nr \([^ ]*\) nbargs \([^ ]*\) '\
'types: (\([^,]*\), \([^,]*\), \([^,]*\), \([^,]*\), \([^,]*\), \([^)]*\)) '\
'args: (\([^,]*\), \([^,]*\), \([^,]*\), \([^,]*\), \([^,]*\), \([^)]*\))/'\
'TRACE_EVENT(sys_\1,\n'\
'	TP_PROTO(\4 \10, \5 \11, \6 \12, \7 \13, \8 \14, \9 \15),\n'\
'	TP_ARGS(\10, \11, \12, \13, \14, \15),\n'\
'	TP_STRUCT__entry(__field(\4, \10) __field(\5, \11) __field(\6, \12) __field(\7, \13) __field(\8, \14) __field(\9, \15)),\n'\
'	TP_fast_assign(tp_assign(\10, \10) tp_assign(\11, \11) tp_assign(\12, 12) tp_assign(\13, \13) tp_assign(\14, \14) tp_assign(\15, \15)),\n'\
'	TP_printk()\n'\
')/g'\
	${TMPFILE} >> ${HEADER}

# Macro for tracing syscall table

rm -f ${TMPFILE}
for NRARGS in $(seq 0 6); do
	grep "^syscall [^ ]* nr [^ ]* nbargs ${NRARGS} " ${SRCFILE} >> ${TMPFILE}
done

echo \
"
#endif /*  _TRACE_SYSCALLS_H */

/* This part must be outside protection */
#include \"../../../probes/define_trace.h\"

#else /* CREATE_SYSCALL_TABLE */
" >> ${HEADER}


NRARGS=0
grep "^syscall [^ ]* nr [^ ]* nbargs ${NRARGS} " ${SRCFILE} > ${TMPFILE}

#noargs
sed 's/^syscall \([^ ]*\) nr \([^ ]*\) nbargs \([^ ]*\) .*$/'\
'TRACE_SYSCALL_TABLE(syscalls_noargs, sys_\1, \2, \3)/g'\
	${TMPFILE} >> ${HEADER}

#others.
grep -v "^syscall [^ ]* nr [^ ]* nbargs ${NRARGS} " ${SRCFILE} > ${TMPFILE}
sed 's/^syscall \([^ ]*\) nr \([^ ]*\) nbargs \([^ ]*\) .*$/'\
'TRACE_SYSCALL_TABLE(sys_\1, sys_\1, \2, \3)/g'\
	${TMPFILE} >> ${HEADER}

echo -n \
"
#endif /* CREATE_SYSCALL_TABLE */
" >> ${HEADER}

rm -f ${INPUTFILE}.tmp
rm -f ${TMPFILE}
rm -f ${SRCFILE}
