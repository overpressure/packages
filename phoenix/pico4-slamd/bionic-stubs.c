// SPDX-License-Identifier: Unlicense
// Link-time stand-ins for bionic's libc.so, libm.so and libdl.so (docs/xr.md#head-tracking).
//
// pico4-slamd and its libraries are linked against these instead of PICO OS's own bionic
// libraries, so that the build needs no file of PICO's: each stub has the real library's soname
// and puts every symbol under bionic's version LIBC (bionic-stubs.map), so the binaries name
// exactly what they named when linked against the real ones (libc.so: LIBC). On the headset the
// tracker's root has PICO OS's real libraries, whose symbols the bionic linker binds at run time;
// these bodies never run. A call to a function missing here fails the link: add it to its library.
//
// Built once per library: -DSTUB_LIBC, -DSTUB_LIBM or -DSTUB_LIBDL.

#define STUB(name) __asm__(".globl " #name "\n.type " #name ", %function\n" #name ":\n\tret\n");

#if defined(STUB_LIBC)
STUB(__errno)
STUB(__libc_init)
STUB(atoi)
STUB(calloc)
STUB(clock_gettime)
STUB(close)
STUB(exit)
STUB(fchmod)
STUB(fchown)
STUB(fflush)
STUB(fprintf)
STUB(free)
STUB(ftruncate)
STUB(getenv)
STUB(malloc)
STUB(memcpy)
STUB(memset)
STUB(mmap)
STUB(open)
STUB(pthread_create)
STUB(pthread_join)
STUB(read)
STUB(rename)
STUB(signal)
STUB(snprintf)
STUB(strcmp)
STUB(syscall)
STUB(unlink)
STUB(usleep)
STUB(vsnprintf)
STUB(write)
// FILE *stderr: a pointer, as bionic's.
__asm__(".data\n.globl stderr\n.type stderr, %object\n.size stderr, 8\n.balign 8\nstderr:\n\t.quad 0\n.text\n");
#elif defined(STUB_LIBM)
STUB(atan2)
STUB(cos)
STUB(sin)
STUB(sqrt)
#elif defined(STUB_LIBDL)
STUB(dlerror)
STUB(dlopen)
STUB(dlsym)
#else
#error "build with -DSTUB_LIBC, -DSTUB_LIBM or -DSTUB_LIBDL"
#endif
