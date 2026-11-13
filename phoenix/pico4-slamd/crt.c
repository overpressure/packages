// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: Unlicense
/*
 * Entry point of bionic executables without the NDK's crtbegin: hand the kernel's argument block
 * to bionic's __libc_init, which runs main and exits (also when started through the linker).
 */
typedef struct
{
	void (**preinit_array)(void);
	void (**init_array)(void);
	void (**fini_array)(void);
} structors_array_t;

int
main(int, char **, char **);
__attribute__((noreturn)) void
__libc_init(void *raw_args, void (*onexit)(void), int (*slingshot)(int, char **, char **),
            structors_array_t const *const structors);

__asm__(".text\n.globl _start\n.type _start,%function\n_start:\n\tmov x0, sp\n\tb _start_main\n");

__attribute__((used)) void
_start_main(void *raw_args)
{
	static const structors_array_t none = {0, 0, 0};
	__libc_init(raw_args, 0, &main, &none);
}
