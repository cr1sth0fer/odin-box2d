package box2d

// Prototype for user allocation function.
// * 'size' the allocation size in bytes
// * 'alignment' the required alignment, guaranteed to be a power of 2
Alloc_Fcn :: #type proc "c" (size: u32, alignment: i32) -> rawptr

// Prototype for user free function.
// * "mem' the memory previously allocated through `b2AllocFcn`
Free_Fcn :: #type proc "c" (mem: rawptr)

// Prototype for the user assert callback. Return 0 to skip the debugger break.
Assert_Fcn :: #type proc "c" (condition, file_name: cstring, line_number: i32) -> i32