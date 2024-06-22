package box2d

// Prototype for user allocation function.
// * ```size``` the allocation size in bytes
// * ```alignment``` the required alignment, guaranteed to be a power of ```2```
Alloc_Fcn :: #type proc "c" (size: u32, alignment: i32) -> rawptr

// Prototype for user free function.
// * ```mem``` the memory previously allocated through ```Alloc_Fcn```
Free_Fcn :: #type proc "c" (mem: rawptr)

// Prototype for the user assert callback. Return ```0``` to skip the debugger break.
Assert_Fcn :: #type proc "c" (condition, file_name: cstring, line_number: i32) -> i32

// Version numbering scheme.
//
// See https://semver.org/
Version :: struct
{
	// Significant changes
	major: i32,

	// Incremental changes
	minor: i32,

	// Bug fixes
	revision: i32,
}

when ODIN_OS == .Windows
{
    // Timer for profiling. This has platform specific code and may
    // not work on every platform.
    Timer :: struct
    {
        start: i64,
    }
}
else when ODIN_OS == .Linux
{
    // Timer for profiling. This has platform specific code and may
    // not work on every platform.
    Timer :: struct
    {
        start_sec,
        start_usec: u64,
    }
}
else
{
    // Timer for profiling. This has platform specific code and may
    // not work on every platform.
    Timer :: struct
    {
        dummy: i32,
    }
}