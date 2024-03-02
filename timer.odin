package box2d

Statistics :: struct
{
	island_count,
	body_count,
	contact_count,
	joint_count,
	proxy_count,
	tree_height,
	stack_capacity,
	stack_used,
	byte_count: i32,
	colors_count: [GRAPH_COLORS_COUNT + 1]i32,
}

when ODIN_OS == .Windows
{
    /// Timer for profiling. This has platform specific code and may
    /// not work on every platform.
    Timer :: struct
    {
        start: i64,
    }
}
else when ODIN_OS == .Linux
{
    /// Timer for profiling. This has platform specific code and may
    /// not work on every platform.
    Timer :: struct
    {
        start_sec,
        start_usec: u64,
    }
}
else
{
    /// Timer for profiling. This has platform specific code and may
    /// not work on every platform.
    Timer :: struct
    {
        dummy: i32,
    }
}