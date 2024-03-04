package box2d

// References a world instance
World_ID :: struct
{
	index: u16,
	revision: u16,
}

// Body identifier
Body_ID :: struct
{
	index: i32,
	world: u16,
	revision: u16,
}

// References a shape instance
Shape_ID :: struct
{
	index: i32,
	world: u16,
	revision: u16,
}

// References a joint instance
Joint_ID :: struct
{
	index: i32,
	world: u16,
	revision: u16,
}

Chain_ID :: struct
{
	index: i32,
	world: i16,
	revision: u16,
}

NULL_WORLD_ID :: World_ID{}
NULL_BODY_ID :: Body_ID{}
NULL_SHAPE_ID :: Shape_ID{}
NULL_JOINT_ID :: Joint_ID{}
NULL_CHAIN_ID :: Chain_ID{}

// Determine if any id is null
is_null :: #force_inline proc "contextless" (id: $T) -> bool
{
    return id.index == 0
}

// Determine if any id is non-null
is_non_null :: #force_inline proc "contextless" (id: $T) -> bool
{
    return id.index != 0
}

// Compare two ids for equality. Doesn't work for World_ID.
id_equals :: #force_inline proc "contextless" (id1, id2: $T) -> bool
{
    return id1.index == id2.index && id1.world == id2.world && id1.revision == id2.revision
}