package box2d

// World id references a world instance. This should be treated as an opaque handle.
World_ID :: struct
{
	index1: u16,
	revision: u16,
}

// Body id references a body instance. This should be treated as an opaque handle.
Body_ID :: struct
{
	index1: i32,
	world0: u16,
	revision: u16,
}

// Shape id references a shape instance. This should be treated as an opaque handle.
Shape_ID :: struct
{
	index1: i32,
	world0: u16,
	revision: u16,
}

// Joint id references a joint instance. This should be treated as an opaque handle.
Joint_ID :: struct
{
	index1: i32,
	world0: u16,
	revision: u16,
}

// Chain id references a chain instances. This should be treated as an opaque handle.
Chain_ID :: struct
{
	index1: i32,
	world0: u16,
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
    return id.index1 == 0
}

// Determine if any id is non-null
is_non_null :: #force_inline proc "contextless" (id: $T) -> bool
{
    return id.index1 != 0
}

// Compare two ids for equality. Doesn't work for ```World_ID```.
id_equals :: #force_inline proc "contextless" (id1, id2: $T) -> bool
{
    return id1.index1 == id2.index1 && id1.world0 == id2.world0 && id1.revision == id2.revision
}