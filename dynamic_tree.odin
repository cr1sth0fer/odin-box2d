package box2d

DEFAULT_CATEGORY_BITS ::  0x00000001
DEFAULT_MASK_BITS ::  0xFFFFFFFF

// A node in the dynamic tree. The client does not interact with this directly.
// 16 + 16 + 8 + pad(8)
Tree_Node :: struct
{
	aabb: AABB, // 16

	// Category bits for collision filtering
	category_bits: u32, // 4

    parent_or_next: struct #raw_union 
	{
		parent, next: i32,
	}, // 4

	child1, // 4
	child2: i32, // 4

	// TODO_ERIN could be union with child index
	user_data: i32, // 4

	// leaf = 0, free node = -1
	height: i16, // 2

	enlarged: bool, // 1

	pad: [9]u8,
}

// A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
//
// A dynamic tree arranges data in a binary tree to accelerate
// queries such as AABB queries and ray casts. Leaf nodes are proxies
// with an AABB. These are used to hold a user collision object, such as a reference to a b2Shape.
//
// Nodes are pooled and relocatable, so I use node indices rather than pointers.
//
// The dynamic tree is made available for advanced users that would like to use it to organize
// spatial game data besides rigid bodies.
Dynamic_Tree :: struct
{
	nodes: [^]Tree_Node,

	root,
	node_count,
	node_capacity,
	free_list,
	proxy_count: i32,

	leaf_indices: [^]i32,
	leaf_boxes: [^]AABB,
	leaf_centers: [^]Vec2,
	bin_indices: [^]i32,
	rebuild_capacity: i32,
}

// This function receives proxies found in the AABB query.
// - return true if the query should continue
Tree_Query_Callback_Fcn :: #type proc "c" (proxy_id, user_data: i32, context_: rawptr) -> bool

// This function receives clipped raycast input for a proxy. The function
// returns the new ray fraction.
// - return a value of 0 to terminate the ray cast
// - return a value less than input->maxFraction to clip the ray
// - return a value of input->maxFraction to continue the ray cast without clipping
Tree_Ray_Cast_Callback_Fcn :: #type proc "c" (input: ^Ray_Cast_Input, proxy_id, user_data: i32, context_: rawptr) -> f32

// This function receives clipped raycast input for a proxy. The function
// returns the new ray fraction.
// - return a value of 0 to terminate the ray cast
// - return a value less than input->maxFraction to clip the ray
// - return a value of input->maxFraction to continue the ray cast without clipping
Tree_Shape_Cast_Callback_Fcn :: #type proc "c" (input: ^Shape_Cast_Input, proxy_id, user_data: i32, context_: rawptr) -> f32

// Get proxy user data
// - return the proxy user data or 0 if the id is invalid
dynamic_tree_get_user_data :: #force_inline proc(tree: ^Dynamic_Tree, proxy_id: i32) -> i32
{
	return tree.nodes[proxy_id].user_data;
}

// Get the AABB of a proxy
dynamic_tree_get_aabb :: #force_inline proc(tree: ^Dynamic_Tree, proxy_id: i32) -> AABB
{
	return tree.nodes[proxy_id].aabb;
}