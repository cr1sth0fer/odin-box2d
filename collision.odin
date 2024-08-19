package box2d

// The maximum number of vertices on a convex polygon. Changing this affects performance even if you
// don't use more vertices.
MAX_POLYGON_VERTICES :: #config(BOX2D_MAX_POLYGON_VERTICES, 8)

// Low level ray-cast input data
Ray_Cast_Input :: struct
{
    // Start point of the ray cast
	origin: Vec2,

    // Translation of the ray cast
    translation: Vec2,

    // The maximum fraction of the translation to consider, typically 1
	max_fraction: f32,
}

// Low level shape cast input in generic form. This allows casting an arbitrary point
// cloud wrap with a radius. For example, a circle is a single point with a non-zero radius.
//
// A capsule is two points with a non-zero radius. A box is four points with a zero radius.
Shape_Cast_Input :: struct
{
    // A point cloud to cast
	points: [MAX_POLYGON_VERTICES]Vec2,

    // The number of points
	count: i32,

    // The radius around the point cloud
	radius: f32,

    // The translation of the shape cast
	translation: Vec2,

    // The maximum fraction of the translation to consider, typically 1
	max_fraction: f32,
}

// Low level ray-cast or shape-cast output data
Cast_Output :: struct
{
    // The surface normal at the hit point
	normal: Vec2,

    // The surface hit point
	point: Vec2,

    // The fraction of the input translation at collision
	fraction: f32,

    // The number of iterations used
	iterations: i32,

    // Did the cast hit?
	hit: bool,
}

// This holds the mass data computed for a shape.
Mass_Data :: struct
{
	// The mass of the shape, usually in kilograms.
	mass: f32,

	// The position of the shape's centroid relative to the shape's origin.
	center: Vec2,

	// The rotational inertia of the shape about the local origin.
	rotational_inertia: f32,
}

// A solid circle
Circle :: struct
{
    // The local center
	point: Vec2,

    // The radius
	radius: f32,
}

// A solid capsule can be viewed as two semicircles connected
// by a rectangle.
Capsule :: struct
{
    // Local center of the first semicircle
	point1: Vec2,

    // Local center of the second semicircle
    point2: Vec2,

    // The radius of the semicircles
	radius: f32,
}

// A solid convex polygon. It is assumed that the interior of the polygon is to
// the left of each edge.
//
// Polygons have a maximum number of vertices equal to ```MAX_POLYGON_VERTICES```.
//
// In most cases you should not need many vertices for a convex polygon.
//
// **WARNING: DO NOT** fill this out manually, instead use a helper function like
//	```make_polygon``` or ```make_box```.
Polygon :: struct
{
    // The polygon vertices
	vertices: [MAX_POLYGON_VERTICES]Vec2,

    // The outward normal vectors of the polygon sides
	normals: [MAX_POLYGON_VERTICES]Vec2,

    // The centroid of the polygon
	centroid: Vec2,

    // The external radius for rounded polygons
	radius: f32,

    // The number of polygon vertices
	count: i32,
}

// A line segment with two-sided collision.
Segment :: struct
{
    // The first point
	point1: Vec2,

    // The second point
    point2: Vec2,
}

// A smooth line segment with one-sided collision. Only collides on the right side.
//
// Several of these are generated for a chain shape.
// ghost1 -> point1 -> point2 -> ghost2
Smooth_Segment :: struct
{
	// The tail ghost vertex
	ghost1: Vec2,

	// The line segment
	segment: Segment,

	// The head ghost vertex
	ghost2: Vec2,

	// The owning chain shape index (internal usage only)
	chain_id: i32,
}

// A convex hull. Used to create convex polygons.
Hull :: struct
{
    // The final points of the hull
	points: [MAX_POLYGON_VERTICES]Vec2,

    // The number of points
	count: i32,
}

// Result of computing the distance between two line segments
Segment_Distance_Result :: struct
{
    // The closest point on the first segment
	closest1: Vec2,

    // The closest point on the second segment
	closest2: Vec2,

    // The barycentric coordinate on the first segment
	fraction1: f32,

    // The barycentric coordinate on the second segment
	fraction2: f32,

    // The squared distance between the closest points
	distance_squared: f32,
}

// A distance proxy is used by the GJK algorithm. It encapsulates any shape.
Distance_Proxy :: struct
{
    // The point cloud
	vertices: [MAX_POLYGON_VERTICES]Vec2,

    // The number of points
	count: i32,

    // The external radius of the point cloud
	radius: f32,
}

// Used to warm start b2Distance. Set count to zero on first call or
// use zero initialization.
Distance_Cache :: struct
{
    // The number of stored simplex points
	count: u16,

    // The cached simplex indices on shape A
	index_a: [3]u8,

    // The cached simplex indices on shape B
	index_b: [3]u8,
}

EMPTY_DISTANCE_CACHE :: Distance_Cache{}

// Input for b2Distance
Distance_Input :: struct
{
    // The proxy for shape A
	proxy_a: Distance_Proxy,

    // The proxy for shape B
	proxy_b: Distance_Proxy,

    // The world transform for shape A
	transform_a: Transform,

    // The world transform for shape B
	transform_b: Transform,

    // Should the proxy radius be considered?
	use_radii: bool,
}

// Output for b2ShapeDistance.
Distance_Output :: struct
{
    // Closest point on shape A
	point_a: Vec2,

    // closest point on shape B
	point_b: Vec2,

    // The final distance, zero if overlapped
	distance: f32,

    // Number of GJK iterations used
	iterations: i32,

	// The number of simplexes stored in the simplex array
	simplex_count: i32,
}

// Simplex vertex for debugging the GJK algorithm
Simplex_Vertex :: struct
{
	wA: Vec2,    // support point in proxyA
	wB: Vec2,    // support point in proxyB
	w:  Vec2,    // wB - wA
	a:  f32,     // barycentric coordinate for closest point
	indexA: i32, // wA index
	indexB: i32, // wB index
}

// Simplex from the GJK algorithm
Simplex :: struct
{
	v1, v2, v3: Simplex_Vertex, // vertices
	count: i32                  // number of valid vertices
}

// Input parameters for b2ShapeCast
Shape_Cast_Pair_Input :: struct
{
	// The proxy for shape A
    proxy_a: Distance_Proxy,

	// The proxy for shape B
    proxy_b: Distance_Proxy,

	// The world transform for shape A
    transform_a: Transform,

	// The world transform for shape B
    transform_b: Transform,

	// The translation of shape B
    translation_b: Vec2,

	// The fraction of the translation to consider, typically 1
    max_fraction: f32,
}

// This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to the body origin,
// which may not coincide with the center of mass. However, to support dynamics we must interpolate the center of mass
// position.
Sweep :: struct
{
	// Local center of mass position
	local_center: Vec2,

	// center world positions
	c1: Vec2,

    // Ending center of mass world position
    c2: Vec2,

	// Starting world rotation
	q1: Rot,

    // Ending world rotation
    q2: Rot,
}

// Input parameters for ```time_of_impact```
TOI_Input :: struct
{
	// The proxy for shape A
	proxy_a: Distance_Proxy,

	// The proxy for shape B
	proxy_b: Distance_Proxy,

	// The movement of shape A
	sweep_a: Sweep,

	// The movement of shape B
	sweep_b: Sweep,

	// Defines the sweep interval [0, tMax]
	t_max: f32,
}

// Describes the TOI output
TOI_State :: enum i32
{
	Unknown,
	Failed,
	Overlapped,
	Hit,
	Separated,
}

// Output parameters for time_of_impact.
TOI_Output :: struct
{
	// The type of result
	state: TOI_State,

	// The time of the collision
	t: f32,
}

// A manifold point is a contact point belonging to a contact
// manifold. It holds details related to the geometry and dynamics
// of the contact points.
Manifold_Point :: struct
{
	// Location of the contact point in world space. Subject to precision loss at large coordinates.
	//	Should only be used for debugging.
	point: Vec2,

	// Location of the contact point relative to bodyA's origin in world space
	// When used internally to the Box2D solver, these are relative to the center of mass.
	anchor_a: Vec2,

	// Location of the contact point relative to bodyB's origin in world space
	anchor_b: Vec2,

	// The separation of the contact point, negative if penetrating
	separation: f32,

	// The impulse along the manifold normal vector.
	normal_impulse: f32,

	// The friction impulse
	tangent_impulse: f32,

	// The maximum normal impulse applied during sub-stepping
	// todo not sure this is needed
	max_normal_impulse: f32,

	// Relative normal velocity pre-solve. Used for hit events. If the normal impulse is
	// zero then there was no hit. Negative means shapes are approaching.
	normal_velocity: f32,

	// Uniquely identifies a contact point between two shapes
	id: u16,

	// Did this contact point exist the previous step?
	persisted: bool,
}

// Conact manifold convex shapes.
Manifold :: struct
{
	// The manifold points, up to two are possible in 2D
	points: [2]Manifold_Point,

	// The unit normal vector in world space, points from shape A to bodyB
	normal: Vec2,

	// The number of contacts points, will be 0, 1, or 2
	point_count: i32,
}

// The default category bit for a tree proxy. Used for collision filtering.
DEFAULT_CATEGORY_BITS ::  0x00000001

// Convenience mask bits to use when you don't need collision filtering and just want
// all results.
DEFAULT_MASK_BITS ::  0xFFFFFFFF

// A node in the dynamic tree. This is private data placed here for performance reasons.
// 16 + 16 + 8 + pad(8)
Tree_Node :: struct
{
	// The node bounding box
	aabb: AABB, // 16

	// Category bits for collision filtering
	category_bits: u32, // 4

	using _: struct #raw_union
	{
		// The node parent index
		parent: i32,

		// The node freelist next index
		next: i32,
	}, // 4

	// Child 1 index
	child1: i32, // 4

	// Child 2 index
	child2: i32, // 4

	// User data
	// todo could be union with child index
	user_data: i32, // 4

	// Leaf = 0, free node = -1
	height: i16, // 2

	// Has the AABB been enlarged?
	enlarged: bool, // 1

	// Padding for clarity
	pad: [9]u8,
}

// The dynamic tree structure. This should be considered private data.
// It is placed here for performance reasons.
Dynamic_Tree :: struct
{
	// The tree nodes
	nodes: [^]Tree_Node,

	// The root index
	root: i32,

	// The number of nodes
	node_count: i32,

	// The allocated node space
	node_capacity: i32,

	// Node free list
	free_list: i32,

	// Number of proxies created
	proxy_count: i32,

	// Leaf indices for rebuild
	leaf_indices: [^]i32,

	// Leaf bounding boxes for rebuild
	leaf_boxes: [^]AABB,

	// Leaf bounding box centers for rebuild
	leaf_centers: [^]Vec2,

	// Bins for sorting during rebuild
	bin_indices: [^]i32,

	// Allocated space for rebuilding
	rebuild_capacity: i32,
}

// This function receives proxies found in the AABB query.
// * *return* ```true``` if the query should continue
Tree_Query_Callback_Fcn :: #type proc "c" (proxy_id: i32, user_data: i32, context_: rawptr) -> bool

// This function receives clipped raycast input for a proxy. The function
// returns the new ray fraction.
// * *return* a value of ```0``` to terminate the ray cast
// * *return* a value less than ```input.max_fraction``` to clip the ray
// * *return* a value of ```input.max_fraction``` to continue the ray cast without clipping
Tree_Ray_Cast_Callback_Fcn :: #type proc "c" (input: ^Ray_Cast_Input, proxy_id: i32, user_data: i32, context_: rawptr) -> f32

// This function receives clipped raycast input for a proxy. The function
// returns the new ray fraction.
// * *return* a value of ```0``` to terminate the ray cast
// * *return* a value less than ```input->max_fraction``` to clip the ray
// * *return* a value of ```input.max_fraction``` to continue the ray cast without clipping
Tree_Shape_Cast_Callback_Fcn :: #type proc "c" (input: ^Shape_Cast_Input, proxy_id: i32, user_data: i32, context_: rawptr) -> f32

// Get proxy user data
//
// *return* the proxy user data or ```0``` if the id is invalid
dynamic_tree_get_user_data :: #force_inline proc(tree: ^Dynamic_Tree, proxy_id: i32) -> i32
{
	return tree.nodes[proxy_id].user_data
}

// Get the AABB of a proxy
dynamic_tree_get_aabb :: #force_inline proc(tree: ^Dynamic_Tree, proxy_id: i32) -> AABB
{
	return tree.nodes[proxy_id].aabb
}
