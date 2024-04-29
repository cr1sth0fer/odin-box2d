package box2d

// Result of computing the distance between two line segments
Segment_Distance_Result :: struct
{
	closest1,
	closest2: Vec2,
	fraction1,
	fraction2,
	distance_squared: f32,
}

// A distance proxy is used by the GJK algorithm. It encapsulates any shape.
Distance_Proxy :: struct
{
	vertices: [MAX_POLYGON_VERTICES]Vec2,
	count: i32,
	radius: f32,
}

// Used to warm start distance.
// Set count to zero on first call.
Distance_Cache :: struct
{
	metric: f32, ///< length or area
	count: u16,
	index_a: [3]u8, ///< vertices on shape A
	index_b: [3]u8, ///< vertices on shape B
}

EMPTY_DISTANCE_CACHE :: Distance_Cache{}

// Input for b2Distance.
// You have to option to use the shape radii
// in the computation. Even
Distance_Input :: struct
{
	proxy_a,
	proxy_b: Distance_Proxy,
	transform_a,
	transform_b: Transform,
	use_radii: bool,
}

// Output for b2Distance.
Distance_Output :: struct
{
	point_a, ///< closest point on shapeA
	point_b: Vec2, ///< closest point on shapeB
	distance: f32,
	iterations: i32, ///< number of GJK iterations used
}

// Input parameters for b2ShapeCast
Shape_Cast_Pair_Input :: struct
{
	proxy_a,
	proxy_b: Distance_Proxy,
	transform_a,
	transform_b: Transform,
	translation_b: Vec2,
	max_fraction: f32,
}

// This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to the body origin,
// which may not coincide with the center of mass. However, to support dynamics we must interpolate the center of mass
// position.
Sweep :: struct
{
	// local center of mass position
	local_center,

	// center world positions
	c1, c2: Vec2,

	// world rotations
	q1, q2: Rot,
}

// Input parameters for time_of_impact
TOI_Input :: struct
{
	proxy_a,
	proxy_b: Distance_Proxy,
	sweep_a,
	sweep_b: Sweep,

	// defines sweep interval [0, tMax]
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
	state: TOI_State,
	t: f32,
}