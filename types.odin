package box2d

// Use an instance of this structure and the callback below to get the closest hit.
Ray_Result :: struct
{
	shape_id: Shape_ID,
	point,
	normal: Vec2,
	fraction: f32,
	hit: bool,
}

// World definition used to create a simulation world. Must be initialized using DEFAULT_WORLD_DEF.
World_Def :: struct
{
	// Gravity vector. Box2D has no up-vector defined.
	gravity: Vec2,

	// Restitution velocity threshold, usually in m/s. Collisions above this
	// speed have restitution applied (will bounce).
	restitution_threshold,

	// This parameter controls how fast overlap is resolved and has units of meters per second
	contact_pushout_velocity,

	// Contact stiffness. Cycles per second.
	contact_hertz,

	/// Contact bounciness. Non-dimensional.
	contact_damping_ratio,

	// Joint stiffness. Cycles per second.
	joint_hertz,

	// Joint bounciness. Non-dimensional.
	joint_damping_ratio: f32,

	// Can bodies go to sleep to improve performance
	enable_sleep,

	// Enable continuous collision
	enable_continous: bool,
	
	// Capacity for bodies. This may not be exceeded.
	body_capacity,

	// initial capacity for shapes
	shape_capacity,

	// Capacity for contacts. This may not be exceeded.
	contact_capacity,

	// Capacity for joints
	joint_capacity,

	// Stack allocator capacity. This controls how much space box2d reserves for per-frame calculations.
	//
	// Larger worlds require more space. b2Statistics can be used to determine a good capacity for your
	// application.
	stack_allocator_capacity: i32,

	// task system hookup
	worker_count: u32,

	// function to spawn task
	enqueue_task: Enqueue_Task_Callback,

	// function to finish a task
	finish_task: Finish_Task_Callback,

	// User context that is provided to enqueue_task and finish_task
	user_task_context: rawptr,
}

// The body type.
// static: zero mass, zero velocity, may be manually moved
// kinematic: zero mass, non-zero velocity set by user, moved by solver
// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
Body_Type :: enum i32
{
	Static = 0,
	Kinematic = 1,
	Dynamic = 2,
}

// A body definition holds all the data needed to construct a rigid body.
// You can safely re-use body definitions. Shapes are added to a body after construction.
Body_Def :: struct
{
	// The body type: static, kinematic, or dynamic.
	// Note: if a dynamic body would have zero mass, the mass is set to one.
	type: Body_Type,

	// The world position of the body. Avoid creating bodies at the origin
	// since this can lead to many overlapping shapes.
	position: Vec2,

	// The world angle of the body in radians.
	angle: f32,

	// The linear velocity of the body's origin in world co-ordinates.
	linear_velocity: Vec2,

	// The angular velocity of the body.
	angular_velocity,

	// Linear damping is use to reduce the linear velocity. The damping parameter
	// can be larger than 1.0f but the damping effect becomes sensitive to the
	// time step when the damping parameter is large.
	linear_damping,

	// Angular damping is use to reduce the angular velocity. The damping parameter
	// can be larger than 1.0f but the damping effect becomes sensitive to the
	// time step when the damping parameter is large.
	angular_damping,

	// Scale the gravity applied to this body.
	gravity_scale: f32,

	// Use this to store application specific body data.
	user_data: rawptr,

	// Set this flag to false if this body should never fall asleep. Note that
	// this increases CPU usage.
	enable_sleep,

	// Is this body initially awake or sleeping?
	is_awake,

	// Should this body be prevented from rotating? Useful for characters.
	fixed_rotation,

	// Treat this body as high speed object that performs continuous collision detection
	// against dynamic and kinematic bodies, but not other bullet bodies.
	is_bullet,

	// Does this body start out enabled?
	is_enabled: bool
}

// This holds contact filtering data.
Filter :: struct
{
	// The collision category bits. Normally you would just set one bit.
	category_bits,

	// The collision mask bits. This states the categories that this
	// shape would accept for collision.
	mask_bits: u32,

	// Collision groups allow a certain group of objects to never collide (negative)
	// or always collide (positive). Zero means no collision group. Non-zero group
	// filtering always wins against the mask bits.
	group_index: i32,
}

// This holds contact filtering data.
Query_Filter :: struct
{
	// The collision category bits. Normally you would just set one bit.
	category_bits,

	// The collision mask bits. This states the categories that this
	// shape would accept for collision.
	mask_bits: u32,
}

// Shape type
Shape_Type :: enum i32
{
	Capsule,
	Circle,
	Polygon,
	Segment,
	Smooth_Segment,
}

// Used to create a shape
Shape_Def :: struct
{
	// Use this to store application specific shape data.
	user_data: rawptr,

	// The friction coefficient, usually in the range [0,1].
	friction,

	// The restitution (elasticity) usually in the range [0,1].
	restitution,

	// The density, usually in kg/m^2.
	density: f32,

	// Contact filtering data.
	filter: Filter,

	// A sensor shape collects contact information but never generates a collision
	// response.
	is_sensor,

	// Enable sensor events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
	enable_sensor_events,

	// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
	enable_contact_events,

	// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
	// and must be carefully handled due to multi-threading. Ignored for sensors.
	enable_pre_solve_events: bool,
}

// Used to create a chain of edges. This is designed to eliminate ghost collisions with some limitations.
// - DO NOT use chain shapes unless you understand the limitations. This is an advanced feature!
// - chains are one-sided
// - chains have no mass and should be used on static bodies
// - the front side of the chain points the right of the point sequence
// - chains are either a loop or open
// - a chain must have at least 4 points
// - the distance between any two points must be greater than b2_linearSlop
// - a chain shape should not self intersect (this is not validated)
// - an open chain shape has NO COLLISION on the first and final edge
// - you may overlap two open chains on their first three and/or last three points to get smooth collision
// - a chain shape creates multiple hidden shapes on the body
// https://en.wikipedia.org/wiki/Polygonal_chain
Chain_Def :: struct
{
	// An array of at least 4 points. These are cloned and may be temporary.
	points: [^]Vec2,

	// The point count, must be 4 or more.
	count: u32,

	// Indicates a closed chain formed by connecting the first and last points
	is_loop: bool,

	// Use this to store application specific shape data.
	user_data: rawptr,

	// The friction coefficient, usually in the range [0,1].
	friction,

	// The restitution (elasticity) usually in the range [0,1].
	restitution: f32,

	// Contact filtering data.
	filter: Filter,
}

// Profiling data. Times are in milliseconds.
Profile :: struct
{
	step,
	pairs,
	collide,
	solve,
	build_islands,
	solve_constraints,
	broadphase,
	continuous: f32,
}

// Counters that give details of the simulation size
Counters :: struct
{
	island_count,
	body_count,
	contact_count,
	joint_count,
	proxy_count,
	pair_count,
	tree_height,
	stack_capacity,
	stack_used,
	byte_count,
	task_count: i32,
	color_counts: [GRAPH_COLORS_COUNT + 1]i32,
}