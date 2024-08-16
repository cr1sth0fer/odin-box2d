package box2d

// Task interface
//
// This is prototype for a Box2D task. Your task system is expected to invoke the Box2D task with these arguments.
//
// The task spans a range of the parallel-for: ```[start_index, end_index)```
//
// The worker index must correctly identify each worker in the user thread pool, expected in ```[0, worker_count)```.
//
// A worker must only exist on only one thread at a time and is analogous to the thread index.
// The task context is the context pointer sent from Box2D when it is enqueued.
//
// The ```start_index``` and ```end_index``` are expected in the range ```[0, item_count)``` where ```item_count``` is the argument to ```Enqueue_Task_Callback```
// below. Box2D expects ```start_index < end_index``` and will execute a loop like this:
//
//	for i := start_index; i < end_index; i += 1
//	{
//		do_work()
//	}
Task_Callback :: #type proc "c" (start_index, end_index: i32, worker_index: u32, task_context: rawptr)

// These functions can be provided to Box2D to invoke a task system. These are designed to work well with **enkiTS**.
//
// Returns a pointer to the user's task object. May be ```nil```. A ```nil``` indicates to **Box2D** that the work was executed
// serially within the callback and there is no need to call ```Finish_Task_Callback```.
//
// The ```item_count``` is the number of **Box2D** work items that are to be partitioned among workers by the user's task system.
//
// This is essentially a parallel-for. The ```min_range``` parameter is a suggestion of the minimum number of items to assign
// per worker to reduce overhead. For example, suppose the task is small and that ```item_count``` is ```16```. A ```min_range``` of ```8``` suggests
// that your task system should split the work items among just two workers, even if you have more available.
//
// In general the range ```[start_index, end_index)``` send to ```Task_Callback``` should obey:
// ```end_index - start_index >= min_range```
//
// The exception of course is when ```item_count < min_range```.
Enqueue_Task_Callback :: #type proc "c" (task: ^Task_Callback, item_count, min_range: i32, task_context, user_context: rawptr) -> rawptr

// Finishes a user task object that wraps a **Box2D** task.
Finish_Task_Callback :: #type proc "c" (user_task, user_context: rawptr)

// Result from ```world_ray_cast_closest```
Ray_Result :: struct
{
	shape_id: Shape_ID,
	point,
	normal: Vec2,
	fraction: f32,
	hit: bool,
}

// World definition used to create a simulation world.
//
// Must be initialized using ```default_world_def```.
World_Def :: struct
{
	// Gravity vector. Box2D has no up-vector defined.
	gravity: Vec2,

	// Restitution velocity threshold, usually in m/s. Collisions above this
	// speed have restitution applied (will bounce).
	restitution_threshold,

	// This parameter controls how fast overlap is resolved and has units of meters per second
	contact_pushout_velocity,

	// Threshold velocity for hit events. Usually meters per second.
	hit_event_threshold,

	// Contact stiffness. Cycles per second.
	contact_hertz,

	// Contact bounciness. Non-dimensional.
	contact_damping_ratio,

	// Joint stiffness. Cycles per second.
	joint_hertz,

	// Joint bounciness. Non-dimensional.
	joint_damping_ratio: f32,

	// Can bodies go to sleep to improve performance
	enable_sleep,

	// Enable continuous collision
	enable_continous: bool,

	// Number of workers to use with the provided task system. Box2D performs best when using only
	// performance cores and accessing a single L2 cache. Efficiency cores and hyper-threading provide
	// little benefit and may even harm performance.
	worker_count: u32,

	// function to spawn tasks
	enqueue_task: Enqueue_Task_Callback,

	// function to finish a task
	finish_task: Finish_Task_Callback,

	// User context that is provided to ```enqueue_task``` and ```finish_task```
	user_task_context: rawptr,

	// Used internally to detect a valid definition. DO **NOT SET**.
	internal_value: i32,
}

// The body simulation type.
//
// Each body is one of these three types. The type determines how the body behaves in the simulation.
Body_Type :: enum i32
{
	Static = 0,
	Kinematic = 1,
	Dynamic = 2,
}

// A body definition holds all the data needed to construct a rigid body.
//
// You can safely re-use body definitions. Shapes are added to a body after construction.
//
// Body definitions are temporary objects used to bundle creation parameters.
//
// Must be initialized using ```default_body_def```.
Body_Def :: struct
{
	// The body type: static, kinematic, or dynamic.
	type: Body_Type,

	// The initial world position of the body. Bodies should be created with the desired position.
	// Creating bodies at the origin and then moving them nearly doubles the cost of body creation, especially
	// if the body is moved after shapes have been added.
	position: Vec2,

	// The initial world rotation of the body. Use b2MakeRot() if you have an angle.
	rotation: Rot,

	// The initial linear velocity of the body's origin. Typically in meters per second.
	linear_velocity: Vec2,

	// The initial angular velocity of the body. Typically in meters per second.
	angular_velocity: f32,

	// Linear damping is use to reduce the linear velocity. The damping parameter
	// can be larger than 1 but the damping effect becomes sensitive to the
	// time step when the damping parameter is large.
	//
	// Generally linear damping is undesirable because it makes objects move slowly
	// as if they are floating.
	linear_damping: f32,

	// Angular damping is use to reduce the angular velocity. The damping parameter
	// can be larger than 1.0f but the damping effect becomes sensitive to the
	//
	// Angular damping can be use slow down rotating bodies.
	// time step when the damping parameter is large.
	angular_damping: f32,

	// Scale the gravity applied to this body. Non-dimensional.
	gravity_scale: f32,

	// Sleep velocity threshold, default is 0.05 meter per second
	sleep_threshold: f32,

	// Use this to store application specific body data.
	user_data: rawptr,

	// Set this flag to false if this body should never fall asleep.
	enable_sleep: bool,

	// Is this body initially awake or sleeping?
	is_awake: bool,

	// Should this body be prevented from rotating? Useful for characters.
	fixed_rotation: bool,

	// Treat this body as high speed object that performs continuous collision detection
	// against dynamic and kinematic bodies, but not other bullet bodies.
	//
	// Bullets should be used sparingly. They are not a solution for general dynamic-versus-dynamic
	// continuous collision. They may interfere with joint constraints.
	is_bullet: bool,

	// Used to disable a body. A disabled body does not move or collide.
	is_enabled: bool,

	// Automatically compute mass and related properties on this body from shapes.
	//
	// Triggers whenever a shape is add/removed/changed. Default is true.
	automatic_mass: bool,

	// Used internally to detect a valid definition. **DO NOT SET**.
	internal_value: i32,
}

// This is used to filter collision on shapes. It affects shape-vs-shape collision
// and shape-versus-query collision (such as ```World_Cast_Ray```).
Filter :: struct
{
	// The collision category bits. Normally you would just set one bit. The category bits should
	// represent your application object types. For example:
	// @code{.cpp}
	// My_Categories :: enum u32
	// {
	//	   Static  = 0x00000001,
	//	   Dynamic = 0x00000002,
	//	   Debris  = 0x00000004,
	//	   Player  = 0x00000008,
	//	   etc..
	// };
	category_bits: u32,

	// The collision mask bits. This states the categories that this
	// shape would accept for collision.
	// For example, you may want your player to only collide with static objects
	// and other players.
	// mask_bits := u32(My_Categories.Static | My_Categories.Player)
	mask_bits: u32,

	// Collision groups allow a certain group of objects to never collide (negative)
	// or always collide (positive). A group index of zero has no effect. Non-zero group filtering
	// always wins against the mask bits.
	//
	// For example, you may want ragdolls to collide with other ragdolls but you don't want
	// ragdoll self-collision. In this case you would give each ragdoll a unique negative group index
	// and apply that group index to all shapes on the ragdoll.
	group_index: i32,
}

// The query filter is used to filter collisions between queries and shapes. For example,
// you may want a ray-cast representing a projectile to hit players and the static environment
// but not debris.
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
	// A circle with an offset
	Circle,

	// A capsule is an extruded circle
	Capsule,

	// A line segment
	Segment,

	// A convex polygon
	Polygon,

	// A smooth segment owned by a chain shape
	Smooth_Segment,
}

// Used to create a shape.
// This is a temporary object used to bundle shape creation parameters. You may use
// the same shape definition to create multiple shapes.
//
// Must be initialized using ```default_shape_def```.
Shape_Def :: struct
{
	// Use this to store application specific shape data.
	user_data: rawptr,

	// The friction coefficient, usually in the range [0,1].
	friction: f32,

	// The restitution (elasticity) usually in the range [0,1].
	restitution: f32,

	// The density, usually in kg/m^2.
	density: f32,

	// Contact filtering data.
	filter: Filter,

	// Custom debug draw color.
	custom_color: i32,

	// A sensor shape collects contact information but never generates a collision
	// response.
	is_sensor: bool,

	// Enable sensor events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
	enable_sensor_events: bool,

	// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
	enable_contact_events: bool,

	// Enable hit events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
	enable_hit_events: bool,

	// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
	// and must be carefully handled due to multi-threading. Ignored for sensors.
	enable_pre_solve_events: bool,

	// Normally shapes on static bodies don't invoke contact creation when they are added to the world. This overrides
	// that behavior and causes contact creation. This significantly slows down static body creation which can be important
	// when there are many static shapes.
	force_contact_creation: bool,

	// Used internally to detect a valid definition. **DO NOT SET**.
	internal_value: i32,
}

// Used to create a chain of edges. This is designed to eliminate ghost collisions with some limitations.
//	- chains are one-sided
//	- chains have no mass and should be used on static bodies
//	- chains have a counter-clockwise winding order
//	- chains are either a loop or open
// - a chain must have at least 4 points
//	- the distance between any two points must be greater than b2_linearSlop
//	- a chain shape should not self intersect (this is not validated)
//	- an open chain shape has NO COLLISION on the first and final edge
//	- you may overlap two open chains on their first three and/or last three points to get smooth collision
//	- a chain shape creates multiple smooth edges shapes on the body
// https://en.wikipedia.org/wiki/Polygonal_chain
// Must be initialized using ```default_chain_def```.
//
// Do not use chain shapes unless you understand the limitations. This is an advanced feature.
Chain_Def :: struct
{
	// Use this to store application specific shape data.
	user_data: rawptr,

	// An array of at least 4 points. These are cloned and may be temporary.
	points: [^]Vec2,

	// The point count, must be 4 or more.
	count: i32,

	// The friction coefficient, usually in the range [0,1].
	friction: f32,

	// The restitution (elasticity) usually in the range [0,1].
	restitution: f32,

	// Contact filtering data.
	filter: Filter,

	// Indicates a closed chain formed by connecting the first and last points
	is_loop: bool,

	// Used internally to detect a valid definition. **DO NOT SET**.
	internalValue: i32,
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
	prepare_tasks,
	solver_tasks,
	prepare_constraints,
	integrate_velocities,
	warm_start,
	solve_velocities,
	integrate_positions,
	relax_velocities,
	apply_restitution,
	store_impulses,
	finalize_bodies,
	split_islands,
	sleep_islands,
	hit_events,
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
	color_counts: [12]i32,
}

// Joint type enumeration
//
// This is useful because all joint types use b2JointId and sometimes you
// want to get the type of a joint.
Joint_Type :: enum i32
{
	Distance,
	Motor,
	Mouse,
	Prismatic,
	Revolute,
	Weld,
	Wheel,
}

// Distance joint definition
//
// This requires defining an anchor point on both
// bodies and the non-zero distance of the distance joint. The definition uses
// local anchor points so that the initial configuration can violate the
// constraint slightly. This helps when saving and loading a game.
Distance_Joint_Def :: struct
{
	// The first attached body.
	body_id_a: Body_ID,

	// The second attached body.
	body_id_b: Body_ID,

	// The local anchor point relative to bodyA's origin.
	local_anchor_a: Vec2,

	// The local anchor point relative to bodyB's origin.
	local_anchor_b: Vec2,

	// The rest length of this joint. Clamped to a stable minimum value.
	length: f32,

	// Enable the distance constraint to behave like a spring. If false
	// then the distance joint will be rigid, overriding the limit and motor.
	enable_spring: bool,

	// The linear stiffness hertz (cycles per second)
	hertz: f32,

	// The linear damping ratio (non-dimensional)
	damping_ratio: f32,

	// Enable/disable the joint limit
	enable_limit: bool,

	// Minimum length. Clamped to a stable minimum value.
	min_length: f32,

	// Maximum length. Must be greater than or equal to the minimum length.
	max_length: f32,

	// Enable/disable the joint motor
	enable_motor: bool,

	// The maximum motor force, usually in newtons
	max_motor_force: f32,

	// The desired motor speed, usually in meters per second
	motor_speed: f32,

	// Set this flag to true if the attached bodies should collide.
	collide_connected: bool,

	// User data pointer
	user_data: rawptr,

	// Used internally to detect a valid definition. **DO NOT SET**.
	internal_value: i32,
}

// A motor joint is used to control the relative motion between two bodies
//
// A typical usage is to control the movement of a dynamic body with respect to the ground.
Motor_Joint_Def :: struct
{
	// The first attached body.
	body_id_a: Body_ID,

	// The second attached body.
	body_id_b: Body_ID,

	// Position of body_b minus the position of body_a, in body_a's frame.
	linear_offset: Vec2,

	// The body_b angle minus body_a angle in radians
	angular_offset: f32,

	// The maximum motor force in newtons
	max_force: f32,

	// The maximum motor torque in newton-meters
	max_torque: f32,

	// Position correction factor in the range [0,1]
	correction_factor: f32,

	// Set this flag to true if the attached bodies should collide
	collide_connected: bool,

	// User data pointer
	user_data: rawptr,

	// Used internally to detect a valid definition. **DO NOT SET**.
	internal_value: i32,
}

// A mouse joint is used to make a point on a body track a specified world point.
//
// This a soft constraint and allows the constraint to stretch without
// applying huge forces. This also applies rotation constraint heuristic to improve control.
Mouse_Joint_Def :: struct
{
	// The first attached body.
	body_id_a,

	// The second attached body.
	body_id_b: Body_ID,

	// The initial target point in world space
	target: Vec2,

	// Stiffness in hertz
	hertz: f32,

	// Damping ratio, non-dimensional
	damping_ratio: f32,

	// Maximum force, typically in newtons
	max_force: f32,

	// Set this flag to true if the attached bodies should collide.
	collide_connected: bool,

	// User data pointer
	user_data: rawptr,

	// Used internally to detect a valid definition. **DO NOT SET**.
	internal_value: i32,
}

// Prismatic joint definition
//
// This requires defining a line of motion using an axis and an anchor point.
// The definition uses local anchor points and a local axis so that the initial
// configuration can violate the constraint slightly. The joint translation is zero
// when the local anchor points coincide in world space.
Prismatic_Joint_Def :: struct
{
	// The first attached body.
	body_id_a: Body_ID,

	// The second attached body.
	body_id_b: Body_ID,

	/// The local anchor point relative to body_a's origin.
	local_anchor_a: Vec2,

	// The local anchor point relative to body_b's origin.
	local_anchor_b: Vec2,

	// The local translation unit axis in body_a.
	local_axis_a: Vec2,

	// The constrained angle between the bodies: body_b_angle - body_a_angle.
	reference_angle: f32,

	// Enable a linear spring along the prismatic joint axis
	enable_spring: bool,

	// The spring stiffness Hertz, cycles per second
	hertz: f32,

	// The spring damping ratio, non-dimensional
	damping_ratio: f32,

	// Enable/disable the joint limit.
	enable_limit: bool,

	// The lower translation limit, usually in meters.
	lower_translation: f32,

	// The upper translation limit, usually in meters.
	upper_translation: f32,

	// Enable/disable the joint motor.
	enable_motor: bool,

	// The maximum motor torque, usually in N-m.
	max_motor_force: f32,

	// The desired motor speed in radians per second.
	motor_speed: f32,

	// Set this flag to true if the attached bodies should collide.
	collide_connected: bool,

	// User data pointer
	user_data: rawptr,

	// Used internally to detect a valid definition. **DO NOT SET**.
	internal_value: i32,
}

// Revolute joint definition
//
// This requires defining an anchor point where the bodies are joined.
//
// The definition uses local anchor points so that the
// initial configuration can violate the constraint slightly. You also need to
// specify the initial relative angle for joint limits. This helps when saving
// and loading a game.
//
// The local anchor points are measured from the body's origin
// rather than the center of mass because:
// 1. you might not know where the center of mass will be
// 2. if you add/remove shapes from a body and recompute the mass, the joints will be broken
Revolute_Joint_Def :: struct
{
	// The first attached body.
	body_id_a: Body_ID,

	// The second attached body.
	body_id_b: Body_ID,

	// The local anchor point relative to bodyA's origin.
	local_anchor_a: Vec2,

	// The local anchor point relative to bodyB's origin.
	local_anchor_b: Vec2,

	// The bodyB angle minus bodyA angle in the reference state (radians).
	// This defines the zero angle for the joint limit.
	reference_angle: f32,

	// Enable a rotational spring on the revolute hinge axis
	enable_spring: bool,

	// The spring stiffness Hertz, cycles per second
	hertz: f32,

	// The spring damping ratio, non-dimensional
	damping_ratio: f32,

	// A flag to enable joint limits.
	enable_limit: bool,

	// The lower angle for the joint limit (radians).
	lower_angle: f32,

	// The upper angle for the joint limit (radians).
	upper_angle: f32,

	// A flag to enable the joint motor.
	enable_motor: bool,

	// The maximum motor torque, typically in newton-meters
	max_motor_torque: f32,

	// The desired motor speed in radians per second
	motor_speed: f32,

	// Scale the debug draw
	draw_size: f32,

	// Set this flag to true if the attached bodies should collide
	collide_connected: bool,

	// User data pointer
	user_data: rawptr,

	// Used internally to detect a valid definition. **DO NOT SET**.
	internal_value: i32,
}

// Weld joint definition
//
// A weld joint connect to bodies together rigidly. This constraint provides springs to mimic
// soft-body simulation.
//
// The approximate solver in Box2D cannot hold many bodies together rigidly
Weld_Joint_Def :: struct
{
	// The first attached body.
	body_id_a: Body_ID,

	// The second attached body.
	body_id_b: Body_ID,

	// The local anchor point relative to body_a's origin.
	local_anchor_a: Vec2,

	// The local anchor point relative to body_b's origin.
	local_anchor_b: Vec2,

	// The bodyB angle minus bodyA angle in the reference state (radians).
	reference_angle: f32,

	// Linear stiffness expressed as hertz (oscillations per second). Use zero for maximum stiffness.
	linear_hertz: f32,

	// Angular stiffness as hertz (oscillations per second). Use zero for maximum stiffness.
	angular_hertz: f32,

	// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	linear_damping_ratio: f32,

	// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	angular_damping_ratio: f32,

	// Set this flag to true if the attached bodies should collide.
	collide_connected: bool,

	// User data pointer
	user_data: rawptr,

	// Used internally to detect a valid definition. **DO NOT SET**.
	internal_value: i32,
}

// Wheel joint definition
//
// This requires defining a line of motion using an axis and an anchor point.
//
// The definition uses local  anchor points and a local axis so that the initial
// configuration can violate the constraint slightly. The joint translation is zero
// when the local anchor points coincide in world space.
Wheel_Joint_Def :: struct
{
	// The first attached body.
	body_id_a: Body_ID,

	// The second attached body.
	body_id_b: Body_ID,

	// The local anchor point relative to bodyA's origin.
	local_anchor_a: Vec2,

	// The local anchor point relative to bodyB's origin.
	local_anchor_b: Vec2,

	// The local translation unit axis in bodyA.
	local_axis_a: Vec2,

	// Enable a linear spring along the local axis
	enable_spring: bool,

	// Spring stiffness in Hertz
	hertz: f32,

	// Spring damping ratio, non-dimensional
	damping_ratio: f32,

	// Enable/disable the joint limit.
	enable_limit: bool,

	// The lower translation limit
	lower_translation,

	// The upper translation limit
	upper_translation: f32,

	// Enable/disable the joint rotational motor
	enable_motor: bool,

	// The maximum motor torque, typically in newton-meters
	max_motor_torque: f32,

	// The desired motor speed in radians per second.
	motor_speed: f32,

	// Set this flag to true if the attached bodies should collide
	collide_connected: bool,

	// User data pointer
	user_data: rawptr,

	// Used internally to detect a valid definition. **DO NOT SET**.
	internal_value: i32,
}

// A begin touch event is generated when a shape starts to overlap a sensor shape.
Sensor_Begin_Touch_Event :: struct
{
	// The id of the sensor shape
	sensor_shape_id: Shape_ID,

	// The id of the dynamic shape that began touching the sensor shape
	visitor_shape_id: Shape_ID,
}

// An end touch event is generated when a shape stops overlapping a sensor shape.
Sensor_End_Touch_Event :: struct
{
	// The id of the sensor shape
	sensor_shape_id: Shape_ID,

	// The id of the dynamic shape that stopped touching the sensor shape
	visitor_shape_id: Shape_ID,
}

// Sensor events are buffered in the Box2D world and are available
// as begin/end overlap event arrays after the time step is complete.
//
// **NOTE:** these may become invalid if bodies and/or shapes are destroyed
Sensor_Events :: struct
{
	// Array of sensor begin touch events
	begin_events: [^]Sensor_Begin_Touch_Event,

	// Array of sensor end touch events
	end_events: [^]Sensor_End_Touch_Event,

	// The number of begin touch events
	begin_count: i32,

	// The number of end touch events
	end_count: i32,
}

// A begin touch event is generated when two shapes begin touching.
Contact_Begin_Touch_Event :: struct
{
	// Id of the first shape
	shape_id_a: Shape_ID,

	// Id of the second shape
	shape_id_b: Shape_ID,
}

// An end touch event is generated when two shapes stop touching.
Contact_End_Touch_Event :: struct
{
	// Id of the first shape
	shape_id_a: Shape_ID,

	// Id of the second shape
	shape_id_b: Shape_ID,
}

// A hit touch event is generated when two shapes collide with a speed faster than the hit speed threshold.
Contact_Hit_Event :: struct
{
	// Id of the first shape
	shape_id_a: Shape_ID,

	// Id of the second shape
	shape_id_b: Shape_ID,

	// Point where the shapes hit
	point: Vec2,

	// Normal vector pointing from shape A to shape B
	normal: Vec2,

	// The speed the shapes are approaching. Always positive. Typically in meters per second.
	approach_speed: f32,
}

// Contact events are buffered in the Box2D world and are available
// as event arrays after the time step is complete.
//
// **NOTE:** these may become invalid if bodies and/or shapes are destroyed
Contact_Events :: struct
{
	// Array of begin touch events
	begin_events: [^]Contact_Begin_Touch_Event,

	// Array of end touch events
	end_events: [^]Contact_End_Touch_Event,

	// Array of hit events
	hit_events: [^]Contact_Hit_Event,

	// Number of begin touch events
	begin_count: i32,

	/// Number of end touch events
	end_count: i32,

	// Number of hit events
	hit_count: i32,
}

// Body move events triggered when a body moves.
//
// Triggered when a body moves due to simulation. Not reported for bodies moved by the user.
//
// This also has a flag to indicate that the body went to sleep so the application can also
// sleep that actor/entity/object associated with the body.
//
// On the other hand if the flag does not indicate the body went to sleep then the application
// can treat the actor/entity/object associated with the body as awake.
//
// This is an efficient way for an application to update game object transforms rather than
// calling functions such as b2Body_GetTransform() because this data is delivered as a contiguous array
// and it is only populated with bodies that have moved.
//
// **NOTE:** If sleeping is disabled all dynamic and kinematic bodies will trigger move events.
Body_Move_Event :: struct
{
	transform: Transform,
	body_id: Body_ID,
	user_data: rawptr,
	fell_asleep: bool,
}

// Body events are buffered in the Box2D world and are available
// as event arrays after the time step is complete.
//
// **NOTE:** this date becomes invalid if bodies are destroyed
Body_Events :: struct
{
	// Array of move events
	move_events: [^]Body_Move_Event,

	// Number of move events
	moveCount: i32,
}

// The contact data for two shapes. By convention the manifold normal points
// from shape **A** to shape **B**.
//
// **SEE:** ```shape_get_contact_data``` and ```body_get_contact_data```
Contact_Data :: struct
{
	shape_id_a: Shape_ID,
	shape_id_b: Shape_ID,
	manifold: Manifold,
}

// Prototype for a contact filter callback.
//
// This is called when a contact pair is considered for collision. This allows you to
//	perform custom logic to prevent collision between shapes. This is only called if
//	one of the two shapes has custom filtering enabled. **SEE:** ```shape_def```.
//
// Notes:
// * this function must be thread-safe
// * this is only called if one of the two shapes has enabled custom filtering
// * this is called only for awake dynamic bodies
//
// Return ```false``` if you want to disable the collision
//
// **WARNING:** Do not attempt to modify the world inside this callback
Custom_Filter_Fcn :: #type proc "c" (shape_id_a, shape_id_b: Shape_ID, context_: rawptr) -> bool

// Prototype for a pre-solve callback.
//
// This is called after a contact is updated. This allows you to inspect a
// contact before it goes to the solver. If you are careful, you can modify the
// contact manifold (e.g. modify the normal).
//
// Notes:
// * this function must be thread-safe
// * this is only called if the shape has enabled presolve events
// * this is called only for awake dynamic bodies
// * this is not called for sensors
// * the supplied manifold has impulse values from the previous step
//
//	Return ```false``` if you want to disable the contact this step
//	**WARNING:** Do not attempt to modify the world inside this callback
Pre_Solve_Fcn :: #type proc "c" (shape_id_a, shape_id_b: Shape_ID, manifold: ^Manifold, context_: rawptr) -> bool

// Prototype callback for overlap queries.
//
// Called for each shape found in the query.
//
// **SEE:** ```world_query_aabb```
//
// **RETURN:** ```false``` to terminate the query.
Overlap_Result_Fcn :: #type proc "c" (shape_id: Shape_ID, context_: rawptr) -> bool

// Prototype callback for ray casts.
//
// Called for each shape found in the query. You control how the ray cast
// proceeds by returning a ```f32```:
// * *return* ```-1```: ignore this shape and continue
// * *return* ```0```: terminate the ray cast
// * *return* ```fraction```: clip the ray to this point
// * *return* ```1```: don't clip the ray and continue
// * ```shape_id``` the shape hit by the ray
// * ```point``` the point of initial intersection
// * ```normal``` the normal vector at the point of intersection
// * ```fraction``` the fraction along the ray at the point of intersection
// * ```context_``` the user context
//
// **RETURN:** ```-1``` to filter, ```0``` to terminate, fraction to clip the ray for closest hit, ```1``` to continue
// **SEE:** ```world_cast_ray```
Cast_Result_Fcn :: #type proc "c" (shape: Shape_ID, point, normal: Vec2, fraction: f32, context_: rawptr) -> f32

// These colors are used for debug draw.
HEX_Color :: enum u32
{
	Alice_Blue = 0xf0f8ff,
	Antique_White = 0xfaebd7,
	Antique_White1 = 0xffefdb,
	Antique_White2 = 0xeedfcc,
	Antique_White3 = 0xcdc0b0,
	Antique_White4 = 0x8b8378,
	Aqua = 0x00ffff,
	Aquamarine = 0x7fffd4,
	Aquamarine1 = 0x7fffd4,
	Aquamarine2 = 0x76eec6,
	Aquamarine3 = 0x66cdaa,
	Aquamarine4 = 0x458b74,
	Azure = 0xf0ffff,
	Azure1 = 0xf0ffff,
	Azure2 = 0xe0eeee,
	Azure3 = 0xc1cdcd,
	Azure4 = 0x838b8b,
	Beige = 0xf5f5dc,
	Bisque = 0xffe4c4,
	Bisque1 = 0xffe4c4,
	Bisque2 = 0xeed5b7,
	Bisque3 = 0xcdb79e,
	Bisque4 = 0x8b7d6b,
	Black = 0x000000,
	Blanched_Almond = 0xffebcd,
	Blue = 0x0000ff,
	Blue1 = 0x0000ff,
	Blue2 = 0x0000ee,
	Blue3 = 0x0000cd,
	Blue4 = 0x00008b,
	Blue_Violet = 0x8a2be2,
	Brown = 0xa52a2a,
	Brown1 = 0xff4040,
	Brown2 = 0xee3b3b,
	Brown3 = 0xcd3333,
	Brown4 = 0x8b2323,
	Burlywood = 0xdeb887,
	Burlywood1 = 0xffd39b,
	Burlywood2 = 0xeec591,
	Burlywood3 = 0xcdaa7d,
	Burlywood4 = 0x8b7355,
	Cadet_Blue = 0x5f9ea0,
	Cadet_Blue1 = 0x98f5ff,
	Cadet_Blue2 = 0x8ee5ee,
	Cadet_Blue3 = 0x7ac5cd,
	Cadet_Blue4 = 0x53868b,
	Chartreuse = 0x7fff00,
	Chartreuse1 = 0x7fff00,
	Chartreuse2 = 0x76ee00,
	Chartreuse3 = 0x66cd00,
	Chartreuse4 = 0x458b00,
	Chocolate = 0xd2691e,
	Chocolate1 = 0xff7f24,
	Chocolate2 = 0xee7621,
	Chocolate3 = 0xcd661d,
	Chocolate4 = 0x8b4513,
	Coral = 0xff7f50,
	Coral1 = 0xff7256,
	Coral2 = 0xee6a50,
	Coral3 = 0xcd5b45,
	Coral4 = 0x8b3e2f,
	Cornflower_Blue = 0x6495ed,
	Cornsilk = 0xfff8dc,
	Cornsilk1 = 0xfff8dc,
	Cornsilk2 = 0xeee8cd,
	Cornsilk3 = 0xcdc8b1,
	Cornsilk4 = 0x8b8878,
	Crimson = 0xdc143c,
	Cyan = 0x00ffff,
	Cyan1 = 0x00ffff,
	Cyan2 = 0x00eeee,
	Cyan3 = 0x00cdcd,
	Cyan4 = 0x008b8b,
	Dark_Blue = 0x00008b,
	Dark_Cyan = 0x008b8b,
	Dark_Goldenrod = 0xb8860b,
	Dark_Goldenrod1 = 0xffb90f,
	Dark_Goldenrod2 = 0xeead0e,
	Dark_Goldenrod3 = 0xcd950c,
	Dark_Goldenrod4 = 0x8b6508,
	Dark_Gray = 0xa9a9a9,
	Dark_Green = 0x006400,
	Dark_Khaki = 0xbdb76b,
	Dark_Magenta = 0x8b008b,
	Dark_Olive_Green = 0x556b2f,
	Dark_Olive_Green1 = 0xcaff70,
	Dark_Olive_Green2 = 0xbcee68,
	Dark_Olive_Green3 = 0xa2cd5a,
	Dark_Olive_Green4 = 0x6e8b3d,
	Dark_Orange = 0xff8c00,
	Dark_Orange1 = 0xff7f00,
	Dark_Orange2 = 0xee7600,
	Dark_Orange3 = 0xcd6600,
	Dark_Orange4 = 0x8b4500,
	Dark_Orchid = 0x9932cc,
	Dark_Orchid1 = 0xbf3eff,
	Dark_Orchid2 = 0xb23aee,
	Dark_Orchid3 = 0x9a32cd,
	Dark_Orchid4 = 0x68228b,
	Dark_Red = 0x8b0000,
	Dark_Salmon = 0xe9967a,
	Dark_Sea_Green = 0x8fbc8f,
	Dark_Sea_Green1 = 0xc1ffc1,
	Dark_Sea_Green2 = 0xb4eeb4,
	Dark_Sea_Green3 = 0x9bcd9b,
	Dark_Sea_Green4 = 0x698b69,
	Dark_Slate_Blue = 0x483d8b,
	Dark_Slate_Gray = 0x2f4f4f,
	Dark_Slate_Gray1 = 0x97ffff,
	Dark_Slate_Gray2 = 0x8deeee,
	Dark_Slate_Gray3 = 0x79cdcd,
	Dark_Slate_Gray4 = 0x528b8b,
	Dark_Turquoise = 0x00ced1,
	Dark_Violet = 0x9400d3,
	Deep_Pink = 0xff1493,
	Deep_Pink1 = 0xff1493,
	Deep_Pink2 = 0xee1289,
	Deep_Pink3 = 0xcd1076,
	Deep_Pink4 = 0x8b0a50,
	Deep_Sky_Blue = 0x00bfff,
	Deep_Sky_Blue1 = 0x00bfff,
	Deep_Sky_Blue2 = 0x00b2ee,
	Deep_Sky_Blue3 = 0x009acd,
	Deep_Sky_Blue4 = 0x00688b,
	Dim_Gray = 0x696969,
	Dodger_Blue = 0x1e90ff,
	Dodger_Blue1 = 0x1e90ff,
	Dodger_Blue2 = 0x1c86ee,
	Dodger_Blue3 = 0x1874cd,
	Dodger_Blue4 = 0x104e8b,
	Firebrick = 0xb22222,
	Firebrick1 = 0xff3030,
	Firebrick2 = 0xee2c2c,
	Firebrick3 = 0xcd2626,
	Firebrick4 = 0x8b1a1a,
	Floral_White = 0xfffaf0,
	Forest_Green = 0x228b22,
	Fuchsia = 0xff00ff,
	Gainsboro = 0xdcdcdc,
	Ghost_White = 0xf8f8ff,
	Gold = 0xffd700,
	Gold1 = 0xffd700,
	Gold2 = 0xeec900,
	Gold3 = 0xcdad00,
	Gold4 = 0x8b7500,
	Goldenrod = 0xdaa520,
	Goldenrod1 = 0xffc125,
	Goldenrod2 = 0xeeb422,
	Goldenrod3 = 0xcd9b1d,
	Goldenrod4 = 0x8b6914,
	Gray = 0xbebebe,
	Gray0 = 0x000000,
	Gray1 = 0x030303,
	Gray10 = 0x1a1a1a,
	Gray100 = 0xffffff,
	Gray11 = 0x1c1c1c,
	Gray12 = 0x1f1f1f,
	Gray13 = 0x212121,
	Gray14 = 0x242424,
	Gray15 = 0x262626,
	Gray16 = 0x292929,
	Gray17 = 0x2b2b2b,
	Gray18 = 0x2e2e2e,
	Gray19 = 0x303030,
	Gray2 = 0x050505,
	Gray20 = 0x333333,
	Gray21 = 0x363636,
	Gray22 = 0x383838,
	Gray23 = 0x3b3b3b,
	Gray24 = 0x3d3d3d,
	Gray25 = 0x404040,
	Gray26 = 0x424242,
	Gray27 = 0x454545,
	Gray28 = 0x474747,
	Gray29 = 0x4a4a4a,
	Gray3 = 0x080808,
	Gray30 = 0x4d4d4d,
	Gray31 = 0x4f4f4f,
	Gray32 = 0x525252,
	Gray33 = 0x545454,
	Gray34 = 0x575757,
	Gray35 = 0x595959,
	Gray36 = 0x5c5c5c,
	Gray37 = 0x5e5e5e,
	Gray38 = 0x616161,
	Gray39 = 0x636363,
	Gray4 = 0x0a0a0a,
	Gray40 = 0x666666,
	Gray41 = 0x696969,
	Gray42 = 0x6b6b6b,
	Gray43 = 0x6e6e6e,
	Gray44 = 0x707070,
	Gray45 = 0x737373,
	Gray46 = 0x757575,
	Gray47 = 0x787878,
	Gray48 = 0x7a7a7a,
	Gray49 = 0x7d7d7d,
	Gray5 = 0x0d0d0d,
	Gray50 = 0x7f7f7f,
	Gray51 = 0x828282,
	Gray52 = 0x858585,
	Gray53 = 0x878787,
	Gray54 = 0x8a8a8a,
	Gray55 = 0x8c8c8c,
	Gray56 = 0x8f8f8f,
	Gray57 = 0x919191,
	Gray58 = 0x949494,
	Gray59 = 0x969696,
	Gray6 = 0x0f0f0f,
	Gray60 = 0x999999,
	Gray61 = 0x9c9c9c,
	Gray62 = 0x9e9e9e,
	Gray63 = 0xa1a1a1,
	Gray64 = 0xa3a3a3,
	Gray65 = 0xa6a6a6,
	Gray66 = 0xa8a8a8,
	Gray67 = 0xababab,
	Gray68 = 0xadadad,
	Gray69 = 0xb0b0b0,
	Gray7 = 0x121212,
	Gray70 = 0xb3b3b3,
	Gray71 = 0xb5b5b5,
	Gray72 = 0xb8b8b8,
	Gray73 = 0xbababa,
	Gray74 = 0xbdbdbd,
	Gray75 = 0xbfbfbf,
	Gray76 = 0xc2c2c2,
	Gray77 = 0xc4c4c4,
	Gray78 = 0xc7c7c7,
	Gray79 = 0xc9c9c9,
	Gray8 = 0x141414,
	Gray80 = 0xcccccc,
	Gray81 = 0xcfcfcf,
	Gray82 = 0xd1d1d1,
	Gray83 = 0xd4d4d4,
	Gray84 = 0xd6d6d6,
	Gray85 = 0xd9d9d9,
	Gray86 = 0xdbdbdb,
	Gray87 = 0xdedede,
	Gray88 = 0xe0e0e0,
	Gray89 = 0xe3e3e3,
	Gray9 = 0x171717,
	Gray90 = 0xe5e5e5,
	Gray91 = 0xe8e8e8,
	Gray92 = 0xebebeb,
	Gray93 = 0xededed,
	Gray94 = 0xf0f0f0,
	Gray95 = 0xf2f2f2,
	Gray96 = 0xf5f5f5,
	Gray97 = 0xf7f7f7,
	Gray98 = 0xfafafa,
	Gray99 = 0xfcfcfc,
	Green = 0x00ff00,
	Green1 = 0x00ff00,
	Green2 = 0x00ee00,
	Green3 = 0x00cd00,
	Green4 = 0x008b00,
	Green_Yellow = 0xadff2f,
	Honeydew = 0xf0fff0,
	Honeydew1 = 0xf0fff0,
	Honeydew2 = 0xe0eee0,
	Honeydew3 = 0xc1cdc1,
	Honeydew4 = 0x838b83,
	Hot_Pink = 0xff69b4,
	Hot_Pink1 = 0xff6eb4,
	Hot_Pink2 = 0xee6aa7,
	Hot_Pink3 = 0xcd6090,
	Hot_Pink4 = 0x8b3a62,
	Indian_Red = 0xcd5c5c,
	Indian_Red1 = 0xff6a6a,
	Indian_Red2 = 0xee6363,
	Indian_Red3 = 0xcd5555,
	Indian_Red4 = 0x8b3a3a,
	Indigo = 0x4b0082,
	Ivory = 0xfffff0,
	Ivory1 = 0xfffff0,
	Ivory2 = 0xeeeee0,
	Ivory3 = 0xcdcdc1,
	Ivory4 = 0x8b8b83,
	Khaki = 0xf0e68c,
	Khaki1 = 0xfff68f,
	Khaki2 = 0xeee685,
	Khaki3 = 0xcdc673,
	Khaki4 = 0x8b864e,
	Lavender = 0xe6e6fa,
	Lavender_Blush = 0xfff0f5,
	Lavender_Blush1 = 0xfff0f5,
	Lavender_Blush2 = 0xeee0e5,
	Lavender_Blush3 = 0xcdc1c5,
	Lavender_Blush4 = 0x8b8386,
	Lawn_Green = 0x7cfc00,
	Lemon_Chiffon = 0xfffacd,
	Lemon_Chiffon1 = 0xfffacd,
	Lemon_Chiffon2 = 0xeee9bf,
	Lemon_Chiffon3 = 0xcdc9a5,
	Lemon_Chiffon4 = 0x8b8970,
	Light_Blue = 0xadd8e6,
	Light_Blue1 = 0xbfefff,
	Light_Blue2 = 0xb2dfee,
	Light_Blue3 = 0x9ac0cd,
	Light_Blue4 = 0x68838b,
	Light_Coral = 0xf08080,
	Light_Cyan = 0xe0ffff,
	Light_Cyan1 = 0xe0ffff,
	Light_Cyan2 = 0xd1eeee,
	Light_Cyan3 = 0xb4cdcd,
	Light_Cyan4 = 0x7a8b8b,
	Light_Goldenrod = 0xeedd82,
	Light_Goldenrod1 = 0xffec8b,
	Light_Goldenrod2 = 0xeedc82,
	Light_Goldenrod3 = 0xcdbe70,
	Light_Goldenrod4 = 0x8b814c,
	Light_Goldenrod_Yellow = 0xfafad2,
	Light_Gray = 0xd3d3d3,
	Light_Green = 0x90ee90,
	Light_Pink = 0xffb6c1,
	Light_Pink1 = 0xffaeb9,
	Light_Pink2 = 0xeea2ad,
	Light_Pink3 = 0xcd8c95,
	Light_Pink4 = 0x8b5f65,
	Light_Salmon = 0xffa07a,
	Light_Salmon1 = 0xffa07a,
	Light_Salmon2 = 0xee9572,
	Light_Salmon3 = 0xcd8162,
	Light_Salmon4 = 0x8b5742,
	Light_Sea_Green = 0x20b2aa,
	Light_Sky_Blue = 0x87cefa,
	Light_Sky_Blue1 = 0xb0e2ff,
	Light_Sky_Blue2 = 0xa4d3ee,
	Light_Sky_Blue3 = 0x8db6cd,
	Light_Sky_Blue4 = 0x607b8b,
	Light_Slate_Blue = 0x8470ff,
	Light_Slate_Gray = 0x778899,
	Light_Steel_Blue = 0xb0c4de,
	Light_Steel_Blue1 = 0xcae1ff,
	Light_Steel_Blue2 = 0xbcd2ee,
	Light_Steel_Blue3 = 0xa2b5cd,
	Light_Steel_Blue4 = 0x6e7b8b,
	Light_Yellow = 0xffffe0,
	Light_Yellow1 = 0xffffe0,
	Light_Yellow2 = 0xeeeed1,
	Light_Yellow3 = 0xcdcdb4,
	Light_Yellow4 = 0x8b8b7a,
	Lime = 0x00ff00,
	Lime_Green = 0x32cd32,
	Linen = 0xfaf0e6,
	Magenta = 0xff00ff,
	Magenta1 = 0xff00ff,
	Magenta2 = 0xee00ee,
	Magenta3 = 0xcd00cd,
	Magenta4 = 0x8b008b,
	Maroon = 0xb03060,
	Maroon1 = 0xff34b3,
	Maroon2 = 0xee30a7,
	Maroon3 = 0xcd2990,
	Maroon4 = 0x8b1c62,
	Medium_Aquamarine = 0x66cdaa,
	Medium_Blue = 0x0000cd,
	Medium_Orchid = 0xba55d3,
	Medium_Orchid1 = 0xe066ff,
	Medium_Orchid2 = 0xd15fee,
	Medium_Orchid3 = 0xb452cd,
	Medium_Orchid4 = 0x7a378b,
	Medium_Purple = 0x9370db,
	Medium_Purple1 = 0xab82ff,
	Medium_Purple2 = 0x9f79ee,
	Medium_Purple3 = 0x8968cd,
	Medium_Purple4 = 0x5d478b,
	Medium_Sea_Green = 0x3cb371,
	Medium_Slate_Blue = 0x7b68ee,
	Medium_Spring_Green = 0x00fa9a,
	Medium_Turquoise = 0x48d1cc,
	Medium_Violet_Red = 0xc71585,
	Midnight_Blue = 0x191970,
	Mint_Cream = 0xf5fffa,
	Misty_Rose = 0xffe4e1,
	Misty_Rose1 = 0xffe4e1,
	Misty_Rose2 = 0xeed5d2,
	Misty_Rose3 = 0xcdb7b5,
	Misty_Rose4 = 0x8b7d7b,
	Moccasin = 0xffe4b5,
	Navajo_White = 0xffdead,
	Navajo_White1 = 0xffdead,
	Navajo_White2 = 0xeecfa1,
	Navajo_White3 = 0xcdb38b,
	Navajo_White4 = 0x8b795e,
	Navy = 0x000080,
	Navy_Blue = 0x000080,
	Old_Lace = 0xfdf5e6,
	Olive = 0x808000,
	Olive_Drab = 0x6b8e23,
	Olive_Drab1 = 0xc0ff3e,
	Olive_Drab2 = 0xb3ee3a,
	Olive_Drab3 = 0x9acd32,
	Olive_Drab4 = 0x698b22,
	Orange = 0xffa500,
	Orange1 = 0xffa500,
	Orange2 = 0xee9a00,
	Orange3 = 0xcd8500,
	Orange4 = 0x8b5a00,
	Orange_Red = 0xff4500,
	Orange_Red1 = 0xff4500,
	Orange_Red2 = 0xee4000,
	Orange_Red3 = 0xcd3700,
	Orange_Red4 = 0x8b2500,
	Orchid = 0xda70d6,
	Orchid1 = 0xff83fa,
	Orchid2 = 0xee7ae9,
	Orchid3 = 0xcd69c9,
	Orchid4 = 0x8b4789,
	Pale_Goldenrod = 0xeee8aa,
	Pale_Green = 0x98fb98,
	Pale_Green1 = 0x9aff9a,
	Pale_Green2 = 0x90ee90,
	Pale_Green3 = 0x7ccd7c,
	Pale_Green4 = 0x548b54,
	Pale_Turquoise = 0xafeeee,
	Pale_Turquoise1 = 0xbbffff,
	Pale_Turquoise2 = 0xaeeeee,
	Pale_Turquoise3 = 0x96cdcd,
	Pale_Turquoise4 = 0x668b8b,
	Pale_Violet_Red = 0xdb7093,
	Pale_Violet_Red1 = 0xff82ab,
	Pale_Violet_Red2 = 0xee799f,
	Pale_Violet_Red3 = 0xcd6889,
	Pale_Violet_Red4 = 0x8b475d,
	Papaya_Whip = 0xffefd5,
	Peach_Puff = 0xffdab9,
	Peach_Puff1 = 0xffdab9,
	Peach_Puff2 = 0xeecbad,
	Peach_Puff3 = 0xcdaf95,
	Peach_Puff4 = 0x8b7765,
	Peru = 0xcd853f,
	Pink = 0xffc0cb,
	Pink1 = 0xffb5c5,
	Pink2 = 0xeea9b8,
	Pink3 = 0xcd919e,
	Pink4 = 0x8b636c,
	Plum = 0xdda0dd,
	Plum1 = 0xffbbff,
	Plum2 = 0xeeaeee,
	Plum3 = 0xcd96cd,
	Plum4 = 0x8b668b,
	Powder_Blue = 0xb0e0e6,
	Purple = 0xa020f0,
	Purple1 = 0x9b30ff,
	Purple2 = 0x912cee,
	Purple3 = 0x7d26cd,
	Purple4 = 0x551a8b,
	Rebecca_Purple = 0x663399,
	Red = 0xff0000,
	Red1 = 0xff0000,
	Red2 = 0xee0000,
	Red3 = 0xcd0000,
	Red4 = 0x8b0000,
	Rosy_Brown = 0xbc8f8f,
	Rosy_Brown1 = 0xffc1c1,
	Rosy_Brown2 = 0xeeb4b4,
	Rosy_Brown3 = 0xcd9b9b,
	Rosy_Brown4 = 0x8b6969,
	Royal_Blue = 0x4169e1,
	Royal_Blue1 = 0x4876ff,
	Royal_Blue2 = 0x436eee,
	Royal_Blue3 = 0x3a5fcd,
	Royal_Blue4 = 0x27408b,
	Saddle_Brown = 0x8b4513,
	Salmon = 0xfa8072,
	Salmon1 = 0xff8c69,
	Salmon2 = 0xee8262,
	Salmon3 = 0xcd7054,
	Salmon4 = 0x8b4c39,
	Sandy_Brown = 0xf4a460,
	Sea_Green = 0x2e8b57,
	Sea_Green1 = 0x54ff9f,
	Sea_Green2 = 0x4eee94,
	Sea_Green3 = 0x43cd80,
	Sea_Green4 = 0x2e8b57,
	Seashell = 0xfff5ee,
	Seashell1 = 0xfff5ee,
	Seashell2 = 0xeee5de,
	Seashell3 = 0xcdc5bf,
	Seashell4 = 0x8b8682,
	Sienna = 0xa0522d,
	Sienna1 = 0xff8247,
	Sienna2 = 0xee7942,
	Sienna3 = 0xcd6839,
	Sienna4 = 0x8b4726,
	Silver = 0xc0c0c0,
	Sky_Blue = 0x87ceeb,
	Sky_Blue1 = 0x87ceff,
	Sky_Blue2 = 0x7ec0ee,
	Sky_Blue3 = 0x6ca6cd,
	Sky_Blue4 = 0x4a708b,
	Slate_Blue = 0x6a5acd,
	Slate_Blue1 = 0x836fff,
	Slate_Blue2 = 0x7a67ee,
	Slate_Blue3 = 0x6959cd,
	Slate_Blue4 = 0x473c8b,
	Slate_Gray = 0x708090,
	Slate_Gray1 = 0xc6e2ff,
	Slate_Gray2 = 0xb9d3ee,
	Slate_Gray3 = 0x9fb6cd,
	Slate_Gray4 = 0x6c7b8b,
	Snow = 0xfffafa,
	Snow1 = 0xfffafa,
	Snow2 = 0xeee9e9,
	Snow3 = 0xcdc9c9,
	Snow4 = 0x8b8989,
	Spring_Green = 0x00ff7f,
	Spring_Green1 = 0x00ff7f,
	Spring_Green2 = 0x00ee76,
	Spring_Green3 = 0x00cd66,
	Spring_Green4 = 0x008b45,
	Steel_Blue = 0x4682b4,
	Steel_Blue1 = 0x63b8ff,
	Steel_Blue2 = 0x5cacee,
	Steel_Blue3 = 0x4f94cd,
	Steel_Blue4 = 0x36648b,
	Tan = 0xd2b48c,
	Tan1 = 0xffa54f,
	Tan2 = 0xee9a49,
	Tan3 = 0xcd853f,
	Tan4 = 0x8b5a2b,
	Teal = 0x008080,
	Thistle = 0xd8bfd8,
	Thistle1 = 0xffe1ff,
	Thistle2 = 0xeed2ee,
	Thistle3 = 0xcdb5cd,
	Thistle4 = 0x8b7b8b,
	Tomato = 0xff6347,
	Tomato1 = 0xff6347,
	Tomato2 = 0xee5c42,
	Tomato3 = 0xcd4f39,
	Tomato4 = 0x8b3626,
	Turquoise = 0x40e0d0,
	Turquoise1 = 0x00f5ff,
	Turquoise2 = 0x00e5ee,
	Turquoise3 = 0x00c5cd,
	Turquoise4 = 0x00868b,
	Violet = 0xee82ee,
	Violet_Red = 0xd02090,
	Violet_Red1 = 0xff3e96,
	Violet_Red2 = 0xee3a8c,
	Violet_Red3 = 0xcd3278,
	Violet_Red4 = 0x8b2252,
	Web_Gray = 0x808080,
	Web_Green = 0x008000,
	Web_Maroon = 0x800000,
	Web_Purple = 0x800080,
	Wheat = 0xf5deb3,
	Wheat1 = 0xffe7ba,
	Wheat2 = 0xeed8ae,
	Wheat3 = 0xcdba96,
	Wheat4 = 0x8b7e66,
	White = 0xffffff,
	White_Smoke = 0xf5f5f5,
	X11_Gray = 0xbebebe,
	X11_Green = 0x00ff00,
	X11_Maroon = 0xb03060,
	X11_Purple = 0xa020f0,
	Yellow = 0xffff00,
	Yellow1 = 0xffff00,
	Yellow2 = 0xeeee00,
	Yellow3 = 0xcdcd00,
	Yellow4 = 0x8b8b00,
	Yellow_Green = 0x9acd32,
}

// This struct holds callbacks you can implement to draw a Box2D world.
Debug_Draw :: struct
{
	// Draw a closed polygon provided in CCW order.
    draw_polygon: proc "c" (vertices: [^]Vec2, vertex_count: i32, color: HEX_Color, context_: rawptr),

	// Draw a solid closed polygon provided in CCW order.
    draw_solid_polygon: proc "c" (transform: Transform, vertices: [^]Vec2, vertex_count: i32, radius: f32, color: HEX_Color, context_: rawptr),

	// Draw a circle.
    draw_circle: proc "c" (center: Vec2, radius: f32, color: HEX_Color, context_: rawptr),

	// Draw a solid circle.
    draw_solid_circle: proc "c" (transform: Transform, radius: f32, color: HEX_Color, context_: rawptr),

	// Draw a capsule.
    draw_capsule: proc "c" (p1, p2: Vec2, radius: f32, color: HEX_Color, context_: rawptr),

	// Draw a solid capsule.
    draw_solid_capsule: proc "c" (p1, p2: Vec2, radius: f32, color: HEX_Color, context_: rawptr),

	// Draw a line segment.
    draw_segment: proc "c" (p1, p2: Vec2, color: HEX_Color, context_: rawptr),

	// Draw a transform. Choose your own length scale.
    draw_transform: proc "c" (transform: Transform, context_: rawptr),

	// Draw a point.
    draw_point: proc "c" (p: Vec2, size: f32, color: HEX_Color, context_: rawptr),

	// Draw a string.
	draw_string: proc "c" (p: Vec2, s: cstring, context_: rawptr),

	// Bounds to use if restricting drawing to a rectangular region
	drawing_bounds: AABB,

	// Option to restrict drawing to a rectangular region. May suffer from unstable depth sorting.
	use_drawing_bounds: bool,

	// Option to draw shapes
	draw_shapes: bool,

	// Option to draw joints
	draw_joints: bool,

	// Option to draw additional information for joints
	draw_aabbs: bool,

	// Option to draw the bounding boxes for shapes
	draw_mass: bool,

	// Option to draw the mass and center of mass of dynamic bodies
	draw_contacts: bool,

	// Option to draw contact points
	draw_graph_colors: bool,

	// Option to visualize the graph coloring used for contacts and joints
	draw_contact_normals: bool,

	// Option to draw contact normals
	draw_contact_impulses: bool,

	// Option to draw contact normal impulses
	draw_friction_impulses: bool,

	// User context that is passed as an argument to drawing callback functions
	context_: rawptr,
}
