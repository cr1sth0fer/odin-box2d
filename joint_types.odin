package box2d

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

// Distance joint definition. This requires defining an anchor point on both
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

	// Minimum length. Clamped to a stable minimum value.
	min_length: f32,

	// Maximum length. Must be greater than or equal to the minimum length.
	max_length: f32,

	// The linear stiffness hertz (cycles per second)
	hertz: f32,

	// The linear damping ratio (non-dimensional)
	damping_ratio: f32,

	// Set this flag to true if the attached bodies should collide.
	collide_connected: bool,
}

DEFAULT_DISTANCE_JOINT_DEF :: Distance_Joint_Def {
	NULL_BODY_ID,
	NULL_BODY_ID,
	{0, 0},
	{0, 0},
	1,
	0,
	HUGE,
	0,
	0,
	false,
}

// A motor joint is used to control the relative motion
// between two bodies. A typical usage is to control the movement
// of a dynamic body with respect to the ground.
Motor_Joint_Def :: struct
{
	// The first attached body.
	body_id_a,

	// The second attached body.
	body_id_b: Body_ID,

	// Position of bodyB minus the position of bodyA, in bodyA's frame, in meters.
	linear_offset: Vec2,

	// The bodyB angle minus bodyA angle in radians.
	angular_offset,

	// The maximum motor force in N.
	max_force,

	// The maximum motor torque in N-m.
	max_torque,

	// Position correction factor in the range [0,1].
	correction_factor: f32
}

// Use this to initialize your joint definition
DEFAULT_MOTOR_JOINT_DEF :: Motor_Joint_Def {
	NULL_BODY_ID,
	NULL_BODY_ID,
	{0, 0},
	0,
	1,
	1,
	0.3,
};

// A mouse joint is used to make a point on a body track a
// specified world point. This a soft constraint and allows the constraint to stretch without
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
	hertz,

	// Damping ratio, non-dimensional
	damping_ratio: f32,
}

DEFAULT_MOUSE_JOINT_DEF :: Mouse_Joint_Def {
	NULL_BODY_ID,
	NULL_BODY_ID,
	{0, 0},
	4.0,
	1.0,
}

/// Prismatic joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space.
Prismatic_Joint_Def :: struct
{
	// The first attached body.
	body_id_a: Body_ID,

	// The second attached body.
	body_id_b: Body_ID,

	/// The local anchor point relative to bodyA's origin.
	local_anchor_a: Vec2,

	// The local anchor point relative to bodyB's origin.
	local_anchor_b: Vec2,

	// The local translation unit axis in bodyA.
	local_axis_a: Vec2,

	// The constrained angle between the bodies: bodyB_angle - bodyA_angle.
	reference_angle: f32,

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
}

DEFAULT_PRISMATIC_JOINT_DEF :: Prismatic_Joint_Def {
	NULL_BODY_ID,
	NULL_BODY_ID,
	{0, 0},
	{0, 0},
	{1, 0},
	0,
	false,
	0,
	0,
	false,
	0,
	0,
	false,
}

// Revolute joint definition. This requires defining an anchor point where the
// bodies are joined. The definition uses local anchor points so that the
// initial configuration can violate the constraint slightly. You also need to
// specify the initial relative angle for joint limits. This helps when saving
// and loading a game.
// The local anchor points are measured from the body's origin
// rather than the center of mass because:
// 1. you might not know where the center of mass will be.
// 2. if you add/remove shapes from a body and recompute the mass,
// the joints will be broken.
Revolute_Joint_Def :: struct
{
	// The first attached body.
	body_id_a,

	// The second attached body.
	body_id_b: Body_ID,

	// The local anchor point relative to bodyA's origin.
	local_anchor_a,

	// The local anchor point relative to bodyB's origin.
	local_anchor_b: Vec2,

	// The bodyB angle minus bodyA angle in the reference state (radians).
	// This defines the zero angle for the joint limit.
	reference_angle: f32,

	// A flag to enable joint limits.
	enable_limit: bool,

	// The lower angle for the joint limit (radians).
	lower_angle,

	// The upper angle for the joint limit (radians).
	upper_angle: f32,

	// A flag to enable the joint motor.
	enable_motor: bool,

	// The maximum motor torque used to achieve the desired motor speed.
	// Usually in N-m.
	max_motor_torque,

	// The desired motor speed. Usually in radians per second.
	motor_speed,

	// Scale the debug draw
	draw_size: f32,

	// Set this flag to true if the attached bodies should collide.
	collide_connected: bool,
}

DEFAULT_REVOLUTE_JOINT_DEF :: Revolute_Joint_Def {
	NULL_BODY_ID,
	NULL_BODY_ID,
    {0, 0},
    {0, 0},
	0,
    false,
	0,
	0,
    false,
	0,
	0,
	0.25,
	false,
}

// A weld joint connect to bodies together rigidly. This constraint can be made soft to mimic
// soft-body simulation.
// * warning the approximate solver in Box2D cannot hold many bodies together rigidly
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
	reference_angle,

	// Linear stiffness expressed as hertz (oscillations per second). Use zero for maximum stiffness.
	linear_hertz,

	// Angular stiffness as hertz (oscillations per second). Use zero for maximum stiffness.
	angular_hertz,

	// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	linear_damping_ratio,

	// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	angular_damping_ratio: f32,

	// Set this flag to true if the attached bodies should collide.
	collide_connected: bool,
}

// Use this to initialize your joint definition
DEFAULT_WELD_JOINT_DEF :: Weld_Joint_Def{
	NULL_BODY_ID,
	NULL_BODY_ID,
	{0, 0},
	{0, 0},
	0,
	0,
	0,
	0,
	0,
	false,
}

// Wheel joint definition. This requires defining a line of
// motion using an axis and an anchor point. The definition uses local
// anchor points and a local axis so that the initial configuration
// can violate the constraint slightly. The joint translation is zero
// when the local anchor points coincide in world space. Using local
// anchors and a local axis helps when saving and loading a game.
Wheel_Joint_Def :: struct
{
	// The first attached body.
	body_id_a,

	// The second attached body.
	body_id_b: Body_ID,

	// The local anchor point relative to bodyA's origin.
	local_anchor_a,

	// The local anchor point relative to bodyB's origin.
	local_anchor_b,

	// The local translation unit axis in bodyA.
	local_axis_a: Vec2,

	// Enable/disable the joint limit.
	enable_limit: bool,

	// The lower translation limit, usually in meters.
	lower_translation,

	// The upper translation limit, usually in meters.
	upper_translation: f32,

	// Enable/disable the joint motor.
	enable_motor: bool,

	// The maximum motor torque, usually in N-m.
	max_motor_torque,

	// The desired motor speed in radians per second.
	motor_speed,

	// Spring stiffness in Hertz
	hertz,

	// Spring damping ratio, non-dimensional
	damping_ratio: f32,

	// Set this flag to true if the attached bodies should collide.
	collide_connected: bool,
}

// Use this to initialize your joint definition
DEFAULT_WHEEL_JOINT_DEF :: Wheel_Joint_Def {
	NULL_BODY_ID,
	NULL_BODY_ID,
	{0, 0},
	{0, 0},
	{0, 1},
	false,
	0,
	0,
	false,
	0,
	0,
	1.0,
	0.7,
	false,
};