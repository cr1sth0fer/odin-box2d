package box2d

NULL_FEATURE :: max(u8)

make_id :: #force_inline proc "contextless" (a, b: $T) -> u16
{
    return (u8(a) << 8 | u8(b))
}

// A manifold point is a contact point belonging to a contact
// manifold. It holds details related to the geometry and dynamics
// of the contact points.
Manifold_Point :: struct
{
	// location of the contact in world space
	// subject to precision loss at large coordinates
	point: Vec2,

	// location of contact point relative to body center of mass in world space
	anchor_a, anchor_b: Vec2,

	// the separation of the contact point, negative if penetrating
	separation,

	// the non-penetration impulse
	normal_impulse,

	// the friction impulse
	tangent_impulse,

	// the maximum normal impulse applied during sub-stepping
	max_normal_impulse: f32,

	// uniquely identifies a contact point between two shapes
	id: u16,

	// did this contact point exist the previous step?
	persisted: bool,
}

// Conact manifold convex shapes.
Manifold :: struct
{
	points: [2]Manifold_Point,
	normal: Vec2,
	point_count: i32,
}

EMPTY_MANIFOLD :: Manifold{}