package box2d

// https://en.wikipedia.org/wiki/Pi
PI :: 3.14159265359

// 2D vector
//
// This can be used to represent a point or free vector
Vec2 :: [2]f32

// 2D rotation
//
// This is similar to using a complex number for rotation
Rot :: struct
{
	// cosine and sine
	c, s: f32,
}

// A 2D rigid transform
Transform :: struct
{
	p: Vec2,
	q: Rot,
}

// A 2-by-2 Matrix
Mat22 :: matrix[2,2]f32

// Axis-aligned bounding box
AABB :: struct
{
	lower_bound,
	upper_bound: Vec2,
}
