package box2d

// 2D vector
Vec2 :: [2]f32

// 3D vector
Vec3 :: [3]f32

// 2D rotation
Rot :: struct
{
	// sine and cosine
	s, c: f32,
}

// A 2D rigid transform
Transform :: struct
{
	p: Vec2,
	q: Rot,
}

// A 2-by-2 Matrix
Mat22 :: struct
{
	// columns
	cx, cy: Vec2,
}

// Axis-aligned bounding box
AABB :: struct
{
	lower_bound,
	upper_bound: Vec2,
}

// Color for debug drawing. Each value has the range [0,1].
Color :: [4]f32