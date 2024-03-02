package box2d

// A convex hull. Used to create convex polygons.
Hull :: struct
{
	points: [MAX_POLYGON_VERTICES]Vec2,
	count: i32,
}