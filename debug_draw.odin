package box2d

// This struct holds callbacks you can implement to draw a box2d world.
Debug_Draw :: struct
{
	// Draw a closed polygon provided in CCW order.
    draw_polygon: proc "c" (vertices: [^]Vec2, vertex_count: i32, color: Color, context_: rawptr),

	// Draw a solid closed polygon provided in CCW order.
    draw_solid_polygon: proc "c" (vertices: [^]Vec2, vertex_count: i32, color: Color, context_: rawptr),

	// Draw a rounded polygon provided in CCW order.
    draw_rounded_polygon: proc "c" (vertices: [^]Vec2, vertex_count: i32, radius: f32, line_color, fill_color: Color, context_: rawptr),
    
	// Draw a circle.
    draw_circle: proc "c" (center: Vec2, radius: f32, color: Color, context_: rawptr),
    
	// Draw a solid circle.
    draw_solid_circle: proc "c" (center: Vec2, radius: f32, axis: Vec2, color: Color, context_: rawptr),
    
	// Draw a capsule.
    draw_capsule: proc "c" (p1, p2: Vec2, radius: f32, color: Color, context_: rawptr),
    
	// Draw a solid capsule.
    draw_solid_capsule: proc "c" (p1, p2: Vec2, radius: f32, color: Color, context_: rawptr),
    
	// Draw a line segment.
    draw_segment: proc "c" (p1, p2: Vec2, color: Color, context_: rawptr),
    
	// Draw a transform. Choose your own length scale.
	// - param xf a transform.
    draw_transform: proc "c" (xf: Transform, context_: rawptr),
    
	// Draw a point.
    draw_point: proc "c" (p: Vec2, size: f32, color: Color, context_: rawptr),

	// Draw a string.
	draw_string: proc "c" (p: Vec2, s: cstring, context_: rawptr),

	draw_shapes,
	draw_joints,
	draw_aabbs,
	draw_mass,
	draw_contacts,
	draw_graph_colors,
	draw_contact_normals,
	draw_contact_impulses,
	draw_friction_impulses: bool,
	context_: rawptr,
}