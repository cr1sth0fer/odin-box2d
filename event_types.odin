package box2d

// A begin touch event is generated when a shape starts to overlap a sensor shape.
Sensor_Begin_Touch_Event :: struct
{
	sensor_shape_id,
	visitor_shape_id: Shape_ID,
}

// An end touch event is generated when a shape stops overlapping a sensor shape.
Sensor_End_Touch_Event :: struct
{
	sensor_shape_id,
	visitor_shape_id: Shape_ID,
}

// Sensor events are buffered in the Box2D world and are available
//	as begin/end overlap event arrays after the time step is complete.
//
// Note: these may become invalid if bodies and/or shapes are destroyed
Sensor_Events :: struct
{
	begin_events: [^]Sensor_Begin_Touch_Event,
	end_events: [^]Sensor_End_Touch_Event,
	begin_count,
	end_count: i32,
}

// A begin touch event is generated when two shapes begin touching. By convention the manifold
// normal points from shape A to shape B.
Contact_Begin_Touch_Event :: struct
{
	shape_id_a,
	shape_id_b: Shape_ID,
	manifold: Manifold,
}

// An end touch event is generated when two shapes stop touching.
Contact_End_Touch_Event :: struct
{
	shape_id_a,
	shape_id_b: Shape_ID,
}

// Contact events are buffered in the Box2D world and are available
// as event arrays after the time step is complete.
//
// Note: these may become invalid if bodies and/or shapes are destroyed
Contact_Events :: struct
{
	begin_events: [^]Contact_Begin_Touch_Event,
	end_events: [^]Contact_End_Touch_Event,
	begin_count,
	end_count: i32,
}

// The contact data for two shapes. By convention the manifold normal points
// from shape A to shape B.
Contact_Data :: struct
{
	shape_id_a,
	shape_id_b: Shape_ID,
	manifold: Manifold,
}