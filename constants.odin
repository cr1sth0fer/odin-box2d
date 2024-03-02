package box2d

// box2d bases all length units on meters, but you may need different units for your game.
//
// You can override this value to use different units.
LENGTH_UNITS_PER_METER :: #config(BOX2D_LENGTH_UNITS_PER_METER, 1)

// https://en.wikipedia.org/wiki/Pi
PI :: 3.14159265359

// This is used to fatten AABBs in the dynamic tree. This allows proxies
// to move by a small amount without triggering a tree adjustment.
//
// This is in meters.
// * Warning modifying this can have a significant impact on performance
AABB_MARGIN :: 0.1 * LENGTH_UNITS_PER_METER

// A small length used as a collision and constraint tolerance. Usually it is
// chosen to be numerically significant, but visually insignificant. In meters.
// @warning modifying this can have a significant impact on stability
LINEAR_SLOP :: 0.005 * LENGTH_UNITS_PER_METER

// A small angle used as a collision and constraint tolerance. Usually it is
// chosen to be numerically significant, but visually insignificant.
// * Warning modifying this can have a significant impact on stability
ANGULAR_SLOP :: 2 / 180 * PI

// The maximum number of vertices on a convex polygon. Changing this affects performance even if you
// don't use more vertices.
MAX_POLYGON_VERTICES :: #config(BOX2D_MAX_POLYGON_VERTICES, 8)

// Maximum number of simultaneous worlds that can be allocated
MAX_WORLDS :: #config(BOX2D_MAX_WORLDS, 128)

// The maximum linear translation of a body per time step. This limit is very large and is used
// to prevent numerical problems. You shouldn't need to adjust this. Meters.
// * Warning modifying this can have a significant impact on stability
MAX_TRANSLATION :: 4.0 * LENGTH_UNITS_PER_METER

// The maximum rotation of a body per time step. This limit is very large and is used
// to prevent numerical problems. You shouldn't need to adjust this.
// * Warning modifying this can have a significant impact on stability
MAX_ROTATION :: 0.25 * PI

// * Warning modifying this can have a significant impact on performance and stability
SPECULATIVE_DISTANCE :: 4.0 * LINEAR_SLOP

// The time that a body must be still before it will go to sleep. In seconds.
TIME_TO_SLEEP :: #config(BOX2D_TIME_TO_SLEEP, 0.5)

// A body cannot sleep if its linear velocity is above this tolerance.
LINEAR_SLEEP_TOLERANCE :: #config(BOX2D_LINEAR_SLEEP_TOLERANCE, 0.01 * LENGTH_UNITS_PER_METER)

// A body cannot sleep if its angular velocity is above this tolerance.
ANGULAR_SLEEP_TOLERANCE :: #config(BOX2D_ANGULAR_SLEEP_TOLERANCE, 2 / 180 * PI)

// Used to detect bad values. Positions greater than about 16km will have precision
// problems, so 100km as a limit should be fine in all cases.
HUGE :: 100_000.0 * LENGTH_UNITS_PER_METER

// Maximum parallel workers. Used to size some static arrays.
MAX_WORKERS :: 64

// Solver graph coloring
GRAPH_COLORS_COUNT :: 12

// Version numbering scheme.
// See http://en.wikipedia.org/wiki/Software_versioning
Version :: struct
{
	// significant changes
	major,

	// incremental changes
	minor,

	// bug fixes
	revision: i32,
}

// Current version.
VERSION :: Version{3, 0, 0}