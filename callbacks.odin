package box2d

// Task interface
//
// This is prototype for a Box2D task. Your task system is expected to invoke the Box2D task with these arguments.
//
// The task spans a range of the parallel-for: [startIndex, endIndex)
//
// The worker index must correctly identify each worker in the user thread pool, expected in [0, workerCount).
//
// A worker must only exist on only one thread at a time and is analogous to the thread index.
// The task context is the context pointer sent from Box2D when it is enqueued.
//
//	The startIndex and end_index are expected in the range [0, item_count) where item_count is the argument to b2EnqueueTaskCallback
// below. Box2D expects start_index < end_index and will execute a loop like this:
//
//	for i := start_index; i < end_index; i += 1
//	{
//		do_work()
//	}
Task_Callback :: #type proc "c" (start_index, end_index: i32, worker_index: u32, task_context: rawptr)

// These functions can be provided to Box2D to invoke a task system. These are designed to work well with enkiTS.
//
// Returns a pointer to the user's task object. May be nullptr. A nullptr indicates to Box2D that the work was executed
// serially within the callback and there is no need to call b2FinishTaskCallback.
//
// The itemCount is the number of Box2D work items that are to be partitioned among workers by the user's task system.
//
// This is essentially a parallel-for. The min_range parameter is a suggestion of the minimum number of items to assign
// per worker to reduce overhead. For example, suppose the task is small and that itemCount is 16. A min_range of 8 suggests
// that your task system should split the work items among just two workers, even if you have more available.
//
// In general the range [start_index, end_index) send to b2TaskCallback should obey:
// end_index - start_index >= min_range
//
// The exception of course is when itemCount < min_range.
Enqueue_Task_Callback :: #type proc "c" (task: ^Task_Callback, item_count, min_range: i32, task_context, user_context: rawptr) -> rawptr

// Finishes a user task object that wraps a Box2D task.
Finish_Task_Callback :: #type proc "c" (user_task, user_context: rawptr)

// Prototype for a pre-solve callback.
//
// This is called after a contact is updated. This allows you to inspect a
// contact before it goes to the solver. If you are careful, you can modify the
// contact manifold (e.g. disable contact).
//
// Notes:
// - this function must be thread-safe
// - this is only called if the shape has enabled presolve events
// - this is called only for awake dynamic bodies
// - this is not called for sensors
// - the supplied manifold has impulse values from the previous step
// - return false if you want to disable the contact this step
Pre_Solve_Fcn :: #type proc "c" (shape_id_a, shape_id_b: Shape_ID, manifold: ^Manifold, context_: rawptr) -> bool

// Prototype callback for AABB queries.
//
// See world_query_aabb
//
// Called for each shape found in the query AABB.
// - return false to terminate the query.
Overlap_Result_Fcn :: #type proc "c" (shape_id: Shape_ID, context_: rawptr) -> bool

// Prototype callback for ray casts.
// See b2World::RayCast
// Called for each shape found in the query. You control how the ray cast
// proceeds by returning a float:
// - return -1: ignore this shape and continue
// - return 0: terminate the ray cast
// - return fraction: clip the ray to this point
// - return 1: don't clip the ray and continue
// - param shape the shape hit by the ray
// - param point the point of initial intersection
// - param normal the normal vector at the point of intersection
// - param fraction the fraction along the ray at the point of intersection
// - return -1 to filter, 0 to terminate, fraction to clip the ray for
// closest hit, 1 to continue
Cast_Result_Fcn :: #type proc "c" (shape: Shape_ID, point, normal: Vec2, fraction: f32, context_: rawptr) -> f32

// Joints and shapes are destroyed when their associated
// body is destroyed. Implement this listener so that you
// may nullify references to these joints and shapes.
Joint_Destroyed_Fcn :: #type proc "c" (joint_id: Joint_ID, context_: rawptr)
Shape_Destroyed_Fcn :: #type proc "c" (shape_id: Shape_ID, context_: rawptr)

// Implement this class to provide collision filtering. In other words, you can implement
// this class if you want finer control over contact creation.
// - return true if contact calculations should be performed between these two shapes.
// - warning for performance reasons this is only called when the AABBs begin to overlap.
Should_Collide_Fcn :: #type proc "c" (shape_id_a, shape_id_b: Shape_ID, context_: rawptr) -> bool

// Implement these callbacks to get contact information. You can use these results for
// things like sounds and game logic. You can also get contact results by
// traversing the contact lists after the time step. However, you might miss
// some contacts because continuous physics leads to sub-stepping.
//
// Additionally you may receive multiple callbacks for the same contact in a
// single time step.
//
// You should strive to make your callbacks efficient because there may be
// many callbacks per time step.
// - warning You cannot create/destroy Box2D entities inside these callbacks.
//
// Called when two shapes begin to touch.
Begin_Contact_Fcn :: #type proc "c" (shape_id_a, shape_id_b: Shape_ID, context_: rawptr)

// Called when two shapes cease to touch.
End_Contact_Fcn :: #type proc "c" (shape_id_a, shape_id_b: Shape_ID, context_: rawptr)


// This lets you inspect a contact after the solver is finished. This is useful
// for inspecting impulses.
//
// Note: the contact manifold does not include time of impact impulses, which can be
// arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
// in a separate data structure.
//
// Note: this is only called for contacts that are touching, solid, and awake.
Post_Solve_Fcn :: #type proc "c" (shape_id_a, shape_id_b: Shape_ID, manifold: ^Manifold, context_: rawptr)

//TODO: world_set_post_solve_callback :: proc "c" (world_id: World_ID, fcn: Post_Solve_Fcn, context_: rawptr)

World_Callbacks :: struct
{
	joint_destroyed_fcn: Joint_Destroyed_Fcn,
	shape_destroyed_fcn: Shape_Destroyed_Fcn,
	should_collide_fcn: Should_Collide_Fcn,
	begin_contact_fcn: Begin_Contact_Fcn,
	end_contact_fcn: End_Contact_Fcn,
	pre_solve_fcn: Pre_Solve_Fcn,
	post_solve_fcn: Post_Solve_Fcn,
}