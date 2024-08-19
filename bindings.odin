package box2d

AVX2 :: #config(BOX2D_AVX2, true)

when ODIN_OS == .Windows && ODIN_ARCH == .amd64
{
    when AVX2
    {
        foreign import box2d {
            "binaries/box2d_windows_amd64_avx2.lib",
        }
    }
    else
    {
        foreign import box2d {
            "binaries/box2d_windows_amd64_sse2.lib",
        }
    }
}
else when ODIN_OS == .Linux && ODIN_ARCH == .amd64
{
    when AVX2
    {
        foreign import box2d {
            "binaries/box2d_linux_amd64_avx2.a",
        }
    }
    else
    {
        foreign import box2d {
        	"binaries/box2d_linux_amd64_sse2.a",
        }
    }
}

@(default_calling_convention="c")
foreign box2d
{
    /* base.h */


    // This allows the user to override the allocation functions. These should be
    // set during application startup.
    @(link_name="b2SetAllocator")
    set_allocator :: proc(alloc_fcn: Alloc_Fcn, free_fcn: Free_Fcn) ---

    // *return* the total bytes allocated by Box2D
    @(link_name="b2GetByteCount")
    get_byte_count :: proc() -> i32 ---

    // Override the default assert callback.
    // * ```assert_fcn``` a non-null assert callback
    @(link_name="b2SetAssertFcn")
    set_assert_fcn :: proc(assert_fcn: Assert_Fcn) ---

    // Get the current version of Box2D
    @(link_name="b2GetVersion")
    get_version :: proc() -> Version ---

    @(link_name="b2CreateTimer")
    create_timer :: proc() -> Timer ---

    @(link_name="b2GetTicks")
    get_ticks :: proc(timer: ^Timer) -> i64 ---

    @(link_name="b2GetMilliseconds")
    get_milliseconds :: proc(timer: ^Timer) -> f32 ---

    @(link_name="b2GetMillisecondsAndReset")
    get_milliseconds_and_reset :: proc(timer: ^Timer) -> f32 ---

    @(link_name="b2SleepMilliseconds")
    sleep_milliseconds :: proc(milliseconds: i32) ---

    @(link_name="b2Yield")
    yield :: proc() ---


    /* box2d.h */


    // Create a world for rigid body simulation. A world contains bodies, shapes, and constraints. You make create
    // up to ```128``` worlds. Each world is completely independent and may be simulated in parallel.
    //
    // *return* the world id.
    @(link_name="b2CreateWorld")
    create_world :: proc(def: ^World_Def) -> World_ID ---

    // Destroy a world.
    @(link_name="b2DestroyWorld")
    destroy_world :: proc(world_id: World_ID) ---

    // World identifier validation. Provides validation for up to 64K allocations.
    @(link_name="b2World_IsValid")
    world_is_valid :: proc(id: World_ID) -> bool ---

    // Simulate a world for one time step. This performs collision detection, integration, and constraint solution.
    // * ```world_id``` The world to simulate
    // * ```time_step``` The amount of time to simulate, this should be a fixed number. Typically ```1/60```.
    // * ```sub_step_count``` The number of sub-steps, increasing the sub-step count can increase accuracy. Typically ```4```.
    @(link_name="b2World_Step")
    world_step :: proc(world_id: World_ID, time_step: f32, sub_step_count: i32) ---

    // Call this to draw shapes and other debug draw data
    @(link_name="b2World_Draw")
    world_draw :: proc(world_id: World_ID, debug_draw: ^Debug_Draw) ---

    // Get the body events for the current time step. The event data is transient. Do not store a reference to this data.
    @(link_name="b2World_GetBodyEvents")
    world_get_body_events :: proc(worldId: World_ID) -> Body_Events ---

    // Get sensor events for the current time step. The event data is transient. Do not store a reference to this data.
    @(link_name="b2World_GetSensorEvents")
    world_get_sensor_events :: proc(world_id: World_ID) -> Sensor_Events ---

    // Get contact events for this current time step. The event data is transient. Do not store a reference to this data.
    @(link_name="b2World_GetContactEvents")
    world_get_contact_events :: proc(world_id: World_ID) -> Contact_Events ---

    // Overlap test for all shapes that **potentially** overlap the provided AABB.
    @(link_name="b2World_OverlapAABB")
    world_overlap_aabb :: proc(world_id: World_ID, aabb: AABB, filter: Query_Filter, fcn: Overlap_Result_Fcn, context_: rawptr) ---

    // Query the world for all shapes that overlap the provided circle.
    @(link_name="b2World_OverlapCircle")
    world_overlap_circle :: proc(world_id: World_ID, circle: ^Circle, transform: Transform, filter: Query_Filter, fcn: Overlap_Result_Fcn, context_: rawptr) ---

    // Query the world for all shapes that overlap the provided capsule.
    @(link_name="b2World_OverlapCapsule")
    world_overlap_capsule :: proc(world_id: World_ID, capsule: ^Capsule, transform: Transform, filter: Query_Filter, fcn: Overlap_Result_Fcn, context_: rawptr) ---

    // Query the world for all shapes that overlap the provided polygon.
    @(link_name="b2World_OverlapPolygon")
    world_overlap_polygon :: proc(world_id: World_ID, polygon: ^Polygon, transform: Transform, filter: Query_Filter, fcn: Overlap_Result_Fcn, context_: rawptr) ---

    // Cast a ray into the world to collect shapes in the path of the ray.
    //
    // Your callback function controls whether you get the closest point, any point, or n-points.
    //
    // The ray-cast ignores shapes that contain the starting point.
    // * ```world_id``` The world to cast the ray against
    // * ```origin``` The start point of the ray
    // * ```translation``` The translation of the ray from the start point to the end point
    // * ```filter``` Contains bit flags to filter unwanted shapes from the results
    // * ```fcn``` A user implemented callback function
    // * ```context_``` A user context that is passed along to the callback function
    //
    // **NOTE:** The callback function may receive shapes in any order
    @(link_name="b2World_CastRay")
    world_cast_ray :: proc(world_id: World_ID, origin, translation: Vec2, filter: Query_Filter, fcn: Cast_Result_Fcn, context_: rawptr) ---

    // Cast a ray into the world to collect the closest hit. This is a convenience function.
    //
    // This is less general than ```world_cast_ray``` and does not allow for custom filtering.
    @(link_name="b2World_CastRayClosest")
    world_cast_ray_closest :: proc(world_id: World_ID, origin, translation: Vec2, filter: Query_Filter) -> Ray_Result ---

    // Cast a circle through the world. Similar to a ray-cast except that a circle is cast instead of a point.
    @(link_name="b2World_CastCircle")
    world_cast_circle :: proc(world_id: World_ID, circle: ^Circle, origin_transform: Transform, translation: Vec2, filter: Query_Filter, fcn: Cast_Result_Fcn, context_: rawptr) ---

    // Cast a capsule through the world. Similar to a ray-cast except that a capsule is cast instead of a point.
    @(link_name="b2World_CastCapsule")
    world_cast_capsule :: proc(world_id: World_ID, capsule: ^Capsule, origin_transform: Transform, translation: Vec2, filter: Query_Filter, fcn: Cast_Result_Fcn, context_: rawptr) ---

    // Cast a capsule through the world. Similar to a ray-cast except that a polygon is cast instead of a point.
    @(link_name="b2World_CastPolygon")
    world_cast_polygon :: proc(world_id: World_ID, polygon: ^Polygon, origin_transform: Transform, translation: Vec2, filter: Query_Filter, fcn: Cast_Result_Fcn, context_: rawptr) ---

    // Enable/disable sleep. If your application does not need sleeping, you can gain some performance
    // by disabling sleep completely at the world level.
    //
    // **SEE:** ```World_Def```
    @(link_name="b2World_EnableSleeping")
    world_enable_sleeping :: proc(world_id: World_ID, flag: bool) ---

    // Enable/disable continuous collision between dynamic and static bodies. Generally you should keep continuous
    // collision enabled to prevent fast moving objects from going through static objects. The performance gain from
    // disabling continuous collision is minor.
    //
    // **SEE:** ```World_Def```
    @(link_name="b2World_EnableContinuous")
    world_enable_continuous :: proc(world_id: World_ID, flag: bool) ---

    // Adjust the restitution threshold. It is recommended not to make this value very small
    // because it will prevent bodies from sleeping. Typically in meters per second.
    //
    // **SEE:** ```World_Def```
    @(link_name="b2World_SetRestitutionThreshold")
    world_set_restitution_threshold :: proc(world_id: World_ID, value: f32) ---

    // Adjust the hit event threshold. This controls the collision velocity needed to generate a b2ContactHitEvent.
    //
    // Typically in meters per second.
    //
    // **SEE:** ```World_Def.hit_event_threshold```
    @(link_name="b2World_SetHitEventThreshold")
    world_set_hit_event_threshold :: proc(world_id: World_ID, value: f32) ---

    // Register the custom filter callback. This is optional.
    @(link_name="b2World_SetCustomFilterCallback")
    world_set_custom_filter_callback :: proc(world_id: World_ID, fcn: Custom_Filter_Fcn, context_: rawptr) ---

    // Register the pre-solve callback. This is optional.
    @(link_name="b2World_SetPreSolveCallback")
    world_set_pre_solve_callback :: proc(world_id: World_ID, fcn: Pre_Solve_Fcn, context_: rawptr) ---

    // Set the gravity vector for the entire world. Box2D has no concept of an up direction and this
    // is left as a decision for the application. Typically in ```m/s^2```.
    //
    // **SEE:** ```World_Def```
    @(link_name="b2World_SetGravity")
    world_set_gravity :: proc(world_id: World_ID, gravity: Vec2) ---

    // Get the gravity vector
    @(link_name="b2World_GetGravity")
    world_get_gravity :: proc(world_id: World_ID) -> Vec2 ---

    // Apply a radial explosion
    // * ```world_id``` The world id
    // * ```position``` The center of the explosion
    // * ```radius``` The radius of the explosion
    // * ```impulse``` The impulse of the explosion, typically in ```kg * m / s``` or ```N * s```.
    @(link_name="b2World_Explode")
    world_explode :: proc(world_id: World_ID, position: Vec2, radius: f32, impulse: f32) ---

    // Adjust contact tuning parameters
    // * ```world_id``` The world id
    // * ```hertz``` The contact stiffness (cycles per second)
    // * ```damping_ratio``` The contact bounciness with 1 being critical damping (non-dimensional)
    // * ```push_velocity``` The maximum contact constraint push out velocity (meters per second)
    //
    // **NOTE:** Advanced feature
    @(link_name="b2World_SetContactTuning")
    world_set_contact_tuning :: proc(world_id: World_ID, hertz, damping_ratio, push_velocity: f32) ---

    // Enable/disable constraint warm starting. Advanced feature for testing. Disabling
    // sleeping greatly reduces stability and provides no performance gain.
    @(link_name="b2World_EnableWarmStarting")
    world_enable_warm_starting :: proc(world_id: World_ID, flag: bool) ---

    // Get the current world performance profile
    @(link_name="b2World_GetProfile")
    world_get_profile :: proc(world_id: World_ID) -> Profile ---

    // Get world counters and sizes
    @(link_name="b2World_GetCounters")
    world_get_counters :: proc(world_id: World_ID) -> Counters ---

    // Dump memory stats to ```box2d_memory.txt```
    @(link_name="b2World_DumpMemoryStats")
    world_dump_memory_stats :: proc(world_id: World_ID) ---

    // Create a rigid body given a definition. No reference to the definition is retained. So you can create the definition
    // on the stack and pass it as a pointer.
    //
    //      body_def := b2.default_body_def()
    //      my_body_id := b2.create_body(my_world_id, &body_def)
    //
    // **WARNING:** This function is locked during callbacks.
    @(link_name="b2CreateBody")
    create_body :: proc(world_id: World_ID, def: ^Body_Def) -> Body_ID ---

    // Destroy a rigid body given an id. This destroys all shapes and joints attached to the body.
    //
    // Do not keep references to the associated shapes and joints.
    @(link_name="b2DestroyBody")
    destroy_body :: proc(body_id: Body_ID) ---

    // Body identifier validation. Can be used to detect orphaned ids. Provides validation for up to 64K allocations.
    @(link_name="b2Body_IsValid")
    body_is_valid :: proc(body_id: Body_ID) -> bool ---

    // Destroy a rigid body given an id. Destroys all joints attached to the body. Be careful
    // because this may invalidate some b2JointId that you have stored.
    // - warning This function is locked during callbacks.
    @(link_name="b2DestroyBodyAndJoints")
    destroy_body_and_joints :: proc(body_id: Body_ID) ---

    // Get the type of a body
    @(link_name="b2Body_GetType")
    body_get_type :: proc(body_id: Body_ID) -> Body_Type ---

    // Set the type of a body. This has a similar cost to re-creating the body.
    @(link_name="b2Body_SetType")
    body_set_type :: proc(body_id: Body_ID, type: Body_Type) ---

    // Set the user data for a body
    @(link_name="b2Body_SetUserData")
    body_set_user_data :: proc(body_id: Body_ID, user_data: rawptr) ---

    // Get the user data stored in a body
    @(link_name="b2Body_GetUserData")
    body_get_user_data :: proc(body_id: Body_ID) -> rawptr ---

    // Get the world position of a body. This is the location of the body origin.
    @(link_name="b2Body_GetPosition")
    body_get_position :: proc(body_id: Body_ID) -> Vec2 ---

    /// Get the world rotation of a body as a sine/cosine pair.
    @(link_name="b2Body_GetRotation")
    body_get_rotation :: proc(body_id: Body_ID) -> Rot ---

    // Get the world angle of a body in radians.
    @(link_name="b2Body_GetAngle")
    body_get_angle :: proc(body_id: Body_ID) -> f32 ---

    // Get the world transform of a body.
    @(link_name="b2Body_GetTransform")
    body_get_transform :: proc(body_id: Body_ID) -> Transform ---

    // Set the world transform of a body. This acts as a teleport and is fairly expensive.
    @(link_name="b2Body_SetTransform")
    body_set_transform :: proc(body_id: Body_ID, position: Vec2, rotation: Rot) ---

    // Get a local point on a body given a world point
    @(link_name="b2Body_GetLocalPoint")
    body_get_local_point :: proc(body_id: Body_ID, global_point: Vec2) -> Vec2 ---

    // Get a world point on a body given a local point
    @(link_name="b2Body_GetWorldPoint")
    body_get_world_point :: proc(body_id: Body_ID, local_point: Vec2) -> Vec2 ---

    // Get a local vector on a body given a world vector
    @(link_name="b2Body_GetLocalVector")
    body_get_local_vector :: proc(body_id: Body_ID, global_vector: Vec2) -> Vec2 ---

    // Get a world vector on a body given a local vector
    @(link_name="b2Body_GetWorldVector")
    body_get_world_vector :: proc(body_id: Body_ID, local_vector: Vec2) -> Vec2 ---

    // Get the linear velocity of a body's center of mass
    @(link_name="b2Body_GetLinearVelocity")
    body_get_linear_velocity :: proc(body_id: Body_ID) -> Vec2 ---

    // Get the angular velocity of a body in radians per second
    @(link_name="b2Body_GetAngularVelocity")
    body_get_angular_velocity :: proc(body_id: Body_ID) -> f32 ---

    // Set the linear velocity of a body
    @(link_name="b2Body_SetLinearVelocity")
    body_set_linear_velocity :: proc(body_id: Body_ID, linear_velocity: Vec2) ---

    // Set the angular velocity of a body in radians per second
    @(link_name="b2Body_SetAngularVelocity")
    body_set_angular_velocity :: proc(body_id: Body_ID, angular_velocity: f32) ---

    // Apply a force at a world point. If the force is not applied at the center of mass,
    // it will generate a torque and affect the angular velocity. This optionally wakes up the body.
    //
    // The force is ignored if the body is not awake.
    // * ```body_id``` The body id
    // * ```force``` The world force vector, typically in newtons ```(N)```
    // * ```point``` The world position of the point of application
    // * ```wake``` Option to wake up the body
    @(link_name="b2Body_ApplyForce")
    body_apply_force :: proc(body_id: Body_ID, force, point: Vec2, wake: bool) ---

    // Apply a force to the center of mass. This optionally wakes up the body.
    //
    // The force is ignored if the body is not awake.
    // * ```body_id``` The body id
    // * ```force``` the world force vector, usually in newtons ```(N)```.
    // * ```wake``` also wake up the body
    @(link_name="b2Body_ApplyForceToCenter")
    body_apply_force_to_center :: proc(body_id: Body_ID, force: Vec2, wake: bool) ---

    // Apply a torque. This affects the angular velocity without affecting the linear velocity.
    //
    // This optionally wakes the body. The torque is ignored if the body is not awake.
    // * ```bodyId``` The body id
    // * ```torque``` about the ```z-axis``` (out of the screen), typically in ```N*m```.
    // * ```wake``` also wake up the body
    @(link_name="b2Body_ApplyTorque")
    body_apply_torque :: proc(body_id: Body_ID, torque: f32, wake: bool) ---

    // Apply an impulse at a point. This immediately modifies the velocity.
    //
    // It also modifies the angular velocity if the point of application
    // is not at the center of mass. This optionally wakes the body.
    //
    // The impulse is ignored if the body is not awake.
    // * ```body_id``` The body id
    // * ```impulse``` the world impulse vector, typically in ```N*s``` or ```kg*m/s```.
    // * ```point``` the world position of the point of application.
    // * ```wake``` also wake up the body
    //
    // **WARNING:** This should be used for one-shot impulses. If you need a steady force,
    // use a force instead, which will work better with the sub-stepping solver.
    @(link_name="b2Body_ApplyLinearImpulse")
    body_apply_linear_impulse :: proc(body_id: Body_ID, impulse, point: Vec2, wake: bool) ---

    // Apply an impulse to the center of mass. This immediately modifies the velocity.
    //
    // The impulse is ignored if the body is not awake. This optionally wakes the body.
    // * ```body_id``` The body id
    // * ```impulse``` the world impulse vector, typically in ```N*s``` or ```kg*m/s```.
    // * ```wake``` also wake up the body
    //
    // **WARNING:** This should be used for one-shot impulses. If you need a steady force,
    // use a force instead, which will work better with the sub-stepping solver.
    @(link_name="b2Body_ApplyLinearImpulseToCenter")
    body_apply_linear_impulse_to_center :: proc(body_id: Body_ID, impulse: Vec2, wake: bool) ---

    // Apply an angular impulse. The impulse is ignored if the body is not awake.
    //
    // This optionally wakes the body.
    // * ```body_id``` The body id
    // * ```impulse``` the angular impulse, typically in units of ```kg*m*m/s```
    // * ```wake``` also wake up the body
    //
    // **WARNING:** This should be used for one-shot impulses. If you need a steady force,
    // use a force instead, which will work better with the sub-stepping solver.
    @(link_name="b2Body_ApplyAngularImpulse")
    body_apply_angular_impulse :: proc(body_id: Body_ID, impulse: f32, wake: bool) ---

    // Get the mass of the body, typically in kilograms
    @(link_name="b2Body_GetMass")
    body_get_mass :: proc(body_id: Body_ID) -> f32 ---

    /// Get the inertia tensor of the body, typically in ```kg*m^2```
    @(link_name="b2Body_GetInertiaTensor")
    body_get_inertia_tensor :: proc(body_id: Body_ID) -> f32 ---

    // Get the center of mass position of the body in local space.
    @(link_name="b2Body_GetLocalCenterOfMass")
    body_get_local_center_of_mass :: proc(body_id: Body_ID) -> Vec2 ---

    // Get the center of mass position of the body in world space.
    @(link_name="b2Body_GetWorldCenterOfMass")
    body_get_world_center_of_mass :: proc(body_id: Body_ID) -> Vec2 ---

    // Override the body's mass properties. Normally this is computed automatically using the
    // shape geometry and density. This information is lost if a shape is added or removed or if the
    // body type changes.
    @(link_name="b2Body_SetMassData")
    body_set_mass_data :: proc(body_id: Body_ID, mass_data: Mass_Data) ---

    // Get the mass data for a body.
    @(link_name="b2Body_GetMassData")
    body_get_mass_data :: proc(body_id: Body_ID) -> Mass_Data ---

    // This update the mass properties to the sum of the mass properties of the shapes.
    //
    // This normally does not need to be called unless you called ```set_mass_data``` to override
    // the mass and you later want to reset the mass.
    //
    // You may also use this when automatic mass computation has been disabled.
    //
    // You should call this regardless of body type.
    @(link_name="b2Body_ApplyMassFromShapes")
    body_apply_mass_from_shapes :: proc(body_id: Body_ID) ---

    // Set the automatic mass setting. Normally this is set in b2BodyDef before creation.
    //
    // **SEE:** ```Body_Def.automatic_mass```
    @(link_name="b2Body_SetAutomaticMass")
    body_set_automatic_mass :: proc(body_id: Body_ID, automatic_mass: bool) ---

    // Get the automatic mass setting
    @(link_name="b2Body_GetAutomaticMass")
    body_get_automatic_mass :: proc(body_id: Body_ID) -> bool ---

    // Adjust the linear damping. Normally this is set in b2BodyDef before creation.
    @(link_name="b2Body_SetLinearDamping")
    body_set_linear_damping :: proc(body_id: Body_ID, linear_damping: f32) ---

    // Get the current linear damping.
    @(link_name="b2Body_GetLinearDamping")
    body_get_linear_damping :: proc(body_id: Body_ID) -> f32 ---

    // Adjust the angular damping. Normally this is set in b2BodyDef before creation.
    @(link_name="b2Body_SetAngularDamping")
    body_set_angular_damping :: proc(body_id: Body_ID, angular_damping: f32) ---

    // Get the current angular damping.
    @(link_name="b2Body_GetAngularDamping")
    body_get_angular_damping :: proc(body_id: Body_ID) -> f32 ---

    // Adjust the gravity scale. Normally this is set in b2BodyDef before creation.
    //
    // **SEE:** ```Body_Def.gravity_scale```
    @(link_name="b2Body_SetGravityScale")
    body_set_gravity_scale :: proc(body_id: Body_ID, gravity_scale: f32) ---

    // Get the current gravity scale.
    @(link_name="b2Body_GetGravityScale")
    body_get_gravity_scale :: proc(body_id: Body_ID) -> f32 ---

    // *return* ```true``` if this body is awake
    @(link_name="b2Body_IsAwake")
    body_is_awake :: proc(body_id: Body_ID) -> bool ---

    // Wake a body from sleep. This wakes the entire island the body is touching.
    //
    // **WARNING:** Putting a body to sleep will put the entire island of bodies touching this body to sleep,
    // which can be expensive and possibly unintuitive.
    @(link_name="b2Body_Wake")
    body_set_awake :: proc(body_id: Body_ID, awake: bool) ---

    // Enable or disable sleeping for this body. If sleeping is disabled the body will wake.
    @(link_name="b2Body_EnableSleep")
    body_enable_sleep :: proc(body_id: Body_ID, enable_sleep: bool) ---

    // *returns* ```true``` if sleeping is enabled for this body
    @(link_name="b2Body_IsSleepEnabled")
    body_is_sleep_enabled :: proc(body_id: Body_ID) -> bool ---

    // *returns* ```true``` if this body is enabled
    @(link_name="b2Body_IsEnabled")
    body_is_enabled :: proc(body_id: Body_ID) -> bool ---

    // Disable a body by removing it completely from the simulation. This is expensive.
    @(link_name="b2Body_Disable")
    body_disable :: proc(body_id: Body_ID) ---

    // Enable a body by adding it to the simulation. This is expensive.
    @(link_name="b2Body_Enable")
    body_enable :: proc(body_id: Body_ID) ---

    // Set this body to have fixed rotation. This causes the mass to be reset in all cases.
    @(link_name="b2Body_SetFixedRotation")
    body_set_fixed_rotation :: proc(body_id: Body_ID, flag: bool) ---

    // Does this body have fixed rotation?
    @(link_name="b2Body_IsFixedRotation")
    body_is_fixed_rotation :: proc(body_id: Body_ID) -> bool ---

    // Set this body to be a bullet. A bullet does continuous collision detection
    // against dynamic bodies (but not other bullets).
    @(link_name="b2Body_SetBullet")
    body_set_bullet :: proc(body_id: Body_ID, flag: bool) ---

    // Is this body a bullet?
    @(link_name="b2Body_IsBullet")
    body_is_bullet :: proc(body_id: Body_ID) -> bool ---

    // Enable/disable hit events on all shapes
    //
    // **SEE:** ```Shape_Def.enable_hit_events```
    @(link_name="b2Body_EnableHitEvents")
    body_enable_hit_events :: proc(body_id: Body_ID, enable_hit_events: bool) ---

    // Get the number of shapes on this body
    @(link_name="b2Body_GetShapeCount")
    body_get_shape_count :: proc(body_id: Body_ID) -> i32 ---

    // Get the shape ids for all shapes on this body, up to the provided capacity.
    //
    // *returns* the number of shape ids stored in the user array
    @(link_name="b2Body_GetShapes")
    body_get_shapes :: proc(body_id: Body_ID, shape_array: [^]Shape_ID, capacity: i32) -> i32 ---

    // Get the number of joints on this body
    @(link_name="b2Body_GetJointCount")
    body_get_joint_count :: proc(body_id: Body_ID) -> i32 ---

    // Get the joint ids for all joints on this body, up to the provided capacity
    //
    // *returns* the number of joint ids stored in the user array
    @(link_name="b2Body_GetJoints")
    body_get_joints :: proc(body_id: Body_ID, joint_array: [^]Joint_ID, capacity: i32) -> i32 ---

    // Get the maximum capacity required for retrieving all the touching contacts on a body
    @(link_name="b2Body_GetContactCapacity")
    body_get_contact_capacity :: proc(body_id: Body_ID) -> i32 ---

    // Get the touching contact data for a body
    @(link_name="b2Body_GetContactData")
    body_get_contact_data :: proc(body_id: Body_ID, contact_data: ^Contact_Data, capacity: i32) -> i32 ---

    // Get the current world AABB that contains all the attached shapes. Note that this may not emcompass the body origin.
    //
    // If there are no shapes attached then the returned AABB is empty and centered on the body origin.
    @(link_name="b2Body_ComputeAABB")
    body_compute_aabb :: proc(body_id: Body_ID) -> AABB ---

    // Create a circle shape and attach it to a body. The shape definition and geometry are fully cloned.
    //
    // Contacts are not created until the next time step.
    // *return* the shape id for accessing the shape
    @(link_name="b2CreateCircleShape")
    create_circle_shape :: proc(body_id: Body_ID, def: ^Shape_Def, circle: ^Circle) -> Shape_ID ---

    // Create a line segment shape and attach it to a body. The shape definition and geometry are fully cloned.
    //
    // Contacts are not created until the next time step.
    // *return* the shape id for accessing the shape
    @(link_name="b2CreateSegmentShape")
    create_segment_shape :: proc(body_id: Body_ID, def: ^Shape_Def, segment: ^Segment) -> Shape_ID ---

    // Create a capsule shape and attach it to a body. The shape definition and geometry are fully cloned.
    //
    // Contacts are not created until the next time step.
    // *return* the shape id for accessing the shape
    @(link_name="b2CreateCapsuleShape")
    create_capsule_shape :: proc(body_id: Body_ID, def: ^Shape_Def, capsule: ^Capsule) -> Shape_ID ---

    // Create a polygon shape and attach it to a body. The shape definition and geometry are fully cloned.
    //
    // Contacts are not created until the next time step.
    // *return* the shape id for accessing the shape
    @(link_name="b2CreatePolygonShape")
    create_polygon_shape :: proc(body_id: Body_ID, def: ^Shape_Def, polygon: ^Polygon) -> Shape_ID ---

    // Destroy a shape
    @(link_name="b2DestroyShape")
    destroy_shape :: proc(shape_id: Shape_ID) ---

    // Shape identifier validation. Provides validation for up to 64K allocations.
    @(link_name="b2Shape_IsValid")
    shape_is_valid :: proc(shape_id: Shape_ID) -> bool ---

    // Get the type of a shape
    @(link_name="b2Shape_GetType")
    shape_get_type :: proc(shape_id: Shape_ID) -> Shape_Type ---

    // Get the id of the body that a shape is attached to
    @(link_name="b2Shape_GetBody")
    shape_get_body :: proc(shape_id: Shape_ID) -> Body_ID ---

    // *return* ```true``` If the shape is a sensor
    @(link_name="b2Shape_IsSensor")
    shape_is_sensor :: proc(shape_id: Shape_ID) -> bool ---

    // Set the user data for a shape
    @(link_name="b2Shape_SetUserData")
    shape_set_user_data :: proc(shape_id: Shape_ID, user_data: rawptr) ---

    // Get the user data for a shape. This is useful when you get a shape id
    // from an event or query.
    @(link_name="b2Shape_GetUserData")
    shape_get_user_data :: proc(shape_id: Shape_ID) -> rawptr ---

    // Set the mass density of a shape, typically in ```kg/m^2```.
    //
    // This will not update the mass properties on the parent body.
    //
    // **SEE:** ```Shape_Def.density```, ```body_apply_mass_from_shapes```
    @(link_name="b2Shape_SetDensity")
    shape_set_density :: proc(shape_id: Shape_ID, density: f32) ---

    // Get the density of a shape, typically in ```kg/m^2```
    @(link_name="b2Shape_GetDensity")
    shape_get_density :: proc(shape_id: Shape_ID) -> f32 ---

    // Set the friction on a shape
    //
    // **SEE:** ```Shape_Def.friction```
    @(link_name="b2Shape_SetFriction")
    shape_set_friction :: proc(shape_id: Shape_ID, friction: f32) ---

    // Get the friction on a shape.
    @(link_name="b2Shape_GetFriction")
    shape_get_friction :: proc(shape_id: Shape_ID) -> f32 ---

    // Set the shape restitution (bounciness)
    //
    // **SEE:** ```Shape_Def.restitution```
    @(link_name="b2Shape_SetRestitution")
    shape_set_restitution :: proc(shape_id: Shape_ID, restitution: f32) ---

    // Get the shape restitution
    @(link_name="b2Shape_GetRestitution")
    shape_get_restitution :: proc(shape_id: Shape_ID) -> f32 ---

    // Get the shape filter
    @(link_name="b2Shape_GetFilter")
    shape_get_filter :: proc(shape_id: Shape_ID) -> Filter ---

    // Set the current filter. This is almost as expensive as recreating the shape.
    //
    // **SEE:** ```Shape_Def.filter```
    @(link_name="b2Shape_SetFilter")
    shape_set_filter :: proc(shape_id: Shape_ID, filter: Filter) ---

    // Enable sensor events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
    //
    // **SEE:** ```Shape_Def.is_sensor```
    @(link_name="b2Shape_EnableSensorEvents")
    shape_enable_sensor_events :: proc(shape_id: Shape_ID, flag: bool) ---

    // *return* ```true``` if sensor events are enabled
    @(link_name="b2Shape_AreSensorEventsEnabled")
    shape_are_sensor_events_enabled :: proc(shape_id: Shape_ID) -> bool ---

    // Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
    //
    // **SEE:** ```Shape_Def.enable_contact_events```
    @(link_name="b2Shape_EnableContactEvents")
    shape_enable_contact_events :: proc(shape_id: Shape_ID, flag: bool) ---

    // *return* true if contact events are enabled
    @(link_name="b2Shape_AreContactEventsEnabled")
    shape_are_contact_events_enabled :: proc(shape_id: Shape_ID) -> bool ---

    // Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
    // and must be carefully handled due to multithreading. Ignored for sensors.
    //
    // **SEE:** ```Pre_Solve_Fcn```
    @(link_name="b2Shape_EnablePreSolveEvents")
    shape_enable_pre_solve_events :: proc(shape_id: Shape_ID, flag: bool) ---

    // *return* ```true``` if pre-solve events are enabled
    @(link_name="b2Shape_ArePreSolveEventsEnabled")
    shape_are_pre_solve_events_enabled :: proc(shape_id: Shape_ID) -> bool ---

    // Enable contact hit events for this shape. Ignored for sensors.
    //
    // **SEE:** ```World_Def.hit_event_threshold```
    @(link_name="b2Shape_EnableHitEvents")
    shape_enable_hit_events :: proc(shape_id: Shape_ID, flag: bool) ---

    // *return* ```true``` if hit events are enabled
    @(link_name="b2Shape_AreHitEventsEnabled")
    shape_are_hit_events_enabled :: proc(shape_id: Shape_ID) -> bool ---

    // Test a point for overlap with a shape
    @(link_name="b2Shape_TestPoint")
    shape_test_point :: proc(shape_id: Shape_ID, point: Vec2) -> bool ---

    //Ray cast a shape directly
    @(link_name="b2Shape_RayCast")
    shape_ray_cast :: proc(shape_id: Shape_ID, origin, translation: Vec2) -> Cast_Output ---

    // Get a copy of the shape's circle. Asserts the type is correct.
    @(link_name="b2Shape_GetCircle")
    shape_get_circle :: proc(shape_id: Shape_ID) -> Circle ---

    // Get a copy of the shape's line segment. Asserts the type is correct.
    @(link_name="b2Shape_GetSegment")
    shape_get_segment :: proc(shape_id: Shape_ID) -> Segment ---

    // Get a copy of the shape's smooth line segment. These come from chain shapes.
    //
    // Asserts the type is correct.
    @(link_name="b2Shape_GetSmoothSegment")
    shape_get_smooth_segment :: proc(shape_id: Shape_ID) -> Smooth_Segment ---

    // Get a copy of the shape's capsule. Asserts the type is correct.
    @(link_name="b2Shape_GetCapsule")
    shape_get_capsule :: proc(shape_id: Shape_ID) -> Capsule ---

    // Get a copy of the shape's convex polygon. Asserts the type is correct.
    @(link_name="b2Shape_GetPolygon")
    shape_get_polygon :: proc(shape_id: Shape_ID) -> Polygon ---

    // Allows you to change a shape to be a circle or update the current circle.
    //
    // This does not modify the mass properties.
    //
    // **SEE:** ```body_apply_mass_from_shapes```
    @(link_name="b2Shape_SetCircle")
    shape_set_circle :: proc(shape_id: Shape_ID, circle: ^Circle) ---

    // Allows you to change a shape to be a capsule or update the current capsule.
    //
    // This does not modify the mass properties.
    //
    // **SEE:** ```body_apply_mass_from_shapes```
    @(link_name="b2Shape_SetCapsule")
    shape_set_capsule :: proc(shape_id: Shape_ID, capsule: ^Capsule) ---

    // Allows you to change a shape to be a segment or update the current segment.
    @(link_name="b2Shape_SetSegment")
    shape_set_segment :: proc(shape_id: Shape_ID, segment: ^Segment) ---

    // Allows you to change a shape to be a polygon or update the current polygon.
    //
    // This does not modify the mass properties.
    //
    // **SEE:** ```body_apply_mass_from_shapes```
    @(link_name="b2Shape_SetPolygon")
    shape_set_polygon :: proc(shape_id: Shape_ID, polygon: ^Polygon) ---

    // Get the parent chain id if the shape type is b2_smoothSegmentShape, otherwise
    // return ```NULL_CHAIN_ID```.
    @(link_name="b2Shape_GetParentChain")
    shape_get_parent_chain :: proc(shape_id: Shape_ID) -> Chain_ID ---

    // Get the maximum capacity required for retrieving all the touching contacts on a shape
    @(link_name="b2Shape_GetContactCapacity")
    shape_get_contact_capacity :: proc(shape_id: Shape_ID) -> i32 ---

    // Get the touching contact data for a shape. The provided ```shape id``` will be either ```shape id a``` or ```shape id b``` on the contact data.
    @(link_name="b2Shape_GetContactData")
    shape_get_contact_data :: proc(shape_id: Shape_ID, contact_data: [^]Contact_Data, capacity: i32) -> i32 ---

    // Get the current world AABB
    @(link_name="b2Shape_GetAABB")
    shape_get_aabb :: proc(shape_id: Shape_ID) -> AABB ---

    // Get the closest point on a shape to a target point. Target and result are in world space.
    @(link_name="b2Shape_GetClosestPoint")
    shape_get_closest_point :: proc(shape_id: Shape_ID, target: Vec2) -> Vec2 ---

    // Create a chain shape
    //
    // **SEE:** ```Chain_Def``` for details
    @(link_name="b2CreateChain")
    create_chain :: proc(body_id: Body_ID, def: ^Chain_Def) -> Chain_ID ---

    // Destroy a chain shape
    @(link_name="b2DestroyChain")
    destroy_chain :: proc(chain_id: Chain_ID) ---

    // Set the chain friction
    // **SEE:** ```Chain_Def::friction```
    @(link_name="b2Chain_SetFriction")
    chain_set_friction :: proc(chain_id: Chain_ID, friction: f32) ---

    // Set the chain restitution (bounciness)
    //
    // **SEE:** ```Chain_Def::restitution```
    @(link_name="b2Chain_SetFriction")
    chain_set_restitution :: proc(chain_id: Chain_ID, restitution: f32) ---

    // Chain identifier validation. Provides validation for up to 64K allocations.
    @(link_name="b2Chain_IsValid")
    chain_is_valid :: proc(chain_id: Chain_ID) -> bool ---

    // Destroy a joint
    @(link_name="b2DestroyJoint")
    destroy_joint :: proc(joint_id: Joint_ID) ---

    // Joint identifier validation. Provides validation for up to 64K allocations.
    @(link_name="b2Joint_IsValid")
    joint_is_valid :: proc (joint_id: Joint_ID) -> bool ---

    // Get the joint type
    @(link_name="b2Joint_GetType")
    joint_get_type :: proc(joint_id: Joint_ID) -> Joint_Type ---

    // Get body A on a joint
    @(link_name="b2Joint_GetBodyA")
    joint_get_body_a :: proc(joint_id: Joint_ID) -> Body_ID ---

    // Get body B on a joint
    @(link_name="b2Joint_GetBodyB")
    joint_get_body_b :: proc(joint_id: Joint_ID) -> Body_ID ---

    // Get local anchor on body A
    @(link_name="b2Joint_GetLocalAnchorA")
    joint_get_local_anchor_a :: proc(joint_id: Joint_ID) -> Vec2 ---

    // Get local anchor on body B
    @(link_name="b2Joint_GetLocalAnchorB")
    joint_get_local_anchor_b :: proc(joint_id: Joint_ID) -> Vec2 ---

    // Toggle collision between connected bodies
    @(link_name="b2Joint_SetCollideConnected")
    joint_set_collide_connected :: proc(joint_id: Joint_ID, should_collide: bool) ---

    // Is collision allowed between connected bodies?
    @(link_name="b2Joint_GetCollideConnected")
    joint_get_collide_connected :: proc(joint_id: Joint_ID) -> bool ---

    // Set the user data on a joint
    @(link_name="b2Joint_SetUserData")
    joint_set_user_data :: proc(joint_id: Joint_ID, user_data: rawptr) ---

    // Get the user data on a joint
    @(link_name="b2Joint_GetUserData")
    joint_get_user_data :: proc(joint_id: Joint_ID) -> rawptr ---

    // Wake the bodies connect to this joint
    @(link_name="b2Joint_WakeBodies")
    joint_wake_bodies :: proc(joint_id: Joint_ID) ---

    // Get the current constraint force for this joint
    @(link_name="b2Joint_GetConstraintForce")
    joint_get_constraint_force :: proc(joint_id: Joint_ID) -> Vec2 ---

    // Get the current constraint torque for this joint
    @(link_name="b2Joint_GetConstraintTorque")
    joint_get_constraint_torque :: proc(joint_id: Joint_ID) -> f32 ---

    // Create a distance joint
    //
    // **SEE:** ```Distance_Joint_Def``` for details
    @(link_name="b2CreateDistanceJoint")
    create_distance_joint :: proc(world_id: World_ID, def: ^Distance_Joint_Def) -> Joint_ID ---

    // Set the rest length of a distance joint
    // * ```joint_id``` The id for a distance joint
    // * ```length``` The new distance joint length
    @(link_name="b2DistanceJoint_SetLength")
    distance_joint_set_length :: proc(joint_id: Joint_ID, length, min_length, max_length: f32) ---

    // Get the rest length of a distance joint
    @(link_name="b2DistanceJoint_GetLength")
    distance_joint_get_length :: proc(joint_id: Joint_ID) -> f32 ---

    // Enable/disable the distance joint spring. When disabled the distance joint is rigid.
    @(link_name="b2DistanceJoint_EnableSpring")
    distance_joint_enable_spring :: proc(joint_id: Joint_ID, enable_spring: bool) ---

    // Is the distance joint spring enabled?
    @(link_name="b2DistanceJoint_IsSpringEnabled")
    distance_joint_is_spring_enabled :: proc(joint_id: Joint_ID) -> bool ---

    // Set the spring stiffness in Hertz
    @(link_name="b2DistanceJoint_SetSpringHertz")
    distance_joint_set_spring_hertz :: proc(joint_id: Joint_ID, hertz: f32) ---

    // Set the spring damping ratio, non-dimensional
    @(link_name="b2DistanceJoint_SetSpringDampingRatio")
    distance_joint_set_spring_damping_ratio :: proc(joint_id: Joint_ID, damping_ratio: f32) ---

    // Get the spring Hertz
    @(link_name="b2DistanceJoint_GetHertz")
    distance_joint_get_hertz :: proc(joint_id: Joint_ID) -> f32 ---

    // Get the spring damping ratio
    @(link_name="b2DistanceJoint_GetDampingRatio")
    distance_joint_get_damping_ratio :: proc(joint_id: Joint_ID) -> f32 ---

    // Enable joint limit. The limit only works if the joint spring is enabled. Otherwise the joint is rigid
    // and the limit has no effect.
    @(link_name="b2DistanceJoint_EnableLimit")
    distance_joint_enable_limit :: proc(joint_id: Joint_ID, enable_limit: bool) ---

    // Is the distance joint limit enabled?
    @(link_name="b2DistanceJoint_IsLimitEnabled")
    distance_joint_is_limit_enabled :: proc(joint_id: Joint_ID) -> bool ---

    // Set the minimum and maximum length parameters of a distance joint
    @(link_name="b2DistanceJoint_SetLengthRange")
    distance_joint_set_length_range :: proc(joint_id: Joint_ID, min_length, max_length: f32) ---

    /// Get the minimum distance joint length
    @(link_name="b2DistanceJoint_GetMinLength")
    distance_joint_get_min_length :: proc(joint_id: Joint_ID) -> f32 ---

    /// Get the maximum distance joint length
    @(link_name="b2DistanceJoint_GetMaxLength")
    distance_joint_get_max_length :: proc(joint_id: Joint_ID) -> f32 ---

    // Get the current length of a distance joint
    @(link_name="b2DistanceJoint_GetCurrentLength")
    distance_joint_get_current_length :: proc(joint_id: Joint_ID) -> f32 ---

    // Enable/disable the distance joint motor
    @(link_name="b2DistanceJoint_EnableMotor")
    distance_joint_enable_motor :: proc(joint_id: Joint_ID, enable_motor: f32) ---

    // Is the distance joint motor enabled?
    @(link_name="b2DistanceJoint_IsMotorEnabled")
    distance_joint_is_motor_enabled :: proc(joint_id: Joint_ID) -> bool ---

    // Set the distance joint motor speed, typically in meters per second
    @(link_name="b2DistanceJoint_SetMotorSpeed")
    distance_joint_set_motor_speed :: proc(joint_id: Joint_ID, motor_speed: f32) ---

    // Get the distance joint motor speed, typically in meters per second
    @(link_name="b2DistanceJoint_GetMotorSpeed")
    distance_joint_get_motor_speed :: proc(joint_id: Joint_ID) -> f32 ---

    // Set the distance joint maximum motor force, typically in newtons
    @(link_name="b2DistanceJoint_SetMaxMotorForce")
    distance_joint_set_max_motor_force :: proc(joint_id: Joint_ID, force: f32) ---

    // Get the distance joint maximum motor force, typically in newtons
    @(link_name="b2DistanceJoint_GetMaxMotorForce")
    distance_joint_get_max_motor_force :: proc(joint_id: Joint_ID) -> f32 ---

    // Get the distance joint current motor force, typically in newtons
    @(link_name="b2DistanceJoint_GetMotorForce")
    distance_joint_get_motor_force :: proc(joint_id: Joint_ID) -> f32 ---

    // Create a motor joint
    //
    // **SEE:** ```MotorJoint_Def``` for details
    @(link_name="b2CreateMotorJoint")
    create_motor_joint :: proc(world_id: World_ID, def: ^Motor_Joint_Def) -> Joint_ID ---

    // Set the motor joint linear offset target
    @(link_name="b2MotorJoint_SetLinearOffset")
    motor_joint_set_linear_offset :: proc(joint_id: Joint_ID, linear_offset: Vec2) ---

    // Get the motor joint linear offset target
    @(link_name="b2MotorJoint_GetLinearOffset")
    motor_joint_get_linear_offset :: proc(joint_id: Joint_ID) -> Vec2 ---

    // Set the motor joint angular offset target in radians
    @(link_name="b2MotorJoint_SetAngularOffset")
    motor_joint_set_angular_offset :: proc(joint_id: Joint_ID, angular_offset: f32) ---

    // Get the motor joint angular offset target in radians
    @(link_name="b2MotorJoint_GetAngularOffset")
    motor_joint_get_angular_offset :: proc(joint_id: Joint_ID) -> f32 ---

    // Set the motor joint maximum force, typically in newtons
    @(link_name="b2MotorJoint_SetMaxForce")
    motor_joint_set_max_force :: proc(joint_id: Joint_ID, max_force: f32) ---

    // Get the motor joint maximum force, typically in newtons
    @(link_name="b2MotorJoint_GetMaxForce")
    motor_joint_get_max_force :: proc(joint_id: Joint_ID) -> f32 ---

    // Set the motor joint maximum torque, typically in newton-meters
    @(link_name="b2MotorJoint_SetMaxTorque")
    motor_joint_set_max_torque :: proc(joint_id: Joint_ID, max_torque: f32) ---

    // Get the motor joint maximum torque, typically in newton-meters
    @(link_name="b2MotorJoint_GetMaxTorque")
    motor_joint_get_max_torque :: proc(joint_id: Joint_ID) -> f32 ---

    // Set the motor joint correction factor, typically in ```[0, 1]```
    @(link_name="b2MotorJoint_SetCorrectionFactor")
    motor_joint_set_correction_factor :: proc(joint_id: Joint_ID, correction_factor: f32) ---

    // Get the motor joint correction factor, typically in ```[0, 1]```
    @(link_name="b2MotorJoint_GetCorrectionFactor")
    motor_joint_get_correction_factor :: proc(joint_id: Joint_ID) -> f32 ---

    // Create a mouse joint
    //
    // **SEE:** ```Mouse_Joint_Def``` for details
    @(link_name="b2CreateMouseJoint")
    create_mouse_joint :: proc(world_id: World_ID, def: ^Mouse_Joint_Def) -> Joint_ID ---

    // Set the mouse joint target
    @(link_name="b2MouseJoint_SetTarget")
    mouse_joint_set_target :: proc(joint_id: Joint_ID, target: Vec2) ---

    // Get the mouse joint target
    @(link_name="b2MouseJoint_GetTarget")
    mouse_joint_get_target :: proc(joint_id: Joint_ID) -> Vec2 ---

    // Set the mouse joint spring stiffness in Hertz
    @(link_name="b2MouseJoint_SetSpringHertz")
    mouse_joint_set_spring_hertz :: proc(joint_id: Joint_ID, hertz: f32) ---

    // Get the mouse joint spring stiffness in Hertz
    @(link_name="b2MouseJoint_GetSpringHertz")
    mouse_joint_get_spring_hertz :: proc(joint_id: Joint_ID) -> f32 ---

    // Set the mouse joint spring damping ratio, non-dimensional
    @(link_name="b2MouseJoint_SetSpringDampingRatio")
    mouse_joint_set_spring_damping_ratio :: proc(joint_id: Joint_ID, damping_ratio: f32) ---

    // Get the mouse joint damping ratio, non-dimensional
    @(link_name="b2MouseJoint_GetSpringDampingRatio")
    mouse_joint_get_spring_damping_ratio :: proc(joint_id: Joint_ID) -> f32 ---

    // Set the mouse joint maximum force, typically in newtons
    @(link_name="b2MouseJoint_SetMaxForce")
    mouse_joint_set_max_force :: proc(joint_id: Joint_ID, max_force: f32) ---

    // Get the mouse joint maximum force, typically in newtons
    @(link_name="b2MouseJoint_GetMaxForce")
    mouse_joint_get_max_force :: proc(joint_id: Joint_ID) -> f32 ---

    // Create a prismatic (slider) joint.
    // **SEE:** ```Prismatic_Joint_Def``` for details
    @(link_name="b2CreatePrismaticJoint")
    create_prismatic_joint :: proc(world_id: World_ID, def: ^Prismatic_Joint_Def) -> Joint_ID ---

    // Enable/disable the joint spring.
    @(link_name="b2PrismaticJoint_EnableSpring")
    prismatic_joint_enable_spring :: proc(joint_id: Joint_ID, enable_spring: bool) ---

    // Is the prismatic joint spring enabled or not?
    @(link_name="b2PrismaticJoint_IsSpringEnabled")
    prismatic_joint_is_spring_enabled :: proc(joint_id: Joint_ID) -> bool ---

    // Set the prismatic joint stiffness in Hertz.
    //
    // This should usually be less than a quarter of the simulation rate. For example, if the simulation
    // runs at 60Hz then the joint stiffness should be 15Hz or less.
    @(link_name="b2PrismaticJoint_SetSpringHertz")
    prismatic_joint_set_spring_hertz :: proc(joint_id: Joint_ID, hertz: f32) ---

    // Get the prismatic joint stiffness in Hertz
    @(link_name="b2PrismaticJoint_GetSpringHertz")
    prismatic_joint_get_spring_hertz :: proc(joint_id: Joint_ID) -> f32 ---

    // Set the prismatic joint damping ratio (non-dimensional)
    @(link_name="b2PrismaticJoint_SetSpringDampingRatio")
    prismatic_joint_set_spring_damping_ratio :: proc(joint_id: Joint_ID, damping_ratio: f32) ---

    // Get the prismatic spring damping ratio (non-dimensional)
    @(link_name="b2PrismaticJoint_GetSpringDampingRatio")
    prismatic_joint_get_spring_damping_ratio :: proc(joint_id: Joint_ID) -> f32 ---

    // Enable/disable a prismatic joint limit
    @(link_name="b2PrismaticJoint_EnableLimit")
    prismatic_joint_enable_limit :: proc(joint_id: Joint_ID, enable_limit: bool) ---

    // Is the prismatic joint limit enabled?
    @(link_name="b2PrismaticJoint_IsLimitEnabled")
    prismatic_joint_is_limit_enabled :: proc(joint_id: Joint_ID) -> bool ---

    // Get the prismatic joint lower limit
    @(link_name="b2PrismaticJoint_GetLowerLimit")
    prismatic_joint_get_lower_limit :: proc(joint_id: Joint_ID) -> f32 ---

    // Get the prismatic joint upper limit
    @(link_name="b2PrismaticJoint_GetUpperLimit")
    prismatic_joint_get_upper_limit :: proc(joint_id: Joint_ID) -> f32 ---

    // Set the prismatic joint limits
    @(link_name="b2PrismaticJoint_SetLimits")
    prismatic_joint_set_limits :: proc(joint_id: Joint_ID, lower, upper: f32) ---

    // Enable/disable a prismatic joint motor
    @(link_name="b2PrismaticJoint_EnableMotor")
    prismatic_joint_enable_motor :: proc(joint_id: Joint_ID, enable_motor: bool) ---

    // Is the prismatic joint motor enabled?
    @(link_name="b2PrismaticJoint_IsMotorEnabled")
    prismatic_joint_is_motor_enabled :: proc(joint_id: Joint_ID) -> bool ---

    // Set the prismatic joint motor speed, typically in meters per second
    @(link_name="b2PrismaticJoint_SetMotorSpeed")
    prismatic_joint_set_motor_speed :: proc(joint_id: Joint_ID, motor_speed: f32) ---

    // Get the prismatic joint motor speed, typically in meters per second
    @(link_name="b2PrismaticJoint_GetMotorSpeed")
    prismatic_joint_get_motor_speed :: proc(joint_id: Joint_ID) -> f32 ---

    // Set the prismatic joint maximum motor force, typically in newtons
    @(link_name="b2PrismaticJoint_SetMaxMotorForce")
    prismatic_joint_set_max_motor_force :: proc(joint_id: Joint_ID, force: f32) ---

    // Get the prismatic joint maximum motor force, typically in newtons
    @(link_name="b2PrismaticJoint_GetMaxMotorForce")
    prismatic_joint_get_max_motor_force :: proc(joint_id: Joint_ID) -> f32 ---

    // Get the prismatic joint current motor force, typically in newtons
    @(link_name="b2PrismaticJoint_GetMotorForce")
    prismatic_joint_get_motor_force :: proc(joint_id: Joint_ID) -> f32 ---

    // Create a revolute joint
    // **SEE:** ```Revolute_Joint_Def``` for details
    @(link_name="b2CreateRevoluteJoint")
    create_revolute_joint :: proc(world_id: World_ID, def: ^Revolute_Joint_Def) -> Joint_ID ---

    // Enable/disable the revolute joint spring
    @(link_name="b2RevoluteJoint_EnableSpring")
    revolute_joint_enable_spring :: proc(joint_id: Joint_ID, enable_spring: bool) ---

    // Is the revolute joint limit enabled?
    @(link_name="b2RevoluteJoint_IsLimitEnabled")
    revolute_joint_is_limit_enabled :: proc(joint_id: Joint_ID) -> bool ---

    // Set the revolute joint spring stiffness in Hertz
    @(link_name="b2RevoluteJoint_SetSpringHertz")
    revolute_joint_set_spring_hertz :: proc(joint_id: Joint_ID, hertz: f32) ---

    // Get the revolute joint spring stiffness in Hertz
    @(link_name="b2RevoluteJoint_GetSpringHertz")
    revolute_joint_get_spring_hertz :: proc(joint_id: Joint_ID) -> f32 ---

    // Set the revolute joint spring damping ratio, non-dimensional
    @(link_name="b2RevoluteJoint_SetSpringDampingRatio")
    revolute_joint_set_spring_damping_ratio :: proc(joint_id: Joint_ID, damping_ratio: f32) ---

    // Get the revolute joint spring damping ratio, non-dimensional
    @(link_name="b2RevoluteJoint_GetSpringDampingRatio")
    revolute_joint_get_spring_damping_ratio :: proc(joint_id: Joint_ID) -> f32 ---

    // Get the revolute joint current angle in radians relative to the reference angle
    //
    // **SEE:** ```Revolute_Joint_Def.reference_angle```
    @(link_name="b2RevoluteJoint_GetAngle")
    revolute_joint_get_angle :: proc(joint_id: Joint_ID) -> f32 ---

    // Enable/disable the revolute joint limit
    @(link_name="b2RevoluteJoint_EnableLimit")
    revolute_joint_enable_limit :: proc(joint_id: Joint_ID, enable_limit: bool) ---

    // Get the revolute joint lower limit in radians
    @(link_name="b2RevoluteJoint_GetLowerLimit")
    revolute_joint_get_lower_limit :: proc(joint_id: Joint_ID) -> f32 ---

    // Get the revolute joint upper limit in radians
    @(link_name="b2RevoluteJoint_GetUpperLimit")
    revolute_joint_get_upper_limit :: proc(joint_id: Joint_ID) -> f32 ---

    // Set the revolute joint limits in radians
    @(link_name="b2RevoluteJoint_SetLimits")
    revolute_joint_set_limits :: proc(joint_id: Joint_ID, lower, upper: f32) ---

    // Enable/disable a revolute joint motor
    @(link_name="b2RevoluteJoint_EnableMotor")
    revolute_joint_enable_motor :: proc(joint_id: Joint_ID, enable_motor: bool) ---

    // Is the revolute joint motor enabled?
    @(link_name="b2RevoluteJoint_IsMotorEnabled")
    revolute_joint_is_motor_enabled :: proc(joint_id: Joint_ID) -> bool ---

    // Set the revolute joint motor speed in radians per second
    @(link_name="b2RevoluteJoint_SetMotorSpeed")
    revolute_joint_set_motor_speed :: proc(joint_id: Joint_ID, motor_speed: f32) ---

    // Get the revolute joint motor speed in radians per second
    @(link_name="b2RevoluteJoint_GetMotorSpeed")
    revolute_joint_get_motor_speed :: proc(joint_id: Joint_ID) -> f32 ---

    // Get the revolute joint current motor torque, typically in newton-meters
    @(link_name="b2RevoluteJoint_GetMotorTorque")
    revolute_joint_get_motor_torque :: proc(joint_id: Joint_ID) -> f32 ---

    // Set the revolute joint maximum motor torque, typically in newton-meters
    @(link_name="b2RevoluteJoint_SetMaxMotorTorque")
    revolute_joint_set_max_motor_torque :: proc(joint_id: Joint_ID, torque: f32) ---

    // Get the revolute joint maximum motor torque, typically in newton-meters
    @(link_name="b2RevoluteJoint_GetMaxMotorTorque")
    revolute_joint_get_max_motor_torque :: proc(joint_id: Joint_ID) -> f32 ---

    // Create a weld joint
    //
    // **SEE:** ```Weld_Joint_Def``` for details
    @(link_name="b2CreateWeldJoint")
    create_weld_joint :: proc(world_id: World_ID, def: ^Weld_Joint_Def) -> Joint_ID ---

    // Set weld joint linear stiffness in Hertz. ```0``` is rigid.
    @(link_name="b2WeldJoint_SetLinearHertz")
    weld_joint_set_linear_hertz :: proc(joint_id: Joint_ID, hertz: f32) ---

    // Get the weld joint linear stiffness in Hertz
    @(link_name="b2WeldJoint_GetLinearHertz")
    weld_joint_get_linear_hertz :: proc(joint_id: Joint_ID) -> f32 ---

    // Set weld joint linear damping ratio (non-dimensional)
    @(link_name="b2WeldJoint_SetLinearDampingRatio")
    weld_joint_set_linear_damping_ratio :: proc(joint_id: Joint_ID, damping_ratio: f32) ---

    // Get the weld joint linear damping ratio (non-dimensional)
    @(link_name="b2WeldJoint_GetLinearDampingRatio")
    weld_joint_get_linear_damping_ratio :: proc(joint_id: Joint_ID) -> f32 ---

    // Set the weld joint angular stiffness in Hertz. ```0``` is rigid.
    @(link_name="b2WeldJoint_SetAngularHertz")
    weld_joint_set_angular_hertz :: proc(joint_id: Joint_ID, hertz: f32) ---

    // Get the weld joint angular stiffness in Hertz
    @(link_name="b2WeldJoint_GetAngularHertz")
    weld_joint_get_angular_hertz :: proc(joint_id: Joint_ID) -> f32 ---

    // Set weld joint angular damping ratio, non-dimensional
    @(link_name="b2WeldJoint_SetAngularDampingRatio")
    weld_joint_set_angular_damping_ratio :: proc(joint_id: Joint_ID, damping_ratio: f32) ---

    // Get the weld joint angular damping ratio, non-dimensional
    @(link_name="b2WeldJoint_GetAngularDampingRatio")
    weld_joint_get_angular_damping_ratio :: proc(joint_id: Joint_ID) -> f32 ---

    // Create a wheel joint
    //
    // **SEE:** ```Wheel_Joint_Def``` for details
    @(link_name="b2CreateWheelJoint")
    create_wheel_joint :: proc(world_id: World_ID, def: ^Wheel_Joint_Def) -> Joint_ID ---

    // Enable/disable the wheel joint spring
    @(link_name="b2WheelJoint_EnableSpring")
    wheel_joint_enable_spring :: proc(joint_id: Joint_ID, enable_spring: bool) ---

    // Is the wheel joint spring enabled?
    @(link_name="b2WheelJoint_IsSpringEnabled")
    wheel_joint_is_spring_enabled :: proc(joint_id: Joint_ID) -> bool ---

    // Enable/disable the wheel joint spring
    @(link_name="b2WheelJoint_SetSpringHertz")
    wheel_joint_set_spring_hertz :: proc(joint_id: Joint_ID, hertz: f32) ---

    // Get the wheel joint stiffness in Hertz
    @(link_name="b2WheelJoint_GetSpringHertz")
    wheel_joint_get_spring_hertz :: proc(joint_id: Joint_ID) -> f32 ---

    // Set the wheel joint damping ratio, non-dimensional
    @(link_name="b2WheelJoint_SetSpringDampingRatio")
    wheel_joint_set_spring_damping_ratio :: proc(joint_id: Joint_ID, damping_ratio: f32) ---

    // Get the wheel joint damping ratio, non-dimensional
    @(link_name="b2WheelJoint_GetSpringDampingRatio")
    wheel_joint_get_spring_damping_ratio :: proc(joint_id: Joint_ID) -> f32 ---

    // Enable/disable the wheel joint limit
    @(link_name="b2WheelJoint_EnableLimit")
    wheel_joint_enable_limit :: proc(joint_id: Joint_ID, enable_limit: f32) ---

    // Is the wheel joint limit enabled?
    @(link_name="b2WheelJoint_IsLimitEnabled")
    wheel_joint_is_limit_enabled :: proc(joint_id: Joint_ID) -> bool ---

    // Get the wheel joint lower limit
    @(link_name="b2WheelJoint_GetLowerLimit")
    wheel_joint_get_lower_limit :: proc(joint_id: Joint_ID) -> f32 ---

    // Get the wheel joint upper limit
    @(link_name="b2WheelJoint_GetUpperLimit")
    wheel_joint_get_upper_limit :: proc(joint_id: Joint_ID) -> f32 ---

    // Set the wheel joint limits
    @(link_name="b2WheelJoint_SetLimits")
    wheel_joint_set_limits :: proc(joint_id: Joint_ID, lower, upper: f32) ---

    // Enable/disable the wheel joint motor
    @(link_name="b2WheelJoint_EnableMotor")
    wheel_joint_enable_motor :: proc(joint_id: Joint_ID, enable_motor: bool) ---

    // Is the wheel joint motor enabled?
    @(link_name="b2WheelJoint_IsMotorEnabled")
    wheel_joint_is_motor_enabled :: proc(joint_id: Joint_ID) -> bool ---

    // Set the wheel joint motor speed in radians per second
    @(link_name="b2WheelJoint_SetMotorSpeed")
    wheel_joint_set_motor_speed :: proc(joint_id: Joint_ID, motor_speed: f32) ---

    // Get the wheel joint motor speed in radians per second
    @(link_name="b2WheelJoint_GetMotorSpeed")
    wheel_joint_get_motor_speed :: proc(joint_id: Joint_ID) -> f32 ---

    // Set the wheel joint maximum motor torque, typically in newton-meters
    @(link_name="b2WheelJoint_SetMaxMotorTorque")
    wheel_joint_set_max_motor_torque :: proc(joint_id: Joint_ID, torque: f32) ---

    // Get the wheel joint maximum motor torque, typically in newton-meters
    @(link_name="b2WheelJoint_GetMaxMotorTorque")
    wheel_joint_get_max_motor_torque :: proc(joint_id: Joint_ID) -> f32 ---

    // Get the wheel joint current motor torque, typically in newton-meters
    @(link_name="b2WheelJoint_GetMotorTorque")
    wheel_joint_get_motor_torque :: proc(joint_id: Joint_ID) -> f32 ---


    /* joint_util.h */


    // Utility to compute linear stiffness values from frequency and damping ratio
    @(link_name="b2LinearStiffness")
    linear_stiffness :: proc(stiffness, damping: ^f32, frequency_hertz, damping_ratio: f32, body_a, body_b: Body_ID) ---

    // Utility to compute rotational stiffness values frequency and damping ratio
    @(link_name="b2AngularStiffness")
    angular_stiffness :: proc(stiffness, damping: ^f32, frequency_hertz, damping_ratio: f32, body_a, body_b: Body_ID) ---


    /* collision.h */


    // Validate ray cast input data (NaN, etc)
    @(link_name="b2IsValidRay")
    is_valid_ray :: proc(input: ^Ray_Cast_Input) -> bool ---

    // Make a convex polygon from a convex hull. This will assert if the hull is not valid.
    @(link_name="b2MakePolygon")
    make_polygon :: proc(hull: ^Hull, radius: f32) -> Polygon ---

    // Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
    @(link_name="b2MakeOffsetPolygon")
    make_offset_polygon :: proc(hull: ^Hull, radius: f32, transform: Transform) -> Polygon ---

    // Make a square polygon, bypassing the need for a convex hull.
    @(link_name="b2MakeSquare")
    make_square :: proc(h: f32) -> Polygon ---

    // Make a box (rectangle) polygon, bypassing the need for a convex hull.
    @(link_name="b2MakeBox")
    make_box :: proc(hx, hy: f32) -> Polygon ---

    // Make a rounded box, bypassing the need for a convex hull.
    @(link_name="b2MakeRoundedBox")
    make_rounded_box :: proc(hx, hy, radius: f32) -> Polygon ---

    // Make an offset box, bypassing the need for a convex hull.
    @(link_name="b2MakeOffsetBox")
    make_offset_box :: proc(hx, hy: f32, center: Vec2, angle: f32) -> Polygon ---

    // Transform a polygon. This is useful for transfering a shape from one body to another.
    @(link_name="b2TransformPolygon")
    transform_polygon :: proc(transform: Transform, polygon: ^Polygon) -> Polygon ---

    // Compute mass properties of a circle
    @(link_name="b2ComputeCircleMass")
    compute_circle_mass :: proc(shape: ^Circle, density: f32) -> Mass_Data ---

    // Compute mass properties of a capsule
    @(link_name="b2ComputeCapsuleMass")
    compute_capsule_mass :: proc(shape: ^Capsule, density: f32) -> Mass_Data ---

    // Compute mass properties of a polygon
    @(link_name="b2ComputePolygonMass")
    compute_polygon_mass :: proc(shape: ^Polygon, density: f32) -> Mass_Data ---

    // Compute the bounding box of a transformed circle
    @(link_name="b2ComputeCircleAABB")
    compute_circle_aabb :: proc(shape: ^Circle, transform: Transform) -> AABB ---

    // Compute the bounding box of a transformed capsule
    @(link_name="b2ComputeCapsuleAABB")
    compute_capsule_aabb :: proc(shape: ^Capsule, transform: Transform) -> AABB ---

    // Compute the bounding box of a transformed polygon
    @(link_name="b2ComputePolygonAABB")
    compute_polygon_aabb :: proc(shape: ^Polygon, transform: Transform) -> AABB ---

    // Compute the bounding box of a transformed line segment
    @(link_name="b2ComputeSegmentAABB")
    compute_segment_aabb :: proc(shape: ^Segment, transform: Transform) -> AABB ---

    // Test a point for overlap with a circle in local space
    @(link_name="b2PointInCircle")
    point_in_circle :: proc(point: Vec2, shape: ^Circle) -> bool ---

    // Test a point for overlap with a capsule in local space
    @(link_name="b2PointInCapsule")
    point_in_capsule :: proc(point: Vec2, shape: ^Capsule) -> bool ---

    // Test a point for overlap with a convex polygon in local space
    @(link_name="b2PointInPolygon")
    point_in_polygon :: proc(point: Vec2, shape: ^Polygon) -> bool ---

    // Ray cast versus circle in shape local space. Initial overlap is treated as a miss.
    @(link_name="b2RayCastCircle")
    ray_cast_circle :: proc(input: ^Ray_Cast_Input, shape: ^Circle) -> Cast_Output ---

    // Ray cast versus capsule in shape local space. Initial overlap is treated as a miss.
    @(link_name="b2RayCastCapsule")
    ray_cast_capsule :: proc(input: ^Ray_Cast_Input, shape: ^Capsule) -> Cast_Output ---

    // Ray cast versus segment in shape local space. Optionally treat the segment as one-sided with hits from
    // the left side being treated as a miss.
    @(link_name="b2RayCastSegment")
    ray_cast_segment :: proc(input: ^Ray_Cast_Input, shape: ^Segment) -> Cast_Output ---

    // Ray cast versus polygon in shape local space. Initial overlap is treated as a miss.
    @(link_name="b2RayCastPolygon")
    ray_cast_polygon :: proc(input: ^Ray_Cast_Input, shape: ^Polygon) -> Cast_Output ---

    // Shape cast versus a circle. Initial overlap is treated as a miss.
    @(link_name="b2ShapeCastCircle")
    shape_cast_circle :: proc(input: ^Shape_Cast_Input, shape: ^Circle) -> Cast_Output ---

    // Shape cast versus a capsule. Initial overlap is treated as a miss.
    @(link_name="b2ShapeCastCapsule")
    shape_cast_capsule :: proc(input: ^Shape_Cast_Input, shape: ^Capsule) -> Cast_Output ---

    // Shape cast versus a line segment. Initial overlap is treated as a miss.
    @(link_name="b2ShapeCastSegment")
    shape_cast_segment :: proc(input: ^Shape_Cast_Input, shape: ^Segment) -> Cast_Output ---

    // Shape cast versus a convex polygon. Initial overlap is treated as a miss.
    @(link_name="b2ShapeCastPolygon")
    shape_cast_polygon :: proc(input: ^Shape_Cast_Input, shape: ^Polygon) -> Cast_Output ---

    // Compute the convex hull of a set of points. Returns an empty hull if it fails.
    //
    // Some failure cases:
    // * all points very close together
    // * all points on a line
    // * less than 3 points
    // * more than ```MAX_POLYGON_VERTICES``` points
    //
    // This welds close points and removes collinear points.
    @(link_name="b2ComputeHull")
    compute_hull :: proc(points: [^]Vec2, count: i32) -> Hull ---

    // This determines if a hull is valid. Checks for:
    // * convexity
    // * collinear points
    //
    // This is expensive and should not be called at runtime.
    @(link_name="b2ValidateHull")
    validate_hull :: proc(hull: ^Hull) -> bool ---

    // Compute the distance between two line segments, clamping at the end points if needed.
    @(link_name="b2SegmentDistance")
    segment_distance :: proc(p1, q1, p2, q2: Vec2) -> Segment_Distance_Result ---

    // Compute the closest points between two shapes. Supports any combination of:
    //
    // ```Circle```, ```Polygon```, ```Edge_Shape```. The simplex cache is input/output.
    //
    // On the first call set b2SimplexCache.count to zero.
	// Compute the closest points between two shapes represented as point clouds.
	// b2DistanceCache cache is input/output. On the first call set b2DistanceCache.count to zero.
	//    The underlying GJK algorithm may be debugged by passing in debug simplexes and capacity. You may pass in NULL and 0 for these.
    @(link_name="b2ShapeDistance")
    shape_distance :: proc(cache: ^Distance_Cache, input: ^Distance_Input, simplexes: ^Simplex = nil, simplex_capacity: i32 = 0) -> Distance_Output ---

    // Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
    //
    // Returns true if hit, ```false``` if there is no hit or an initial overlap
    @(link_name="b2ShapeCast")
    shape_cast :: proc(input: ^Shape_Cast_Pair_Input) -> Cast_Output ---

    // Make a proxy for use in GJK and related functions.
    @(link_name="b2MakeProxy")
    make_proxy :: proc(vertices: [^]Vec2, count: i32, radius: f32) -> Distance_Proxy ---

    // Evaluate the transform sweep at a specific time.
    @(link_name="b2GetSweepTransform")
    get_sweep_transform :: proc(sweep: ^Sweep, time: f32) -> Transform ---

    // Compute the upper bound on time before two shapes penetrate. Time is represented as
    // a fraction between ```[0,t_max]```. This uses a swept separating axis and may miss some intermediate,
    // non-tunneling collisions. If you change the time interval, you should call this function
    // again.
    @(link_name="b2TimeOfImpact")
    time_of_impact :: proc(input: ^TOI_Input) -> TOI_Output ---

    // Compute the collision manifold between two circles.
    @(link_name="b2CollideCircles")
    collide_circles :: proc(circle_a: ^Circle, xf_a: Transform, circleB: ^Circle, xf_b: Transform) -> Manifold ---

    // Compute the collision manifold between a capsule and circle
    @(link_name="b2CollideCapsuleAndCircle")
    collide_capsule_and_circle :: proc(capsule_a: ^Capsule, xf_a: Transform, circle_b: ^Circle, xf_b: Transform) -> Manifold ---

    // Compute the collision manifold between an segment and a circle.
    @(link_name="b2CollideSegmentAndCircle")
    collide_segment_and_circle :: proc(segment_a: Segment, xf_a: Transform, circle_b: ^Circle, xf_b: Transform) -> Manifold ---

    // Compute the collision manifold between a polygon and a circle.
    @(link_name="b2CollidePolygonAndCircle")
    collide_polygon_and_circle :: proc(polygon_a: ^Polygon, xf_a: Transform, circle_b: ^Circle, xf_b: Transform) -> Manifold ---

    // Compute the collision manifold between a capsule and circle
    @(link_name="b2CollideCapsules")
    collide_capsules :: proc(capsule_a: ^Capsule, xf_a: Transform, capsule_b: ^Capsule, xf_b: Transform, cache: ^Distance_Cache) -> Manifold ---

    // Compute the collision manifold between an segment and a capsule.
    @(link_name="b2CollideSegmentAndCapsule")
    collide_segment_and_capsule :: proc(segment_a: Segment, xf_a: Transform, capsule_b: ^Capsule, xf_b: Transform, cache: ^Distance_Cache) -> Manifold ---

    // Compute the collision manifold between a polygon and capsule
    @(link_name="b2CollidePolygonAndCapsule")
    collide_polygon_and_capsule :: proc(polygon_a: ^Polygon, xf_a: Transform, capsule_b: ^Capsule, xf_b: Transform, cache: ^Distance_Cache) -> Manifold ---

    // Compute the collision manifold between two polygons.
    @(link_name="b2CollidePolygons")
    collide_polygons :: proc(polygon_a: ^Polygon, xf_a: Transform, polygon_b: ^Polygon, xf_b: Transform, cache: ^Distance_Cache) -> Manifold ---

    // Compute the collision manifold between an segment and a polygon.
    @(link_name="b2CollideSegmentAndPolygon")
    collide_segment_and_polygon :: proc(segment_a: Segment, xf_a: Transform, polygon_b: ^Polygon, xf_b: Transform, cache: ^Distance_Cache) -> Manifold ---

    // Compute the collision manifold between a smooth segment and a circle.
    @(link_name="b2CollideSmoothSegmentAndCircle")
    collide_smooth_segment_and_circle :: proc(smooth_segment_a: ^Smooth_Segment, xf_a: Transform, circle_b: ^Circle, xf_b: Transform) -> Manifold ---

    // Compute the collision manifold between an segment and a capsule.
    @(link_name="b2CollideSmoothSegmentAndCapsule")
    collide_smooth_segment_and_capsule :: proc(smooth_segment_a: ^Smooth_Segment, xf_a: Transform, capsule_b: ^Capsule, xf_b: Transform, cache: ^Distance_Cache) -> Manifold ---

    // Compute the collision manifold between a smooth segment and a rounded polygon.
    @(link_name="b2CollideSmoothSegmentAndPolygon")
    collide_smooth_segment_and_polygon :: proc(smooth_segment_a: ^Smooth_Segment, xf_a: Transform, polygon_b: ^Polygon, xf_b: Transform, cache: ^Distance_Cache) -> Manifold ---

    // Constructing the tree initializes the node pool.
    @(link_name="b2DynamicTree_Create")
    dynamic_tree_create :: proc() -> Dynamic_Tree ---

    // Destroy the tree, freeing the node pool.
    @(link_name="b2DynamicTree_Destroy")
    dynamic_tree_destroy :: proc(tree: ^Dynamic_Tree) ---

    // Create a proxy. Provide a tight fitting AABB and a user_data value.
    @(link_name="b2DynamicTree_CreateProxy")
    dynamic_tree_create_proxy :: proc(tree: ^Dynamic_Tree, aabb: AABB, category_bits: u32, user_data: i32) -> i32 ---

    /// Destroy a proxy. This asserts if the id is invalid.
    @(link_name="b2DynamicTree_DestroyProxy")
    dynamic_tree_destroy_proxy :: proc(tree: ^Dynamic_Tree, proxy_id:  i32) ---

    // Clone one tree to another, reusing storage in the outTree if possible
    @(link_name="b2DynamicTree_Clone")
    dynamic_tree_clone :: proc(out_tree, in_tree: ^Dynamic_Tree) ---

    // Move a proxy to a new AABB by removing and reinserting into the tree.
    @(link_name="b2DynamicTree_MoveProxy")
    dynamic_tree_move_proxy :: proc(tree: ^Dynamic_Tree, proxy_id:  i32, aabb: AABB) ---

    // Enlarge a proxy and enlarge ancestors as necessary.
    @(link_name="b2DynamicTree_EnlargeProxy")
    dynamic_tree_enlarge_proxy :: proc(tree: ^Dynamic_Tree, proxy_id:  i32, aabb: AABB) ---

    // Query an AABB for overlapping proxies. The callback class
    // is called for each proxy that overlaps the supplied AABB.
    @(link_name="b2DynamicTree_QueryFiltered")
    dynamic_tree_query_filtered :: proc(tree: ^Dynamic_Tree, aabb: AABB, mask_bits: u32, callback: Tree_Query_Callback_Fcn, context_: rawptr) ---

    // Query an AABB for overlapping proxies. The callback class
    // is called for each proxy that overlaps the supplied AABB.
    @(link_name="b2DynamicTree_Query")
    dynamic_tree_query :: proc(tree: Dynamic_Tree, aabb: AABB, callback: Tree_Query_Callback_Fcn, context_: rawptr) ---

    // Ray-cast against the proxies in the tree. This relies on the callback
    // to perform a exact ray-cast in the case were the proxy contains a shape.
    //
    // The callback also performs the any collision filtering. This has performance
    // roughly equal to k - log(n), where k is the number of collisions and n is the
    // number of proxies in the tree.
    // - param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction - (p2 - p1).
    // - param callback a callback class that is called for each proxy that is hit by the ray.
    @(link_name="b2DynamicTree_RayCast")
    dynamic_tree_ray_cast :: proc(tree: ^Dynamic_Tree, input: ^Ray_Cast_Input, mask_bits: u32, callback: Tree_Ray_Cast_Callback_Fcn, context_: rawptr) ---


    // Ray-cast against the proxies in the tree. This relies on the callback
    // to perform a exact ray-cast in the case were the proxy contains a shape.
    //
    // The callback also performs the any collision filtering. This has performance
    // roughly equal to k - log(n), where k is the number of collisions and n is the
    // number of proxies in the tree.
    // - param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction - (p2 - p1).
    // - param callback a callback class that is called for each proxy that is hit by the ray.
    @(link_name="b2DynamicTree_ShapeCast")
    dynamic_tree_shape_cast :: proc(tree: ^Dynamic_Tree, input: ^Shape_Cast_Input, mask_bits: u32, callback: Tree_Shape_Cast_Callback_Fcn, context_: rawptr) ---

    // Validate this tree. For testing.
    @(link_name="b2DynamicTree_Validate")
    dynamic_tree_validate :: proc(tree: ^Dynamic_Tree) ---

    // Compute the height of the binary tree in O(N) time. Should not be
    // called often.
    @(link_name="b2DynamicTree_GetHeight")
    dynamic_tree_get_height :: proc(tree: ^Dynamic_Tree) -> i32 ---

    // Get the maximum balance of the tree. The balance is the difference in height of the two children of a node.
    @(link_name="b2DynamicTree_GetMaxBalance")
    dynamic_tree_get_max_balance :: proc(tree: ^Dynamic_Tree) -> i32 ---

    // Get the ratio of the sum of the node areas to the root area.
    @(link_name="b2DynamicTree_GetAreaRatio")
    dynamic_tree_get_area_ratio :: proc(tree: ^Dynamic_Tree) -> f32 ---

    // Build an optimal tree. Very expensive. For testing.
    @(link_name="b2DynamicTree_RebuildBottomUp")
    dynamic_tree_rebuild_bottom_up :: proc(tree: ^Dynamic_Tree) ---

    // Get the number of proxies created
    @(link_name="b2DynamicTree_GetProxyCount")
    dynamic_tree_get_proxy_count :: proc(tree: ^Dynamic_Tree) -> i32 ---

    // Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.
    @(link_name="b2DynamicTree_Rebuild")
    dynamic_tree_rebuild :: proc(tree: ^Dynamic_Tree, full_build: bool) -> i32 ---

    // Shift the world origin. Useful for large worlds.
    // The shift formula is: ```position -= new_origin```
    //
    // * ```tree``` the tree to shift
    // * ```new_origin``` the new origin with respect to the old origin
    @(link_name="b2DynamicTree_ShiftOrigin")
    dynamic_tree_shift_origin :: proc(tree: ^Dynamic_Tree, new_origin: Vec2) ---

    // Get the number of bytes used by this tree
    @(link_name="b2DynamicTree_GetByteCount")
    dynamic_tree_get_byte_count :: proc(tree: ^Dynamic_Tree) -> i32 ---


    /* math.h */


    @(link_name="b2IsValid")
    is_valid :: proc(a: f32) -> bool ---

    @(link_name="b2Vec2_IsValid")
    vec2_is_valid :: proc(v: Vec2) -> bool ---

    @(link_name="b2AABB_IsValid")
    aabb_is_valid :: proc(aabb: AABB) -> bool ---

    // Convert this vector into a unit vector
    @(link_name="b2Normalize")
    normalize :: proc(v: Vec2) -> Vec2 ---

    // This asserts of the vector is too short
    @(link_name="b2NormalizeChecked")
    normalize_checked :: proc(v: Vec2) -> Vec2 ---

    @(link_name="b2GetLengthAndNormalize")
    get_length_and_normalize :: proc(length: ^f32, v: Vec2) -> Vec2 ---


    /* types.h */


    // Use this to initialize your world definition
    @(link_name="b2DefaultWorldDef")
    default_world_def :: proc() -> World_Def ---

    // Use this to initialize your body definition
    @(link_name="b2DefaultBodyDef")
    default_body_def :: proc() -> Body_Def ---

    // Use this to initialize your filter
    @(link_name="b2DefaultFilter")
    default_filter :: proc() -> Filter ---

    // Use this to initialize your query filter
    @(link_name="b2DefaultQueryFilter")
    default_query_filter :: proc() -> Query_Filter ---

    // Use this to initialize your shape definition
    @(link_name="b2DefaultShapeDef")
    default_shape_def :: proc() -> Shape_Def ---

    // Use this to initialize your chain definition
    @(link_name="b2DefaultChainDef")
    default_chain_def :: proc() -> Chain_Def ---

    // Use this to initialize your joint definition
    @(link_name="b2DefaultDistanceJointDef")
    default_distance_joint_def :: proc() -> Distance_Joint_Def ---

    // Use this to initialize your joint definition
    @(link_name="b2DefaultMotorJointDef")
    default_motor_joint_def :: proc() -> Motor_Joint_Def ---

    // Use this to initialize your joint definition
    @(link_name="b2DefaultPrismaticJointDef")
    default_prismatic_joint_def :: proc() -> Prismatic_Joint_Def ---

    // Use this to initialize your joint definition
    @(link_name="b2DefaultRevoluteJointDef")
    default_revolute_joint_def :: proc() -> Revolute_Joint_Def ---

    // Use this to initialize your joint definition
    @(link_name="b2DefaultWeldJointDef")
    default_weld_joint_def :: proc() -> Weld_Joint_Def ---

    // Use this to initialize your joint definition
    @(link_name="b2DefaultWheelJointDef")
    default_wheel_joint_def :: proc() -> Wheel_Joint_Def ---
}
