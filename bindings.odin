package box2d

// This function receives shapes found in the AABB query.
// - Return true if the query should continue
Query_Callback_Fcn :: #type proc "c" (shape_id: Shape_ID, context_: rawptr) -> bool

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
    /* api.h */


    // This allows the user to override the allocation functions. These should be
    // set during application startup.
    set_allocator :: proc(alloc_fcn: Alloc_Fcn, free_fcn: Free_Fcn) ---

    // Total bytes allocated by Box2D
    get_byte_count :: proc() -> u32 ---

    // Override the default assert callback.
    // - param assert_fcn a non-null assert callback
    set_assert_fcn :: proc(assert_fcn: Assert_Fcn) ---

    
    /* box2d.h */


    // Create a world for rigid body simulation. This contains all the bodies, shapes, and constraints.
    @(link_name="b2CreateWorld")
    create_world :: proc(def: ^World_Def) -> World_ID ---

    // Destroy a world.
    @(link_name="b2DestroyWorld")
    destroy_world :: proc(world_id: World_ID) ---
    
    /// World identifier validation. Provides validation for up to 64K allocations.
    @(link_name="b2World_IsValid")
    world_is_valid :: proc(id: World_ID) -> bool ---

    // Take a time step. This performs collision detection, integration,
    // and constraint solution.
    // - param time_step the amount of time to simulate, this should not vary.
    // - param velocity_iterations for the velocity constraint solver.
    // - param position_iterations for the position constraint solver.
    @(link_name="b2World_Step") 
    world_step :: proc(world_id: World_ID, time_step: f32, sub_step_count: i32) ---

    // Call this to draw shapes and other debug draw data. This is intentionally non-const.
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

    // Overlap test for all shapes that *potentially* overlap the provided AABB.
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

    // Ray-cast the world for all shapes in the path of the ray. Your callback
    // controls whether you get the closest point, any point, or n-points.
    //
    // The ray-cast ignores shapes that contain the starting point.
    // - param callback a user implemented callback class.
    // - param point1 the ray starting point
    // - param point2 the ray ending point
    @(link_name="b2World_RayCast")
    world_ray_cast :: proc(world_id: World_ID, origin, translation: Vec2, filter: Query_Filter, fcn: Cast_Result_Fcn, context_: rawptr) ---
        
    // Ray-cast closest hit. Convenience function. This is less general than b2World_RayCast and does not allow for custom filtering.
    @(link_name="b2World_RayCastClosest")
    world_ray_cast_closest :: proc(world_id: World_ID, origin, translation: Vec2, filter: Query_Filter) -> Ray_Result ---
    
    // Cast a circle through the world. Similar to a ray-cast except that a circle is cast instead of a point.
    @(link_name="b2World_CircleCast")
    world_circle_cast :: proc(world_id: World_ID, circle: ^Circle, origin_transform: Transform, translation: Vec2, filter: Query_Filter, fcn: Cast_Result_Fcn, context_: rawptr) ---
    
    // Cast a capsule through the world. Similar to a ray-cast except that a capsule is cast instead of a point.
    @(link_name="b2World_CapsuleCast")
    world_capsule_cast :: proc(world_id: World_ID, capsule: ^Capsule, origin_transform: Transform, translation: Vec2, filter: Query_Filter, fcn: Cast_Result_Fcn, context_: rawptr) ---
    
    // Cast a capsule through the world. Similar to a ray-cast except that a polygon is cast instead of a point.
    @(link_name="b2World_PolygonCast")
    world_polygon_cast :: proc(world_id: World_ID, polygon: ^Polygon, origin_transform: Transform, translation: Vec2, filter: Query_Filter, fcn: Cast_Result_Fcn, context_: rawptr) ---
    
    // Enable/disable sleep. Advanced feature for testing.
    @(link_name="b2World_EnableSleeping")
    world_enable_sleeping :: proc(world_id: World_ID, flag: bool) ---

    // Enable/disable constraint warm starting. Advanced feature for testing.
    @(link_name="b2World_EnableWarmStarting")
    world_enable_warm_starting :: proc(world_id: World_ID, flag: bool) ---

    // Enable/disable continuous collision. Advanced feature for testing.
    @(link_name="b2World_EnableContinuous")
    world_enable_continuous :: proc(world_id: World_ID, flag: bool) ---

    // Adjust the restitution threshold. Advanced feature for testing.
    @(link_name="b2World_SetRestitutionThreshold")
    world_set_restitution_threshold :: proc(world_id: World_ID, value: f32) ---

    // Register the pre-solve callback. This is optional.
    @(link_name="b2World_SetPreSolveCallback")
    world_set_pre_solve_callback :: proc(world_id: World_ID, fcn: Pre_Solve_Fcn, context_: rawptr) ---

    // Adjust contact tuning parameters:
    // - hertz is the contact stiffness (cycles per second)
    // - damping ratio is the contact bounciness with 1 being critical damping (non-dimensional)
    // - push velocity is the maximum contact constraint push out velocity (meters per second)
    //
    // Advanced feature
    @(link_name="b2World_SetContactTuning")
    world_set_contact_tuning :: proc(world_id: World_ID, hertz, damping_ratio, push_velocity: f32) ---

    // Get the current profile.
    @(link_name="b2World_GetProfile")
    world_get_profile :: proc(world_id: World_ID) -> Profile ---

    // Get counters and sizes
    @(link_name="b2World_GetCounters")
    world_get_counters :: proc(world_id: World_ID) -> Counters ---

    // Create a rigid body given a definition. No reference to the definition is retained.
    // - Warning This function is locked during callbacks.
    @(link_name="b2CreateBody")
    create_body :: proc(world_id: World_ID, def: ^Body_Def) -> Body_ID ---

    // Destroy a rigid body given an id. This destroys all shapes attached to the body
    // but does not destroy the joints.
    // - Warning This function is locked during callbacks.
    @(link_name="b2DestroyBody")
    destroy_body :: proc(body_id: Body_ID) ---
    
    // Body identifier validation. Provides validation for up to 64K allocations.
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
    body_set_transform :: proc(body_id: Body_ID, position: Vec2, angle: f32) ---

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
    
    // Apply a force at a world point. If the force is not
    //
    // applied at the center of mass, it will generate a torque and
    // affect the angular velocity. This wakes up the body.
    // - param force the world force vector, usually in Newtons (N).
    // - param point the world position of the point of application.
    // - param wake also wake up the body
    @(link_name="b2Body_ApplyForce")
    body_apply_force :: proc(body_id: Body_ID, force, point: Vec2, wake: bool) ---
    
    // Apply a force to the center of mass. This wakes up the body.
    // - param force the world force vector, usually in Newtons (N).
    // - param wake also wake up the body
    @(link_name="b2Body_ApplyForceToCenter")
    body_apply_force_to_center :: proc(body_id: Body_ID, force: Vec2, wake: bool) ---
    
    // Apply a torque. This affects the angular velocity
    // without affecting the linear velocity of the center of mass.
    // - param torque about the z-axis (out of the screen), usually in N-m.
    // - param wake also wake up the body
    @(link_name="b2Body_ApplyTorque")
    body_apply_torque :: proc(body_id: Body_ID, torque: f32, wake: bool) ---
    
    // Apply an impulse at a point. This immediately modifies the velocity.
    //
    // It also modifies the angular velocity if the point of application
    // is not at the center of mass. This wakes up the body.
    //
    // This should be used for one-shot impulses. If you need a steady force,
    // use a force instead, which will work better with the sub-stepping solver.
    // - param impulse the world impulse vector, usually in N-seconds or kg-m/s.
    // - param point the world position of the point of application.
    // - param wake also wake up the body
    @(link_name="b2Body_ApplyLinearImpulse")
    body_apply_linear_impulse :: proc(body_id: Body_ID, impulse, point: Vec2, wake: bool) ---
    
    // Apply an impulse to the center of mass. This immediately modifies the velocity.
    //
    // This should be used for one-shot impulses. If you need a steady force,
    // use a force instead, which will work better with the sub-stepping solver.
    // - param impulse the world impulse vector, usually in N-seconds or kg-m/s.
    // - param wake also wake up the body
    @(link_name="b2Body_ApplyLinearImpulseToCenter")
    body_apply_linear_impulse_to_center :: proc(body_id: Body_ID, impulse: Vec2, wake: bool) ---
    
    // Apply an angular impulse.
    //
    // This should be used for one-shot impulses. If you need a steady force,
    // use a force instead, which will work better with the sub-stepping solver.
    // - param impulse the angular impulse in units of
    // kg*m*m/s
    // - param wake also wake up the body
    @(link_name="b2Body_ApplyAngularImpulse")
    body_apply_angular_impulse :: proc(body_id: Body_ID, impulse: f32, wake: bool) ---

    // Get the mass of the body (kilograms)
    @(link_name="b2Body_GetMass")
    body_get_mass :: proc(body_id: Body_ID) -> f32 ---

    // Get the inertia tensor of the body. In 2D this is a single number. (kilograms - meters^2)
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
    
    // This resets the mass properties to the sum of the mass properties of the fixtures.
    //
    // This normally does not need to be called unless you called SetMassData to override
    // the mass and you later want to reset the mass.
    @(link_name="b2Body_ResetMassData")
    body_reset_mass_data :: proc(body_id: Body_ID) ---
    
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
    @(link_name="b2Body_SetGravityScale")
    body_set_gravity_scale :: proc(body_id: Body_ID, gravity_scale: f32) ---
    
    // Get the current gravity scale.
    @(link_name="b2Body_GetGravityScale")
    body_get_gravity_scale :: proc(body_id: Body_ID) -> f32 ---
    
    // Is this body awake?
    @(link_name="b2Body_IsAwake")
    body_is_awake :: proc(body_id: Body_ID) ---

    // Wake a body from sleep. This wakes the entire island the body is touching.
    @(link_name="b2Body_Wake")
    body_wake :: proc(body_id: Body_ID) ---
    
    // Enable or disable sleeping this body. If sleeping is disabled the body will wake.
    @(link_name="b2Body_EnableSleep")
    body_enable_sleep :: proc(body_id: Body_ID, enable_sleep: bool) ---
    
    // - return is sleeping enabled for this body?
    @(link_name="b2Body_IsSleepEnabled")
    body_is_sleep_enabled :: proc(body_id: Body_ID) -> bool ---

    // Is this body enabled?
    @(link_name="b2Body_IsEnabled")
    body_is_enabled :: proc(body_id: Body_ID) -> bool ---

    // Disable a body by removing it completely from the simulation
    @(link_name="b2Body_Disable")
    body_disable :: proc(body_id: Body_ID) ---

    // Enable a body by adding it to the simulation
    @(link_name="b2Body_Enable")
    body_enable :: proc(body_id: Body_ID) ---
    
    // Set this body to have fixed rotation. This causes the mass to be reset.
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

    // Iterate over shapes on a body
    @(link_name="b2Body_GetFirstShape")
    body_get_first_shape :: proc(body_id: Body_ID) -> Shape_ID ---
    
    // Iterate over shapes on a body
    @(link_name="b2Body_GetNextShape")
    body_get_next_shape :: proc(shape_id: Shape_ID) -> Shape_ID ---
    
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
    // - return the shape id for accessing the shape
    @(link_name="b2CreateCircleShape")
    create_circle_shape :: proc(body_id: Body_ID, def: ^Shape_Def, circle: ^Circle) -> Shape_ID ---

    // Create a line segment shape and attach it to a body. The shape definition and geometry are fully cloned.
    //
    // Contacts are not created until the next time step.
    // - return the shape id for accessing the shape
    @(link_name="b2CreateSegmentShape")
    create_segment_shape :: proc(body_id: Body_ID, def: ^Shape_Def, segment: ^Segment) -> Shape_ID ---
    
    // Create a capsule shape and attach it to a body. The shape definition and geometry are fully cloned.
    //
    // Contacts are not created until the next time step.
    // - return the shape id for accessing the shape
    @(link_name="b2CreateCapsuleShape")
    create_capsule_shape :: proc(body_id: Body_ID, def: ^Shape_Def, capsule: ^Capsule) -> Shape_ID ---
    
    // Create a polygon shape and attach it to a body. The shape definition and geometry are fully cloned.
    //
    // Contacts are not created until the next time step.
    // - return the shape id for accessing the shape
    @(link_name="b2CreatePolygonShape")
    create_polygon_shape :: proc(body_id: Body_ID, def: ^Shape_Def, polygon: ^Polygon) -> Shape_ID ---

    // Destroy any shape type
    @(link_name="b2DestroyShape")
    destroy_shape :: proc(shape_id: Shape_ID) ---

    // Shape identifier validation. Provides validation for up to 64K allocations.
    @(link_name="b2Shape_IsValid")
    shape_is_valid :: proc(shape_id: Shape_ID) -> bool ---
    
    // Get the type of a shape.
    @(link_name="b2Shape_GetType")
    shape_get_type :: proc(shape_id: Shape_ID) -> Shape_Type ---

    // Get the body that a shape is attached to
    @(link_name="b2Shape_GetBody")
    shape_get_body :: proc(shape_id: Shape_ID) -> Body_ID ---

    // Is this shape a sensor? See Shape_Def.
    @(link_name="b2Shape_IsSensor")
    shape_is_sensor :: proc(shape_id: Shape_ID) -> bool ---
    
    // Set the user data for a shape.
    @(link_name="b2Shape_SetUserData")
    shape_set_user_data :: proc(shape_id: Shape_ID, user_data: rawptr) ---

    // Get the user data for a shape. This is useful when you get a shape id
    // from an event or query.
    @(link_name="b2Shape_GetUserData")
    shape_get_user_data :: proc(shape_id: Shape_ID) -> rawptr ---
    
    // Set the density on a shape. Normally this is specified in Shape_Def.
    //
    // This will not update the mass properties on the parent body until you
    // call body_reset_mass_data.
    @(link_name="b2Shape_SetDensity")
    shape_set_density :: proc(shape_id: Shape_ID, density: f32) ---
    
    // Get the density on a shape.
    @(link_name="b2Shape_GetDensity")
    shape_get_density :: proc(shape_id: Shape_ID) -> f32 ---

    // Set the friction on a shape. Normally this is specified in b2ShapeDef.
    @(link_name="b2Shape_SetFriction")
    shape_set_friction :: proc(shape_id: Shape_ID, friction: f32) ---

    // Get the friction on a shape.
    @(link_name="b2Shape_GetFriction")
    shape_get_friction :: proc(shape_id: Shape_ID) -> f32 ---

    // Set the restitution (bounciness) on a shape. Normally this is specified in b2ShapeDef.
    @(link_name="b2Shape_SetRestitution")
    shape_set_restitution :: proc(shape_id: Shape_ID, restitution: f32) ---

    // Get the restitution on a shape.
    @(link_name="b2Shape_GetRestitution")
    shape_get_restitution :: proc(shape_id: Shape_ID) -> f32 ---

    // Get the current filter
    @(link_name="b2Shape_GetFilter")
    shape_get_filter :: proc(shape_id: Shape_ID) -> Filter ---

    // Set the current filter. This is almost as expensive as recreating the shape.
    @(link_name="b2Shape_SetFilter")
    shape_set_filter :: proc(shape_id: Shape_ID, filter: Filter) ---
    
    // Enable sensor events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
    @(link_name="b2Shape_EnableSensorEvents")
    shape_enable_sensor_events :: proc(shape_id: Shape_ID, flag: bool) ---
    
    // - return are sensor events enabled?
    @(link_name="b2Shape_AreSensorEventsEnabled")
    b2_shape_are_sensor_events_enabled :: proc(shape_id: Shape_ID) -> bool ---
    
    // Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
    @(link_name="b2Shape_EnableContactEvents")
    Shape_EnableContactEvents :: proc(shape_id: Shape_ID, flag: bool) ---
    
    // - return are contact events enabled?
    @(link_name="b2Shape_AreContactEventsEnabled")
    shape_are_contact_events_enabled :: proc(shape_id: Shape_ID) -> bool ---
    
    // Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
    //	and must be carefully handled due to multi-threading. Ignored for sensors.
    @(link_name="b2Shape_EnablePreSolveEvents")
    Shape_EnablePreSolveEvents :: proc(shape_id: Shape_ID, flag: bool) ---

    // - return are pre-solve events enabled?
    shape_are_pre_solve_events_enabled :: proc(shape_id: Shape_ID) -> bool ---

    // Test a point for overlap with a shape
    @(link_name="b2Shape_TestPoint")
    shape_test_point :: proc(shape_id: Shape_ID, point: Vec2) -> bool ---
    
    //Ray cast a shape directly
    @(link_name="b2Shape_RayCast")
    shape_ray_cast :: proc(shape_id: Shape_ID, origin, translation: Vec2) -> Cast_Output ---
    
    // Access the circle geometry of a shape.
    @(link_name="b2Shape_GetCircle")
    shape_get_circle :: proc(shape_id: Shape_ID) -> ^Circle ---
    
    // Access the line segment geometry of a shape.
    @(link_name="b2Shape_GetSegment")
    shape_get_segment :: proc(shape_id: Shape_ID) -> ^Segment ---
    
    // Access the smooth line segment geometry of a shape. These come from chain shapes.
    @(link_name="b2Shape_GetSmoothSegment")
    shape_get_smooth_segment :: proc(shape_id: Shape_ID) -> ^Smooth_Segment ---
    
    // Access the capsule geometry of a shape.
    @(link_name="b2Shape_GetCapsule")
    shape_get_capsule :: proc(shape_id: Shape_ID) -> ^Capsule ---
    
    // Access the convex polygon geometry of a shape.
    @(link_name="b2Shape_GetPolygon")
    shape_get_polygon :: proc(shape_id: Shape_ID) -> ^Polygon ---
    
    // Allows you to change a shape to be a circle or update the current circle.
    //
    // This does not modify the mass properties.
    @(link_name="b2Shape_SetCircle")
    shape_set_circle :: proc(shape_id: Shape_ID, circle: ^Circle) ---
    
    // Allows you to change a shape to be a capsule or update the current capsule.
    @(link_name="b2Shape_SetCapsule")
    shape_set_capsule :: proc(shape_id: Shape_ID, capsule: ^Capsule) ---
    
    // Allows you to change a shape to be a segment or update the current segment.
    @(link_name="b2Shape_SetSegment")
    shape_set_segment :: proc(shape_id: Shape_ID, segment: ^Segment) ---
    
    // Allows you to change a shape to be a segment or update the current segment.
    @(link_name="b2Shape_SetPolygon")
    shape_set_polygon :: proc(shape_id: Shape_ID, polygon: ^Polygon) ---
    
    // If the type is SMOOTH_SEGMENT_SHAPE then you can get the parent chain id.
    //
    // If the shape is not a smooth segment then this will return NULL_CHAIN_ID.
    @(link_name="b2Shape_GetParentChain")
    shape_get_parent_chain :: proc(shape_id: Shape_ID) -> Chain_ID ---
    
    // Get the maximum capacity required for retrieving all the touching contacts on a shape
    @(link_name="b2Shape_GetContactCapacity")
    shape_get_contact_capacity :: proc(shape_id: Shape_ID) -> i32 ---
    
    // Get the touching contact data for a shape. The provided shapeId will be either shapeIdA or shapeIdB on the contact data.
    @(link_name="b2Shape_GetContactData")
    shape_get_contact_data :: proc(shape_id: Shape_ID, contact_data: [^]Contact_Data, capacity: i32) -> i32 ---
    
    // Get the current world AABB
    @(link_name="b2Shape_GetAABB")
    shape_get_aabb :: proc(shape_id: Shape_ID) -> AABB ---
    
    // Create a chain shape
    // - see Chain_Def for details
    @(link_name="b2CreateChain")
    create_chain :: proc(body_id: Body_ID, def: ^Chain_Def) -> Chain_ID ---
    
    // Destroy a chain shape
    @(link_name="b2DestroyChain")
    destroy_chain :: proc(chain_id: Chain_ID) ---
    
    // Set the friction of a chain. Normally this is set in b2ChainDef.
    @(link_name="b2Chain_SetFriction")
    chain_set_friction :: proc(chain_id: Chain_ID, friction: f32) ---
    
    // Set the restitution (bounciness) on a chain. Normally this is specified in b2ChainDef.
    @(link_name="b2Chain_SetFriction")
    chain_set_restitution :: proc(chain_id: Chain_ID, restitution: f32) ---

    // Chain identifier validation. Provides validation for up to 64K allocations.
    @(link_name="b2Chain_IsValid")
    chain_is_valid :: proc(chain_id: Chain_ID) -> bool ---

    // Create a distance joint
    // see Distance_Joint_Def for details
    @(link_name="b2CreateDistanceJoint")
    create_distance_joint :: proc(world_id: World_ID, def: ^Distance_Joint_Def) -> Joint_ID ---
    
    // Create a motor joint
    // see Motor_Join_Def for details
    @(link_name="b2CreateMotorJoint")
    create_motor_joint :: proc(world_id: World_ID, def: ^Motor_Joint_Def) -> Joint_ID ---

    // Create a mouse joint
    // - see Mouse_Joint_Def for details
    @(link_name="b2CreateMouseJoint")
    create_mouse_joint :: proc(world_id: World_ID, def: ^Mouse_Joint_Def) -> Joint_ID ---

    // Create a prismatic (slider) joint
    // see Prismatic_Joint_Def for details
    @(link_name="b2CreatePrismaticJoint")
    create_prismatic_joint :: proc(world_id: World_ID, def: ^Prismatic_Joint_Def) -> Joint_ID ---

    // Create a revolute (hinge) joint
    // see Revolute_Joint_Def for details
    @(link_name="b2CreateRevoluteJoint")
    create_revolute_joint :: proc(world_id: World_ID, def: ^Revolute_Joint_Def) -> Joint_ID ---

    // Create a weld joint
    // - see Weld_Joint_Def for details
    @(link_name="b2CreateWeldJoint")
    create_weld_joint :: proc(world_id: World_ID, def: ^Weld_Joint_Def) -> Joint_ID ---

    // Create a wheel joint
    // - see Wheel_Joint_Def for details
    @(link_name="b2CreateWheelJoint")
    create_wheel_joint :: proc(world_id: World_ID, def: ^Wheel_Joint_Def) -> Joint_ID ---

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
    
    // Get local anchor on body_a
    @(link_name="b2Joint_GetLocalAnchorA")
    joint_get_local_anchor_a :: proc(joint_id: Joint_ID) -> Vec2 ---

    // Get local anchor on body_b
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
    Joint_GetUserData :: proc(joint_id: Joint_ID) -> rawptr ---
    
    // Wake the bodies connect to this joint
    @(link_name="b2Joint_WakeBodies")
    joint_wake_bodies :: proc(joint_id: Joint_ID) ---

    // Get the constraint force on a distance joint
    @(link_name="b2DistanceJoint_GetConstraintForce")
    distance_joint_get_constraint_force :: proc(joint_id: Joint_ID, time_step: f32) -> f32 ---

    // Set the length parameters of a distance joint
    // - see Distance_Joint_Def for details
    @(link_name="b2DistanceJoint_SetLength")
    distance_joint_set_length :: proc(joint_id: Joint_ID, length, min_length, max_length: f32) ---

    // Get the rest length of a distance joint
    @(link_name="b2DistanceJoint_GetLength")
    distance_joint_get_length :: proc(joint_id: Joint_ID) -> f32 ---
    
    /// Set the minimum and maximum length parameters of a distance joint
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

    // Adjust the softness of a distance joint
    // - see Distance_Joint_Def for details
    @(link_name="b2DistanceJoint_SetTuning")
    distance_joint_set_tuning :: proc(joint_id: Joint_ID, hertz, damping_ratio: f32) ---
    
    // Get the Hertz of a distance joint
    @(link_name="b2DistanceJoint_GetHertz")
    distance_joint_get_hertz :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Get the damping ratio of a distance joint
    @(link_name="b2DistanceJoint_GetDampingRatio")
    distance_joint_get_damping_ratio :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Set the linear offset target for a motor joint
    @(link_name="b2MotorJoint_SetLinearOffset")
    motor_joint_set_linear_offset :: proc(joint_id: Joint_ID, linear_offset: Vec2) ---
    
    // - return the linear offset target for a motor joint
    @(link_name="b2MotorJoint_GetLinearOffset")
    motor_joint_get_linear_offset :: proc(joint_id: Joint_ID) -> Vec2 ---
    
    // Set the angular offset target for a motor joint in radians
    @(link_name="b2MotorJoint_SetAngularOffset")
    motor_joint_set_angular_offset :: proc(joint_id: Joint_ID, angular_offset: f32) ---

    // - return the angular offset target for a motor joint in radians
    @(link_name="b2MotorJoint_GetAngularOffset")
    motor_joint_get_angular_offset :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Set the maximum force for a motor joint
    @(link_name="b2MotorJoint_SetMaxForce")
    motor_joint_set_max_force :: proc(joint_id: Joint_ID, max_force: f32) ---

    // - return the maximum force for a motor joint
    @(link_name="b2MotorJoint_GetMaxForce")
    motor_joint_get_max_force :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Set the maximum torque for a motor joint
    @(link_name="b2MotorJoint_SetMaxTorque")
    motor_joint_set_max_torque :: proc(joint_id: Joint_ID, max_torque: f32) ---

    // - return the maximum torque for a motor joint
    @(link_name="b2MotorJoint_GetMaxTorque")
    motor_joint_get_max_torque :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Set the correction factor for a motor joint
    @(link_name="b2MotorJoint_SetCorrectionFactor")
    motor_joint_set_correction_factor :: proc(joint_id: Joint_ID, correction_factor: f32) ---
    
    // - return the correction factor for a motor joint
    @(link_name="b2MotorJoint_GetCorrectionFactor")
    motor_joint_get_correction_factor :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Get the current constraint force for a motor joint
    @(link_name="b2MotorJoint_GetConstraintForce")
    motor_joint_get_constraint_force :: proc(joint_id: Joint_ID) -> Vec2 ---
    
    // Get the current constraint torque for a motor joint
    @(link_name="b2MotorJoint_GetConstraintTorque")
    motor_joint_get_constraint_torque :: proc(joint_id: Joint_ID) -> f32 ---

    // Set the target for a mouse joint
    @(link_name="b2MouseJoint_SetTarget")
    mouse_joint_set_target :: proc(joint_id: Joint_ID, target: Vec2) ---
    
    // - return the target for a mouse joint
    @(link_name="b2MouseJoint_GetTarget")
    mouse_joint_get_target :: proc(joint_id: Joint_ID) -> Vec2 ---
    
    // Adjust the softness parameters of a mouse joint
    @(link_name="b2MouseJoint_SetTuning")
    mouse_joint_set_tuning :: proc(joint_id: Joint_ID, hertz, damping_ratio: f32) ---

    // Get the Hertz of a mouse joint
    @(link_name="b2MouseJoint_GetHertz")
    mouse_joint_get_hertz :: proc(joint_id: Joint_ID) -> f32 ---

    // Get the damping ratio of a mouse joint
    @(link_name="b2MouseJoint_GetDampingRatio")
    mouse_joint_get_damping_ratio :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Enable/disable a prismatic joint limit
    @(link_name="b2PrismaticJoint_EnableLimit")
    prismatic_joint_enable_limit :: proc(joint_id: Joint_ID, enable_limit: bool) ---
    
    // - return is the prismatic joint limit enabled
    @(link_name="b2PrismaticJoint_IsLimitEnabled")
    prismatic_joint_is_limit_enabled :: proc(joint_id: Joint_ID) -> bool ---
    
    // Get the lower joint limit in length units (meters).
    @(link_name="b2PrismaticJoint_GetLowerLimit")
    prismatic_joint_get_lower_limit :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Get the upper joint limit in length units (meters).
    @(link_name="b2PrismaticJoint_GetUpperLimit")
    prismatic_joint_get_upper_limit :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Set the joint limits in length units (meters).
    @(link_name="b2PrismaticJoint_SetLimits")
    prismatic_joint_set_limits :: proc(joint_id: Joint_ID, lower, upper: f32) ---
    
    // Enable/disable a prismatic joint motor
    @(link_name="b2PrismaticJoint_EnableMotor")
    prismatic_joint_enable_motor :: proc(joint_id: Joint_ID, enable_motor: bool) ---
    
    // - return is the prismatic joint motor enabled
    @(link_name="b2PrismaticJoint_IsMotorEnabled")
    prismatic_joint_is_motor_enabled :: proc(joint_id: Joint_ID) -> bool ---
    
    // Set the motor speed for a prismatic joint
    @(link_name="b2PrismaticJoint_SetMotorSpeed")
    prismatic_joint_set_motor_speed :: proc(joint_id: Joint_ID, motor_speed: f32) ---
    
    // - return the motor speed for a prismatic joint
    @(link_name="b2PrismaticJoint_GetMotorSpeed")
    prismatic_joint_get_motor_speed :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Get the current motor force for a prismatic joint
    @(link_name="b2PrismaticJoint_GetMotorForce")
    prismatic_joint_get_motor_force :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Set the maximum force for a pristmatic joint motor
    @(link_name="b2PrismaticJoint_SetMaxMotorForce")
    prismatic_joint_set_max_motor_force :: proc(joint_id: Joint_ID, force: f32) ---
    
    // - return the maximum force for a prismatic joint motor
    @(link_name="b2PrismaticJoint_GetMaxMotorForce")
    prismatic_joint_get_max_motor_force :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Get the current constraint force for a prismatic joint
    @(link_name="b2PrismaticJoint_GetConstraintForce")
    prismatic_joint_get_constraint_force :: proc(joint_id: Joint_ID) -> Vec2 ---
    
    // Get the current constraint torque for a prismatic joint
    @(link_name="b2PrismaticJoint_GetConstraintTorque")
    prismatic_joint_get_constraint_torque :: proc(joint_id: Joint_ID) -> f32 ---

    // Enable/disable a revolute joint limit
    @(link_name="b2RevoluteJoint_EnableLimit")
    revolute_joint_enable_limit :: proc(joint_id: Joint_ID, enable_limit: bool) ---
    
    // - return is the revolute joint limit enabled
    @(link_name="b2RevoluteJoint_IsLimitEnabled")
    revolute_joint_is_limit_enabled :: proc(joint_id: Joint_ID) -> bool ---
    
    // Get the lower joint limit in radians.
    @(link_name="b2RevoluteJoint_GetLowerLimit")
    revolute_joint_get_lower_limit :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Get the upper joint limit in radians.
    @(link_name="b2RevoluteJoint_GetUpperLimit")
    revolute_joint_get_upper_limit :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Set the joint limits in radians.
    @(link_name="b2RevoluteJoint_SetLimits")
    revolute_joint_set_limits :: proc(joint_id: Joint_ID, lower, upper: f32) ---
    
    // Enable/disable a revolute joint motor
    @(link_name="b2RevoluteJoint_EnableMotor")
    revolute_joint_enable_motor :: proc(joint_id: Joint_ID, enable_motor: bool) ---
    
    // - return is the revolute joint motor enabled
    @(link_name="b2RevoluteJoint_IsMotorEnabled")
    revolute_joint_is_motor_enabled :: proc(joint_id: Joint_ID) -> bool ---

    // Set the motor speed for a revolute joint in radians per second
    @(link_name="b2RevoluteJoint_SetMotorSpeed")
    revolute_joint_set_motor_speed :: proc(joint_id: Joint_ID, motor_speed: f32) ---
    
    // - return the motor speed for a revolute joint in radians per second
    @(link_name="b2RevoluteJoint_GetMotorSpeed")
    revolute_joint_get_motor_speed :: proc(joint_id: Joint_ID) -> f32 ---

    // Get the current motor torque for a revolute joint
    @(link_name="b2RevoluteJoint_GetMotorTorque")
    revolute_joint_get_motor_torque :: proc(joint_id: Joint_ID) -> f32 ---

    // Set the maximum torque for a revolute joint motor
    @(link_name="b2RevoluteJoint_SetMaxMotorTorque")
    revolute_joint_set_max_motor_torque :: proc(joint_id: Joint_ID, torque: f32) ---
    
    // - return the maximum torque for a revolute joint motor
    @(link_name="b2RevoluteJoint_GetMaxMotorTorque")
    revolute_joint_get_max_motor_torque :: proc(joint_id: Joint_ID) -> f32 ---

    // Get the current constraint force for a revolute joint
    @(link_name="b2RevoluteJoint_GetConstraintForce")
    revolute_joint_get_constraint_force :: proc(joint_id: Joint_ID) -> Vec2 ---

    // Get the current constraint torque for a revolute joint
    @(link_name="b2RevoluteJoint_GetConstraintTorque")
    revolute_joint_get_constraint_torque :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Set the wheel joint stiffness in Hertz
    @(link_name="b2WheelJoint_SetSpringHertz")
    wheel_joint_set_spring_hertz :: proc(joint_id: Joint_ID, hertz: f32) ---
    
    // - return the wheel joint stiffness in Hertz
    @(link_name="b2WheelJoint_GetSpringHertz")
    wheel_joint_get_spring_hertz :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Set the wheel joint damping ratio (non-dimensional)
    @(link_name="b2WheelJoint_SetSpringDampingRatio")
    wheel_joint_set_spring_damping_ratio :: proc(joint_id: Joint_ID, damping_ratio: f32) ---
    
    // - return the wheel joint damping ratio (non-dimensional)
    @(link_name="b2WheelJoint_GetSpringDampingRatio")
    wheel_joint_get_spring_damping_ratio :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Enable/disable the wheel joint limit
    @(link_name="b2WheelJoint_EnableLimit")
    wheel_joint_enable_limit :: proc(joint_id: Joint_ID, enable_limit: f32) ---
    
    // - return is the wheel joint limit enabled
    @(link_name="b2WheelJoint_IsLimitEnabled")
    wheel_joint_is_limit_enabled :: proc(joint_id: Joint_ID) -> bool ---
    
    // Get the lower joint limit in length units (meters).
    @(link_name="b2WheelJoint_GetLowerLimit")
    wheel_joint_get_lower_limit :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Get the upper joint limit in length units (meters).
    @(link_name="b2WheelJoint_GetUpperLimit")
    wheel_joint_get_upper_limit :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Set the joint limits in length units (meters).
    @(link_name="b2WheelJoint_SetLimits")
    wheel_joint_set_limits :: proc(joint_id: Joint_ID, lower, upper: f32) ---

    // Enable/disable the wheel joint motor
    @(link_name="b2WheelJoint_EnableMotor")
    wheel_joint_enable_motor :: proc(joint_id: Joint_ID, enable_motor: bool) ---
    
    // - return is the wheel joint motor enabled
    @(link_name="b2WheelJoint_IsMotorEnabled")
    wheel_joint_is_motor_enabled :: proc(joint_id: Joint_ID) -> bool ---
    
    // Set the wheel joint motor speed in radians per second
    @(link_name="b2WheelJoint_SetMotorSpeed")
    wheel_joint_set_motor_speed :: proc(joint_id: Joint_ID, motor_speed: f32) ---
    
    // - return the wheel joint motor speed in radians per second
    @(link_name="b2WheelJoint_GetMotorSpeed")
    wheel_joint_get_motor_speed :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Get the wheel joint current motor torque
    @(link_name="b2WheelJoint_GetMotorTorque")
    wheel_joint_get_motor_torque :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Set the wheel joint maximum motor torque
    @(link_name="b2WheelJoint_SetMaxMotorTorque")
    wheel_joint_set_max_motor_torque :: proc(joint_id: Joint_ID, torque: f32) ---
    
    // - return the wheel joint maximum motor torque
    @(link_name="b2WheelJoint_GetMaxMotorTorque")
    wheel_joint_get_max_motor_torque :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Get the current wheel joint constraint force
    @(link_name="b2WheelJoint_GetConstraintForce")
    wheel_joint_get_constraint_force :: proc(joint_id: Joint_ID) -> Vec2 ---
    
    // Get the current wheel joint constraint torque
    @(link_name="b2WheelJoint_GetConstraintTorque")
    wheel_joint_get_constraint_torque :: proc(joint_id: Joint_ID) -> f32 ---
    
    // Set weld joint linear stiffness in Hertz. 0 is rigid.
    @(link_name="b2WeldJoint_SetLinearHertz")
    weld_joint_set_linear_hertz :: proc(joint_id: Joint_ID, hertz: f32) ---
    
    // - return the weld joint linear stiffness in Hertz.
    @(link_name="b2WeldJoint_GetLinearHertz")
    b2_weld_joint_get_linear_hertz :: proc(joint_id: Joint_ID) -> f32 ---
    
    /// Set weld joint linear damping ratio (non-dimensional)
    @(link_name="b2WeldJoint_SetLinearDampingRatio")
    weld_joint_set_linear_damping_ratio :: proc(joint_id: Joint_ID, damping_ratio: f32) ---
    
    // - return the weld joint linear damping ratio (non-dimensional)
    @(link_name="b2WeldJoint_GetLinearDampingRatio")
    weld_joint_get_linear_damping_ratio :: proc(joint_id: Joint_ID) -> f32 ---
    
    /// Set weld joint angular stiffness in Hertz. 0 is rigid.
    @(link_name="b2WeldJoint_SetAngularHertz")
    weld_joint_set_angular_hertz :: proc(joint_id: Joint_ID, hertz: f32) ---
    
    // - return the weld joint angular stiffness in Hertz.
    @(link_name="b2WeldJoint_GetAngularHertz")
    weld_joint_get_angular_hertz :: proc(joint_id: Joint_ID) -> f32 ---
    
    /// Set weld joint angular damping ratio (non-dimensional)
    @(link_name="b2WeldJoint_SetAngularDampingRatio")
    weld_joint_set_angular_damping_ratio :: proc(joint_id: Joint_ID, damping_ratio: f32) ---
    
    // - return the weld joint angular damping ratio (non-dimensional)
    @(link_name="b2WeldJoint_GetAngularDampingRatio")
    weld_joint_get_angular_damping_ratio :: proc(joint_id: Joint_ID) -> f32 ---


    /* joint_util.h */
    
    
    // Utility to compute linear stiffness values from frequency and damping ratio
    @(link_name="b2LinearStiffness")
    linear_stiffness :: proc(stiffness, damping: ^f32, frequency_hertz, damping_ratio: f32, body_a, body_b: Body_ID) ---
    
    // Utility to compute rotational stiffness values frequency and damping ratio
    @(link_name="b2AngularStiffness")
    angular_stiffness :: proc(stiffness, damping: ^f32, frequency_hertz, damping_ratio: f32, body_a, body_b: Body_ID) ---
    
    
    /* distance.h */
    

    // Compute the distance between two line segments, clamping at the end points if needed.
    @(link_name="b2SegmentDistance")
    segment_distance :: proc(p1, q1, p2, q2: Vec2) -> Segment_Distance_Result ---
    
    // Compute the closest points between two shapes. Supports any combination of:
    //
    // Circle, Polygon, Edge_Shape. The simplex cache is input/output.
    //
    // On the first call set b2SimplexCache.count to zero.
    @(link_name="b2ShapeDistance")
    shape_distance :: proc(cache: ^Distance_Cache, input: ^Distance_Input) -> Distance_Output ---
    
    // Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
    // - returns true if hit, false if there is no hit or an initial overlap
    @(link_name="b2ShapeCast")
    shape_cast :: proc(input: ^Shape_Cast_Pair_Input) -> Cast_Output ---
    
    // Make a proxy for use in GJK and related functions.
    @(link_name="b2MakeProxy")
    make_proxy :: proc(vertices: [^]Vec2, count: i32, radius: f32) -> Distance_Proxy ---
    
    @(link_name="b2GetSweepTransform")
    get_sweep_transform :: proc(sweep: ^Sweep, time: f32) -> Transform ---

    // Compute the upper bound on time before two shapes penetrate. Time is represented as
    // a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
    // non-tunneling collisions. If you change the time interval, you should call this function
    // again.
    @(link_name="b2TimeOfImpact")
    time_of_impact :: proc(input: ^TOI_Input) -> TOI_Output ---


    /* dynamic_tree.h */


    // Constructing the tree initializes the node pool.
    @(link_name="b2DynamicTree_Create")
    dynamic_tree_create :: proc() -> Dynamic_Tree ---
    
    // Destroy the tree, freeing the node pool.
    @(link_name="b2DynamicTree_Destroy")
    dynamic_tree_destroy :: proc(tree: ^Dynamic_Tree) ---
    
    // Create a proxy. Provide a tight fitting AABB and a userData value.
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
    //
    // The shift formula is: position -= new_origin
    // - 'new_origin' the new origin with respect to the old origin
    @(link_name="b2DynamicTree_ShiftOrigin")
    dynamic_tree_shift_origin :: proc(tree: ^Dynamic_Tree, new_origin: Vec2) ---


    /* geometry.h */
    

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


    /* hull.h */


    // Compute the convex hull of a set of points. Returns an empty hull if it fails.
    //
    // Some failure cases:
    // - all points very close together
    // - all points on a line
    // - less than 3 points
    // - more than b2_maxPolygonVertices points
    //
    // This welds close points and removes collinear points.
    @(link_name="b2ComputeHull")
    compute_hull :: proc(points: [^]Vec2, count: i32) -> Hull ---

    // This determines if a hull is valid. Checks for:
    // - convexity
    // - collinear points
    // This is expensive and should not be called at runtime.
    @(link_name="b2ValidateHull")
    validate_hull :: proc(hull: ^Hull) -> bool ---

    
    /* joint_types.h */
    
    
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


    /* manifold.h */


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

    
    /* timer.h */


    @(link_name="b2CreateTimer")
    create_timer :: proc() -> Timer ---

    @(link_name="b2GetTicks")
    get_ticks :: proc(timer: ^Timer) -> i64 ---

    @(link_name="b2GetMilliseconds")
    get_milliseconds :: proc(timer: ^Timer) -> f32 ---

    @(link_name="b2GetMillisecondsAndReset")
    get_milliseconds_and_reset :: proc(timer: ^Timer) -> f32 ---

    @(link_name="b2SleepMilliseconds")
    sleep_milliseconds :: proc(milliseconds: f32) ---


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
}
