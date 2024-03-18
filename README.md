# Description

Box2D 3.0 https://github.com/erincatto/box2c

Current commit: https://github.com/erincatto/box2c/commit/29adfcfb4dbc371a50e37d8e17b072d990ad7ae6

Included platform binaries:
* windows_amd64

All binaries are build using the default settings.
All binaries are located in binaries directory if you want to use your own build of Box2D.

Defines for Box2D Constants:
* BOX2D_AVX2 (default: true)
* BOX2D_LENGTH_UNITS_PER_METER
* BOX2D_MAX_POLYGON_VERTICES
* BOX2D_MAX_WORLDS
* BOX2D_TIME_TO_SLEEP
* BOX2D_LINEAR_SLEEP_TOLERANCE
* BOX2D_ANGULAR_SLEEP_TOLERANCE

Only change it if you build your own version using diferent constant values.

# Example:

```odin
package box2d_example

import b2 "odin-box2d"
import "core:fmt"

main :: proc()
{
    world_def := b2.default_world_def()
    world_def.gravity = b2.Vec2{0, -10}
    world_id := b2.create_world(&world_def)
    defer b2.destroy_world(world_id)
    
    ground_body_def := b2.default_body_def()
    ground_body_def.position = b2.Vec2{0, -10}
    ground_body_id := b2.create_body(world_id, &ground_body_def)

    ground_box := b2.make_box(50, 10)
    ground_shape_def := b2.default_shape_def()
    b2.create_polygon_shape(ground_body_id, &ground_shape_def, &ground_box)

    body_def := b2.default_body_def()
    body_def.type = .Dynamic
    body_def.position = b2.Vec2{0, 4}
    body_id := b2.create_body(world_id, &body_def)

    shape_def := b2.default_shape_def(0)
    shape_def.density = 1
    shape_def.friction = 0.3

    circle: b2.Circle
    circle.radius = 1
    b2.create_circle_shape(body_id, &shape_def, &circle)

    time_step: f32 = 1.0 / 60
    sub_steps: i32 = 4
    
    for i in 0..<60
    {
        b2.world_step(world_id, time_step, sub_steps)
        position := b2.body_get_position(body_id)
        angle := b2.body_get_angle(body_id)
        fmt.println(position, angle)
    }
}
```
