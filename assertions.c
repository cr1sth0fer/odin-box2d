#define _CRT_SECURE_NO_WARNINGS

#include ".box2c/include/box2d/box2d.h"

#ifdef ASSERTIONS

#include "stdio.h"

#define ODIN_ASSERT(_C_Type_, _Odin_Type_) fprintf(file, "#assert(size_of(%s) == %llu)\n", _Odin_Type_, sizeof(_C_Type_))

int main()
{
    FILE* file = fopen("assertions.odin", "w");
    fprintf(file, "package box2d\n\n");

    ODIN_ASSERT(b2Version, "Version");
    ODIN_ASSERT(b2Timer, "Timer");
    ODIN_ASSERT(b2RayCastInput, "Ray_Cast_Input");
    ODIN_ASSERT(b2ShapeCastInput, "Shape_Cast_Input");
    ODIN_ASSERT(b2CastOutput, "Cast_Output");
    ODIN_ASSERT(b2MassData, "Mass_Data");
    ODIN_ASSERT(b2Circle, "Circle");
    ODIN_ASSERT(b2Polygon, "Polygon");
    ODIN_ASSERT(b2Segment, "Segment");
    ODIN_ASSERT(b2SmoothSegment, "Smooth_Segment");
    ODIN_ASSERT(b2Hull, "Hull");
    ODIN_ASSERT(b2SegmentDistanceResult, "Segment_Distance_Result");
    ODIN_ASSERT(b2DistanceProxy, "Distance_Proxy");
    ODIN_ASSERT(b2DistanceCache, "Distance_Cache");
    ODIN_ASSERT(b2DistanceInput, "Distance_Input");
    ODIN_ASSERT(b2DistanceOutput, "Distance_Output");
    ODIN_ASSERT(b2ShapeCastPairInput, "Shape_Cast_Pair_Input");
    ODIN_ASSERT(b2Sweep, "Sweep");
    ODIN_ASSERT(b2TOIInput, "TOI_Input");
    ODIN_ASSERT(b2TOIState, "TOI_State");
    ODIN_ASSERT(b2TOIOutput, "TOI_Output");
    ODIN_ASSERT(b2ManifoldPoint, "Manifold_Point");
    ODIN_ASSERT(b2Manifold, "Manifold");
    ODIN_ASSERT(b2Manifold, "Manifold");
    ODIN_ASSERT(b2TreeNode, "Tree_Node");
    ODIN_ASSERT(b2DynamicTree, "Dynamic_Tree");
    ODIN_ASSERT(b2WorldId, "World_ID");
    ODIN_ASSERT(b2BodyId, "Body_ID");
    ODIN_ASSERT(b2ShapeId, "Shape_ID");
    ODIN_ASSERT(b2JointId, "Joint_ID");
    ODIN_ASSERT(b2ChainId, "Chain_ID");
    ODIN_ASSERT(b2Vec2, "Vec2");
    ODIN_ASSERT(b2Rot, "Rot");
    ODIN_ASSERT(b2Transform, "Transform");
    ODIN_ASSERT(b2Mat22, "Mat22");
    ODIN_ASSERT(b2AABB, "AABB");
    ODIN_ASSERT(b2RayResult, "Ray_Result");
    ODIN_ASSERT(b2WorldDef, "World_Def");
    ODIN_ASSERT(b2BodyDef, "Body_Def");
    ODIN_ASSERT(b2Filter, "Filter");
    ODIN_ASSERT(b2QueryFilter, "Query_Filter");
    ODIN_ASSERT(b2ShapeDef, "Shape_Def");
    ODIN_ASSERT(b2ChainDef, "Chain_Def");
    ODIN_ASSERT(b2Profile, "Profile");
    ODIN_ASSERT(b2Counters, "Counters");
    ODIN_ASSERT(b2DistanceJointDef, "Distance_Joint_Def");
    ODIN_ASSERT(b2MotorJointDef, "Motor_Joint_Def");
    ODIN_ASSERT(b2MouseJointDef, "Mouse_Joint_Def");
    ODIN_ASSERT(b2PrismaticJointDef, "Prismatic_Joint_Def");
    ODIN_ASSERT(b2RevoluteJointDef, "Revolute_Joint_Def");
    ODIN_ASSERT(b2WeldJointDef, "Weld_Joint_Def");
    ODIN_ASSERT(b2WheelJointDef, "Wheel_Joint_Def");
    ODIN_ASSERT(b2SensorBeginTouchEvent, "Sensor_Begin_Touch_Event");
    ODIN_ASSERT(b2SensorEndTouchEvent, "Sensor_End_Touch_Event");
    ODIN_ASSERT(b2SensorEvents, "Sensor_Events");
    ODIN_ASSERT(b2ContactBeginTouchEvent, "Contact_Begin_Touch_Event");
    ODIN_ASSERT(b2ContactEndTouchEvent, "Contact_End_Touch_Event");
    ODIN_ASSERT(b2ContactHitEvent, "Contact_Hit_Event");
    ODIN_ASSERT(b2ContactEvents, "Contact_Events");
    ODIN_ASSERT(b2BodyMoveEvent, "Body_Move_Event");
    ODIN_ASSERT(b2BodyEvents, "Body_Events");
    ODIN_ASSERT(b2ContactData, "Contact_Data");
    ODIN_ASSERT(b2DebugDraw, "Debug_Draw");

    fclose(file);
    return 0;
}

#endif