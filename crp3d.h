#ifndef __CRP3D_H__
#define __CRP3D_H__

#include <stdint.h>

#ifdef __cplusplus
    #define CRP3D_API extern "C"
#else
    #define CRP3D_API
#endif

typedef float f32;
typedef double f64;
typedef char b8;
typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;
typedef int8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef char* cstring;

#ifdef CRP3D_F64
    typedef f64 decimal;
#else
    typedef f32 decimal;
#endif

typedef struct
{
    decimal x;
    decimal y;
} Vector2;

typedef struct
{
    decimal x;
    decimal y;
    decimal z;
} Vector3;

typedef struct
{
    decimal x;
    decimal y;
    decimal z;
    decimal w;
} Quaternion;

typedef struct
{
    Vector3 position;
    Quaternion rotation;
} Transform;

typedef struct
{
    cstring world_name;
    Vector3 gravity;
    decimal persistent_contact_distance_threshold;
    decimal default_friction_coefficient;
    decimal default_bounciness;
    decimal restitution_velocity_threshold;
    b8 is_sleeping_enabled;
    u16 default_velocity_solver_nb_iterations;
    u16 default_position_solver_nb_iterations;
    f32 default_time_before_sleep;
    decimal default_sleep_linear_velocity;
    decimal default_sleep_angular_velocity;
    decimal cos_angle_similar_contact_manifold;
} World_Settings;

typedef void* Physics_Common;
typedef void* Physics_World;
typedef void* Sphere_Shape;
typedef void* Box_Shape;
typedef void* Capsule_Shape;
typedef void* Collision_Body;
typedef void* Rigid_Body;

CRP3D_API Physics_Common physics_common_create();
CRP3D_API void physics_common_destroy(Physics_Common common);
CRP3D_API Physics_World physics_common_create_world(Physics_Common common, World_Settings* settings);
CRP3D_API void physics_common_destroy_world(Physics_Common common, Physics_World world);
CRP3D_API Sphere_Shape physics_common_create_sphere_shape(Physics_Common common, decimal radius);
CRP3D_API void physics_common_destroy_sphere_shape(Physics_Common common, Sphere_Shape shape);
CRP3D_API Box_Shape physics_common_create_box_shape(Physics_Common common, Vector3 extent);
CRP3D_API void physics_common_destroy_box_shape(Physics_Common common, Box_Shape shape);
CRP3D_API Capsule_Shape physics_common_create_capsule_shape(Physics_Common common, decimal radius, decimal height);
CRP3D_API void physics_common_destroy_capsule_shape(Physics_Common common, Capsule_Shape shape);

CRP3D_API Collision_Body physics_world_create_collision_body(Physics_World world, Transform transform);
CRP3D_API void physics_world_destroy_collision_body(Physics_World world, Collision_Body body);
CRP3D_API cstring physics_world_get_name(Physics_World world);
CRP3D_API Rigid_Body physics_world_create_rigid_body(Physics_World world, Transform transform);
CRP3D_API void physics_world_destroy_rigid_body(Physics_World world, Rigid_Body body);

#endif//__CRP3D_H__