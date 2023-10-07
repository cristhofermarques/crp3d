#include "crp3d.h"
#include <reactphysics3d/reactphysics3d.h>

/* Conversion */
#define VECTOR2(_Variable_) (reactphysics3d::Vector2(_Variable_.x, _Variable_.y))
#define VECTOR3(_Variable_) (reactphysics3d::Vector3(_Variable_.x, _Variable_.y, _Variable_.z))
#define QUATERNION(_Variable_) (reactphysics3d::Quaternion(_Variable_.x, _Variable_.y, _Variable_.z, _Variable_.w))
#define TRANSFORM(_Variable_) (reactphysics3d::Transform(VECTOR3(_Variable_.position), QUATERNION(_Variable_.rotation)))

#define AS_PHYSICS_COMMON(_Variable_) ((reactphysics3d::PhysicsCommon*)_Variable_)
#define AS_PHYSICS_WORLD(_Variable_) ((reactphysics3d::PhysicsWorld*)_Variable_)
#define AS_SPHERE_SHAPE(_Variable_) ((reactphysics3d::SphereShape*)_Variable_)
#define AS_BOX_SHAPE(_Variable_) ((reactphysics3d::BoxShape*)_Variable_)
#define AS_CAPSULE_SHAPE(_Variable_) ((reactphysics3d::CapsuleShape*)_Variable_)
#define AS_COLLISION_BODY(_Variable_) ((reactphysics3d::CollisionBody*)_Variable_)
#define AS_RIGID_BODY(_Variable_) ((reactphysics3d::RigidBody*)_Variable_)

void To_Vector2(reactphysics3d::Vector2* rp_vector, Vector2* vector)
{
    rp_vector->x = vector->x;
    rp_vector->y = vector->y;
}

void To_Vector3(reactphysics3d::Vector3* rp_vector, Vector3* vector)
{
    rp_vector->x = vector->x;
    rp_vector->y = vector->y;
    rp_vector->z = vector->z;
}

void To_WorldSettings(reactphysics3d::PhysicsWorld::WorldSettings* rp_settings, World_Settings* settings)
{
    rp_settings->worldName = settings->world_name;
    rp_settings->gravity = VECTOR3(settings->gravity);
    rp_settings->persistentContactDistanceThreshold = settings->persistent_contact_distance_threshold;
    rp_settings->defaultFrictionCoefficient = settings->default_friction_coefficient;
    rp_settings->defaultBounciness = settings->default_bounciness;
    rp_settings->restitutionVelocityThreshold = settings->restitution_velocity_threshold;
    rp_settings->isSleepingEnabled = settings->is_sleeping_enabled;
    rp_settings->defaultVelocitySolverNbIterations = settings->default_velocity_solver_nb_iterations;
    rp_settings->defaultPositionSolverNbIterations = settings->default_position_solver_nb_iterations;
    rp_settings->defaultTimeBeforeSleep = settings->default_time_before_sleep;
    rp_settings->defaultSleepLinearVelocity = settings->default_sleep_linear_velocity;
    rp_settings->defaultSleepAngularVelocity = settings->default_sleep_angular_velocity;
    rp_settings->cosAngleSimilarContactManifold = settings->cos_angle_similar_contact_manifold;
}

/* Wrap */
Physics_Common physics_common_create()
{
    return (Physics_Common) new reactphysics3d::PhysicsCommon();
}

void physics_common_destroy(Physics_Common common)
{
    delete AS_PHYSICS_COMMON(common);
}

Physics_World physics_common_create_world(Physics_Common common, World_Settings* settings)
{
    if (settings == NULL)
    {
        return (Physics_World)AS_PHYSICS_COMMON(common)->createPhysicsWorld();
    }
    else
    {
        reactphysics3d::PhysicsWorld::WorldSettings rp_settings;
        To_WorldSettings(&rp_settings, settings);
        return (Physics_World)AS_PHYSICS_COMMON(common)->createPhysicsWorld(rp_settings);
    }
}

void physics_common_destroy_world(Physics_Common common, Physics_World world)
{
    AS_PHYSICS_COMMON(common)->destroyPhysicsWorld(AS_PHYSICS_WORLD(world));
}

Sphere_Shape physics_common_create_sphere_shape(Physics_Common common, decimal radius)
{
    rp3d::decimal rp_radius = (rp3d::decimal)radius;
    return (Sphere_Shape)AS_PHYSICS_COMMON(common)->createSphereShape(rp_radius);
}

void physics_common_destroy_sphere_shape(Physics_Common common, Sphere_Shape shape)
{
    AS_PHYSICS_COMMON(common)->destroySphereShape(AS_SPHERE_SHAPE(shape));
}

Box_Shape physics_common_create_box_shape(Physics_Common common, Vector3 extent)
{
    return (Box_Shape)AS_PHYSICS_COMMON(common)->createBoxShape(VECTOR3(extent));
}

void physics_common_destroy_box_shape(Physics_Common common, Box_Shape shape)
{
    AS_PHYSICS_COMMON(common)->destroyBoxShape(AS_BOX_SHAPE(shape));
}

Capsule_Shape physics_common_create_capsule_shape(Physics_Common common, decimal radius, decimal height)
{
    return (Capsule_Shape)AS_PHYSICS_COMMON(common)->createCapsuleShape(radius, height);
}

void physics_common_destroy_capsule_shape(Physics_Common common, Capsule_Shape shape)
{
    AS_PHYSICS_COMMON(common)->destroyCapsuleShape(AS_CAPSULE_SHAPE(shape));
}

Collision_Body physics_world_create_collision_body(Physics_World world, Transform transform)
{
    return (Collision_Body)AS_PHYSICS_WORLD(world)->createCollisionBody(TRANSFORM(transform));
}

void physics_world_destroy_collision_body(Physics_World world, Collision_Body body)
{
    AS_PHYSICS_WORLD(world)->destroyCollisionBody(AS_COLLISION_BODY(body));
}

cstring physics_world_get_name(Physics_World world)
{
    return (cstring)AS_PHYSICS_WORLD(world)->getName().c_str();
}

Rigid_Body physics_world_create_rigid_body(Physics_World world, Transform transform)
{
    return (Rigid_Body)AS_PHYSICS_WORLD(world)->createRigidBody(TRANSFORM(transform));
}

void physics_world_destroy_rigid_body(Physics_World world, Rigid_Body body)
{
    AS_PHYSICS_WORLD(world)->destroyRigidBody(AS_RIGID_BODY(body));
}