use rapier3d::geometry::{BroadPhase, ColliderSet, NarrowPhase};
use rapier3d::pipeline::PhysicsPipeline;
use rapier3d::{dynamics::BodyStatus, dynamics::RigidBodyBuilder, na::Vector3};
use rapier3d::{
    dynamics::{IntegrationParameters, JointSet, RigidBodySet},
    geometry::ColliderBuilder,
};

fn main() {
    let mut pipeline = PhysicsPipeline::new();
    let gravity = Vector3::new(0.0, -9.81, 0.0);

    let mut integration_parameters = IntegrationParameters::default();
    integration_parameters.set_inv_dt(100.0);

    let mut broad_phase = BroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut joints = JointSet::new();
    // We ignore contact events for now.
    let event_handler = ();

    let rigid_body = RigidBodyBuilder::new(BodyStatus::Dynamic)
        // The rigid body translation.
        // Default: zero vector.
        .translation(0.0, 5.0, 0.0)
        // The rigid body rotation.
        // Default: no rotation.
        //.rotation(Vector3::z() * 5.0)
        /*
        // The rigid body position. Will override `.translation(...)` and `.rotation(...)`.
        // Default: the identity isometry.
        .position(Isometry3::new(
            Vector3::new(1.0, 3.0, 2.0),
            Vector3::y() * std::f32::consts::PI,
        ))
        */
        //.linvel(1.0, 3.0, 4.0)
        //.angvel(Vector3::x() * 3.0)
        .can_sleep(true)
        .build();

    let mut rigid_body_set = RigidBodySet::new();
    let body_handle = rigid_body_set.insert(rigid_body);

    let collider = ColliderBuilder::ball(1.5)
        .translation(0.0, 0.0, 0.0) // relative to rigid body
        .friction(0.8)
        .sensor(false)
        .build();

    let mut collider_set = ColliderSet::new();
    let collider_handle = collider_set.insert(collider, body_handle, &mut rigid_body_set);

    for _ in 0..100 {
        let collider = collider_set.get(collider_handle).unwrap();
        let body = rigid_body_set.get(body_handle).unwrap();
        println!(
            "collider: {:?} body {:?}",
            collider.position().translation,
            body.position.translation
        );
        pipeline.step(
            &gravity,
            &integration_parameters,
            &mut broad_phase,
            &mut narrow_phase,
            &mut rigid_body_set,
            &mut collider_set,
            &mut joints,
            &event_handler,
        );
    }
}
