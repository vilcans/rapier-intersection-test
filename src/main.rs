use rapier3d::{
    dynamics::{BodyStatus, IntegrationParameters, JointSet, RigidBodyBuilder, RigidBodySet},
    geometry::{BroadPhase, ColliderBuilder, ColliderSet, NarrowPhase},
    na::Point3,
    na::Vector3,
    pipeline::{ChannelEventCollector, PhysicsPipeline},
};

fn main() {
    let mut pipeline = PhysicsPipeline::new();
    let gravity = Vector3::new(0.0, -9.81, 0.0);

    // See https://rapier.rs/docs/user_guides/rust/integration_parameters
    let mut integration_parameters = IntegrationParameters::default();
    integration_parameters.allowed_linear_error = 0.01; // Allow this much penetration
    integration_parameters.set_inv_dt(100.0);

    let mut broad_phase = BroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut joints = JointSet::new();
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    // Initialize the event collector.
    let (contact_send, contact_recv) = crossbeam::channel::unbounded();
    let (proximity_send, proximity_recv) = crossbeam::channel::unbounded();
    let event_handler = ChannelEventCollector::new(proximity_send, contact_send);

    // Ball

    let ball_body = RigidBodyBuilder::new(BodyStatus::Dynamic)
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

    let ball_body_handle = rigid_body_set.insert(ball_body);

    let ball_collider = ColliderBuilder::ball(1.5)
        .translation(0.0, 0.0, 0.0) // relative to rigid body
        .friction(0.8)
        .sensor(false)
        .build();

    let ball_collider_handle =
        collider_set.insert(ball_collider, ball_body_handle, &mut rigid_body_set);

    // Floor

    let floor_body = RigidBodyBuilder::new(BodyStatus::Static).build();
    let floor_body_handle = rigid_body_set.insert(floor_body);
    let floor_collider = ColliderBuilder::trimesh(
        vec![
            Point3::new(-10.0, 0.0, -10.0), // 0
            Point3::new(10.0, 0.0, -10.0),  // 1
            Point3::new(10.0, 0.0, 10.0),   // 2
            Point3::new(-10.0, 0.0, 10.0),  // 3
        ],
        vec![Point3::new(0, 1, 2), Point3::new(1, 2, 3)],
    )
    .build();
    collider_set.insert(floor_collider, floor_body_handle, &mut rigid_body_set);

    for _ in 0..100 {
        let collider = collider_set.get(ball_collider_handle).unwrap();
        let body = rigid_body_set.get(ball_body_handle).unwrap();
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

        while let Ok(proximity_event) = proximity_recv.try_recv() {
            println!("Got proximity event {:?}", proximity_event);
        }

        while let Ok(contact_event) = contact_recv.try_recv() {
            println!("Got contact event {:?}", contact_event);
        }
    }
}
