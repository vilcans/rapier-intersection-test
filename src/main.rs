use crossbeam::Receiver;
use rapier3d::{
    dynamics::{
        BodyStatus, IntegrationParameters, JointSet, RigidBodyBuilder, RigidBodyHandle,
        RigidBodySet,
    },
    geometry::{
        BroadPhase, ColliderBuilder, ColliderHandle, ColliderSet, ContactEvent, NarrowPhase,
        ProximityEvent,
    },
    na::Isometry3,
    na::Point3,
    na::Vector3,
    pipeline::{ChannelEventCollector, PhysicsPipeline},
};

#[allow(dead_code)]
struct World {
    pipeline: PhysicsPipeline,
    gravity: Vector3<f32>,
    integration_parameters: IntegrationParameters,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    joints: JointSet,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    proximity_recv: Receiver<ProximityEvent>,
    contact_recv: Receiver<ContactEvent>,
    event_handler: ChannelEventCollector,

    ball_body_handle: RigidBodyHandle,
    ball_collider_handle: ColliderHandle,

    sensor_body_handle: RigidBodyHandle,
    sensor_collider_handle: ColliderHandle,

    floor_body_handle: RigidBodyHandle,
    floor_collider_handle: ColliderHandle,
}

impl World {
    fn step(&mut self) -> (Vec<ProximityEvent>, Vec<ContactEvent>) {
        self.pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.joints,
            &self.event_handler,
        );

        let mut proximity_events = Vec::new();
        while let Ok(proximity_event) = self.proximity_recv.try_recv() {
            println!("Got proximity event {:?}", proximity_event);
            proximity_events.push(proximity_event);
        }

        let mut contact_events = Vec::new();
        while let Ok(contact_event) = self.contact_recv.try_recv() {
            println!("Got contact event {:?}", contact_event);
            contact_events.push(contact_event);
        }
        (proximity_events, contact_events)
    }
}

fn create_world() -> World {
    let pipeline = PhysicsPipeline::new();
    let gravity = Vector3::new(0.0, -9.81, 0.0);

    // See https://rapier.rs/docs/user_guides/rust/integration_parameters
    let mut integration_parameters = IntegrationParameters::default();
    integration_parameters.allowed_linear_error = 0.01; // Allow this much penetration
    integration_parameters.set_inv_dt(100.0);

    let broad_phase = BroadPhase::new();
    let narrow_phase = NarrowPhase::new();
    let joints = JointSet::new();
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

    println!(
        "Ball: body handle {:?} collider handle {:?}",
        ball_body_handle, ball_collider_handle
    );

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
    let floor_collider_handle =
        collider_set.insert(floor_collider, floor_body_handle, &mut rigid_body_set);

    println!(
        "Floor: body handle {:?} collider handle {:?}",
        floor_body_handle, floor_collider_handle
    );

    // Sensor

    let sensor_body = RigidBodyBuilder::new(BodyStatus::Kinematic)
        .translation(0.0, 20.0, 0.0)
        .can_sleep(false)
        .build();
    let sensor_body_handle = rigid_body_set.insert(sensor_body);
    let sensor_collider = ColliderBuilder::ball(0.5).sensor(true).build();
    let sensor_collider_handle =
        collider_set.insert(sensor_collider, sensor_body_handle, &mut rigid_body_set);

    println!(
        "Sensor: body handle {:?} collider handle {:?}",
        sensor_body_handle, sensor_collider_handle
    );
    World {
        pipeline,
        gravity,
        integration_parameters,
        broad_phase,
        narrow_phase,
        joints,
        rigid_body_set,
        collider_set,
        proximity_recv,
        contact_recv,
        event_handler,
        ball_body_handle,
        ball_collider_handle,
        sensor_body_handle,
        sensor_collider_handle,
        floor_body_handle,
        floor_collider_handle,
    }
}

pub fn main() {
    let mut world = create_world();

    for frame in 0..200 {
        {
            {
                let mut sensor = world
                    .rigid_body_set
                    .get_mut(world.sensor_body_handle)
                    .unwrap();
                let y = 20.0 - frame as f32;
                println!("Sensor y {}", y);
                sensor.set_next_kinematic_position(Isometry3::new(
                    Vector3::new(0.0, y, 0.0),
                    Vector3::new(0.0, 0.0, 0.0),
                ));
            }

            if frame == 100 {
                //println!("Adding force");
                //body.apply_impulse(Vector3::new(0.0, 0.0, 1000.0));
                /*
                println!("Moving sensor");
                let mut sensor = rigid_body_set.get_mut(sensor_body_handle).unwrap();
                sensor.set_next_kinematic_position(Isometry3::new(
                    Vector3::new(0.0, 0.0, 0.0),
                    Vector3::new(0.0, 0.0, 0.0),
                ));*/
            }
            let body = world.rigid_body_set.get(world.ball_body_handle).unwrap();
            let collider = world.collider_set.get(world.ball_collider_handle).unwrap();
            println!(
                "Frame {}, collider: {:?} body {:?}",
                frame,
                collider.position().translation,
                body.position.translation
            );
        }

        let (_proximity_events, _contact_events) = world.step();
    }
}
