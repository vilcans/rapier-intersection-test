extern crate itertools;

use crossbeam::Receiver;
use itertools::Itertools;
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
    na::{self, Vector3},
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

    moving_body_handle: RigidBodyHandle,
    moving_collider_handle: ColliderHandle,

    floor_body_handle: RigidBodyHandle,
    floor_collider_handle: ColliderHandle,
}

impl World {
    fn step(&mut self) -> (Vec<ProximityEvent>, Vec<ContactEvent>) {
        {
            let mut floor = self.rigid_body_set.get_mut(self.floor_body_handle).unwrap();
            floor.wake_up();
        }
        {
            let mut moving = self
                .rigid_body_set
                .get_mut(self.moving_body_handle)
                .unwrap();
            moving.wake_up();
        }

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
            proximity_events.push(proximity_event);
        }

        let mut contact_events = Vec::new();
        while let Ok(contact_event) = self.contact_recv.try_recv() {
            contact_events.push(contact_event);
        }
        (proximity_events, contact_events)
    }
}

fn create_world(
    floor_is_mesh: bool,
    floor_status: BodyStatus,
    moving_status: BodyStatus,
    moving_is_sensor: bool,
) -> World {
    let pipeline = PhysicsPipeline::new();
    let gravity = Vector3::new(0.0, -9.81, 0.0);

    let integration_parameters = IntegrationParameters::default();
    let broad_phase = BroadPhase::new();
    let narrow_phase = NarrowPhase::new();
    let joints = JointSet::new();
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    // Initialize the event collector.
    let (contact_send, contact_recv) = crossbeam::channel::unbounded();
    let (proximity_send, proximity_recv) = crossbeam::channel::unbounded();
    let event_handler = ChannelEventCollector::new(proximity_send, contact_send);

    // Floor

    let floor_body = RigidBodyBuilder::new(floor_status).build();
    let floor_body_handle = rigid_body_set.insert(floor_body);
    let floor_collider = if floor_is_mesh {
        ColliderBuilder::trimesh(
            vec![
                Point3::new(-10.0, 0.0, -10.0), // 0
                Point3::new(10.0, 0.0, -10.0),  // 1
                Point3::new(10.0, 0.0, 10.0),   // 2
                Point3::new(-10.0, 0.0, 10.0),  // 3
            ],
            vec![Point3::new(0, 1, 2), Point3::new(1, 2, 3)],
        )
        .build()
    } else {
        ColliderBuilder::cuboid(10.0, 0.1, 10.0).build()
    };
    let floor_collider_handle =
        collider_set.insert(floor_collider, floor_body_handle, &mut rigid_body_set);

    // Moving object

    let moving_body = RigidBodyBuilder::new(moving_status)
        .translation(0.0, 20.0, 0.0)
        .can_sleep(false)
        .build();
    let moving_body_handle = rigid_body_set.insert(moving_body);
    let moving_collider = ColliderBuilder::ball(0.5).sensor(moving_is_sensor).build();
    let moving_collider_handle =
        collider_set.insert(moving_collider, moving_body_handle, &mut rigid_body_set);

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
        moving_body_handle,
        moving_collider_handle,
        floor_body_handle,
        floor_collider_handle,
    }
}

pub fn main() {
    let statuses = vec![
        BodyStatus::Dynamic,
        BodyStatus::Static,
        BodyStatus::Kinematic,
    ];
    for &sensor in &[false, true] {
        if sensor {
            println!("\nMoving object is sensor:");
        } else {
            println!("\nMoving object is not a sensor:");
        }
        for (&floor_status, &moving_status) in statuses.iter().cartesian_product(&statuses) {
            let world = create_world(true, floor_status, moving_status, sensor);
            let result = test_world(world);
            println!(
                "Moving {:?} vs {:?} => {}",
                moving_status, floor_status, result
            );
        }
    }
}

fn test_world(mut world: World) -> String {
    let new_pos = Isometry3::new(Vector3::new(0.0, 0.0, 0.0), na::zero());
    {
        let mut moving = world
            .rigid_body_set
            .get_mut(world.moving_body_handle)
            .unwrap();
        if moving.body_status == BodyStatus::Kinematic {
            moving.set_next_kinematic_position(new_pos);
        } else {
            moving.set_position(new_pos)
        }
    }

    let (proximity_events, contact_events) = world.step();
    assert_eq!(contact_events.len(), 0);
    assert_eq!(proximity_events.len(), 0);

    /*{
        let moving = world.rigid_body_set.get(world.moving_body_handle).unwrap();
        assert_eq!(new_pos, moving.position, "Moving object should have moved");
    }*/

    let (proximity_events, contact_events) = world.step();
    //assert_eq!(contact_events.len(), 0);
    match (proximity_events.len(), contact_events.len()) {
        (1, 0) => "Proximity".to_string(),
        (0, 1) => "Contact".to_string(),
        (0, 0) => "None".to_string(),
        (p, c) => panic!(
            "Strange number of proximity events: {} and contact events: {}",
            p, c
        ),
    }
}
