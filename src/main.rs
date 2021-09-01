use rapier2d::prelude::*;
use std::io::Write;
use std::fs::File;

fn main() {
  let mut rigid_body_set = RigidBodySet::new();
  let mut collider_set = ColliderSet::new();

  /* Create the bouncing ball. */
  let rigid_body = RigidBodyBuilder::new_dynamic()
          .translation(vector![0.0, 10.0])
          .rotation(std::f32::consts::PI / 4.0)
          .build();
  let collider = ColliderBuilder::cuboid(0.5, 0.5).build();
  let ball_body_handle = rigid_body_set.insert(rigid_body);
  collider_set.insert_with_parent(collider, ball_body_handle, &mut rigid_body_set);

  /* Create other structures necessary for the simulation. */
  let gravity = vector![0.0, 0.0];
  let integration_parameters = IntegrationParameters::default();
  let mut physics_pipeline = PhysicsPipeline::new();
  let mut island_manager = IslandManager::new();
  let mut broad_phase = BroadPhase::new();
  let mut narrow_phase = NarrowPhase::new();
  let mut joint_set = JointSet::new();
  let mut ccd_solver = CCDSolver::new();
  let physics_hooks = ();
  let event_handler = ();

  let mut file = File::create("test_results.csv").unwrap();

  /* Run the game loop, stepping the simulation once per frame. */
  for _ in 0..20000 {

      let pos = rigid_body_set[ball_body_handle].position().clone();
    rigid_body_set[ball_body_handle].apply_force_at_point(pos * vector![0.0, -0.5], pos * point![0.0, -0.5], true);
    physics_pipeline.step(
      &gravity,
      &integration_parameters,
      &mut island_manager,
      &mut broad_phase,
      &mut narrow_phase,
      &mut rigid_body_set,
      &mut collider_set,
      &mut joint_set,
      &mut ccd_solver,
      &physics_hooks,
      &event_handler,
    );

    let ball_body = &rigid_body_set[ball_body_handle];
    write!(&mut file, 
      "{},{}\n",
      ball_body.rotation().angle(),
      ball_body.angvel()
    );
  }
}
