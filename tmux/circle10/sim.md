1. rbl_controller:
  limited_fov: true
  only_2d: false
  ciri: true
  lidar_tilt: 20.0 # [deg]
  lidar_fov: 59.0 #[deg]
  step_size: 0.2
  boundary_threshold: 2.0
  z_min: 0.0
  z_max: 25.0
  z_ref: 1.0
  d1: 1.0
  d2: 1.0
  d3: 1.0
  d4: 1.0
  d5: 1.0
  d6: 1.0
  d7: 1.0
  radius: 8.0 # max planning horizon
  betaD: 0.5
  beta_min: 0.1
  dt: 0.1
  encumbrance: 0.2
  use_z_rule: false
  cwvd_obs: 0.99
  cwvd_rob: 0.99
  use_garmin_alt: false
  replanner: false
  boundary_threshold_speed: 1.0
  move_centroid_to_sensed_cell: true
  
2. encumbrance: 0.5
3. encumbrance: 1.0
4. limited_fov: false
  only_2d: false
  ciri: true
  lidar_tilt: 0.0 # [deg]
  lidar_fov: 360.0 #[deg]
  encumbrance: 0.2
5. encumbrance: 0.5
6. encumbrance: 1.0
7. only_2d: true
   encumbrance: 0.2
8. encumbrance: 0.5
9. encumbrance: 1.0
10. 1. with obs
11. 2. with obs
