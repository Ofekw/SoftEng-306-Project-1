define mylaser ranger
(
  sensor(
  range [ 0.0 5.0 ] # range
  fov 180       # field of vision
  samples 180   # samples 
  )
  # generic model properties
  color "black"
  size [ 0.5 0.05 0.1 ]
  
)

define tractor position 
(
  color "gray50"
  drive "diff"		 	# Differential steering model.

  obstacle_return 0           	# Can hit things.
  laser_return 1                 # reflects laser beams
  ranger_return 1             	# reflects sonar beams
  blobfinder_return 1         	# Seen by blobfinders  
  fiducial_return 1           	# Seen as "1" fiducial finders

  localization "gps"         	  

  size [3.5 2.5 2.6]
  #origin [-0.25 -0.25 0 0 ]

  mylaser(pose [ 0.000 0.000 -2.5 0.000 ])
  velocity_bounds [-1000 1000 -1000 1000 -1000 1000 -90 90 ]

  # wheel 1
  block
  (
	points 4
	point[0] [-0.15 0]
	point[1] [-0.15 0.25]
	point[2] [0.27 0.25]
	point[3] [0.27 0]
	z [0 0.5]
	color "black"
  )

  # wheel 2
  block
  (
	points 4
	point[0] [-0.15 0.75]
	point[1] [-0.15 1]
	point[2] [0.27 1]
	point[3] [0.27 0.75]
	z [0 0.5]
	color "black"
  )

  # wheel 3
  block
  (
	points 4
	point[0] [0.7 0.15]
	point[1] [0.7 0.35]
	point[2] [0.97 0.35]
	point[3] [0.97 0.15]
	z [0 0.3]
	color "black"
  )

  # wheel 4
  block
  (
	points 4
	point[0] [0.7 0.65]
	point[1] [0.7 0.85]
	point[2] [0.97 0.85]
	point[3] [0.97 0.65]
	z [0 0.3]
	color "black"
  )

  # first half body base
  block
  (
	points 4
	point[0] [0 0.25]
	point[1] [0 0.75]
	point[2] [0.5 0.75]
	point[3] [0.5 0.25]
	z [0.2 0.284]
	color "dark green"
  )

  # back of seat
  block
  (
	points 4
	point[0] [0 0.25]
	point[1] [0 0.75]
	point[2] [0.25 0.75]
	point[3] [0.25 0.25]
	z [0.284 0.826]
	color "dark green"
  )

  # second half body
  block
  (
	points 4
	point[0] [0.5 0.25]
	point[1] [0.5 0.75]
	point[2] [1 0.65]
	point[3] [1 0.35]
	z [0.2 0.55]
	color "dark green"
  )

  # worker torso
  block
  (
	points 4
	point[0] [0.28 0.35]
	point[1] [0.28 0.67]
	point[2] [0.39 0.67]
	point[3] [0.39 0.35]
	z [0.2 0.6]
	color "ivory"
  )

  # neck
  block
  (
	points 4
	point[0] [0.28 0.456]
	point[1] [0.28 0.563]
	point[2] [0.39 0.563]
	point[3] [0.39 0.456]
	z [0.6 0.65]
	color "pink"
  )

  # head
  block
  (
	points 4
	point[0] [0.28 0.4236]
	point[1] [0.28 0.5964]
	point[2] [0.39 0.5964]
	point[3] [0.39 0.4236]
	z [0.65 0.8]
	color "pink"
  )

  # hat part1
  block
  (
	points 4
	point[0] [0.28 0.4236]
	point[1] [0.28 0.5964]
	point[2] [0.39 0.5964]
	point[3] [0.39 0.4236]
	z [0.8 0.9]
	color "ivory"
  )

  # hat part2
  block
  (
	points 4
	point[0] [0.39 0.4236]
	point[1] [0.39 0.5964]
	point[2] [0.441 0.5964]
	point[3] [0.441 0.4236]
	z [0.8 0.85]
	color "ivory"
  )
)

	
