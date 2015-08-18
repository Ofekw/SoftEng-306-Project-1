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

define animal position
(
  color "gray50"
  drive "diff"		 	# Differential steering model.

  obstacle_return 1           	# Can hit things.
  ranger_return 1             	# reflects sonar beams
  fiducial_return 1           	# Seen as "1" fiducial finders

  localization "gps"         	  

  size [1.5 0.5 1.0]
  #origin [-0.25 -0.25 0 0 ]

  mylaser(pose [ 0 0.000 -0.750 0.000 ])
  velocity_bounds [-1000 1000 -1000 1000 -1000 1000 -90 90 ]

  # body
  block
  (
	points 4
	point[0] [0 0]
	point[1] [0 1]
	point[2] [0.65 1]
	point[3] [0.65 0]
	z [0.45 1]
	color "black"
  )

  # neck
  block
  (
	points 4
	point[0] [0.65 0.25]
	point[1] [0.65 0.75]
	point[2] [0.7 0.75]
	point[3] [0.7 0.25]
	z [0.65 0.9]
	color "brown"
  )

  # head
  block
  (
	points 4
	point[0] [0.7 0]
	point[1] [0.7 1]
	point[2] [1 1]
	point[3] [1 0]
	z [0.6 1]
	color "brown"
  )

  # leg 1
  block
  (
	points 4
	point[0] [0 0]
	point[0] [0 0.2]
	point[0] [0.1 0.2]
	point[0] [0.1 0]
	z [0 0.45]
  )

  # leg 2
  block
  (
	points 4
	point[0] [0 0.8]
	point[1] [0 1]
	point[2] [0.1 1]
	point[3] [0.1 0.8]
	z [0 0.45]
  )

  # leg 3
  block
  (
	points 4
	point[0] [0.55 0.8]
	point[1] [0.55 1]
	point[2] [0.65 1]
	point[3] [0.65 0.8]
	z [0 0.45]
  )

  # leg 4
  block
  (
	points 4
	point[0] [0.55 0]
	point[1] [0.55 0.2]
	point[2] [0.65 0.2]
	point[3] [0.65 0]
	z [0 0.45]
  )

)
