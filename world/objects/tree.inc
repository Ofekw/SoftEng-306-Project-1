
define tree position 
(
  color "gray50"

  obstacle_return 1           	# Can hit things.
  laser_return 1                 # reflects laser beams
  ranger_return 1             	# reflects sonar beams
  blobfinder_return 1         	# Seen by blobfinders  
  fiducial_return 1           	# Seen as "1" fiducial finders

  localization "gps"         	  

  size [5 5 5]
  #origin [-0.25 -0.25 0 0 ]

  # body bottom
  block 
  (
    points 4
    point[0] [ 0.1 0.25 ] 
    point[1] [ 0.9 0.25 ] 
    point[2] [ 0.9 0.75 ] 
    point[3] [ 0.1 0.75 ] 
    z [ 0.0 0.05 ]
	 color "black"
  )

  # body top
  block 
  (
    points 4
    point[0] [ 0.1 0.25 ] 
    point[1] [ 0.9 0.25 ] 
    point[2] [ 0.9 0.75 ] 
    point[3] [ 0.1 0.75 ] 
    color "brown"
    z [ 0.05 1 ]
  )



  # tree canopy
  block
  (
	 points 4
	 point[0] [5 0.0]
	 point[1] [5 5]
	 point[2] [0 5]
	 point[3] [0.0 0.0]
	 z [1 1.05]
  color "green"
  )






)
