
define dropoff position 
(

  obstacle_return 0           	# Can hit things.
  laser_return 0                 # reflects laser beams
  ranger_return 0             	# reflects sonar beams
  blobfinder_return 0         	# Seen by blobfinders  
  fiducial_return 0           	# Seen as "1" fiducial finders

  localization "gps"         	  

  size [100 16 0.01]

  # body bottom
  block 
  (
    points 4
    point[0] [ 0.0 0.0 ] 
    point[1] [ 1.0 0.0 ] 
    point[2] [ 1.0 1.0 ] 
    point[3] [ 0.0 1.0 ] 
    z [ 0.0 0.05 ]
	 color "red"
  )

 )
