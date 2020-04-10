
import math 
def reward_function(params):
    '''
    Example of rewarding the agent to follow center line
    
    {'all_wheels_on_track': True, 'x': 2.5, 'y': 0.75, 'distance_from_center': 0, 'heading': 0, 'progress': 0, 'steps': 1, 'speed': 0.5, 'steering_angle': 6, 'track_width': 0.2, 'waypoints': [[2.5, 0.75], [3.33, 0.75], [4.17, 0.75], [5.0, 0.75], [5.83, 0.75], [6.67, 0.75], [7.5, 0.75], [8.33, 0.75], [9.17, 0.75], [9.75, 0.94], [10.0, 1.5], [10.0, 1.875], [9.92, 2.125], [9.58, 2.375], [9.17, 2.75], [8.33, 2.5], [7.5, 2.5], [7.08, 2.56], [6.67, 2.625], [5.83, 3.44], [5.0, 4.375], [4.67, 4.69], [4.33, 4.875], [4.0, 5.0], [3.33, 5.0], [2.5, 4.95], [2.08, 4.94], [1.67, 4.875], [1.33, 4.69], [0.92, 4.06], [1.17, 3.185], [1.5, 1.94], [1.6, 1.5], [1.83, 1.125], [2.17, 0.885]], 'closest_waypoints': [0, 1], 'is_left_of_center': True, 'is_reversed': True, 'track_length': 16.635021275568313, 'closest_objects': [0, 1], 'objects_location': [[4.511289152034186, 1.3292364463761641], [6.537302737755836, 1.4140104486149618], [4.752976532490525, 3.1350845729056838], [3.103370840828792, 4.133062703357412], [0.7094212601659824, 4.217507179944688], [1.5996645329306798, 1.7124666440529925]], 'objects_left_of_center': [True, True, False, True, False, True], 'object_in_camera': True, 'objects_speed': [0.2, 0.2, 0.2, 0.2, 0.2, 0.2], 'objects_heading': [2.717322296283114, 2.368920328229894, -1.9305073380382327, -1.3586571052564007, 0.00025041038280975884, 0.5573269195381345], 'objects_distance': [1.9501724549506574, 4.139700164331426, 7.997164727523002, 10.024219705986598, 12.56561517097048, 15.090712948383509], 'is_crashed': False, 'is_offtrack': True}
    
    '''
     ## initialize constants
    SMOOTH_STEERING_ANGLE_THRESHOLD = 15
    MAX_STEERING_ANGLE = 30
    REWARD_MAX =  9000 ## 10000
    MAX_SPEED = float(3.0)
    MIN_SPEED = float(1.5)
    REWARD_MIN = 1e-3
    SAFE_HORIZONTAL_DISTANCE = 0.8


    # Read input parameters
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    speed = params["speed"]
    all_wheels_on_track  = params["all_wheels_on_track"]
    car_x = params["x"]
    car_y = params["y"]
    steering_angle = params['steering_angle']
    is_left_of_center = params["is_left_of_center"]
    is_reversed = params["is_reversed"]
    heading = params["heading"]
    steps = params["steps"]
    waypoints = params["waypoints"]
    closest_waypoints = params["closest_waypoints"]
    nearest_prev_waypoint_indx = params["closest_waypoints"][0]
    nearest_next_waypoint_index =  params["closest_waypoints"][1]
    progress = params["progress"]
    
    ## compute additional parameters
    ## check if the car is heading is the right direction - using the prev and next wayppoints
    # helper functions
    
    get_waypoint = lambda wayppoints, index: wayppoints[index % len(wayppoints)]
    get_angle_between_waypoints = lambda previous_point,next_point: math.atan2(next_point[1] - previous_point[1], next_point[0]-previous_point[0])
    
    
    ## previous waypoint
    previous_point = get_waypoint(waypoints,nearest_prev_waypoint_indx)
    
    # next wayppoint
    next_point = get_waypoint(waypoints, nearest_next_waypoint_index)
    
    ## check the direction of the car
    track_direction = get_angle_between_waypoints(previous_point,next_point)
    track_direction = math.degrees(track_direction)
    heading_direction = params["heading"]
    
    ## check if the car is in the turn 
    angle_ahead = get_angle_between_waypoints(get_waypoint(waypoints,nearest_prev_waypoint_indx),get_waypoint(waypoints,nearest_prev_waypoint_indx+1))
    angle_behind = get_angle_between_waypoints(get_waypoint(waypoints,nearest_prev_waypoint_indx-1),get_waypoint(waypoints,nearest_prev_waypoint_indx))
    diff = (angle_ahead - angle_behind)
    turn_angle = diff +360 if diff < -90 and diff > 90 else (-180 + (diff-180) if diff > 180 else ( 180 - (diff+180) if diff < -180 else diff ))
    
    dx = distance_from_center - car_x
    dy = distance_from_center - car_y
    
   
    
    ## case no reward reward_function
    """
    When car is crash or bad behavior, track_car_direction is wrong or is_reversed is true or speed is very high
    
    """
    if not (all_wheels_on_track or is_reversed or speed < (0.1 * MAX_SPEED)):
        return float(REWARD_MIN)
        
    ## we are heading in the right direction, no bad steering,
    threshold_heading_alignment = 10
    if abs(track_direction - heading) < threshold_heading_alignment:
        return REWARD_MAX * 0.30
        
    ## if steering angle is less than SMOOTH_STEERING_ANGLE_THRESHOLD - 
    if steering_angle < SMOOTH_STEERING_ANGLE_THRESHOLD:
        return REWARD_MAX * 0.15
  

    ## Optimum speed and is in turn 
    if turn_angle < 3 and speed <  MAX_STEERING_ANGLE*0.67:
        return REWARD_MAX * 0.6
    
    ## if progress well
    TOTAL_NUM_STEPS = 150
    if steps % 100 == 0 and progress >  (steps/TOTAL_NUM_STEPS):
        return REWARD_MAX * 0.4
    
    ## car has reached the final waypoint 
    if nearest_prev_waypoint_indx == len(waypoints)-1:
        print("Reached target")
        return REWARD_MAX
    
    
