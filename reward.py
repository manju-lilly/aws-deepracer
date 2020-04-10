
import math 
def reward_function(params):
    '''
    Example of rewarding the agent to follow center line
    
    {'all_wheels_on_track': True, 'x': 2.5, 'y': 0.75, 'distance_from_center': 0, 'heading': 0, 'progress': 0, 'steps': 1, 'speed': 0.5, 'steering_angle': 6, 'track_width': 0.2, 'waypoints': [[2.5, 0.75], [3.33, 0.75], [4.17, 0.75], [5.0, 0.75], [5.83, 0.75], [6.67, 0.75], [7.5, 0.75], [8.33, 0.75], [9.17, 0.75], [9.75, 0.94], [10.0, 1.5], [10.0, 1.875], [9.92, 2.125], [9.58, 2.375], [9.17, 2.75], [8.33, 2.5], [7.5, 2.5], [7.08, 2.56], [6.67, 2.625], [5.83, 3.44], [5.0, 4.375], [4.67, 4.69], [4.33, 4.875], [4.0, 5.0], [3.33, 5.0], [2.5, 4.95], [2.08, 4.94], [1.67, 4.875], [1.33, 4.69], [0.92, 4.06], [1.17, 3.185], [1.5, 1.94], [1.6, 1.5], [1.83, 1.125], [2.17, 0.885]], 'closest_waypoints': [0, 1], 'is_left_of_center': True, 'is_reversed': True, 'track_length': 16.635021275568313, 'closest_objects': [0, 1], 'objects_location': [[4.511289152034186, 1.3292364463761641], [6.537302737755836, 1.4140104486149618], [4.752976532490525, 3.1350845729056838], [3.103370840828792, 4.133062703357412], [0.7094212601659824, 4.217507179944688], [1.5996645329306798, 1.7124666440529925]], 'objects_left_of_center': [True, True, False, True, False, True], 'object_in_camera': True, 'objects_speed': [0.2, 0.2, 0.2, 0.2, 0.2, 0.2], 'objects_heading': [2.717322296283114, 2.368920328229894, -1.9305073380382327, -1.3586571052564007, 0.00025041038280975884, 0.5573269195381345], 'objects_distance': [1.9501724549506574, 4.139700164331426, 7.997164727523002, 10.024219705986598, 12.56561517097048, 15.090712948383509], 'is_crashed': False, 'is_offtrack': True}
    
    '''
     ## initialize constants
    SMOOTH_STEERING_ANGLE_THRESHOLD = 15
    MAX_STEERING_ANGLE = 30
    REWARD_MAX =  100
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
    heading = params["heading"]
    
    steering = abs(params['steering_angle'])
    ## case no reward reward_function
    """
    When car is crash or bad behavior, track_car_direction is wrong or is_reversed is true or speed is very high
    """
    reward = float(REWARD_MIN)
    w1 = 5
    w2 = 0.5

    ## off the track
    if not (all_wheels_on_track or is_reversed):
        reward = -10 - w1 * speed
        
    ## on the track
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width

    if distance_from_center >= 0.0 and distance_from_center <= marker_1:
        reward = 1 + w2 * speed
    
    elif distance_from_center <= marker_2:
        reward = 0.5 + w2 * speed
    elif distance_from_center <= marker_3:
        reward = 0.1 + w2 * speed
    
    ## prevent zigzag

    # Steering penality threshold, change the number based on your action space setting
    ABS_STEERING_THRESHOLD = 15

    # Penalize reward if the agent is steering too much
    if steering > ABS_STEERING_THRESHOLD:
        reward = 0.8 + w2 * speed

    return reward
        

