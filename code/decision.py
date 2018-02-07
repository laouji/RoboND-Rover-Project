import numpy as np

def try_action(Rover):
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is None:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        return Rover

    return decision_step(Rover)

# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Check for Rover.mode status
    if Rover.mode == 'forward':
        # Check the extent of navigable terrain
        if len(Rover.nav_angles) >= Rover.stop_forward:
            # If mode is forward, navigable terrain looks good
            # and velocity is below max, then throttle
            if Rover.vel < Rover.max_vel:
                # Set throttle value to throttle setting
                advance(Rover, Rover.throttle_set)
            else: # Else coast
                advance(Rover, 0)
        # If there's a lack of navigable terrain pixels then go to 'stop' mode
        elif len(Rover.nav_angles) < Rover.stop_forward:
                ## Set mode to "stop" and hit the brakes!
                apply_break(Rover)

    # If we're already in "stop" mode then make different decisions
    elif Rover.mode == 'stop':
        # If we're in stop mode but still moving keep braking
        if Rover.vel > 0.2:
            apply_brake(Rover)
        # If we're not moving (vel < 0.2) then do something else
        elif Rover.vel <= 0.2:
            # Now we're stopped and we have vision data to see if there's a path forward
            if len(Rover.nav_angles) < Rover.go_forward:
                turn(Rover)
            if len(Rover.nav_angles) >= Rover.go_forward:
                advance(Rover, Rover.throttle_set)

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover

def apply_break(Rover):
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0
    Rover.mode = 'stop'

def turn(Rover):
    Rover.throttle = 0
    # Release the brake to allow turning
    Rover.brake = 0
    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
    Rover.steer = -15 # Could be more clever here about which way to turn

def advance(Rover, throttle):
    # Set throttle back to stored value
    Rover.throttle = throttle
    # Release the brake
    Rover.brake = 0
    # Set steer to mean angle (average angle clipped to the range +/- 15)
    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
    Rover.mode = 'forward'
