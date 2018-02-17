## Project: Search and Sample Return

[//]: # (Image References)

[image1]: ./misc/test_mapping_output_preview.png
[image2]: ./misc/graphics_quality.png

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

Please check the notebook for my modifications

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result.

The function `process_image()` takes a 3d input video and replots it as a 2d overhead map.
The first step is to outline the dimensions of both source and destination images. I used an interactive matplotlib window to determine the coordinates for the source image.

In order to apply a perspective transformation, I simply called the `perspect_transform()` function defined earlier in the notebook.

Next, I created three masks: one which tries to isolate yellow pixels (rock samples), and another that tries to isolate light coloured pixels (terrain).
The third mask (`visible_terrain_mask`) isolates only pixels that are within the camera's view. This third mask will be applied to the navigable terrain view, so the rover won't get confused by data outside of its current field of vision.

Then I applied these masks to the warped image to create three different layers and then called the `rover_coords()` function on them to get the rover-centric x and y coordinates.

In step #5 I use the `pix_to_world()` function to get the world coordinates, using the x, y and yaw values read in from the csv input.

These coordinates can then be plugged into the `data.worldmap`. Each channel containing a different map overlay.
Except for the text. I didn't really change any of the visuals output to the screen in step #7.

![preview of process image() analysis][image1]

Find the video using my test data in `./output/test_mapping_sample.mp4`

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

My `perception_step()` is very similar to `process_image()` function outlined above. The main difference is that unlike the csv data, the data from the rover occasionally suffers from bad readings due to fluctuations in roll and pitch.

Therefore I added a `should_update_map()` function that is called in step #7 that tries to determine whether the telemetry readings it has are reliable enough or not. The function does this simply by setting threshold values. If the roll or pitch angles exceed these values, then `should_update_map()` returns false, and we refrain from updating the map values until we get more reliable data.

```python
def should_update_map(roll, pitch):
    if not is_valid_angle(roll, 1.0):
        return False

    if not is_valid_angle(pitch, 1.5):
        return False

    return True

def is_valid_angle(angle, threshold):
    if angle < threshold or angle > (360 - threshold):
        return True
    else:
        return False
```

My `decision_step()` function has been refactored quite a bit to make it easier for me to understand the different courses of action and later so I could add my own decision branches.
One problem I ran into early on was that the rover would bump into small rocks that were too small for it to recognise as obstacles and it would continue trying to drive forward. To alleviate this I added code to change the rover's mode from `forward` to `stuck` if the rover had full throttle, but had not changed its velocity in awhile.

For this I needed a way to time how long the rover had been still, so I made a mini class for the timer in `driver_rover.py`:

```python
class ActionTimer:
    def __init__(self, delta_sec):
        self.delta_sec = delta_sec
        self.start_time = None

    def start(self):
        if self.start_time is not None:
            return False # already running so can't start

        self.start_time = time.time()
        return True

    def timeout(self):
        now = time.time()
        if ((now - self.start_time) > self.delta_sec):
            self.start_time = None ## reset for next use
            return True

        return False
```

In `RoverState`'s `__init__` function I initialized the timer with a value of 2s and then start when we want to start timing an action 

So anytime the rover was in a `forward` mode, I added code that would check the throttle, velocity and the timer and update the rover's state to `stuck` if it looked like its wheels were spinning at full throttle but it wasn't going anywhere.

```python
if Rover.throttle == Rover.throttle_set and Rover.vel == 0:
        if Rover.action_timer.timeout():
            Rover.mode = 'stuck'
```

Next I added some code to handle the `stuck` mode and also to trigger a `reverse` mode. The function `handle_stuck_state()` simply calls the `advance()` function (which is also used for moving forward), but with negative throttle:

```python
def advance(Rover, throttle):
    # Set throttle to given value
    Rover.throttle = throttle
    # Release the brake
    Rover.brake = 0
    Rover.mode = 'forward'

    nav_angles = average_nav_angle(Rover)

    # if we're going backwards try to go straight back
    if throttle < 0:
        Rover.mode = 'reverse'
        nav_angles = 0

    Rover.steer = np.clip(nav_angles, -15, 15) # clipped to the range +/- 15
```

Finally I added a `handle_reverse_state()` function that checks to see if the rover has successfully moved backwards, and then tries to turn around after the timer expires.

```python
def handle_reverse_state(Rover):
    # in case the Rover gets stuck going backwards
    if Rover.throttle == Rover.throttle_set and Rover.vel == 0:
        if Rover.action_timer.timeout():
            Rover.mode = 'forward'
            Rover.throttle = 0
            return turn_around(Rover)

    if Rover.vel < Rover.threshold_reverse_velocity:
        # we want to know if we succeeded in backing up, but it might not be time to turn around yet
        Rover.has_reached_threshold_reverse_velocity = True

    if Rover.has_reached_threshold_reverse_velocity and Rover.action_timer.timeout():
        Rover.mode = 'forward'
        Rover.throttle = 0
        Rover.has_reached_threshold_reverse_velocity = False # reset
        return turn_around(Rover)

    return advance(Rover, -1.0)
```

The second big problem I had with the default behaviour of the rover is that it had no idea where it had been already, so sometimes it would get stuck in a loop revisting the same area over and over again.
In order to try to improve this I added a `seenmap` to `RoverState`, which would be a numpy bool array that contains 1s anywhere the rover had been, and 0s anywhere it had not yet.

Back in `perception_step()` I call a new function called `update_nav_weights()` using the navigable world coordinates to reduce the weight of any pixels we have already visted so that when we try to determine which angle to turn in later we can reduce their probability of being visited again.

```python
def update_nav_weights(Rover, x_pixels, y_pixels):
    if (len(x_pixels) == 0):
        Rover.nav_weights = np.array([])
        return

    updated_weights = np.ones_like(x_pixels, dtype=np.float)
    for i, (x, y) in enumerate(zip(x_pixels, y_pixels)):
        if Rover.seenmap[y, x]:
            updated_weights[i] = updated_weights[i] * 0.5

    Rover.nav_weights = updated_weights
```

The angle we chose to take back in `decision_step()` can then take advantage of this weight array when we take an average of all the possible explorable angles:

```python
np.average(rad_to_deg(Rover.nav_angles), weights=Rover.nav_weights)
```

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.

Here is a youtube link to the rover autonomously navigating the terrain using my code: [https://youtu.be/MLeakP3-kVs](https://youtu.be/MLeakP3-kVs)

I focused mostly on trying to get the rover to map as much terrain as possible by reducing the chance it would bump into things and get stuck, and trying to encourage it to explore areas it had never been before.
The improvements I made, however, are still a bit naive. The rover still sometimes drives in circles, straying only a few map points from where it had last driven. Improving computer vision by tweaking the navigable terrain mask so the rover can be aware of more possible routes might help a bit in this regard.

The program could be optimised even further by getting the rover to move towards rock samples when they enter its field of vision, since the rover must be very close to the sample for the `near_sample` flag to be triggered. As is, it's very unlikely that this rover will ever pick up a sample.

If I were to persue this project more I would like to get the rover to try to find overlap between the navigable view and the rock sample view, and then prioritise the pixels between it over others by increasing their weight in `Rover.nav_weights`. This would likely increase the chance that the rover would get close enough to the samples to pick them up.

Settings I used running the simulator:

* Screen resolution: 1440 x 900
* Graphics Quality: Good
* FPS: 39

![settings used to run the simulator][image2]
