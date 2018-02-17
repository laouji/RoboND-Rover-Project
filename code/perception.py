import math
import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
#def apply_mask(img, rgb_thresh=(160, 160, 160)):
def apply_mask(img, mask):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])

    # Index the array of zeros with the boolean array and set to 1
    color_select[mask] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel

def update_nav_weights(Rover, x_pixels, y_pixels):
    if (len(x_pixels) == 0):
        Rover.nav_weights = np.array([])
        return

    updated_weights = np.ones_like(x_pixels, dtype=np.float)
    for i, (x, y) in enumerate(zip(x_pixels, y_pixels)):
        if Rover.seenmap[y, x]:
            updated_weights[i] = updated_weights[i] * Rover.seen_weight_multiplier

    Rover.nav_weights = updated_weights

# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))

    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    visible_terrain_mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))# keep same size as input image

    return warped, visible_terrain_mask


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # NOTE: camera image is coming to you in Rover.img

    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset],
                              [Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset],
                              ])

    # 2) Apply perspective transform
    warped, visible_terrain_mask = perspect_transform(Rover.img, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # Require that each pixel be above all three threshold values in RGB
    # masks will now contain a boolean array with "True" where threshold was met
    yellow_mask = (warped[:,:,0] > 110) & (warped[:,:,1] > 110) & (warped[:,:,2] < 50)
    navigable_mask = (warped[:,:,0] > 150) & (warped[:,:,1] > 140) & (warped[:,:,2] > 160)

    rock_view = apply_mask(warped, yellow_mask)
    navigable_view = apply_mask(warped, navigable_mask)
    obstacle_view = np.absolute(np.float32(navigable_view) - 1) * visible_terrain_mask

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacle_view * 255
    Rover.vision_image[:,:,1] = rock_view * 255
    Rover.vision_image[:,:,2] = navigable_view * 255

    # 5) Convert map image pixel values to rover-centric coords
    obstacle_x, obstacle_y = rover_coords(obstacle_view)
    rock_x, rock_y = rover_coords(rock_view)
    navigable_x, navigable_y = rover_coords(navigable_view)

    # 6) Convert rover-centric pixel values to world coordinates
    scale = 10
    navigable_x_world, navigable_y_world = pix_to_world(navigable_x, navigable_y, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale)
    rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale)
    obstacle_x_world, obstacle_y_world = pix_to_world(obstacle_x, obstacle_y, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale)


    # 7) Update Rover worldmap (to be displayed on right side of screen)
    if should_update_map(Rover.roll, Rover.pitch):
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    update_nav_weights(Rover, navigable_x_world, navigable_y_world)

    # 8) Convert rover-centric pixel positions to polar coordinates
    pixel_distances, angles = to_polar_coords(navigable_x, navigable_y)

    # Update Rover pixel distances and angles
    Rover.nav_dists = pixel_distances
    Rover.nav_angles = angles

    return Rover

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
