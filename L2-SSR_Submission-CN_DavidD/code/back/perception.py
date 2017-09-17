import numpy as np
import cv2

#*********************************************************************
# Added by David, Dong 

scale = 10 # 1 pix means 1 x 1 meter in the world map

# Add the Rock Thresholding function
hsv_lower_yellow = np.array([93, 173, 131])
hsv_upper_yellow = np.array([98, 255, 182])

def rock_thresh(wp_img):
    hsv = cv2.cvtColor(wp_img, cv2.COLOR_BGR2HSV)
    # Threshold the HSV image to get only yellow colors
    mask = cv2.inRange(hsv, hsv_lower_yellow, hsv_upper_yellow)
    #bitwise-AND: overlay the mask image and original image
    res = cv2.bitwise_and(wp_img, wp_img, mask = mask)
    return mask, res

#*********************************************************************

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
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
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5 
        # Set a bottom offset to account for the fact that the bottom of the image
        # is not the position of the rover but a bit in front of it
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                          [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                          [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                          [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                          ])
    
    # 2) Apply perspective transform
    wponly_img = perspect_transform(Rover.img,source,destination) 
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    wp_th_imgNavi = color_thresh(wponly_img, rgb_thresh=(160, 160, 160)) # Navigation area
    wp_th_imgObs = 1-wp_th_imgNavi # Obstacles area
        #wp_th_imgObs = ~wp_th_imgNavi
    wp_th_imgRock_mask, wp_th_imgRock_res = (rock_thresh(wponly_img))
    wp_th_imgRock_mask = (wp_th_imgRock_mask/255).astype(np.uint8) # Rock samples
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    y_navi, x_navi = wp_th_imgNavi.nonzero()
    y_obs, x_obs = wp_th_imgObs.nonzero()
    y_rock, x_rock = wp_th_imgRock_mask.nonzero()
    Rover.vision_image[:,:,0] =  wp_th_imgObs*255
    Rover.vision_image[:,:,1] = wp_th_imgRock_mask*255
    Rover.vision_image[:,:,2] = wp_th_imgNavi*255
    
    
    # 5) Convert map image pixel values to rover-centric coords
    x_obs_rc, y_obs_rc = rover_coords(wp_th_imgObs)
    x_rock_rc, y_rock_rc = rover_coords(wp_th_imgRock_mask)
    x_navi_rc, y_navi_rc = rover_coords(wp_th_imgNavi)
    
    # 6) Convert rover-centric pixel values to world coordinates
    x_navi_world, y_navi_world = pix_to_world(x_navi_rc, y_navi_rc, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale)
    x_obs_world, y_obs_world = pix_to_world(x_obs_rc, y_obs_rc, Rover.pos[0], 
                                Rover.pos[1], Rover.yaw,  
                                    Rover.worldmap.shape[0], scale)
    x_rock_world, y_rock_world = pix_to_world(x_rock_rc, y_rock_rc, Rover.pos[0], 
                                Rover.pos[1], Rover.yaw, 
                                    Rover.worldmap.shape[0], scale)
    
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    if (abs(Rover.pitch < 10)) & (abs(Rover.roll < 10)):
        Rover.worldmap[y_obs_world, x_obs_world, 0] += 1
        Rover.worldmap[y_rock_world, x_rock_world, 1] += 1
        Rover.worldmap[y_navi_world, x_navi_world, 2] += 1
        
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(x_navi_rc, y_navi_rc)
      
    return Rover

