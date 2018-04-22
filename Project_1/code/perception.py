import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img,mode = 2, rgb_thresh_lower=(160, 160, 160), rgb_thresh_upper=(240,200,30)):
    # mode: 
    # 1 = get rocks 
    # 2 = get ground
    # 3 = get obstacle
    
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    if mode == 1:
        # Tresh for identifying pixels of a rock
        rgb_thresh_lower=(160, 160, 0)
        thresh = (img[:,:,0] > rgb_thresh_lower[0]) & (img[:,:,0] < rgb_thresh_upper[0])\
                    & (img[:,:,1] > rgb_thresh_lower[1]) & (img[:,:,1] < rgb_thresh_upper[1])\
                    & (img[:,:,2] > rgb_thresh_lower[2]) & (img[:,:,2] < rgb_thresh_upper[2])
    elif mode == 2:
        # Tresh for identifying pixels of navigable ground
        thresh = (img[:,:,0] > rgb_thresh_lower[0]) \
                    & (img[:,:,1] > rgb_thresh_lower[1]) \
                    & (img[:,:,2] > rgb_thresh_lower[2])
    elif mode == 3:
        # Tresh for identifying obstacles 
        thresh = (img[:,:,0] < rgb_thresh_lower[0]) \
                    & (img[:,:,1] < rgb_thresh_lower[1]) \
                    & (img[:,:,2] < rgb_thresh_lower[2])
        
    # Index the array of zeros with the boolean array and set to 1
    color_select[thresh] = 1
    #print(color_select)
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
    # check validity of image
    
    img = Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5 
    bottom_offset = 7
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                  [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    warped = perspect_transform(img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # navigavle terrain
    nt_treshed = color_thresh(warped, mode = 2)
    # obstacle 
    o_treshed = color_thresh(warped, mode = 3)
    # Rock sample 
    rs_treshed = color_thresh(warped, mode = 1)
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = o_treshed*255
    Rover.vision_image[:,:,1] = rs_treshed*255
    Rover.vision_image[:,:,2] = nt_treshed*255
    
    # 5) Convert map image pixel values to rover-centric coords
    nt_xpix, nt_ypix = rover_coords(nt_treshed)
    
    o_xpix, o_ypix = rover_coords(o_treshed)
    
    rs_xpix, rs_ypix = rover_coords(rs_treshed)
    # 6) Convert rover-centric pixel values to world coordinates
    world_size = Rover.worldmap.shape[0]
    scale = 2*dst_size
    nt_x_world,nt_y_world = pix_to_world(nt_xpix,nt_ypix,Rover.pos[0],Rover.pos[1],
                                                   Rover.yaw,world_size,scale)
    
    o_x_world,o_y_world = pix_to_world(o_xpix,o_ypix,Rover.pos[0],Rover.pos[1],
                                                   Rover.yaw,world_size,scale)
    
    rs_x_world,rs_y_world = pix_to_world(rs_xpix,rs_ypix,Rover.pos[0],Rover.pos[1],
                                                   Rover.yaw,world_size,scale)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    # To improve on fidelity, check that pitch and roll is close to zero. 
    if Rover.roll <= 3 and Rover.pitch <=1:
        Rover.worldmap[o_y_world, o_x_world, 0] += 1
        Rover.worldmap[rs_y_world, rs_x_world, 1] += 1
        Rover.worldmap[nt_y_world, nt_x_world, 2] += 1
    # 8) Convert rover-centric pixel positions to polar coordinates
    rover_centric_distanses, rover_centric_angles = to_polar_coords(nt_xpix, nt_ypix)
    
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.nav_dist = rover_centric_distanses
    Rover.nav_angles = rover_centric_angles
 
    
    
    return Rover