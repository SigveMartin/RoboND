#Project Submission Notes


Short writeup to address each rubric item of assigment

## Notebook Analysis

### Running and modifying functions of notebook
In order to add rock and obstacle detection I modified the color_thresh function. I added a mode (1: for checking for rocks, 2: navigable terrain and 3: obstacles). I also modified to use two rgb thresh values, an upper and lower. I used a color picking tool to pinpoint the levels based on the color of rocks. 

Mode 1 filters out rocks, in filtering out all pixels that are not "gold". 
Mode 2 filters out all pixels that are below the default thresh values of 160. 
Mode 3 is basically an inverted Mode 2 filter, filtering out all pixels that are higher than 160. 

### Populate the process_image() function
I followed the guids in the text first defining the source and destination points. Then applying the perspective transform. Then creating threshes for terrain (mode 2), obstacles (mode 3) and rock samples (mode 1). Further I converted the thresheolded image pixels to rover-centric coordinates using rover_coords(thresh) function. Then converting these to world coordinates by using the pix_to_world function. I then updated the worldmap with all three threshes giving their world coordinates. 

For the Make mosaic image, I just kept the example code as is. See output folder for test_mapping.mp4. 

## Autonomous Navigation and Mapping

### Filling in perception_step() and decision_step()

#### perception_step()
I built on the code from the notebook and followed much of the same lines. However now applying it to the Rover object. I iterated some on the source and destination. Especially the bottom_offset by a slight increase in order to imrpove the Rovers ability to not get stuck. In addition to this I also wrapped the update of Rover worldmap within a check on roll and pitch in order to improve fidelity. This had very good results. I iterated on the values, and found that roll <=3 and pitch <= 1 gave a good result. 

#### desicion_step()
I implemented a strategy of hugging the left wall. This I did by adding a bias of 11.5 to the stearing:
		bias = 11.5
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi)+bias, -15, 15) 
I also added a new parameter self.prev_pos to Rover, and made sure Rover.prev_pos = Rover.pos in update_rover in the support_functions.py before updating Rover.pos to the current position. I used this in decision_step() to improve the handling when the rover get stuck. Somethimes the Rover got stuck while it thought the navigable terrain looked good. So then (in addition to adjusting the bottom ofset) I ckecked if the velosity was unaffected and position the same after it had given throttle. If so, then tried to stear right and change mode to stop. In all cases where it has stopped I turned right -15 as we favour the left wall it is likely that the right is the best way to navigable terrain. This was not always ture of course, and could have been improved. 

### Launching in autonomous mode 
I tested and iterated alot in autonomous mode.I run it on OSX El Capitan, with 1024x768 resolution and Good Graphics Quality..  I could have given the Rover better brains, but stoped hear due to time constraints.See screen capture of reference runhere: https://youtu.be/WGR3ko0fcE8

I increased max speed to 2.5 and after updating the world map update function to only map when roll and pitch close to 0 I got a high fidelity of the map. It still somethimes get stuck in places it cant get out of, but generally the "keep left" strategy lets it map above 60 % with a high fidelity on the runs. 
Here is a link to the other section of the movie: had to trim it as it was too big: https://youtu.be/v_hqjrYZWRQ This shows the end of the run where it mapped over 90% of the world with over 70% fidelity. 
