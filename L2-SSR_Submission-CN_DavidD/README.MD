<<<<<<< HEAD
<font size = 6> Writeup Report for RoboND - L.2. Search and Sample Return </font>
- By: David, Dong  
- Udacity username: david.dong@psi-maker.com  
- Date:  Sep. 12, 2017
- Location: Beijing, China
---

# Project environment setup
1. Windows 10 Pro English version 64-bit
2. [Windows Simulator Build](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Windows_Roversim.zip)
 - *Screen resolution: 800 x 600*
 - *Graphic quality: Good*
 - *Display 1*  
 - *Video recording rate in simulator is fps=25, output video is fps=50*
3. Software
  - Anaconda3-4.4.0-Python3.6-Windows-x86_64.exe
  conda upgrade conda  
  conda upgrade --all
  conda install nb_conda (to manage environment)
  - Git-2.14.1-64-bit.exe
  - Git Clone RoboND-Python-StarterKit
  - Create RoboND environment   
  - Install Nbextensions for Jupyter *(notebook table of contents)*
  - jupyter nbextensions_configurator enable
 ---
# Folders structure
> __L2-SSR_Submission-CN_DavidD__
 >> __calibration_images__：*images for python code testing*  
 >> __output__: *Video & picture output by 'test_rover_img_basic.ipynb'*  
 >> __record_data__: *the simulator(Roversim) training mode recorded image and 'robot_log.csv'*  
 >> __ref__: *files, code, tools for future improvement*  
 >> __code__:  
  >>> Autonomous navigation scripts (drive_rover.py, supporting_functions.py, decision.py and perception.py)   
  >>> run_test.ipynb: to run 'drive_rover.py' in Ipython  
  >>> test_perception_py: Python self improvement testing code  
  >>> test_rover_img_basic.ipynb:process_image() code*
---
# Key Knowledge
## Dimensions & space 
- Telemetry Coordinates, Euler angels; naval and aeronautical position  
  ![Example](ref/TelemetryCoordinates.JPG)  

- Matrix is the dimensions symbol, should review more knowledge about this

$$
\left[
\begin{matrix}
x' \\
y'
\end{matrix}
\right]
=
\left[
\begin{matrix}
cos(\theta) & -sin(\theta) \\
sin(\theta) & cos(\theta)
\end{matrix}
\right]
\left[
\begin{matrix}
x \\
y
\end{matrix}
\right]
$$  
## Theory & tools for future improvement
- Calculus, Algebra 
- Image Matrix concepts, Image pixels - different numpy arrays, color channels(RGB; BGR; Gray; HSV; HSL), single channel binary image and differnece
- Python coding skills, libraries for image & data analysis, matplotlib; numpy; pandas; cv2; Mahotas; PIL; SciPy; etc.
- Tool: Anaconda, Python, Jupyter notebook, nbextensions, Atom, Latex, workflow chart by MD
- Raw image Coordinates; Rover-Centric Coordinates; map to World Coordinates; polar Coordinates
- Layout by plt.subplot(m, n, p)  
- The uint8 image can be plotted, but others cannot
- Perspective, thresholding(e.g. OTSU, HSV method etc.),border identification, radar scan
---
# Rubric Points
## Notebook Analysis
### How to  analyze the image  by Python code
Please refer to __'test_rover_img_basic.ipynb'__  
   
 > 1. Three important concepts for image analysis: dimensions' coordinates; image channels; image overlay. Familiar with below python libraries will be helpful:   
matplotlib; numpy; pandas; cv2; Mahotas; PIL; SciPy;
 > 2. Basic image analysis, the image perception step 
  >> __Perspective__: from a camera image to the top-down view, the scale is 10x10 pixels = 0.1x0.1 meter square. It is still the 3 channel color image after perspective.  
  >> __Thresholding__: Seperate color image to single channel binary image  
   >>> 1) __Navigation area thresholding__: There are many ways for thresholding. I tested OTSU(*otsu_threshold()*), and static 3-tuple color threshold values in 'test_rover_img_basic.ipynb'. Both are ok for this project, by that we can get the navigation area on binary image with value '1' for navigatiion, and value '0' for other areas. __Note: by OTSU threshold, the result will include the yellow rock; by static color threshold value, the result will not include the yellow rock__  
   >>> 2) __Obstacle area thresholding__: obstacle = 1-navigation; or obstacle = cv2.bitwise_not(naigation); or obstacle = ~navigation  
   >>> 3) __Rock sample thresholding__: in the functioin rock_thresh() of 'test_rover_img_basic.ipynb', it changes the original BGR image to HSV image by *cv2.cvtColor* method, then use *cv2.inRange(hsv, hsv_lower_yellow, hsv_upper_yellow)* to get the yellow  rock samele mask, the single channel image,at last by integrate the mask image into original image to get three channels color image for rock sample
  >>
  >> __Image border__: by *'cv2.findContours'*, we can find single channel images' borders for future detailed analysis
  >>__Create worldmap__: create the worldmap from a same size PNG file with one square meter per pixel
 >
 > 3. Rover-centric map: All thresholding images can be convert to rover-centirc coordinates

 
### World map integration
__World map integration__ '*process_image()*':  
 > in 'test_rover_img_basic.ipynb -- 8.2 Process_image code', I tested image overlay to plot the worldmap image
  >> 1) use random rover yaw & postion coordinates x,y; and create the worldmap from a same size PNG file with one square meter per pixel  
  >> 2) put the obstacle area binary image to worldmap image channel 0(R channel)  
  >> 3) put the rock area binary image to worldmap image channel 1(G channel)  
  >> 4) put the navigation area binary image to worldmap image channel 2(B channel)  
  >> 5) Integrate the overlay worldmap by '*cv2.addWeighted*'
 >
 > put the above test code into the method 'process_image()', replaced the testing rover yaw, rover position & image by simulator recorded values*(robot_log.csv) in the folder 'record_data'* 

### Output video
By 'ffmpeg' plugin & 'moviepy' library output the video in the folder output\>recordImg2Video_mapping.mp4 for simulator recorded data 

## Autonomous Navigation and Mapping summary
### Information for simulator
 - *Screen resolution: 800 x 600*
 - *Graphic quality: Good*
 - *Display 1*  
 - FPS output to terminal by drive_rover.py: from 20 to 25  
 
### Script result
  
__Please run driver_rover_start.ipynb by jupyter notebook for Autonomous Mode__
- more than 40% of the world map environment
- more than 60% fidelity (accuracy) against the ground truth: Identify the valid image data to improve fidelity(good_nav_pix) in perception.py if (abs(Rover.pitch < 10)) & (abs(Rover.roll < 10))
- located more than 1 rock sample
- not pickup any rocks
- not record any image on autonomous mode

 
### key summary for Autonomous script update 
1. changed 'max_vel' from 2 to 30 in *decision.py*
2. Below parameters are used or added for rover decision.py
    ```
     self.stop_forward = 50 # Threshold to initiate stopping
     self.go_forward = 500  # Threshold to go forward again
     self.time_5s = None   # check rover status by interval 5 seconds, to avoid unexpectly rover jam
     self.s_pos = None     #  rover position before 5 seconds
     self.barrier = 0     # Radar Scan(*perception.py - radar_scan()*) function to check 3 meters(less than rock identify distance) ahead the rover navigation area. Stop the rover if there is rocks or large barrier in front of rover in *decision.py*.
    ```
3. Identify the valid image data to improve fidelity(good_nav_pix) in *perception.py* 
    ```
    if (abs(Rover.pitch < 10)) & (abs(Rover.roll < 10))
    ``` 
4. *Decision.py - decision_step(Rover)* update
 ![rover_path](ref/Decision_Tree.JPG) 
 __Desicion tree description__
 - Rover go along with the right direction `Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 0)` on FORWARD status
 - Stop if not enough going forward navigation area pixels
 - Stop the rover if there are rocks or large barrier in front of rover every 5 seconds
 - Turn left once the rover is facing the wall or rocks, untill -15\<steer angel\<15
5. update the self.time_5s and self.s_pos value if there is new Rover.image in supporting_functions.py - update_rover()

## Important perception.py functions
- def perspect_transform(img, src, dst): Do image perspective for color thresholding
- def rock_thresh(wp_img): get rock samples from image
- def color_thresh(img, rgb_thresh=(160, 160, 160)): get navigation area from image
- Obstacle area = 1 - Navigation area
- def rover_coords(binary_img): get the rover centric coordinates
- def to_polar_coords(x_pixel, y_pixel): get the radial coordinates for rover
- def rotate_pix, translate_pix, pix_to_world: Translate current Rover Yaw & Rover Position to the worldmap coordinates
- def radar_scan(x_navi_rc,y_navi_rc,x_obs_rc,y_obs_rc): The Radar Scan function to check navigation pixels & obstacle pixels ahead 3 meters of rover navigation area
- def perception_step(Rover): do basic image perception and update the Rover status 

# Confusion, Question, Conclusion for future improvement
1. The best way for Rover jam unexpectly more than 5 sends is going backward, but I didn't find the method of Rover backward
2. By Radar Scan function, we can improve rover decision more
3. By image border identification, with rover size, obstacle size & rock sample size calculation in the image, the rover can pass through very narrow place
4. Record Rover starting point coordinates & crossroad coordinates, the Rover can easily traverse the worldmap, pickup the rock sample and return to the starting place
5. I find the HSV lower value & HSV upper value from Internet, the static value to identify yellow rock sample. 
but by [Object Tracking](http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html) , the calculation value is Yellow HSV value=[[[ 30 255 255]]], __then how to find the HSV lower&upper value for yellow rock?__

# Reference
1. [OpenCV 2.3.2 documentation](http://www.opencv.org.cn/opencvdoc/2.3.2/html/index.html)  
2. [Geometric Image Transformations](http://docs.opencv.org/modules/imgproc/doc/geometric_transformations.html)  
3. [Image Thresholding theory](http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_thresholding/py_thresholding.html)  
[Chinese version](http://www.opencv.org.cn/opencvdoc/2.3.2/html/doc/tutorials/imgproc/threshold/threshold.html)  
4. [Rotation matrix](https://en.wikipedia.org/wiki/Rotation_matrix)  
5. [Telemetry](https://en.wikipedia.org/wiki/Telemetry)
6. [SocketIO](https://python-socketio.readthedocs.io/en/latest/)
7. [Polar coordinate system](https://en.wikipedia.org/wiki/Polar_coordinate_system)
8. [Geometry translation](https://en.wikipedia.org/wiki/Translation_(geometry))
9. [Median filter](https://en.wikipedia.org/wiki/Median_filter)
10. [Frame of reference](https://en.wikipedia.org/wiki/Frame_of_reference)
11. [Ipython buildin magic commands](http://ipython.readthedocs.io/en/stable/interactive/magics.html)
12. [Python debugger](https://docs.python.org/3/library/pdb.html)
13. [Matplotlib](http://matplotlib.org/faq/usage_faq.html#what-is-a-backend)
14. [Robot garden waiver](http://www.robotgarden.org/get-involved/waiver/)
15. [Buzz swarm robot](http://the.swarming.buzz/)
||||||| merged common ancestors
# Key Knowledge of Lesson 2 Search and Sample Return Project
 ## 1.1 
=======
# 1. Crawl on Key Knowledge
1. Telemetry Coordinates  
*Euler angels*; *naval and aeronautical position*  
![Example](../Ref/TelemetryCoordinates.JPG)  
2. Python coding
3. Record data from simulator trainning mode for IMG & log in the folder 'record_data'
4. Choose one picture from record data folder [robocam_2017_08_26_17_59_22_096.jpg](record_data/IMG/robocam_2017_08_26_17_59_22_096.jpg) for later coding Example  
![CodingExpPict](record_data/IMG/robocam_2017_08_26_17_59_22_096.jpg)  
5. Basic knowledge about deal with image  
 - Matrix concepts to deal with image  
  e.g.  
$$
\left[
\begin{matrix}
1 & 0 & t_x \\
0 & 1 & t_y
\end{matrix}
\right]
$$
 - *image pixels*: 8 bits information for each pix of each channel (0,255)
 - *image channel*: RGB; BGR; Gray
 - *libraries for image* matplotlib; cv2; Mahotas; PIL etc.
 - *Coordinates for image*: raw image Coordinates; Rover-Centric Coordinates; map to World Coordinates; polar Coordinates  
 - *layout method by plt.subplot(m, n, p)*  
6. Flowchart on the project of 'Search & sample return'  
>>>>>>> origin/master
