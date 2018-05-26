## Project: Perception Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!
[//]: # (Image References)

[image1]: ./images/environment.png
[image2]: ./images/outl_filtered_env.png
[image3]: ./images/vox_filtered_env.png
[image4]: ./images/pt_filtered_env.png
[image5]: ./images/table_ex_env.png
[image6]: ./images/objects_ex_env.png
[image7]: ./images/objects_ex_env.png
[image8]: ./images/euclidian_d.png
[image9]: ./images/rgb_hist.png
[image10]: ./images/hsv_hist.png
[image11]: ./images/m1_final.png
[image12]: ./images/m2_final.png
[image13]: ./images/m3_final.png
[image14]: ./images/cluster_recognized.png

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.
The first part of this project was to implement the perception pipeline as developed through exercise 1, 2 and 3 leading up to this project. In the project we got a cluttered environment, as the screenshot from Rviz below shows. 

![Screenshot from Rviz showing the cluttered environment][image1]

So first filter added was a statistical outlier filter as walked through in [lesson 17.16](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/e5bfcfbd-3f7d-43fe-8248-0c65d910345a/lessons/8d51e0bf-0fa1-49a7-bd45-e062c4a2121f/concepts/fdb3a445-43e0-4a02-81e2-0448432c156f?contentVersion=1.0.0&contentLocale=en-us). As this lesson states we assume a Gaussian distribution, thus all points whose mean distances are outside of an interval defined by the global distances mean+standard deviation are considered to be outliers and removed from the point cloud. 

I published the resulting pointcoud so that I could iterate and find the best setting for the number of nabouring points to analyse and the threshold scale factor. I landed on 50 and 0.01 respectively, which got the result as illustrated in the following screenshot from Rviz while publishing the outlier filtered pointcloud.

![Screenshot from Rviz showing the outlier filtered point cloud][image2]

As RGBD cameras provide feature rich and dens point clouds, we apply a Voxel Grid Downsampling filter (as walked through in [lesson 17.10](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/e5bfcfbd-3f7d-43fe-8248-0c65d910345a/lessons/8d51e0bf-0fa1-49a7-bd45-e062c4a2121f/concepts/77999aa5-19c1-40ee-89bc-6ba621612f85)) to reduce the pointcloud while still having enough data to represent the input point cloud as a whole. 

I experimented with several values, starting with a leaf size of 0.01 that I found to be best in the lecture. However, during the iterations of the project I found that I had to reduce the leaf size in order to have higher resolution of the cloud to properly reqognise the objects. So, i found that a leaf size of 0.003 was good for the purpose of this pipline resulting in the following (rather dens) pointcloud after the downsampling. 

![Screenshot from Rviz showing the pointcloud after Voxel Filter Downsampling with 0.003 leaf size][image3]

Since we already knew enough about the location of the data of interest in our scene the next step was to use a Pass Through Filter (as walked through in [lesson 17.11](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/e5bfcfbd-3f7d-43fe-8248-0c65d910345a/lessons/8d51e0bf-0fa1-49a7-bd45-e062c4a2121f/concepts/d704276f-f535-4505-9270-571fd94d50ee)) to filter out useless data. 

This filter is basically cropping out a point cloud by specifying the axis and what cut-off values, effectively defining the region of interest that you allow to pass through the filter. I added the axis to Rviz and used the measure tool to find applicable cut-off values on each axis, z (up/down), y(left/right) and x(in/out). As we are only interested in the objects on the table, I applied the filter so that only the place on the table where the objects where placed would pass through. This resulted in the following filter: 

Axis | Min Value | Max Value 
--- | --- | --- 
Z | 0.61 | 1.1 
Y | -0.4 | 0.4
Z | 0.35 | 0.9 

The Y axis filter was important as the dropboxes was with in the view of the camera, and got wrongly recognized as an object of interest before filtering it out. The result from this filter stage is illustrated in the screenshot from Rviz below. 

![Screenshot from Rviz showing the pointcloud after Voxel Filter Downsampling with 0.003 leaf size][image4]

In order to remove the table itself from the environment, and thus extraxt the objects, we next performed a [Random Sample Consensus](https://en.wikipedia.org/wiki/Random_sample_consensus) (RANSAC) technique (as walked through in [lesson 17.13](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/e5bfcfbd-3f7d-43fe-8248-0c65d910345a/lessons/8d51e0bf-0fa1-49a7-bd45-e062c4a2121f/concepts/9a7196fc-6812-478c-b1a6-cf6a8fc6885e)). RANSAC is an algoritm that is able to identify points that belongs to a particular model, eg. a plane like in our case. The points that fits with the plane model are filtered out as "inliers" and the other points are filtered out as "outliers" in relation to the model that is fitted. Effectively, our inlier points represent the table and the outliers are the objects on top of the table of our current pointcloud.

During lesson 17 I experimented with different values for "max distance", and found 0.01 to be a resonable choice for the maximum distance a point could be from the best fit model of the plane in order to be considered an inlier. As I also used a plane model in the project, and it had a similar environment at this stage in pipeline, it is not suppricing that the same measure performed well in the project too. The screenshots below shows the extracted pointcloud for the inliers and the outliers, respectively the tabletop and the objects. 

![Screenshot from Rviz showing the pointcloud of the inliers after RANSAC plane fitting][image5]

![Screenshot from Rviz showing the pointcloud of the outliers after RANSAC plane fitting][image6]


#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  
At this stage we have segmented out the tabletop, but the objects are still in one meshed pointcloud. Hence, we need to apply a new techniqe in order to segment out each cluster of points representing each individual object on the table. Since the number of clusters, i.e. objects, on the table is not constant and known, I ise the Density-Based Spatial Clustering of Applications with Noise [(DBSCAN) algoritm](https://en.wikipedia.org/wiki/DBSCAN#Algorithm) as walked through in [lesson 18.7](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/e5bfcfbd-3f7d-43fe-8248-0c65d910345a/lessons/2cc29bbd-5c51-4c3e-b238-1282e4f24f42/concepts/f3abc339-1d6d-42b6-9178-67e3e37eba19). Since we know that each point on each object cluster is closer to eachother, than to the nabouring object cluster, we could filter based on points that are within a threshold distance from the nearest point in the data. This method is also called "Euclidean Clustering" as the the length measure used between points is the "Euclidean Distance" as given by the following formula: 

![Euclidean distance between points p and q in an n-dimensional dataset, ref. lesson 17.7][image7]

In order to perform this clustering method I used the PCL's Euclidean Clustering algoritm as walked through in [lesson 18.12](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/e5bfcfbd-3f7d-43fe-8248-0c65d910345a/lessons/2cc29bbd-5c51-4c3e-b238-1282e4f24f42/concepts/aff79804-e31d-468e-9f12-03536a1b16dc) and tested in exercise 2. This method first constructs a [k-d-tree](http://pointclouds.org/documentation/tutorials/kdtree_search.php) from the extracted objects to improve performance in searching for nabouring points. I got some training in finding the optimal parameters for this algorim, i.e. tolerance on the distance treshold, minimum and maximum number of points in clusters, however it is affected by the other filters in the project pipeline, so I had to iterate on these. I did this also by outputing the cluster cloud into Rviz which the screenshot below illustrates.

![Screenshot from Rviz showing the pointcloud of the clusters from Euclidean Clustering filter][image8]

Especially I found that it got affected by the voxcel leaf size, where smaller leaf size causes higer resolution and effectively affects both distance between points and number of points in clusters. At the end I landed on a tolerance of distance threshold of 0.01 and a minimum and maximum cluster size of respectively 100 and 5000. 


#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
In order to let the computer reqognize each object in the scene, I built on the learning from lesson 19 and exercise 3. We used the colour features of the objects to train the machine to learn how to reqognize each object in the scene based on the extracted object point clouds. 

In order to do this we created colour histograms to hold the features of the objects. The histogram then becomes a colour signature for each object. Based on the RGBD camera data we could extract the RGB colour information of each object cluster and create a feature vector of concatenated histograms for each colour channel, respectively R, G and B as shown in the image below. 

![Histograms of R, G and B colour channel for the blue "Udacity can", ref. lesson 19.7][image9]

In order to compare histograms we needed to normalize them, as each object would have different number of points. Since RGB representation is not the most robust representation for object recognition tasks, as its sensitive to different lighing condition, we also convert it to HSV (Hue, Saturation and Value) wich is particulary robust to different lighting contidion. After doing this and concatenating all tree colour channels (H, S, V) you'd get the following histogram. 

![Normalized histogram of H, S and V colour channel for the blue "Udacity can", ref. lesson 19.7][image10]

In order to teach the machine how to use these features to recognize objects in the scene we used a supervised machine learning algoritm called Support Vector Machine (SVM), that let you characterize the parameter space of your dataset into discrete classes. This means that we could train the SVM by taking several feature vectors of a labeled object, and then have it learn how to properly classify future feature vectors based on what it has learned. 
 
I had a training environment and scripts set up from exercise 3, however we had to modify slightly toi the different scenes with combination of objects in the project. For each scene the picklists of objects was provided in different .yaml files. I used the "capture_feature" and "train_svm" scripts from exercise 3 to generate training data and a knowledge model for the SVM to be able to classify for each scene. I spent some time iterating and finding the optimal values for especially histogram bin size, number of iterations to spin the objects to capture feature vectors from different angles (i.e. training set size) and played around with different classifiers. I landed on spinning it for a 100 iterations and a sigmoid classification kernel [(Ref. SVM docs)](http://scikit-learn.org/stable/modules/svm.html). An important note is to be sure to use HSV both for the histograms that is extracted during training, and for the ones extracted in the project scene for classification. 

The first scene had 3 objects in it, hence the model consists of 300 features (one feature per iteration) and got an accuracy of 0.99 (+/- 0.03). The following image shows the confusion matrix's for the pick list 1 model.

![Confusion matrix's for pick list 1 model, figure 1 without normalization and figure 2 with normalization][image11]

The second scene had 5 objects, hence 500 features and got an accuracy of 0.94 (+/- 0.04). The confusion matrix for pick list 2 model is as follows. 

![Confusion matrix's for pick list 2 model, figure 1 without normalization and figure 2 with normalization][image12]

The last scene had 8 objects in the scene, resulting in 800 features and an accuracy of 0.92 (+/- 0.06). The confusion matrix for pick list 3 is as follows. 

![Confusion matrix's for pick list 3 model, figure 1 without normalization and figure 2 with normalization][image13]

Running this on the last setup with 8 objects, the classifier performs good. The following screenshot from Rviz is showing the last setup with the cluster clouds in the view and proper labels as classified with the SVM. 

![Screenshot from Rviz showing last setup with cluster clouds shown and properly classified][image14]

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

As the attached .yaml files show, the computer where able to properly classify and add the position of where to pick up the objects in the picklist and where to drop them of. I implemented a check to only run the pr2_mover function if the list of detected objects where identical to the picklist. If it wasn't identical, I printed out a statement. While running the project on all three worlds I monitored the console and it seemed to me that it was able to recognize all objects and match the pick list each time. This, and visual feedback from Rviz, gives me confidense that i performs to the requirements to pass the project. 

I have tested some of the parameters of the filter to check for robustness. The leafsize of the voxcel filter could be set to 0.004 and it performs well still at least on the first test world. However, increasing it to 0.005 (all other things being equal) causes it to not reqognize the objects in the scene properly. So it needs a resolution produced by a leaf size of at least 0.004 to perform well. As this filter is mostly to imrpove performance by decreasing amout of points I use 0.003 to be safe. I didn't experience any performance differences between 0.004 and 0.003 anyways. 

I also experienced a bit with the other parameters and found that the max size of clusters could be increased quite much without affecting the result, however it shouldn't be decreased much especially with leaf size of 0.003 of the voxel filter. 

I think the accuracy of the SVM could be imrpoved even further by providing even more data by increasing iterations it spins and gives a new feature vector in traing. However, this is time consuming and my setup spent enough time producing the training material on 100 iterations. I experimented a bit with different bin sizes on the histograms, but found that 32 was good. I tried to train the different models using both linear and sigmoid kernels, but I didn't see any noticable change in accuracy. 

I had some issues setting up the perception pipeline and found the technique of publishing the different clouds to Rviz helping in optimizing it. However, for a couple of days the prediction seemed almost random for a long while despite all optimizing of the perception pipeline. After a while I found that I accidentially had given the wrong potin cloud to the SVM step... 

Other then that I have found that my PC needs some more juice... It is nicely illustrated by the redish colour of the screenshot above and this screen capture while running the last world and showing the recognized clusters. #waitforit

[![3d perception demo in Rviz](https://img.youtube.com/vi/DfiRmKQG6CE/0.jpg)](https://www.youtube.com/watch?v=DfiRmKQG6CE)

This obviously could be improved by implementing the next challenges of the project with collision avoidance and robot motion. It also could be cool to take the challenge of a more complex table setup. Looking forward to it, but submitting this for review now as it should meet the requirements and the next project is up. 



