## Project: Follow Me
### This is write up for Project 4: Follow Me Drone, based on Semantic Segmentation Network, of Udacity RoboND

![Screen Shot of Drone following in simulator ref. Project Intro, Udacity][image1]

---


# Required Steps to complete this project:

* Clone the project repo [here](https://github.com/udacity/RoboND-DeepLearning-Project.git)
* Fill out the TODO's in the project code as mentioned [here](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/cac27683-d5f4-40b4-82ce-d708de8f5373/lessons/197a058e-44f6-47df-8229-0ce633e0a2d0/concepts/b0044631-d356-4ff4-9fd9-4102c28a2efa?contentVersion=1.0.0&contentLocale=en-us)
* Optimize your network and hyper-parameters.
* Train your network and achieve an accuracy of 40% (0.40) using the Intersection over Union IoU metric which is final_grade_score at the bottom of your notebook.
* Make a brief writeup report summarizing why you made the choices you did in building the network.

[//]: # (Image References)

[image1]: ./images/follow-me-project.png
[image2]: ./images/fcn-architecture.png
[image3]: ./images/1x1convolution.png
[image4]: ./images/upsampling.png
[image5]: ./images/skipped_connections.png
[image6]: ./images/skipped_enc_dec.png
[image7]: ./images/semantic-segmentation.png
[image8]: ./images/semantic-segmentation-people.png
[image9]: ./images/encoder_block.png
[image10]: ./images/decoder_block.png 
[image11]: ./images/1x1_conv2d_batch.png
[image12]: ./images/fcn_model.png

The submission includes 

* [model_training.ipynb](https://github.com/SigveMartin/RoboND/blob/master/Project_4/submission/model_training.ipynb)
* [model_training.html](https://github.com/SigveMartin/RoboND/blob/master/Project_4/submission/model_training.html)
* [config_model_weights.h5](https://github.com/SigveMartin/RoboND/blob/master/Project_4/submission/config_model_weights.h5)
* [model_weights.h5](https://github.com/SigveMartin/RoboND/blob/master/Project_4/submission/model_weights.h5)

and the whole project folder together with this write-up / README and its images. 


## [Rubric](https://review.udacity.com/#!/rubrics/1155/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Network Architecture 

After walking through different lessons on PID controllers and quadrotor control using PID, we delved down into Deep Learning with Tensorflow, Deep Neural Networks, Convolutional Neural Networks, Fully Convolutional Networks (FCN) and finally the Semantic Segmentation lesson. This project is using a simulator to provide image data that the neural network could be trained on. 

The network we have built for this project is called a Segmentation Network, which is a type of Fully Convolutional Network, that is able to perform object detection and semantic segmentation. At a high level the architecture consists of an Encoder and a Decoder, as the image below illustrates. 

![Screen Shot of FCN architecture, Lesson 32: Fully Convolutional Networks, Udacity][image2]

The goal of the Encoder is to extract features of the image, whereas the goal of the decoder is to upscale the output from the encoder to the same size as the original imagem resulting in a segmentation or prediction of each individual pixel of the original image. 

Wheras a typical Convolutional Neural Networks might consist of a series of convolutional layers feeding into a fully connected layer (FCL), enabling classification tasks like "is this a person" (2D tensor), we want to find out where in the image the person is (4D tensor). That is why we need to replace the FCL with a 1x1 convolutional layer, as the fully connected layer does not preserve spatial information. So by inserting a 1x1 convolutional layer instead of the FCL we are able to preserve spatial inforamtion throughout the whole network, enabling it to find the person and decode where it is in the image. 

The replacement of the FCL is one of three special teqniues that the FCNs adopt. A 1x1 convolutional layer is created by setting 1x1 filter, stride =1 and zero (same) padding. In my FCN model I used the conv2d_batchnorm() with 64 filters, kernal of 1x1 and stride of 1, producing a 1x1 convolution as the image below illustrates. 


![Image of 1x1 Convolutions, Lesson 32.4, Udacity][image3]

The other teqniques FCN adopt are upsampling through transpose convolutional layers, which is part of the architecture of the decoder where essentially a reverse convolution where the forward and backwards passes are swapped, as illustrated by the image below. You'll see this applied in the code through BilinearUpSampling2D. 

![Screenshot from video, Lesson 32.3, Udacity][image4]

The third technique is skipped connections, which allows the network to use information from multiple resolution scales, enabling more precise segmentation decisions as the image below illustrates. 

![Screenshot from video, Lesson 32.3, Udacity][image5] 

It is implemented in the decoder_block by taking a larger layer and the smaller_layer as inputs where it is concatenated after the upsampling step. It is implemented in the fcn_model by using output_layer1 as the larger layer of output_layer4 of and the inputs as the larger layer of output_layer5 of the decoder. Thus feeding information from larger resolutions of images from the encoder step, as illustrated by the image below. 

![Screenshot from video, Lesson 32.10, Udacity][image6] 

The neural network implemented provides sceen understanding through "semantic segmentation", which is the task of assigning meaning to part of an object (Ref lecture 33.3, Udacity). This is done by assigning each pixel to a target class, as illustrated in the image below where pixels of pedestrians are illustrated in red, road in light purple, car in dark blue, sidewalk in pink, tram in dark green, and signs in yellow. 

![Screenshot from video, Lesson 33.3, Udacity][image7] 

Which in our case enables the drone to learn where the target person in an image stream is, as illustrated in the image below. 

![Screenshot from video, Lesson 33.5, Udacity][image8] 

#### Details of the Fully Concolutional Network as implemented in this notebook 

In both the Encoder and the Decoder we have implemented Seperable Convolutional layers with batch normalization. Sepearable Convolutions, also known as depthwise seperable convolutions, is increasing the efficiency of the network by reducing the amount of parameters needed. This is achieved by a convolution  over each channel of an input layer, followed by a 1x1 convolution that takes the output channels from the previous step and combines them into an output layer. Batchnormalization is an additional way to optimize network training, by normalizing each current batch that get fed into each layer of the network. 

##### Encoder Block
The encoder_block function of the code consists of one sepearable convolutional batch normalized and ReLU activated layer, as the image illustrates below.  This was used to create two encoder layers of the fcn_model first with filter size if 16 and strides of 2, and second of filter size of 32 and strides of 2. Both with kernal size 3. 

![Screenshot from the Encoder Block function of the notebook, project deliverable][image9] 

##### Decoder Block
The decoder_block function of the code consists of an upsampling layer, concatenation of big pixel and small pixel layers, and two sepearable convolutional batch normalized layers as illustrated in the image below. This was used to create two decoding layers, matching the encoder, first of filter size 32 and second of filter size 16. Both with a stride of 1. 

![Screenshot from the Decoder block function of the notebook, project deliverable][image10] 

#### Fully Convolutional Network (FCN) Model 
The FCN_Model also incorporates one 1x1 convolution layer with batch normalization, by a conv2d_batchnorm function as illustrated below. This is the layer that ensures that spatial information is preserved between the encoder and decoder layers.

![Screenshot from the Seperable Convolutions functions part of the notebook, project deliverable][image11] 

All in all this resulted in a FCN_Model as illustrated by the image below. 

![Screenshot from the FCN_Model function part of the notebook, project deliverable][image12] 

I started out with two encoder and decoder blocks. Each layer increases the networks ability to capture more complex shapes in the images, however it also increases the chance of overfitting the data. I played around with different values in the filters, but ended up with 16 and 32 for encoder and decoder and the 64 sized 1x1 convolution between. I think this serves well to pick up the features of the person in the images and found the architecture to be a good basis for tuning the hyper parameters so that network accuracy over 40%. 

#### Maybe as sub sections of architecture

* various uses of encoding/decoding images - when it should be used, when its useful, and any problems that may arise. 
* The student is able to clearly articulate whether this model and data would work well for following another object (dog, cat, car, etc.) instead of a human and if not, what changes would be required.

### Tuning of hyper parameters 

The hyper parameters for this network is defined as; 

* **batch_size:** number of training samples/images that get propagated through the network in a single pass.
* **num_epochs:** number of times the entire training dataset gets propagated through the network.
* **steps_per_epoch:** number of batches of training images that go through the network in 1 epoch. We have provided you with a default value. One recommended value to try would be based on the total number of images in training dataset divided by the batch_size.
* **validation_steps:** number of batches of validation images that go through the network in 1 epoch. This is similar to steps_per_epoch, except validation_steps is for the validation dataset. We have provided you with a default value for this as well.
* **workers:** maximum number of processes to spin up. This can affect your training speed and is dependent on your hardware. We have provided a recommended value to work with.


	* Epoch
	* Learning Rate
	* Batch Size

	Etc.
	All configurable parameters should be explicitly stated and justified.

### Notes on Neural Networks - Applicability and Limitations 

FCN models are good at sceen understanding, more than merely detecting if an object is in an image or not. If the task at hand is to classify correctly objects in sceen a semantic segmentation might not be needed. In such case you could perform the task by applying a convolutional network with a fully connected layer leading up to a soft max function. However, if you need deeper understanding - or understanding of deeper dimmensions of the image - then semantic segmentation, as applied in this project would be relevant. 

A neural neural network that is built to perfom a supervised learning approach is always dependant on the available labeled data it gets to train on. In this case it has been trained to learn how to segment out persons in images. However, if trained on data visualizing eg. dogs, cats, cars or other things, and with some editions to some of the utility functions, it would be able to segment out dogs or those other things too. Semantic Segmentation does perform equally good regardless of type of object it is charged to spot, all other things being equal.

To imrpove the networks robustnes and ability to correctly understand and find the hero in images one could try adding data from real life, or data with more noice. However, that would also require more data to get similar accuracy all other things being equal. 

Obvously semantic segmentation, and other sceen understanding, networks are really useful for perception algoritms of eg. self driving cars. In addition to segmenting out different objects and their location in the image, the sceen understanding network could also be extended by also understanding debt of the objects in the image. 

Bounding box network architectures could also be used on some cases to identify objects in an image, however it lend itself poorly to objects that is not easily bounded by a box such as a road or similar. For detecting humans it could be used with success, however not as accurate as the segmentation netwok providing pixelwise classification to the objects of interest. 

### Project Results and Possible Improvements 

More data 

One tactic to improve the network would be to build a deeper network with eg. one additional encoder and decoder layer, with filters going as deep as 128 in the 1x1 layer, i.e. encoder 16-32-64, 128 1x1, decoder 64-32-16. This would be worth trying, especially if used on more training data. Then dropout could be added to reduce overfitting. 



