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
[image13]: ./images/training_graph.png
[image14]: ./images/following_target.png
[image15]: ./images/patrol_without_target.png
[image16]: ./images/patrol_with_target.png

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

### Tuning of hyper parameters 

The hyper parameters for this network is defined as; 

> * **batch_size:** number of training samples/images that get propagated through the network in a single pass.
> * **num_epochs:** number of times the entire training dataset gets propagated through the network.
> * **steps_per_epoch:** number of batches of training images that go through the network in 1 epoch. We have provided you with a default value. One recommended value to try would be based on the total number of images in training dataset divided by the batch_size.
> * **validation_steps:** number of batches of validation images that go through the network in 1 epoch. This is similar to steps_per_epoch, except validation_steps is for the validation dataset. We have provided you with a default value for this as well.
> * **workers:** maximum number of processes to spin up. This can affect your training speed and is dependent on your hardware. We have provided a recommended value to work with.

I initially started with the same parameters as in the segmentation lab exersice; 

* learning_rate = 0.005
* batch_size = 128
* num_epochs = 10
* steps_per_epoch = 35
* validation_steps = 50
* workers = 2

This resulted in a final score of only ~ 4%. Both the traning loss and the validation loss was quite high, and from the graphs it seemed to reduce quickly and be occilating a bit. It also increased at the end, which I interpreted as learning rate being too high. However, not overfitting (validation loss < training loss). 

Next I tried to lower the learning rate, effectively slowing down the learning process and get smoother convergance. At the same time I decresed the batch size and increased the epochs. I did a couple of iterations ending up with the following setup that (barely) provided a passing final resulting accuracy. 

* learning_rate = 0.0035
* batch_size = 80
* num_epochs = 40
* steps_per_epoch = 50
* validation_steps = 50
* workers = 2

Reviewing the training curves we see the training loss is below the validation loss, but not with a huge difference (loss: 0.0224 - val_loss: 0.0326). It is occilating a bit, and validation loss seems to increase a bit on the end. One could try with a lower learning rate, of eg. 0.003 to get smoother convergence. Also, increasing number of epochs to eg. 50 could improve the result as it might continue converging to a lower traing and validation loss. However, this would require a long training time on my local set-up. Might be usefull to test increasing the number of workers in order to speed up training. However, as this model passed 40% final result, I choose to hand in at this stage, as I'm already late :-D 

![Screenshot from the last training curves of the notebook, project deliverable][image13] 

### Notes on Neural Networks - Applicability and Limitations 

FCN models are good at sceen understanding, more than merely detecting if an object is in an image or not. If the task at hand is to classify correctly objects in sceen a semantic segmentation might not be needed. In such case you could perform the task by applying a convolutional network with a fully connected layer leading up to a soft max function. However, if you need deeper understanding - or understanding of deeper dimmensions of the image - then semantic segmentation, as applied in this project would be relevant. 

A neural neural network that is built to perfom a supervised learning approach is always dependant on the available labeled data it gets to train on. In this case it has been trained to learn how to segment out persons in images. However, if trained on data visualizing eg. dogs, cats, cars or other things, and with some editions to some of the utility functions, it would be able to segment out dogs or those other things too. Semantic Segmentation does perform equally good regardless of type of object it is charged to spot, all other things being equal.

To imrpove the networks robustnes and ability to correctly understand and find the hero in images one could try adding data from real life, or data with more noice. However, that would also require more data to get similar accuracy all other things being equal. 

Obvously semantic segmentation, and other sceen understanding, networks are really useful for perception algoritms of eg. self driving cars. In addition to segmenting out different objects and their location in the image, the sceen understanding network could also be extended by also understanding debt of the objects in the image. 

Bounding box network architectures could also be used on some cases to identify objects in an image, however it lend itself poorly to objects that is not easily bounded by a box such as a road or similar. For detecting humans it could be used with success, however not as accurate as the segmentation netwok providing pixelwise classification to the objects of interest. 

### Project Results and Possible Improvements 

#### Project Results 

The network get scored by two types of errors. The intersection over union for the pixelwise classification and a measure to determine if the network is able to detect the person or not. If more then 3 pixels have probability greater then 0.5 of being the target person then this counts as the network guessing the target is in the image. It also checks if the target is actually in the image, by checking that there are more then 3 pixels containing the target in the label mask. This again is used count the true_positives, false positives, false negatives of the detection. 

The final resulting score is calculated as the pixelwise; 

> average_IoU*(n_true_positive/(n_true_positive+n_false_positive+n_false_negative))


It was captured images of three scenarios; 1) when the drone was behind the target person, 2) when the drone was out on patrol without target, and 3) when the drone was on patrol with target. 

For the first scenario the network scored fairly good on both measures; 

> number of validation samples intersection over the union evaulated on 542
> average intersection over union for background is 0.9944328682908001
> average intersection over union for other people is 0.3064691328128149
> average intersection over union for the hero is 0.8978339702466568
> number true positives: 539, number false positives: 0, number false negatives: 0

This is also illustrated by the image below. From sample data of the scenario where the drone is following the target person, and you see the "real life picture" to the left, the target picture in the midle, and finaly the result from the network to the right. 

![Screenshot of the follow target example images from the notebook, project deliverable][image14] 

For the second scenario the network performed a bit worse, falsely identifying the hero 136 times, however intersection over union for the hero was 0.0 which is good as the hero was not in the images; 

> number of validation samples intersection over the union evaulated on 270
> average intersection over union for background is 0.9846801941406704
> average intersection over union for other people is 0.6556936091110968
> average intersection over union for the hero is 0.0
> number true positives: 0, number false positives: 136, number false negatives: 0

Notice that there was less samples evaluated for this scenario than the first one (542 vs. 270). So might prioritize this scenario if we proceede to add more data. The sample images of the second scenario is presented below. 

![Screenshot of the patrol without target in sceen example images from the notebook, project deliverable][image15] 

The third scenario evaluated a bit more data than the second, but still less than the first scenario (322); 

> number of validation samples intersection over the union evaulated on 322
> average intersection over union for background is 0.9959029081930075
> average intersection over union for other people is 0.40201606407729795
> average intersection over union for the hero is 0.2524588525348087
> number true positives: 159, number false positives: 5, number false negatives: 142

This scenario measure how well the network is able to detect the hero from far away, doing patrol with hero in sceen. As the measure show, it has less ability to classify the hero correctly as sceen both from the IOU score for hero of 0.25 and some false positives and a fairly high amount of false negatives. This is also illustrated by the images below, where you see that the hero is classified both as "other people" (green) and "the hero" (blue), when you compare to the labeled output in the middle. 

![Screenshot of the patrol with target in sceen example images from the notebook, project deliverable][image16] 

Quite a few pixels is miss classified in these images, underpinning the poorer score compared to the first scenario where there is no visible green pixels on the hero (supported by no false positives or false negatives). 

All in all the final resulting measure of the networks accuracy is 0.40922751799259066. Barely above the passing threshold of 40%. However, due to time limitations this would need to suffice for this hand-in. 

#### Posible improvements 

As mentioned earlier, one major improvement would be to add more training data. I only used the training data that was provided by Udacity. Generating more data in general, but especially focusing on scenario 2 and 3. 

Another tactic to improve the network would be to build a deeper network with eg. one additional encoder and decoder layer, with filters going as deep as 128 in the 1x1 layer, i.e. encoder 16-32-64, 128 1x1, decoder 64-32-16. This would be worth trying, especially if used on more training data. Then dropout could be added to reduce overfitting. 

The hyper parameters could also be improved, as mentioned above. In addition they would need to be adjusted anyways when adding more layers to the model. 



