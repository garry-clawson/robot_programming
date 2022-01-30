
### Root Programming - CMP9767M Grapevine Bunch Detection

https://docs.google.com/presentation/d/1FuE3A1aroXwpHDm40s5xWv4lMTm8JGdwhloraxSf0gE/edit?usp=sharing 

## Introduction to Problem

To detect and count the crop yield of a vine yard. This must be accomplished by navigating a Thorvald robot autonomously to count all grape bunches present in the vineyard. 

## Define my area of focus

My area of focus is perception of the grape bunches across the grapevine environment. This means developing a pipeline of tools that will enable detection of grape bunches with only limited variable adjustments rather than a new pipeline configuration or discovery mechanism. 

## What makes the problem hard

Several aspects of the challenge provided require consideration:

1. Occlusion

1. Lighting

1. Lighting direction

1. Colour Gradients of greens

1. Size of grapes

1. Variation on colour of grapes

1. Shadows

1. etc ...

## Explain My Approach to the Problem

Add images here to show results at each stage and discuss limitations and problems I come across

### State Update through the program

### BUG2

1. Explain what it is and why it was chosen over BUG0 or BUG1

1. Homing Beacons

1. Obstacle Avoidance (Wall Follow)

### Vision & OpenCV

1. simpleBlobDetector
1. MorhpologyEx
1. Dilation and Opening
1. Masking
1. etc ...

## Limitations of Solution

### Algorithm

1. Homing beacons require pre defining

1. Sensor (Hoyuka) can get stuck in tight corners or edges

1. Transition through states xcan get caught in a loop

1. Distance to grapevine and Hoyuka Field of View

1. etc ...

### Environment

1. Masking and HSV values

1. Terrain - only deal sin flat land

1. Odometry requirement

1. Use of external walls (not practical)

1. Laser density can go through Grapevine and cause reading errors

1. etc ....