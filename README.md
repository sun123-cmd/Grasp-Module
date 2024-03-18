# Grasp Module
This section contains robot arm grasp manipulation related code. Below figure describes high level pipeline of how a final grasp is generated from a RGBD Image taken from robot head camera.
* [AnyGrasp](https://arxiv.org/abs/2212.08333) generates all possible poses in the scence
* [Lang-SAM](https://github.com/luca-medeiros/lang-segment-anything) is used to extract the object mask, while [GroundingDINO](https://github.com/IDEA-Research/GroundingDINO) source code should be installed.
* [OK-Robot](https://github.com/ok-robot/ok-robot) is the base manipulation model of this module.
* The grasps points are projected back onto the image and filtered to retain only ones that fall on the mask.
* The grasp with max score is chosen as final grasp.


## Code
Run the following code:
```
conda activate grasp-env

cd /home/hs/disk1/sunwenhao/grasp/src
python demo.py --debug
```
**debug flag -** Is for visualizing 3d plots of grasping.
**open_communication flag -** Is for selection from dry run and robot running (here we only want to dry run to test environment installation).

**open_communication -** When you are operating with robot

## Visualizations
Once you run the program it will save the following visualizations in the `save_directory` option in `src/demo.py` file
* **clean_*.jpg -** Image taken by the robot for processing
* **semantic_segmentation_*.jpg -** Segemented object mask of the query
* **poses.jpg -** Screen shot of 3d scene with all the predicted poses
* **grasp_projections.jpg -** Green dots indicate grasps inside the object mask and red dots indicate grasps outside the object mask.
* **best pose.jpg -** Final executed pose

These visualizations help in understanding the output behaviour and also helps in debugging in case of errors.

### Running choices
While running `debug.py`code, you will have choices as follows
`$Enter action [pick/place]: pick`
`$Enter a Object name in the scence: Pick the yellow bottle up`
Then target pictures will be loaded.


### Output results
#### pick action 
has **target postion**, **rotation martix** and **camera depth** return data
* **translation**
[-0.03910572 -0.17605631  0.73799998]
* **rotation_matrix**
[[-0.00318942  0.99998814  0.00369123]
 [-0.6537931  -0.00487828  0.75665766]
 [ 0.75666666  0.          0.6538009 ]]
* **depth and crop_flag** 
0.019999999552965164 False 0

#### place action 
has **target postion** return data
* **Placing point of Object relative to camera**: 
[ 0.12       -0.04195643 -0.67549731]


## Use RGB-D Camera of gazebo
* **First** :`roslaunch realsense2_camera rs_camera.launch`
We need to get camera 3D data from the camera this way.
* **Then** :
