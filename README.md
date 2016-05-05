# srrc_simple_obj_detector

simplest object detector.

## Examples of results:
every blob that's not exactly black is an object candidate. no farther filtering used.
!(frame0077.jpg "precache partially in shadow")
!(frame0155.jpg "pink rock partially in shadow")

brief usage for processing jpg files:

0. check out this repo into your catkin_ws and `catkin_make`
1. open file `test_simple_detector_node.launch`
2. replace value for parameter `src_image_src_folder` with path to your folder containing jpg files
3. replace value for parameter `output_folder` with the path you want to output the processed images to (it will be created if it does not exist)
4. call `roslaunch srrc_simple_obj_detector test_simple_detector_node.launch` and wait.
