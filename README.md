#Install

cd <SOME_PATH>/catkin_ws/src

git clone https://github.com/ChanghaoWang/scenerecognition.git

cd ..

catkin_make

#Project name: scene_recognition

Firstly run:
> roslaunch ewok_ring_buffer ring_buffer.launch (which is finished by ZBY)

Then:
> rosrun scene_recognition scenerecognition

You can also visualize the point cloud after segmentation: The planes segmented by the algorithm are shown in rviz in different colors.
> cd <SOME_PATH>/catkin_ws/src/scene_recognition/rviz

> rviz -d segment.rviz



