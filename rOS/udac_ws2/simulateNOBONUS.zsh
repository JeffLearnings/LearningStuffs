gnome-terminal --tab -- roslaunch my_robot world2.launch
sleep 5
gnome-terminal --tab -- roslaunch ball_chaser ball_chaser_VANILLA.launch 
gnome-terminal --tab -- rosrun rqt_image_view rqt_image_view
