gnome-terminal --tab -- roslaunch my_robot world.launch
sleep 5
gnome-terminal --tab -- roslaunch ball_chaser ball_chaser.launch 
gnome-terminal --tab -- rosrun rqt_image_view rqt_image_view
