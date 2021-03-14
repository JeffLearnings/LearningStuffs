### RUN GUIDE 

## Main simulations bring up
<br>

**1.Normal ros commands in seperate terminal windows/tab/sessions**
```
roslaunch my_robot world2.launch
roslaunch ball_chaser ball_chaser_VANILLA.launch
```
**2.Using shell script prepared**
I prepared a small shell script for bash/zsh to run the commands
```
chmod +x **.zsh **.bash 
./simulateNOBONUS.zsh or ./simulateNOBONUS.bash
```

## BONUSES
**The above launch file will launch the bonus edition**
<br>
To make tracking different colored balls easier, I added walls into the world 
<br>
To run the simulation with different colored balls:
```
roslaunch my_robot world.launch
roslaunch ball_chaser ball_chaser.launch
```
 **OR**

 ```
 ./simulate.zsh or ./simulate.bash
 ```

## HOPE YOU FOUND THIS README USEFUL!!!
