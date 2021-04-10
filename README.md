# warmup_project

## 1. Driving in a square

My approach was to split the problem into two "directives". The first one moves the robot straight for a specified amount of time. The second one rotates the robot in place by 90 degrees. Both directives are handled by publishing to the `/cmd_vel` topic. In order to prevent drift, I decided to rotate the robot relatively slowly. Finally, I added a one second time delay in between these two directives in hopes that this would reduce noise in the environment.

My `run()` function handles the two directives described above in a continuous loop. My for loop on line 27 runs for 5 seconds (since the rate I defined was 2 Hz), and so I publish a Twist message to drive straight for 5 seconds. Similarly, my for loop on line 36 runs for 5 seconds and publishes a Twist message to rotate at 18 deg/s. 18deg/s * 5s = 90 degrees, thus the robot should turn at a right angle, assuming no noise. I decided to turn relatively slowly in hopes that this would prevent drift. 

Between the two directives in `run()`, I call my `_pause()` function which sends an empty Twist message to the robot for 1 second. My thinking here was to enable the robot's motors to completely stop after each directive, so that the robot would be able to turn its 90 degrees with high accuracy and little noise.

Finally, I specify a callback function on the event that the program gets interrupted (with CTRL+C), I called it `_stop_robot()`. This function just publishes an empty Twist message to the topic, which stops the robot in place.

![GIF](https://github.com/vorugantia/warmup_project/blob/main/gifs/drive_square.gif)

## 2. Person follower

My approach as to use the LiDAR scanner to capture the angle and distance of the person, and then to move towards that person until reaching a distance of 0.8m. Rotation is handled by proportional control relative to how far in the peripheral the person is situated relative to the robot. Motion forward is at a constant value, i.e. not proportional control. The robot first rotates towards the person in-place, and once it is roughly facing the person, it begins its motion forward.

`__init__()`: Initialize node, publisher to '/cmd_vel' topic, subscriber to '/scan' topic.

`_publish_vel()`: an internal helper function that publishes a motion directive to the '/cmd_vel' topic with specified linear/angular velocity parameters.

`_scan()`: This is the callback function upon scan messages received from LiDAR scanner. After locating exactly what angle and distance the person is relative to the robot, it handles the following cases: 1) If person is not in the vicinity of the scanner or too close (<0.8m), stop moving; 2) Set the angular velocity of robot according to a proportional control. 3) Stop micro-adjusting angle if the person is within 10 degrees of the robot's FOV (prevents oscillations); 4) Only move the robot forward if it is first facing the person (within 45 degrees of robot's FOV). The result angular/linear velocities are passed to `_publish_vel()`.

`run()`: Spins turtlebot

`_stop_robot()`: See drive_in_square above.

![GIF](https://github.com/vorugantia/warmup_project/blob/main/gifs/follow_person.gif)

## 3. Wall follower

My plan was to have the robot move forward until LiDAR scanner detected the wall in front at a certain distance. Then, have the robot turn clockwise in-place until it the LiDAR scanner picked up data that indicates the robot is parallel to the wall. Finally, repeat the process: have the robot move forward towards the next wall, then turn clockwise until parallel to the second wall.

`__init__()`: Initialize node, publisher to '/cmd_vel' topic, subscriber to '/scan' topic. I also introduce a `stage` variable that basically distinguishes whether the robot is in "move forward" mode or "turn clockwise" mode.

`_publish_vel()`: See above

`_scan()`: This is the callback function upon scan messages received from LiDAR scanner. in `stage = 1`, the robot moves forward at constant speed until a wall is within vicinity directly ahead (I used 0.3m). Then enter `stage = 2`, where the robot turns clockwise until facing parallel to the wall. In order to determine when the robot was facing parallel to the wall, I did some trigonometry:

## TODO: upload photo

Basically, we want to find out when the "front-right" sensor (at 315deg) and the "back-right" sensor (at 225deg) both read the value of the "right" sensor (at 270deg) times sqrt(2). This is trivially derived from Pythagorean's Thm. (I also add an error buffer of 0.025m.) Once this condition is met, the program goes back to `stage = 1` and repeats.

![GIF](https://github.com/vorugantia/warmup_project/blob/main/gifs/follow_wall.gif)

## 4. Challenges

One of the big challenges for me was to figure out how to get the robot to follow the person. Before trying a proportional control approach, I just set the angular velocity to a constant value with the instruction to rotate until the person was within robot's FOV. However this gave me a range of problems, from overshooting to moving erratically and seemingly uncontrollably. Furthermore, the debugging process was quite tedious when I was trying to figure out which lines were giving faulty published messages. Print statements helped me a lot. Furthermore, I thought back to lecture notes and implemented a proportional control approach, and I noticed immediate improvement with the robot's performance. All that I had to do was tweak the constant k a little bit.

Lack of proportional control remained a problem for me in wall follower. Because the robot is moving in the same direction as the forward scan data, it does not do a good job determining when to stop at the wall ahead. I think it would stop at a more accurate position if I implemented proportional control.

## 5. Future work

For my person follower program, I would be interested in applying proportional control to linear velocity. Another cool task within this project would be to have the robot always follow the person to their right side (or left side). This would require the robot to figure out which direction the person themself is facing, or perhaps which direction the person is walking.

For my wall follower program, I would certainly try to implement proportional control into the linear motion. The turning directive seems to be working just fine, but as I mentioned in "Challenges" above, there is some delay when the robot scans the environment directly in front of it and decides when to stop. I think the robot should come to a slow stop as it nears the wall in front of it.

## 6. Takeaways

1. Proportional control helped me solve the person follower task. It would have been useful in the wall follower as well.
2. In general, you have to be careful when you publish overlapping messages to the robot, such that one doesn't incorrectly come before the other, or you have to be careful to ensure a particular message stops getting published when the correct observation of the environment is met.
