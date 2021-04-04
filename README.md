# warmup_project

## 1. Driving in a square

My approach was to split the problem into two "directives". The first one moves the robot straight for a specified amount of time. The second one rotates the robot in place by 90 degrees. Both directives are handled by publishing to the `/cmd_vel` topic. In order to prevent drift, I decided to rotate the robot relatively slowly. Finally, I added a one second time delay in between these two directives in hopes that this would reduce noise in the environment.

My `run()` function handles the two directives described above in a continuous loop. My for loop on line 27 runs for 5 seconds (since the rate I defined was 2 Hz), and so I publish a Twist message to drive straight for 5 seconds. Similarly, my for loop on line 36 runs for 5 seconds and publishes a Twist message to rotate at 18 deg/s. 18deg/s * 5s = 90 degrees, thus the robot should turn at a right angle, assuming no noise. I decided to turn relatively slowly in hopes that this would prevent drift. 

Between the two directives in `run()`, I call my `_pause()` function which sends an empty Twist message to the robot for 1 second. My thinking here was to enable the robot's motors to completely stop after each directive, so that the robot would be able to turn its 90 degrees with high accuracy and little noise.

Finally, I specify a callback function on the event that the program gets interrupted (with CTRL+C), I called it `_stop_robot()`. This function just publishes an empty Twist message to the topic, which stops the robot in place.



## 2. Person follower

Description, code explanation, gif

## 3. Wall follower

Description, code explanation, gif

## 4. Challenges

(1 paragraph)

## 5. Future work

(1 paragraph)

## 6. Takeaways

(2 bullet points)
