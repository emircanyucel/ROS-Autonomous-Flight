# ROS Autonomous Flight
 Lidar Data Extraction and Image Processing with ROS 

I've set up a pretty simple obstacle avoidance algorithm here. Here, the drone moves in AUTO mode until 
it comes within 5 meters of the obstacle and performs normal waypoint tasks. If it comes within 5 meters of the obstacle,
it changes its mode to GUIDED and increases its altitude.

![1](https://user-images.githubusercontent.com/109924168/231140654-fb2fe544-0e7b-4fd8-a9f1-bace305c88f7.png)

![2](https://user-images.githubusercontent.com/109924168/231140678-e083567d-7320-4f49-9d2c-ad4cea34bb4c.png)

![3](https://user-images.githubusercontent.com/109924168/231140691-9b58225f-bd1a-4ddf-be97-2f377f0bcda2.png)


Here, I set up a simple precision landing algorithm. If the drone sees the landing strip while performing 
its waypoint missions in AUTO mode, it switches the mode to GUIDED mode. Then it activates the precision landing 
functions and makes the landing.
Here, the drone primarily focuses on the red zone. Once it aligns and gets close enough, it stops looking for the
red area and focuses on the blue area, making a much more precise landing. When it comes close to 30 cm, it puts 
itself in land mode and lands successfully.

![111](https://user-images.githubusercontent.com/109924168/231142076-124b8655-33f3-4066-842f-a0902092809e.png)

![2222](https://user-images.githubusercontent.com/109924168/231142092-6da5be41-4aba-4bbe-9332-38409e4fecf9.png)

![333](https://user-images.githubusercontent.com/109924168/231142102-61566483-1c54-4ca0-b001-6f1bd2b85499.png)

