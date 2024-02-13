This part was done by Alessio Zeni.

# Precise placement
### Why
We need a precise placement in the system because the Computer vision Part could only get a rough 2D placement of the block in pixel coordinates.
We instead need a precise block position and orientation in world coordinate. Maybe using the depth information from our zed Camera.

### From camera space to world space
It's not as easy as it seem. In particular because it depends a lot on the camera.
But luckily for us the camera has already a good calibration. This means that we could use the usual "dark room" model to transform from camera space to world space.
After some researches we discovered that the fov is 101°=0.88 rad over the x coordinates. With that knowledge if we move the picture with increasing z we get that tan(0.88/2)/(1920/2)*1920/2*(x-1920/2)*z and for y = tan(0.88/2)/(1920/2)*1920/2*(x-1920/2)*z and this is how much the beam diverged from the optical center (assuming 101° of vox and 1920x1080 pixel of resolutions).
Given the camera transformation we can get this rays in world space.

### Get point clouds and images
Our recognition script uses opencv as a library. This means that we must convert the images received from the ros service to suitable opencv images. For that we used the cvbridge library. Instead for the conversion of the point cloud we wrote a simple function that convert it pixel by pixel. This is far from ideal, and is really slow (around 7s for each point cloud). Because of that it has it's own thread that continues to converting non stop.  

### Connect to block_detector
The other part of the code that do the block detection is a simple ros-service,  where we could send an image and get the bounding boxes of the detected blocks.

### Filter
Having the point cloud we have to filter it, and the best way to do it is take the 3d bounding box of the work area, transform it in camera space (cheap, only 8 vertex) and then crop the point cloud. In this way we can filter out a lot of noise from the environment, and reduce considerably the size of the point cloud, from 2 million points to 200000 (more or less, it depends on how many blocks are present on the table).
In addition to that we could use this system to separate the floor points to the blocks points (simplifying the calculation after this), probably on a real camera we should have more depth noises, and probably it will not work as well as in the simulation, but reducing the working area could do the trick.

In the end we could transform the cropped point cloud in the world space, getting ready for the next part.

### Single block detection
firstly we should separate the various blocks with the data that we get from the block_detector service. We could create a "3d projected 2d bounding box" from the camera in this way:
- take the lowest y pixel coordinate, and use that to compute where we intersect the table
- use that dist to create a truncated pyramid, with one base at 10cm from the camera, and the other at the dept we computed
- transform all this mesh to world reference and intersect it with our point cloud.

Now we should have a pretty clean point cloud, with some points that doesn't belong to our block only if there is some overlapping of the bounding box in the camera view (there is not much that can be done to fix this issue)


Now we have to compute the ICP algorithm with the expected block (our source), and with our point cloud (our target).
firstly we compute a rough position (if we take the centroid of the point cloud and place inside our source facing up, and it should be pretty close to the final destination)
then we compute the ICP algorithm for 100 iterations (we observed that more iterations don't increase the precision).

Then we could apply our last optimization: we know that the blocks are facing upward, so we could discard the rotations around x and y axis.


We only push on our Publisher, and let the planner do his job. For letting do his job I take the fitness that I get from the ICP algorithm and multiply with the neural network confidence


### Visualizer
Obviously for debug reason we needed a good visualization on what's going on. Because of that I created a class that wraps the visualization api of Open3d and use it.
For reference, it has the usual pan control, and it must rotate freely. The colored point cloud is from the zed camera, and the blue ones are the placement tentative.
