# Exploration and analysis of Search-based versus Sampling-based motion planning algorithms.

<h1><b> Overview </b></h1>
This project aims to conduct a comprehensive comparison between search-based and sampling-based motion planning algorithms. Specifically, the A* and RRT* algorithms are evaluated and analyzed. The performance of the A* algorithm is further investigated by exploring various heuristic functions. The comparison is conducted across seven distinct environments, each featuring 3D obstacles, to provide a robust assessment of algorithmic capabilities and limitations.


<h1><b> A* Results </b></h1>

Below given images are of A* path with eucledian heursitic for different maps.



<table>
  <tr>
    <td align="center">
      <img src="https://github.com/dhruvtalwar18/motion_planning_algorithms/blob/main/Results/A_star/Cube_e.png" title="A* on Cube Map" style="width: 400px; height: 400px;">
      <br>
      <p align="center">Fig.1 A* path on Cube Map</p>
    </td>
    <td align="center">
      <img src="https://github.com/dhruvtalwar18/motion_planning_algorithms/blob/main/Results/A_star/Flappy_bird_e.png" title="A* on Flappy Bird Map" style="width: 400px; height: 400px;">
      <br>
      <p align="center">Fig.2 A* on Flappy Bird Map</p>
    </td>
  </tr>
</table>
<table>
  <tr>
    <td align="center">
      <img src="https://github.com/dhruvtalwar18/motion_planning_algorithms/blob/main/Results/A_star/Room_e.png" title="A* on Room Map" style="width: 400px; height: 400px;">
      <br>
      <p align="center">Fig.3 A* path on Room Map</p>
    </td>
    <td align="center">
      <img src="https://github.com/dhruvtalwar18/motion_planning_algorithms/blob/main/Results/A_star/Monza_e.png" title="A* on Monza Map" style="width: 400px; height: 400px;">
      <br>
      <p align="center">Fig.4 A* path on Monza Map</p>
    </td>
  </tr>
</table>
<table>
  <tr>
    <td align="center">
      <img src="https://github.com/dhruvtalwar18/motion_planning_algorithms/blob/main/Results/A_star/Tower_e.png" title="A* on Tower Map" style="width: 400px; height: 400px;">
      <br>
      <p align="center">Fig.5 A* path on Tower Map</p>
    </td>
    <td align="center">
      <img src="https://github.com/dhruvtalwar18/motion_planning_algorithms/blob/main/Results/A_star/Window_e.png" title="A* on Window Map" style="width: 400px; height: 400px;">
      <br>
      <p align="center">Fig.6 A* path on Window Map</p>
    </td>
  </tr>
</table>


<p align="center">
    <img src="https://github.com/dhruvtalwar18/motion_planning_algorithms/blob/main/Results/A_star/maze_e.png" title="A* on Maze Map" style="width: 400px; height: 400px;">
  <br>
  <p align="center">Fig.7 A* on Maze Map</p>
</p>

<b> Effect of different heuristic function </b> <br>
In many pathfinding scenarios, especially in grid-based
maps where the optimal path tends to be straighter,
distance heuristics such as Manhattan and Euclidean distances can produce better results. Thus in maps such as
Cube, Flappy Bird, Window, Room and Monza, Manhattan and Euclidean perform better than Chebyshev, in
which Manhattan giving better result for the Room map
In certain maps, such as Maze and Tower, where there
are a high number of obstacles, the Chebyshev heuristic
tends to produce better paths compared to the Euclidean
and Manhattan heuristics. This can be attributed to the
characteristics of the Chebyshev distance metric.
The Chebyshev distance considers diagonal movements
in addition to horizontal and vertical movements, which allows it to better approximate the true distance in a
grid-like environment. In maps with complex obstacle
configurations, the Chebyshev heuristic can account for
diagonal shortcuts that may exist, leading to more direct
paths. 

<p align="center">
    <img src="https://github.com/dhruvtalwar18/motion_planning_algorithms/blob/main/Results/A_star/Heuristic_fn.png" title="Effects of Heuristic Function" style="width: 600px; height: 400px;">
  <br>
  <p align="center">Fig.8 Effects of Heuristic Function on Path length </p>
</p>



<h1><b> RRT* Results </b></h1>

Below given images are of RRT* path with Uniform Sampling for different maps.



<table>
  <tr>
    <td align="center">
      <img src="https://github.com/dhruvtalwar18/motion_planning_algorithms/blob/main/Results/RRT_star/Cube_gb.png" title="RRT* on Cube Map" style="width: 400px; height: 400px;">
      <br>
      <p align="center">Fig.1 RRT* path on Cube Map</p>
    </td>
    <td align="center">
      <img src="https://github.com/dhruvtalwar18/motion_planning_algorithms/blob/main/Results/RRT_star/flappy_bird_gb.png" title="RRT* on Flappy Bird Map" style="width: 400px; height: 400px;">
      <br>
      <p align="center">Fig.2 RRT* on Flappy Bird Map</p>
    </td>
  </tr>
</table>
<table>
  <tr>
    <td align="center">
      <img src="https://github.com/dhruvtalwar18/motion_planning_algorithms/blob/main/Results/RRT_star/Room_gb.png" title="RRT* on Room Map" style="width: 400px; height: 400px;">
      <br>
      <p align="center">Fig.3 RRT* path on Room Map</p>
    </td>
    <td align="center">
      <img src="https://github.com/dhruvtalwar18/motion_planning_algorithms/blob/main/Results/RRT_star/monza_gb.png" title="RRT* on Monza Map" style="width: 400px; height: 400px;">
      <br>
      <p align="center">Fig.4 RRT* path on Monza Map</p>
    </td>
  </tr>
</table>
<table>
  <tr>
    <td align="center">
      <img src="https://github.com/dhruvtalwar18/motion_planning_algorithms/blob/main/Results/RRT_star/tower_gb.png" title="RRT* on Tower Map" style="width: 400px; height: 400px;">
      <br>
      <p align="center">Fig.5 RRT* path on Tower Map</p>
    </td>
    <td align="center">
      <img src="https://github.com/dhruvtalwar18/motion_planning_algorithms/blob/main/Results/RRT_star/window_gb.png" title="RRT* on Window Map" style="width: 400px; height: 400px;">
      <br>
      <p align="center">Fig.6 RRT* path on Window Map</p>
    </td>
  </tr>
</table>


<p align="center">
    <img src="https://github.com/dhruvtalwar18/motion_planning_algorithms/blob/main/Results/RRT_star/Maze_gb.png" title="RRT* on Maze Map" style="width: 400px; height: 400px;">
  <br>
  <p align="center">Fig.7 RRT* on Maze Map</p>
</p>

<b> Effect of different Sampling Methods </b> <br>
In the maps where gaussian Sampling was able to find a
path, it tends to produce shorter path lengths compared
to other sampling methods. All except monza and Maze
where there had to be a direction for the tree to expand,
it was not able to reach to the goal as these maps were
more of guided maps. <br>
This is because this sampling method tends to distribute
the samples evenly across the space, covering a wider
range of configurations. In the context of path planning,
Gaussian sampling can explore different areas of the map,
potentially leading to more diverse and flexible paths. This
increased exploration can sometimes result in shorter path
lengths as it allows the algorithm to find more direct or
efficient routes. The drawback is that gaussian sampling
uses or visits a large number of nodes which result in the
RRT* algorithm to be slow. <br>

Even though from the graph we can see that gaussian
sampling is giving better results but the time taken as
compared between Gaussian and uniform, Gaussian takes
more than 3 times the time as compared to uniform
and goal biased While uniform sampling ensures equal
coverage across the space, it may not capture the specific
characteristics or features of the map that could lead to
shorter paths. <br>


<p align="center">
    <img src="https://github.com/dhruvtalwar18/motion_planning_algorithms/blob/main/Results/A_star/RRT_star.png" title="Effects of Heuristic Function" style="width: 600px; height: 400px;">
  <br>
  <p align="center">Fig.8 Effects of Sampling on Path length </p>
</p>


<h1><b> A* vs RRT* Comparison </b></h1>

<br>
The comparison between the A* and RRT*
algorithms provides valuable insights into their performance characteristics in various map scenarios. The A*
algorithm consistently produces better-quality paths with
lower path lengths across different maps, thanks to its
systematic exploration guided by heuristic functions. On
the other hand, RRT* algorithm demonstrates its strength
in handling maps with high obstacle density, where it can
generate shorter paths using Gaussian sampling compared
to other sampling methods. A* may require more computational time especially in high dimensional spaces. The
choice between A* and RRT* algorithms depends on the
specific requirements of the navigation task, considering
factors such as path quality, runtime, and the characteristics of the map. Further research and experimentation are
necessary to explore the performance of these algorithms
in more diverse and complex environments, ultimately
advancing autonomous navigation capabilities in robotics.



<div style="display: flex;">
  <div style="width: 50%;">
    <p align="center">
      <img src="https://github.com/dhruvtalwar18/motion_planning_algorithms/blob/main/Results/Path_length_rrt_vs_a_.png" title="Path Length Comparison A* vs RRT*" style="width: 600px; height: 400px;">
      <br>
      <p align="center">Fig.1 Path Length Comparison A* vs RRT* </p>
    </p>
  </div>
  <div style="width: 50%;">
    <p align="center">
      <img src="https://github.com/dhruvtalwar18/motion_planning_algorithms/blob/main/Results/rrt_vs_a_.png" title="Number of Nodes" style="width: 600px; height: 400px;">
      <br>
      <p align="center">Fig.2 No. of Nodes comparison A* vs RRT* </p>
    </p>
  </div>
</div>


<h1><b> Code Installation </b></h1>

Git clone the repository and install the requirements
```
git clone https://github.com/dhruvtalwar18/motion_planning_algorithms
cd motion_planning_algorithms
pip3 install -r requirements.txt
```

Change the function to run in the __main__ fn to run A* on different maps
To run A* on flappy bird map

```
test_flappy_bird(resolution,epsilon,True) 
```
To change the heuristic function, change the heuristic param in the heuristic fucntion
```
def heuristic(cell, goal, epsilon, heuristic_type='euclidean'):
```
<br>
Change the map fn to run RRT* on different maps
To run RRT* on flappy bird map, from the RRT_star.py

```
mapfile = './maps/flappy_bird.txt'
start = np.array([0.5,2.5,5.5])
goal = np.array([19.0,2.5,5.5]) #Flappy_bird
```

To change the sampling method select 

```
############################################## Choose Sampling Method  ##########################################
planner = RRTStarPlanner(boundary, blocks, resolution, sampling_method='uniform')
# planner = RRTStarPlanner(boundary, blocks, resolution, sampling_method='gaussian')
# planner = RRTStarPlanner(boundary, blocks, resolution, sampling_method='goal_biased')
```





