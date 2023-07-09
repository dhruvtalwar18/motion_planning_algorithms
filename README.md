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
  <p align="center">Fig.7 Effects of Heuristic Function on Path length </p>
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








