# dwa_planner
接受dijkstra所发送的全局路径，运用dwa算法进行规划，可以自行添加障碍物，rviz中可视化可以看见障碍物（被dwa考虑道德障碍物会被高亮），还会显示dwa所追踪的点<br>

使用方法<br>
直接运行<br>
roslaunch by_dwa all.launch     #此launch包含仿真环境的启动和全局规划以及dwa节点的启动，启动后在rviz中通过2D Nav Goal选择目标点

![image](https://github.com/bydsg/dwa_planner/blob/main/pic/%E5%B1%8F%E5%B9%95%E5%BD%95%E5%83%8F%202023-06-04%2019%2029%2021.gif)
