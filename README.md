# Multi-Agent-Pickup-Delivery
This repo contains a multi-agent multi-label path finding algorithm with simulation in OpenCV

# 01. Introduction
Multi Agent Path Finding problem is a complex problem where multiple agents are assigned to perform a set of pickup-delivery tasks in a pre-defined warehouse map. The problem consists of assigning tasks to each agent and designing a path to pickup the task and deliver it in the desired location while avoiding collisions with other agents. Moreover, multiple agents are allowed to reside at certain blocks in the map (temporary storage, initial agent location, final agent location). The goal is to come up with a optimal approach (minimum make span) to carry out the given tasks. Figure 1 is a sample warehouse map.


![image](https://user-images.githubusercontent.com/48293666/112762331-bc71f100-901c-11eb-9853-a82b8e873dec.png)

# 2. Methodology
The given problem statement is an offline version of Multi Agent Pickup and Delivery (MAPD), as opposed to the online version where tasks get updated in real time. In our case, all the tasks, task pickup and delivery and robot initial and final positions are available to us beforehand. We propose a novel Agent-Task pair scheduling approach using the conventional Iterative Deepening A* (IDA*)[1] algorithm. The heuristics used in this approach is an underestimate of the start (initial position)-> goal (final position of agent after completing all allotted tasks) computed using Floyd Warshall [2] Algorithm for All Pair Shortest Paths (APSP) in the given map. The details of how this heuristic is put to use is explained in the coming sections. We then employ an independent Multi Label A* algorithm for each agent using its start and goal locations along with the labels as defined by the scheduler. This algorithm is further explained in the coming sections. There have been several approaches in the past such as [3], [4], [5]. Figure 2 is a visual overview of the complete approach proposed in this paper.

![image](https://user-images.githubusercontent.com/48293666/112764047-89cbf680-9024-11eb-912c-c02c427f3d93.png)


# 3. Results

We visualize the movement of agents with their task using OpenCV library. The compute and memory complexity of the algorithm is reasonable and generates all the frames withing a few seconds for agents in the order of ~10-20. The proposed algorithm is scalable for a denser grid with a greater number of agents and task too. The following images are retrieved from a simulation for a particular test case obtained at different time steps.

![image](https://user-images.githubusercontent.com/48293666/112764264-5dfd4080-9025-11eb-84c2-64a178585762.png) ![image](https://user-images.githubusercontent.com/48293666/112764276-6b1a2f80-9025-11eb-90e8-ad0575492154.png) 
Step 1												Step 2	


![image](https://user-images.githubusercontent.com/48293666/112764282-71a8a700-9025-11eb-9908-025f53cbb4a7.png)	![image](https://user-images.githubusercontent.com/48293666/112764357-bb918d00-9025-11eb-89d2-f7aeac7fe6ec.png)
Step 3												Step 4

![image](https://user-images.githubusercontent.com/48293666/112764376-d95ef200-9025-11eb-9269-69cec2deb5c4.png)	![image](https://user-images.githubusercontent.com/48293666/112764380-e1b72d00-9025-11eb-98ea-baae4b1b174c.png)
Step 5												Step 6

![image](https://user-images.githubusercontent.com/48293666/112764518-95202180-9026-11eb-83e0-717ac3a9a1d7.png)	![image](https://user-images.githubusercontent.com/48293666/112764530-9bae9900-9026-11eb-8b4b-04c8670090e6.png)
Step 7												Step 8

Fig. 3. A visual representation of the solution obtained using the proposed approach. Red- agent, Green- task, 
Black – blockage, Yellow- temporary storage


# 4. Conclusion

In this assignment we propose a novel algorithm for solving the Offline version of Multi Agent Pick-up and Delivery (MAPD). We leverage the fact that all tasks and agent locations (initial and final) are given to us, and we schedule the agent-task combinations optimally using IDA*. We then employ a modified MLA* algorithm that updates the path for each agent. We resolve collisions using heuristics and optimally provide unique paths to each agent in order to minimize total timespan. We use the Floyd Warshall algorithm to compute heuristics at each time step of MLA* and show that Floyd Warshall is a good underestimate and a good heuristic for path finding algorithms. Out final simulation Figure 3 shows the working simulation of the proposed approach. 


# Algorithm used
1. [Iterative Deepening A* (IDA*)](https://www.geeksforgeeks.org/iterative-deepening-searchids-iterative-deepening-depth-first-searchiddfs/)
2. [Best First Search (A*)](https://www.geeksforgeeks.org/best-first-search-informed-search/)
3. [Depth First Search (DFS)](https://www.geeksforgeeks.org/depth-first-search-or-dfs-for-a-graph/)       

# Required Softwares to be installed
1. C++ 14 compiler
2. cmake
3. OpenCV with C++ (above version 2.8)

	[go to this link for installation](https://medium.com/@theorose49/install-opencv-at-ubuntu-18-04-lts-c60bd45bd040)

# Steps to make project successfully

1. Clone the repository, and navigate to the downloaded folder. This may take a minute or two to clone due to the included image data.
	```
	git clone https://github.com/Abr820/Multi-Agent-Multi-Pickup-Delivery.git
	```
2. go to the directory where you saved
3. make the project bu running following command
    ```
    cmake .
    make MAPD
    ./MAPD
    ```
    
# Different flag used
```
--simulate	|    boolean flag whether to simulate the process or not
--input		|    input file path for the input text
--output	|    output file path to save the path found for each agent
```
for example,
``` ./MAPD --simulate --input=<path to input file> --output=<path to output file> ```


    
    
    
