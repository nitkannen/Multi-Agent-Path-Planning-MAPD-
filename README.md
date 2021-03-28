# Multi-Label-Multi-Agent-Path-Finding
This repo contains a multi-agent multi-label path finding algorithm with simulation in OpenCV

Multi Agent Path Finding problem is a complex problem where multiple agents are assigned to perform a set of pickup-delivery tasks in a pre-defined warehouse map. The problem consists of assigning tasks to each agent and designing a path to pickup the task and deliver it in the desired location while avoiding collisions with other agents. Moreover, multiple agents are allowed to reside at certain blocks in the map (temporary storage, initial agent location, final agent location). The goal is to come up with a optimal approach (minimum make span) to carry out the given tasks. Figure 1 is a sample warehouse map.


![image](https://user-images.githubusercontent.com/48293666/112762331-bc71f100-901c-11eb-9853-a82b8e873dec.png)

# Algorithm used
1. Iterative Deepening A* (IDA*)
2. Best First Search (A*)
3. Depth First Search (DFS)       

# Required Softwares to be installed
1. C++ 14 compiler
2. cmake
3. OpenCV with C++ (above version 2.8)

# Steps to make project successfully

1. Clone the repository, and navigate to the downloaded folder. This may take a minute or two to clone due to the included image data.
	```
	git clone https://github.com/Abr820/Multi-Label-Multi-Agent-Path-Finding.git
	```
2. go to the directory where you saved
3. make the project bu running following command
    ```
    cmake .
    make MLMA
    ./MLMA
    ```
    
# Different flag used
```
--sumlate	|    bollean flag whether to simulate the process or not
--input		|    input file path for the input text
--output	|    output file path to save the path found for each agent
```
for example,
``` ./MLMA --simulate --input=<path to input file> --output=<path to output file> ```


    
    
    
