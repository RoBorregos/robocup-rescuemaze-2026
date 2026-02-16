# robocup-rescuemaze-2026
Our robot is designed to autonomously explore the maze using a Depth First Search (DFS) exploration strategy to systematically map the environment and detect dead ends. Once the maze structure is partially or fully discovered, Dijkstra’s algorithm is applied to compute the shortest paths between key points, optimizing return routes and reducing overall mission time. During navigation, the robot must detect and classify Greek letter victims (Χ, Ψ, Ω) using computer vision (YOLO), as well as identify cognitive targets composed of five colored concentric rings using HSV masks. The health status of each cognitive target is determined by summing the numerical values associated with the ring colors. This integrated approach combines autonomous exploration, path optimization, and visual recognition.

## Roborregos Team Members 
| Name               | Github                                               |       Role   |
| ------------------ | ---------------------------------------------------- | -------------|
| Marco Galindo      | [@m2galindo](https://github.com/m2galindo)           | Programmer   |
| Santiago Ramírez   | [@Champtiago](https://github.com/Champtiago)         | Programer    |
| Salette            |	    			NA					                | Mechanic	   |
| Becca Gómez        |                  NA				                    | Electronic   |

This project follows the official rules of the RoboCup Junior Rescue Maze competition. The robot must:
	•	Autonomously navigate an unknown maze
	•	Detect and classify Greek letter victims (Χ, Ψ, Ω)
	•	Identify and evaluate cognitive targets
	•	Avoid obstacles and operate fully autonomously
	•	Complete the mission within the official time limit

The health status of cognitive targets is calculated by summing the numerical values assigned to each of the five colored concentric rings.

For the complete and official rulebook, see:

Official RoboCup Junior Rescue Maze Rules (2026)
https://junior.robocup.org/wp-content/uploads/2026/01/RCJRescueMaze2026-draft.pdf￼