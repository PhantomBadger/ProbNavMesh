# ProbNavMesh
My project for the DayKnight 30 day project challenge: https://dk30.day9.tv/projects/124225617133568001-1554195689917


# Description
Modifying a NavMesh to give each area a normalised probability value, representing the likelihood of the target being in that spot. 

An AI agent can then use the highest probability value to search around. When an agent spots it's target, the probability value of the area it's in becomes 1, and all others become 0. 

Once the agent loses sight of the target, the probability value propagates to adjacent areas. The use of a NavMesh as opposed to a grid should mean that larger open areas are searched more quickly, whereas winding paths or complex geometry takes longer to search.
