NOTE: STILL UPDATING

References: 
1) ChatGPT with initial help with XML formation.

2) https://mujoco.readthedocs.io/en/stable/python.html - python binding documentation for MuJoCo



In this assignment, I look to apply an evolutionary algorithm to iteratively evolve a pre-selected geometry (inspired by a geodude - a floating mass with two muscular arms, that would aim to roll around and propel itself forward with its arms to learn motion) to potentially optimize the limitations of its geometry for a potentially more efficient mode of locomotion. Another motivation is to potentially study the limit of extra limbs, but cramming too many limbs into a very constrained space. 

The metric for best performing evolved version of a generation is determined entirely by the total displacements from the origin. This metric is largely unstable due to the randomized actuator motion, but given the instability of motion studied, it seems to be the most suitable metric for the time being. Furthermore, this metric is worse than distance for consistent motion optimization but is interesting in the pursuit of short but fast motions.

The best performing specimen of a specific generation is selected as the seed for the next generation, and appropriate metrics are varied by +/- 20% (size and mass). Addition of multiple limbs was considered, but the relative difficulty of randomly introducing multiple limbs onto a spherical body over multiple generations without arbitrary pre-selection deemed this less practical to implement - future work could look to evolve the limbs themselves and potentially look at alternative joint types for locomotion. Actuating segments are randomly added at varying locations within the main body mass, and this does propel the body further over certain amounts of limbs, but very quickly reaches a saturation point at which too many limbs/interfering limbs severely impact the capabilities of locomotion. Since the main body is on a free joint, the propulsion is coming via the action of imbalanced torques from the multiple appendages that are growing on the body.

The evolutionary process further added a degree of variance wherein offspring would randomly not show any mutations from their parents - this is done in order to further add some variance into the generated population and potentially conduct a large number survey over 100s of generations.

**To run this program:**
To run the code, download the following: geodude.xml and ALife.py. 
Execute ALife.py

**Alternatively**
To run the code, download the following: geodude.xml and ALife.ipynb 
Execute ALife.ipynb in its entirety.

**Parameters to modify**
The evolution function consists of 4 parameters:
1) The initial XML file - in this case, geodude.xml.
2) No. of generations to execute the evolutionary algorithm for
3) No. of specimens per generation
4) No. of iterations to simulate and evaluate each speciment

Generation 0 will generate x random specimens as mentioned above, and then the evolutionary algorithm will select the best performing specimen and further evolve that in generation 1, and so on until the last generation.

Observations:
Over the course of the evolution, the specimen tends to evolve such that mass on one arm is minimized and mass on the other is maximized, allowing for potentially explosive but highly unstable and unpredictable motions. It also clearly reached a point of saturation of extra limbs, beyond which functionality is quite compromised.
This unstable motion doesn't look to be constantly iterable on for improved results - adding too much mass on one limb at the cost of mass on the other is detrimental. This is also observable in the fitness measures observed, where in the specimens do not tend to steadily increase in function over further generations.


3D Printed Robot - COMING SOON!
