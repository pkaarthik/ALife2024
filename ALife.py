#!/usr/bin/env python
# coding: utf-8

# In[1]:


import mujoco as mj
import dm_control.mujoco
from mujoco import viewer
import random
import os
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET




# In[2]:


#### Evolutionary Algorithm
def evolution(xml, gen, bodies_per_gen, iterations_body):


    i=0 #generation iteration counter
    j=0 #body iteration counter
    
    best_body_fitness = 0
    best_body = 0
    
    best_gen_fitness = []
    gen_list = []
    xml_list = []
    best_specimen = []
    
    while i<gen:
        while j<bodies_per_gen:
            fitness = mutate(xml,i,j, iterations_body)                    #Mutate the model, simulate and determine the fitness of the model       
            xml_list.append(f"Gen{i}Body{j}.xml")
            if fitness > best_body_fitness:
                best_body_fitness = fitness
                best_body = j
            j+=1
            
        if j==bodies_per_gen:
            print(best_body_fitness, best_body)
            best_gen_fitness.append(best_body_fitness)
            
            if best_body_fitness > np.argmax(best_gen_fitness):              #Compare to fitness value of prev generations to see what the best evolution seed is
                xml = xml_list[best_body]
                best_specimen.append(best_body)
            j=0
            
        
        gen_list.append(i+1)
        #print(best_gen_fitness)
        xml_list = []  #reset values
        best_body_fitness = 0   
        best_body = 0
        
        i+=1
    
    print(best_gen_fitness)
    print(best_specimen)
    plot(gen_list, best_gen_fitness)


# In[3]:


### MUTATE ###
def mutate(best_xml_file,i,j, iterations_body):

    num = random.uniform(0,1)
    print(num)
    if num<=0.5:
        
        tree = ET.parse(best_xml_file)
        root = tree.getroot()
    
        mujoco = ET.Element('mujoco')
        worldbody = ET.SubElement(mujoco, 'worldbody')
        light = ET.SubElement(worldbody, 'light', diffuse=".5 .5 .5", pos="0 0 3", dir="0 0 -1")
        plane = ET.SubElement(worldbody, 'geom', name="ground", type="plane", size="50 50 0.1", rgba=".9 .9 .9 1")
        actuator = ET.SubElement(mujoco, 'actuator')
        bodyframe= ET.SubElement(worldbody, 'body', name="geodude_body", pos="0 0 1") 
    
        joint_geodude = ET.SubElement(bodyframe, 'joint', type = "free")
        body_geodude = ET.SubElement(bodyframe, 'body', name = "main_body", pos = "0 0 0")
        geom_geodude = ET.SubElement(bodyframe, 'geom', type = "sphere", size = "0.12", rgba = "1 0 0 1", pos = "0 0 0", mass ="5")
    
        num_arms = 0
        arm_range = []
        arm_gear = []
        arm_mass = []
        
        
        for body in root.findall('.//body'):
            if body.get('name', '').startswith('arm'):
                num_arms+=1
    
        for body in root.findall('.//joint'):
            if body.get('name', '').startswith('arm'):
                arm_range.append(body.get('range'))
    
        for body in root.findall('.//motor'):
            if body.get('name', '').startswith('arm'):
                arm_gear.append(body.get('gear'))
    

        num_arms+=1
        count = 1
        pos0 = random.uniform(0,0.5)
        pos1 = random.uniform(0,0.5)
        pos2 = 0

        while count <= num_arms:
            if count == 1:
                pos0 = -0.2
                pos1 = 0
            elif count == 2:
                pos0 = 0.2
                pos1 = 0
            else:
                pos0 = random.uniform(-0.1,0.1)
                pos1 = random.uniform(-0.1,0.1)
            arm = ET.SubElement(bodyframe, 'body', name = "arm"+str(count), pos =f"{pos0} {pos1} {pos2}")
            #print("Success 1")       
            joint_geom = ET.SubElement(arm, 'geom', name="arm" + str(count), type="capsule", size="0.05 0.02",  rgba="0 1 0 1", mass="0.01")
            #print("Success 2")
            joint_hinge = ET.SubElement(arm, 'joint', name="arm" + str(count), type="hinge", axis="1 0 0")
            #print("Success 3")
            if count == 1:
                body_hand = ET.SubElement(arm, 'body', name = "hand" + str(count), pos = "-0.08 -0.05 0")
                geom_hand = ET.SubElement(body_hand, 'geom', type = "sphere", size = "0.03", rgba = "0 0 1 1", mass = "1")
            elif count == 2:
                body_hand = ET.SubElement(arm, 'body', name = "hand" + str(count), pos = "0.08 -0.05 0")
                geom_hand = ET.SubElement(body_hand, 'geom', type = "sphere", size = "0.03", rgba = "0 0 1 1", mass = "1")
                
            motor = ET.SubElement(actuator, 'motor', name ="arm" + str(count), joint = "arm" + str(count), gear = "0.2", ctrllimited = "true", ctrlrange = "-60 60")
            #print("Success 4")  
            count+=1 
        
        with open(f"Gen{i}Body{j}.xml", 'wb') as file:
            tree = ET.ElementTree(mujoco)
            #tree.write(file) 
            tree.write(f"Gen{i}Body{j}.xml")

        
        num_arms = 0
    

    
    else:
        
        tree = ET.parse(best_xml_file)
        root = tree.getroot()
        #mutate if random generation is >0.25 to throw further variability into each generation        
        ###Modify main body size and mass and arm size and mass
        body_geom = root.find(".//body[@name='geodude_body']/body/geom")
        body_geom.set('mass', str(float(body_geom.get('mass'))*random.uniform(0.85,1.15))) #body mass change
        #body_geom.set('size', str(float(body_geom.get('size'))*random.uniform(0.85,1.15))) #body size change
        
        arm_geom = root.find(".//body[@name='arm1']/body/geom")
        arm_geom.set('mass', str(float(arm_geom.get('mass'))*random.uniform(0.85,1.15)))  #arm mass change
        arm_geom.set('size', str(float(arm_geom.get('size'))*random.uniform(0.85,1.15)))  #arm size change
        
        arm_geom = root.find(".//body[@name='arm2']/body/geom")
        arm_geom.set('mass', str(float(arm_geom.get('mass'))*random.uniform(0.85,1.15)))  #arm mass change
        arm_geom.set('size', str(float(arm_geom.get('size'))*random.uniform(0.85,1.15)))  #arm size change

        arm_geom = root.find(".//body[@name='arm1']/body/geom")
        arm_mass = arm_geom.get('mass')  #arm mass change
        arm_size = arm_geom.get('size')  #arm size change

         
        tree.write(f"Gen{i}Body{j}.xml")   #write to new xml files
        #tree.write("new.xml") 

    model = mj.MjModel.from_xml_path(f"Gen{i}Body{j}.xml") #read to mujoco model viewer
    data = mj.MjData(model)  
     
    joint_range = 45 #arm motion range
    
    iterations = 0 #iteration counter
    i=0

    with mj.viewer.launch_passive(model, data) as viewer:
        viewer.cam.azimuth = 45                                       #View parameters
        viewer.cam.elevation = -20
        viewer.cam.distance = 10

        while iterations < iterations_body:
            iterations += 1 

            while i < len(data.ctrl):  #
                ctrl = np.deg2rad(joint_range * random.uniform(-1,1))  #Randomize angle of actuation within a joint range
                data.ctrl[i] = ctrl
                i+=1
            i=0

            dm_control.mujoco.mj_step(model, data)
            #viewer.sync()                              ### Comment these two lines out to speed up code heavily
            #time.sleep(1/1000)
                
            
    final_pos = data.qpos[:3]
    #print(final_pos)
    fitness = np.sqrt((final_pos[0])**2+(final_pos[1])**2+(final_pos[2])**2) #fitness is just the total distance traveled by the geodude from the origin, ignoring the drop distance
    #print(fitness)  
    return fitness


# In[4]:


def plot(gen_list, best_gen_fitness):   #plot best fitness of each generation, which is the seed for the next generation
    plt.figure()
    plt.plot(gen_list, best_gen_fitness)  
    plt.xlabel('Generation')
    plt.ylabel('Highest Fitness')
    plt.title('Best Evolved fitness Values')
    plt.show()
    plt.savefig("fitness.png")


# In[7]:


#evolution("geodude.xml", 20, 20, 5000)


# In[6]:


if __name__ == "__main__":
    evolution("geodude.xml", 10, 100, 5000)


# In[ ]:





# In[ ]:




