 


  
## Managing source package dependencies 
A source package must be build in order to use it. It is recommended to manage these packages using a .repos file. In this file, git repositories are listed in a .yaml file format and can be downloaded using vcs. To install vcs you can run: 
``` 
sudo apt-get install python3-vcstool 
``` 
Once installed, you can run the following to download all of the dependencies. Run the follwing in the `smart-home` folder.
``` 
vcs import < external.repos.yaml 
``` 

## Managing binary package dependencies 
A binary package can be installed without the need to build it. Official ROS packages can be installed as binary with the following: 
``` 
sudo apt install ros-{ROS_DISRO}-{PACKAGE_NAME} 
``` 
For example,   
``` 
sudo apt install ros-humble-control 
``` 
However, it is best practice to add all binary packages to the package xml (see [example](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#customize-package-xml)) and then install binary packages with  
``` 
rosdep install --from-paths src --ignore-src -y 
``` 

Note, the above command must be run at the root of the ROS workspace. 

## Managing meshes
Use Blender to view and edit mesh files. When exporting .obj files, make sure that Z is "up" and Y is "forward". 
The values can be chosen from the export options.  


# Smart Home Robot
The project is about i) designing a smart home equipped with a socially assistive robot (SAR) and serval
internet of things (IoT) devices and ii) evaluating the feasibility of using such a smart home to provide care-giving service for
elderly people with dementia. The SAR will execute a range of autonomous behaviors to communicate
with the occupant of the smart home as well as all IoT devices to ensure health and well-being of the
elderly occupant and the safety of the home. 

# Project Related Publication
[1] Tianyi Gu, Momotaz Begum, Naiqian Zhang, Dongpeng Xu, Sajay Arthanat, and Dain P. LaRoche, An Adaptive Software Framework for Dementia-care Robots. Proceedings of the ICAPS Workshop on Planning and Robotics (PlanRob-20), 2020. 

[[pdf]](http://cs.unh.edu/~tg1034/publication/shr_PlanRob2020.pdf) [[video]](https://youtu.be/MjQJuN2I3Vo) [[talk]](https://youtu.be/_laXuQWBT8U) [[slides]](http://cs.unh.edu/~tg1034/slides/PlanRob-2020-shr-slides.pdf)

[2] Sajay Arthanat, Momotaz Begum, Tianyi Gu, Dain P. LaRoche, Dongpeng Xu, and Naiqian Zhang, Caregiver Perspectives on A Smart Home-based Socially Assistive Robot for Individuals with Alzheimer's Disease and Related Dementia. Disability and Rehabilitation: Assistive Technology, 2020.

[[pdf]](http://cs.unh.edu/~tg1034/publication/shr_sajay.pdf)




