# turtle_circle


# Installation

Pour installer ce noeud il faut le cloner dans le dossier src de votre catkin workspace (catkin_ws)


installe turtlesim: http://wiki.ros.org/turtlesim


commande a suivre:


 	cd catkin_ws/src
 	
 	git clone https://github.com/ialy/turtle_circle.git
 	
 	catkin build
 	
	cd ..
	
	source devel/setup.bash
	

# compilation

afin de compiler les noeuds, on peut proceder avec:
	

 ## server

activer la partie serveur de ros



  	Ouvrez un   terminal: ctrl + alt + t  
 
	executer :  roscore
 
 
 ## roslaunch

  	Ouvrez un autre  terminal: ctrl + alt + t  
  
 	aller dans le dossier catkin_ws: cd catkin_ws
  
	sourcer:  source le devel/setup.bash

	executer :  roslaunch turtle_circle turtle_circle.launch


