# A-Star-and-Theta-Star

The project utilizes the A* and Theta* algorithms to find an optimal path from a randomly generated start point to a goal point. 
The grid is made up of 10% blocked cells which the path cannot cross. 

Before running the program, it is important to: 
  * create a 'grids' folder in the same directory as the main file
  * create a 'gridFiles' folder in the same directory as the main file

The grid can be set to any size, however the Default size is set to 100x and 50y.
The program can also generate multiple random grids at the same time by specifying the 'number of grid' value. 
The default number of grids is set to 50, which should be changed to a lower number to speed up the runtime of the program.
'Number of grid' recommended to be set to 1-15 for fast processing of random grids. 

*** NOTE *** 
You need matplotlib to be installed for this program to display the GUI
If not already installed on your device, run this command in the terminal: 
       python -m pip install -U matplotlib
