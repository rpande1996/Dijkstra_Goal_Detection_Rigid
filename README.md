## AR_Plotting
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
---
## Contributors

1) [Rajan Pande](https://github.com/rpande1996)
Graduate Student of M.Eng Robotics at University of Maryland. 
2) [Abhilash Mane](https://github.com/abhilash1998)
Graduate Student of M.Eng Robotics at University of Maryland.

## Overview

This code utilizes for Dijkstra algorithm for path planning for a rigid robot of a certain radius and 
certain clearance from the obstacles. This code visually represents the clearance bubble around obstacles 

## Softwares

* Recommended IDE: PyCharm 2021.2

## Libraries

* Numpy 1.21.2
* OpenCV 3.4.8.29

## Programming Languages

* Python 3.8.12

## License 

```
MIT License

Copyright (c) 2021 Rajan Pande

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
SOFTWARE.
```

## Demo

Following demo is for following parameters:
```
Robot radius: 5
Clearance: 2
Start Point: (8,8)
Goal Point: (390, 290)
```

- [Dijkstra Algorithm Representation:](https://youtu.be/vvrBrp0YNy4)

![ezgif com-gif-maker](https://github.com/rpande1996/Dijkstra_Goal_Detection_Rigid/blob/main/media/gifs/output.gif)

## Build

```
git clone https://github.com/rpande1996/Dijkstra_Goal_Detection_Rigid
cd Dijkstra_Goal_Detection_Rigid/src
python Dijkstra_Goal_Detection_Rigid/.py
```
Enter the following parameters:-
```
Enter radius of the robot:
Enter clearance:
Enter start node x postion:
Enter start node y postion:
Enter goal node x postion:
Enter goal node y postion:
```
Note: The start and goal nodes determine the position of the centre of the robot, consider the robot radius and clearance while selecting these coordinates
