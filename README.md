# ENPM661  Project - 3

## Student Details

|Name|Jay Prajapati|Akash Parmar|
|---|:---:|:---:|
|UID|119208625|118737430|
|Directory ID|jayp|akasparm|
|Email ID|jayp@umd.edu|akasparm@umd.edu

## Dependencies used

|Module|Version|
|---|:---:|
|Python|3|
|Time|Default|
|Numpy|Default|
|heapq|Default|
|OpenCV|4.7.0|

## Links

|Item|Link|
|---|:---:|
|Github Repository|[here](https://github.com/jayprajapati009/astar_implementation.git)|
|Videos|[here](https://drive.google.com/drive/folders/1az9Xc2jjLH1F60jIFIt1WnjcL93X6fYJ?usp=sharing)|

## Run Code

Navigate to the ```Proj3_akash_jay``` folder after unzipping the folder,

```sh
python3 a_star_akash_jay.py
```

Enter the initial node and goal node as per the instruction in the terminal

- First enter the choice for entering the initial and the goal point
  - 1 for Manual entry
  - 2 for Preset Values
- Manual Input Parameters
  - Start Point x-coordinate
  - Start Point y-coordinate
  - Start Point theta (should be multiple of 30)
  - Goal Point x-coordinate
  - Goal Point y-coordinate
  - Goal Point theta (should be multiple of 30)
  - Clearance (should be 5)
  - Robot radius (should be 5)
  - Step size (should be between 1 to 10)

- Once the goal node is found, the simulation will be displayed in a window.
- Press the waitKey to end the code and get the mp4 file of the animation in the working directory
