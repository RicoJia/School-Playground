# ME 495 Final Project - Tic Tac Toe Artist on Sawyer

## Project Overview
The Tic Tac Toe artist is a Sawyer robot based interactive robot Tic Tac Toe player. This project allows a human to play a game of Tic Tac Toe against the Sawyer robot on a dry-erase board. We programmed the robot arm to execute three objectives: read the current board state, compute its next optimal move,
 and perform the trajectory needed to draw an X in that board space. At the current stage of development, the robot goes first and uses the [perfect Tic Tac Toe strategy](https://www.wikihow.com/Win-at-Tic-Tac-Toe), making it nearly impossible to win against the robot. This level of brutality will be neutralized in future work.

##### Game Demonstration
[![Screenshot from 2019-12-13 13-39-59](https://user-images.githubusercontent.com/39393023/70827219-61c46f80-1dae-11ea-81c7-01a2741d246f.png)](https://youtu.be/J4vcd4qHMO0)

## Quickstart Guide
### Equipment
* Sawyer arm
* Handheld dry-erase board (with electrical tape as the board lines and a 2cmx2cm brightly-colored piece of tape placed at the top left corner of the top left board space)
* 2 Dry-erase markers
* Eraser
* 2 Clamps
* Table

### Usage
- To correctly set up the game, the dry-erase board is required with a fiducial point for board orientation recognition. Use the two clamps to secure the board to the table.

![Screenshot from 2019-12-13 13-14-38](https://user-images.githubusercontent.com/39393023/70825660-90404b80-1daa-11ea-80b3-6a5e40704f1a.png)

- On your game computer, Run `rosrun final-project-tic-tac-toe tic_tac_toe`. Make sure the board is not further than 5 cm from the end effector along the vertical direction. Otherwise, initial interaction force might be more than desired.
- When you play the game, hit enter to signal the computer that you have made your move.

## Code Overview
![Screenshot from 2019-12-13 12-46-22](https://user-images.githubusercontent.com/39393023/70823906-99c7b480-1da6-11ea-8725-56962d8e52a5.png)

### Hybrid Motion-Force Control

This part is inside the main code, `tic_tac_toe`.  Desired position on the board plane (x,y) is accesed by `trajectoryGenerator.get_xy()`.
In addition to that, the information of whether the robot should draw, move in the air, or get away from the board is accessed by `trajectoryGenerator.get_draw_status()`.
The details of how these functions work is explained in the **Trajectory Generation** section.

**_When the robot needs to draw_**:
Force control is used to decide on the desired _z_ position of the end effector.
The force control problem is converted to a position control problem.
Desired position along the *z* axis (axis that is perpendicular to the board) is set by using a simple P controller as follows:

```
desired_Z_pos_InHand = desired_Z_posInHand + P*(desired_EE_Force - currentForce)
desired_Z pos_InWorld = Current_Z_pos - desired_Z_pos_InHand
```

Change in the desired `z` position in hand frame is the output of the controller; it does not depend on the current end effector position.
This is important because after the marker gets in contact with the board, its _z_ position is constant.

If the applied force is not enough, the desired z position will be further from the end effector until it overshoots and then comes to a balance.

**_When the robot needs to lose contact with the board_**: desired _z_ position is set 1.5 cm above the current end effector position.

**_When the robot needs to move in the air_**: desired _z_ position is set to the current end effector position.

After the desired _z_ position is found, it is used in combination with the desired (x,y) position obtained from the
TrajectoryGeneration class and given as an input to the built in inverse kinematic function of the Sawyer robot.

### Trajectory Generation

This part encompasses all motions of the robot except drawing using force control. Along the entire trajectory there should not be any jerks or unsafe moves. The motions of the robot are illustrated in the state machine diagram below

![Screenshot from 2019-12-12 22-37-46](https://user-images.githubusercontent.com/39393023/70769669-153a4f00-1d30-11ea-9fe1-8f9bf5a67c2c.png)
**_How the state machine works_**:
The robot starts off at a pre-configured camera position in idle mode. When AI sends the position of the center of a new cross, the robot will move to the Write Standoff Position, which is the pre-configured center position. Afterwards, the robot will generate a trajectory of the cross and move to the first point of the trajectory. When the robot finishes drawing the cross, it will go back to the camera position.

Therefore, to suit your application, adjust the below parameters in trajectory_generation.py
###### Pre-configured Positions and Sizes:
- Camera Position
- Write Standoff Position
- Size of each cell

**_Notes on Parameter Tuning_**:
 When determining the best board position (whose center should be the Write Standoff Position), be aware of the robot's work space (making sure no awkward motions are shown) and glare on the board if there is any.

### Computer Vision
The Sawyer reads the game board using its wrist camera from the predetermined home position. It uses edge detection to find the green square of tape and uses the tape as a point of origin and as a means of finding the number of pixels in a centimeter. The program then knows the placement of all nine board spaces because of the hard-coded measurements of the game board. Each board space in the image is then cropped and isolated from one another so that the Sawyer can iterate through them all.

For each cropped space image, the robot edge detects, dilates the edges to bridge any gaps, finds all contours within the board space, and then draws the largest contour on the image. The code determines if that contour is large enough to be considered a marked shape or if it is an empty space by comparing its area to a threshold. To determine if the shape is an X or an O, assuming the threshold is met, the solidity of the shape is calculated by computing the ratio of the contour area to the convex hull area. If the shape has a solidity above 60%, then it is considered an O, otherwise it is considered an X.

Once every board space has been analyzed, the game state is updated and sent to the game AI.

### TicTacToe AI

This part is called by the trajectory generation when it is in the camera configuration and is waiting for the human player to go. The robot's first turn is hardcoded (regardless of if the player or it goes first).  After that, a weighting algorithm is used to determine the optimal move.  Each win condition, 3 rows, 3 columns, and 2 diagonals, is looped through to see how many Xs and how many Os are in it.  Each win condition is then given a weight corresponding to the total number of robot moves in that win condition and subtracting the player moves (ie. a row with an X and an O in it has a net sum of 0.  While a row with just an X has a net sum of +1).  Win conditions with two of the same symbol are weighted that are not blocked are weighted more heavily with even more weight being given if those symbols correspond to the robot.  The weights are then overlapped onto a 3x3 matrix to give a weight map of the board, with the highest weight being the cell that the robot play in next.

This algorithm will play perfectly and will always win/tie.  However, the human can still win due to anti-cheat not being implemented.

## Future Work
- Currently the robot can only draw crosses. Therefore, drawing circles will be a desirable feature.
- A "kid" mode should be added so the game does not always play perfectly, especially in front of kids (Yep, hopefully kids will like this)
- As of now, this program is only compatible with the dry-erase board that was used for the demonstration. We would like to play a tic tac toe game with the Sawyer on any board with minimal setup. Simply drawing the game board grid before playing a game would make this project more accessible -- no tape or hard-coded measurements required.
