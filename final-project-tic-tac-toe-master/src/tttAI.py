#!/usr/bin/env python

from __future__ import print_function
import numpy as np
import intera_interface
from random import randrange
#import intera_external_devices as kb
import keyboard as kb
import rospy
import rospkg

def printe(string):
    """
    Prints the string without adding a newline
    """
    print(string, end='')

def print_array(gameboard):
    """
    Prints an array in a comma separated format
    """
    print()
    for i in range(len(gameboard)):
        for j in range(len(gameboard[i])):
            printe(gameboard[i][j])
            printe(", ")
        print()

def findXYOffset(tuple):
    """
    Old implementation of implementing cell to cm conversion
    """
    homeX = -4.5
    homeY = 4.5
    baseX = -9
    baseY = 9
    xScale = 2-tuple[0]
    yScale = 2-tuple[1]
    xOffset = homeX + baseX * xScale
    yOffset = homeY + baseY * yScale

    unitScale = 1 #use this for changing units from cm
    return (xOffset*unitScale,yOffset*unitScale)

def displayFace(mode):
    """
    Displays a face on the sawyer robot corresponding to a mode:
    0: display a face for a Tie
    1: display a face for a Win
    2: display a face for a Loss
    11: display a face for Robot's turn
    22: display a face for the Player's turn
    """

    _head = intera_interface.Head()
    head_display = intera_interface.HeadDisplay()

    rospack = rospkg.RosPack()
    pkgpath = rospack.get_path('final-project-tic-tac-toe')

    if mode == 0:
        filepath = pkgpath + "/images/tie" + str(randrange(4)) + ".jpg"
    if mode == 1:
        filepath = pkgpath + "/images/win" + str(randrange(4)) + ".jpg"
    if mode == 2:
        filepath = pkgpath + "/images/lose.jpg"
    if mode == 11:
        filepath = pkgpath + "/images/board.jpg"
    if mode == 22:
        filepath = pkgpath + "/images/turn" + str(randrange(4)) + ".jpg"


    head_display.display_image(filepath)

def isGameOver(b):
    """
    Returns True if the game is over (either board is full or 3 in a row)
    Also returns the player that won if 3 in a row are detected
    """
    board = np.copy(b)
    moves = 0
    for i in range(3):
        for k in range(3):
            if board[i][k] == 2:
                board[i][k] = -1
            if board[i][k] != 0:
                moves += 1
    rows = np.copy(board)
    cols = np.copy(board).T
    diags = np.array([[0,0,0],[0,0,0]])
    for i in range(3):
        diags[0][i] = board[i][i]
        diags[1][i] = board[i][2-i]

    testWin = [0,0,0,0,0,0,0,0,0]
    for i in range(3):
        testWin[i] = np.sum(rows[i])
        testWin[i+3] = np.sum(cols[i])
    testWin[6] = np.sum(diags[0])
    testWin[7] = np.sum(diags[1])

    for i in range(8):
        if abs(testWin[i]) == 3:
            if testWin[i] < 0:
                #displayFace(2)
                return True,2
            #displayFace(1)
            return True,1

    if moves == 9:
        displayFace(0)
        return True, 0
    return False, 0



def findNextMove(gameboard, robot_player):
    """
    Given a 3x3 gameboard filled with 0 (no move), 1 (X), or 2 (O)
    Return a tuple corresponding to a cell on the tictactoe board
    Also return the shape to be drawn at that cell
    """
    nextMove = [-1, -1]
    rowSize = 3
    colSize = 3
    other_player = 1
    if robot_player == 1:
        other_player = 2

    #test for valid gameboard
    if len(gameboard) != rowSize:
        raise TypeError("Tic Tac Toe Board expected " + str(rowSize) + " rows, got " + str(len(gameboard)))
    for i in range(3):
        if len(gameboard[i]) != 3:
            raise TypeError("Tic Tac Toe Board expected " + str(colSize) + " columns, got " + str(len(gameboard[i])))
    ###

    print_array(gameboard)
    #test for gameover
    go,face = isGameOver(gameboard)
    if go:
        #print("GO")
        displayFace(face)
        return (0,0),-1

    #test to see if it is the robots turn
    currentMove = 0
    total_robot_moves = 0
    total_other_moves = 0
    for r in range(rowSize):
        for c in range(colSize):
            val = gameboard[r][c]
            if val == robot_player:
                total_robot_moves += 1
            if val == other_player:
                total_other_moves += 1
    currentMove = total_other_moves+total_robot_moves

    if(total_robot_moves > total_other_moves):
        print("Not the robots turn")
        return (-1,-1), 0
    ###

    #hard coded first move (go for center)
    if currentMove == 0:
        return (1,1), 1
    if currentMove == 1:
        if gameboard[1][1] == 0:
            return (1,1), 1
        return (0,0), 1
    ###

    weights = [[0,0,0],[0,0,0],[0,0,0]]

    for i in range(3):
        # get row weight
        pRobot = 0
        pOther = 0
        for k in range(3):
            cell = gameboard[i][k]
            if cell == robot_player:
                pRobot += 1
            if cell == other_player:
                pOther += 1
        w = pRobot - pOther
        if w > 1:
            w = 10
        if w < -1:
            w = 5

        # save row weight
        for k in range(3):
            if gameboard[i][k] != 0:
                weights[i][k] = -1
            else:
                weights[i][k] += abs(w)


        # get col weight
        pRobot = 0
        pOther = 0
        for k in range(3):
            cell = gameboard[k][i]
            if cell == robot_player:
                pRobot += 1
            if cell == other_player:
                pOther += 1
        w = pRobot - pOther
        if w > 1:
            w = 10
        if w < -1:
            w = 5

        # save col weight
        for k in range(3):
            if gameboard[k][i] != 0:
                weights[k][i] = -1
            else:
                weights[k][i] += abs(w)

        # end of loop

    # get left diagonal weight
    pRobot = 0
    pOther = 0
    for k in range(3):
        cell = gameboard[k][k]
        if cell == robot_player:
            pRobot += 1
        if cell == other_player:
            pOther += 1
    w = pRobot - pOther
    if w > 1:
        w = 10
    if w < -1:
        w = 5

    for k in range(3):
        if gameboard[k][k] != 0:
            weights[k][k] = -1
        else:
            weights[k][k] += abs(w)

    # get Right diagonal weight
    pRobot = 0
    pOther = 0
    for k in range(3):
        cell = gameboard[k][2-k]
        if cell == robot_player:
            pRobot += 1
        if cell == other_player:
            pOther += 1
    w = pRobot - pOther
    if w > 1:
        w = 10
    if w < -1:
        w = 5

    for k in range(3):
        if gameboard[k][2-k] != 0:
            weights[k][2-k] = -1
        else:
            weights[k][2-k] += abs(w)




    #print_array(weights)

    maxWeight = -1
    coor = (-1,-1)
    for i in range(3):
        for k in range(3):
            test_weight = weights[i][k]
            if test_weight > maxWeight:
                #print(test_weight)
                maxWeight = test_weight
                coor = (i,k)

    return coor, 1

def convertCoor(tuple):
    """
    Convert cell tuple from Bennett's gameboard numbering to Rico's numbering
    """
    return (-tuple[0]+1,-tuple[1]+1)

def convertShape2String(shapeInt):
    """
    Get the string corresponding to shape integer
    """
    if shapeInt == -1:
        return "end"
    if shapeInt == 0:
        return "idle"
    if shapeInt == 1:
        return "cross"
    if shapeInt == 2:
        return "circle"
    return "error"

def rotateBoard(board):
    """
    Rotate board from wrist camera
    (since wrist camera is upside down)
    """
    newBoard = np.copy(board)
    for i in range(3):
        for k in range(3):
            newBoard[2-i][2-k] = board[i][k]
    return newBoard

def waitKey():
    """
    Blocking function to wait for Enter key (or Escape key) to be pressed
    """
    kb.getch()
    print("CONTINUE KEY ACCEPTED")


def Update(camera):
    """
    This function takes the camera object from game_state.py
    It enables the camera, waits for a new picture to be taken, and then disables the camera
    It then uses the gameboard saved in the camera object
    (which has been updated now that the camera has taken a new picture)
    to find the cell where the next shape should be drawn.
    It returns a cell and a shape corresponding to that position and which shape the robot is playing as
    """
    rospy.sleep(1)
    camera.updateBoard()
    print(camera.gamestate)
    go, face = isGameOver(camera.gamestate)
    if go:
        displayFace(face)
        return (0,0),-1
    if np.sum(camera.gamestate) != 0:
        displayFace(22)
        print("Press Enter to continue...")
        waitKey()
    camera.cameras.start_streaming(camera.argsCamera)
    camera.once = False
    while(not camera.once):
        rospy.sleep(1)
    camera.cameras.stop_streaming(camera.argsCamera)
    rospy.sleep(1)
    go, face = isGameOver(camera.gamestate)
    if not go:
        displayFace(11)
    else:
        displayFace(face)
        return (0,0),-1
    move,shape = findNextMove(rotateBoard(camera.gamestate), 1)
    #print_array(rotateBoard(camera.gamestate))
    #print(move)

    print(":" + str(move))
    print(":" + str(shape))
    if move[0] == -1:
        return (0,0),0
    if move[1] == -1:
        return (0,0),0
    if shape == -1:
        return (0,0),-1
    return move,1

if __name__ == "__main__":
    """
    Testing for tttAI.py done through running this pyhton script directly
    """
    rospy.init_node('saw')
    board = [[1,1,2],[2,2,1],[1,1,1]]
    print_array(board)
    print()
    waitKey()

    bot = 1
    move,shape = findNextMove(board,bot)
    print("Move: " + str(move))
    print("Shape: " + str(shape))
