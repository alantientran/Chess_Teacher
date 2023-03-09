import sys
import os
import numpy as np
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from xarm.wrapper import XArmAPI


#  * pickup_piece( str -> name of the square) -> pick up a piece from the board
#  * place_piece( str -> name of the square) -> place a piece on the board
#  * move_piece( str -> name of the square, str -> name of the square) -> move a piece from one square to another
#  * constants: size of the board, size of the squares, size of highest piece
#  * (0, [87.0, -0.0, 154.199997, 180.00002, 0.0, -0.0])
#  * tuple when the arm is just touching the board [x, y, z, roll, pitch, yaw]
#  * (0, [163.208542, 0.025312, 135.205475, -179.992629, -3.380279, 0.091444])

#  * positive z values make the arm go up

class Arm_constants:
    # units in mm:
    SIZE_SQUARE : float = 27.49
    SIZE_BOARD : float = 250
    POS_Z_HIGHEST_PIECE : float = 50.71
    POS_Z_BOARD : float = 0
    OFFSET_X1 : float = 0
    OFFSET_Y1 : float = 0
    OFFSET_X2 : float = 0
    OFFSET_Y2 : float = 0
    SQUARE_LOCATIONS = np.empty((8, 8), dtype=tuple)
    ROBOT_COLOR: bool = False
    ARM: XArmAPI
    SQUARES_IN_ROW: int = 8
    # POS_Z_BASE_BOARD : int = 135

    # def __init__(self, size_square, size_board, pos_z_board, pos_z_highest_piece, offset_x, offset_y):
    #     self.SIZE_SQUARE = size_square
    #     self.SIZE_BOARD = size_board
    #     self.POS_Z_BOARD = pos_z_board
    #     self.POS_Z_HIGHEST_PIECE = pos_z_highest_piece
    #     self.OFFSET_X = offset_x
    #     self.OFFSET_Y = offset_y

def instantiateArm():
    Arm_constants.ARM = XArmAPI('192.168.1.166')
    Arm_constants.ARM.motion_enable(enable=True)
    Arm_constants.ARM.set_mode(0)
    Arm_constants.ARM.connect()
    # Arm_constants.ARM.clean_error()
    # Arm_constants.ARM.clean_warn()

def deinstantiateArm():
    Arm_constants.ARM.motion_enable(enable=False)
    Arm_constants.ARM.disconnect()

def calibrate():
    # code to calibrate the arm
    # enter a known position for the arm to move relative to the board
    # arm.set_position(0, 0, 0, wait=True)
    print("CALIBRATION SEQUENCE STARTING...\n")

    color = input("IS THE ROBOT PLAYING AS WHITE? ENTER Y OR N\n")

    while 'Y' not in color and 'N' not in color:
        print("PLEASE ENTER EITHER Y OR N")
        color = input("IS THE ROBOT PLAYING AS WHITE? ENTER Y OR N\n")
    
    if 'Y' in color:
        Arm_constants.ROBOT_COLOR = True
    else:
        Arm_constants.ROBOT_COLOR = False
    
    manualMode()
    
    trash = input("MANUALLY MOVE ARM TO " + indiciesToSquare((0,0)) + " AND PRESS ENTER")

    pos = Arm_constants.ARM.position
    print (pos)
    Arm_constants.OFFSET_X1 = pos[0]
    Arm_constants.OFFSET_Y1 = pos[1]
    Arm_constants.POS_Z_BOARD = pos[2]
    Arm_constants.SQUARE_LOCATIONS[0][0] = (Arm_constants.OFFSET_X1, Arm_constants.OFFSET_Y1)

    trash = input("MANUALLY MOVE ARM TO " + indiciesToSquare((0,7)) + " AND PRESS ENTER")

    pos = Arm_constants.ARM.position
    print(pos)
    Arm_constants.OFFSET_X2 = pos[0]
    Arm_constants.OFFSET_Y2 = pos[1]
    Arm_constants.SQUARE_LOCATIONS[0][7] = (Arm_constants.OFFSET_X2, Arm_constants.OFFSET_Y2)
    if pos[2] < Arm_constants.POS_Z_BOARD:
        Arm_constants.POS_Z_BOARD = pos[2]

    Arm_constants.POS_Z_HIGHEST_PIECE += Arm_constants.POS_Z_BOARD
    print("PERFORMING CALCULATIONS...")

    deltax = Arm_constants.OFFSET_X2 - Arm_constants.OFFSET_X1
    deltay = Arm_constants.OFFSET_Y2 - Arm_constants.OFFSET_Y1

    deltax /= 7
    deltay /= 7

    for i in range(1, Arm_constants.SQUARES_IN_ROW - 1):
        newX = (Arm_constants.SQUARE_LOCATIONS[0][i - 1])[0] + deltax
        newY = (Arm_constants.SQUARE_LOCATIONS[0][i - 1])[1] + deltay
        Arm_constants.SQUARE_LOCATIONS[0][i] = (newX, newY)
    
    for i in range(1, Arm_constants.SQUARES_IN_ROW):
        for j in range(0, Arm_constants.SQUARES_IN_ROW):
            newX = (Arm_constants.SQUARE_LOCATIONS[i - 1][j])[0] + deltay
            newY = (Arm_constants.SQUARE_LOCATIONS[i - 1][j])[1] + deltax
            Arm_constants.SQUARE_LOCATIONS[i][j] = (newX, newY)

    print("CHECKING MOVEMENT FREEDOM...")

    directedMode()

    Arm_constants.ARM.open_lite6_gripper()

    Arm_constants.ARM.set_position(Arm_constants.SQUARE_LOCATIONS[0][0][0], Arm_constants.SQUARE_LOCATIONS[0][0][1], Arm_constants.POS_Z_HIGHEST_PIECE, 180, 0, 0, None, 100, 50, wait=True)

    Arm_constants.ARM.set_position(Arm_constants.SQUARE_LOCATIONS[7][0][0], Arm_constants.SQUARE_LOCATIONS[7][0][1], Arm_constants.POS_Z_HIGHEST_PIECE, 180, 0, 0, None, 100, 50, wait=True)

    Arm_constants.ARM.set_position(Arm_constants.SQUARE_LOCATIONS[7][7][0], Arm_constants.SQUARE_LOCATIONS[7][7][1], Arm_constants.POS_Z_HIGHEST_PIECE, 180, 0, 0, None, 100, 50, wait=True)

    Arm_constants.ARM.set_position(Arm_constants.SQUARE_LOCATIONS[0][7][0], Arm_constants.SQUARE_LOCATIONS[0][7][1], Arm_constants.POS_Z_HIGHEST_PIECE, 180, 0, 0, None, 100, 50, wait=True)

    Arm_constants.ARM.set_position(Arm_constants.SQUARE_LOCATIONS[0][0][0], Arm_constants.SQUARE_LOCATIONS[0][0][1], Arm_constants.POS_Z_HIGHEST_PIECE, 180, 0, 0, None, 100, 50, wait=True)

    # code that outline board here

    print("CALIBRATION COMPLETE!")
    

# given a string name of square in chess notation, returns a tuple of indices into the corresponding square in the data table
# returns (-1, -1) if invalid input
def squareToIndices(square : str):
    x = int(square[1])
    y = square[0].lower()
    if x < 1 or x > 8 or ord(y) < 97 or ord(y) > 104:
        return (-1, -1)
    if Arm_constants.ROBOT_COLOR:
        x = x - 8
        x = abs(x)
        y = ord(y) - 97
    else:
        x = x-1
        y = ord(y) - 97 - 7
        y = abs(y)
    return (x, y)


# given a tuple representing indices for the stored table, returns the name of the square in chess notation
# returns a i9 if invalid inputs
def indiciesToSquare(indices : tuple):
    x = indices[0]
    y = indices[1]
    if x > 7 or y > 7 or x < 0 or y < 0:
        return 'i9'
    if Arm_constants.ROBOT_COLOR:
        x = -x
        x += 8
        y += 97
        y = chr(y)
    else:
        x += 1
        y = -y
        y += 7
        y += 97
        y = chr(y)
    return (str(y) + str(x))

def printStatus():
    print('------------------------------------')
    print('Mode: ' + str(Arm_constants.ARM.mode))
    print('State: ' + str(Arm_constants.ARM.state))
    print('Has Error Warn: ' + str(Arm_constants.ARM.has_err_warn))
    print('Has Error: ' + str(Arm_constants.ARM.has_error))
    print('Has Warn: ' + str(Arm_constants.ARM.has_warn))
    print('Error Code: ' + str(Arm_constants.ARM.error_code))
    print('Warn Code: ' + str(Arm_constants.ARM.warn_code))
    print('------------------------------------')

def printStatusInfinite():
    while True:
        time.sleep(0.1)
        print('------------------------------------')
        print('Mode: ' + str(Arm_constants.ARM.mode))
        print('State: ' + str(Arm_constants.ARM.state))
        print('Has Error Warn: ' + str(Arm_constants.ARM.has_err_warn))
        print('Has Error: ' + str(Arm_constants.ARM.has_error))
        print('Has Warn: ' + str(Arm_constants.ARM.has_warn))
        print('Error Code: ' + str(Arm_constants.ARM.error_code))
        print('Warn Code: ' + str(Arm_constants.ARM.warn_code))
        print('------------------------------------')

def manualMode():
    Arm_constants.ARM.set_mode(2)
    Arm_constants.ARM.clean_error()
    Arm_constants.ARM.clean_warn()

    Arm_constants.ARM.set_state(0)

def directedMode():
    Arm_constants.ARM.set_mode(0)
    Arm_constants.ARM.clean_error()
    Arm_constants.ARM.clean_warn()

    Arm_constants.ARM.set_state(0)

def pickupPiece():
    Arm_constants.ARM.open_lite6_gripper()
    time.sleep(1)

    curPos = Arm_constants.ARM.position
    Arm_constants.ARM.set_position(curPos[0], curPos[1], Arm_constants.POS_Z_BOARD, 180, 0, 0, None, 100, 50, wait=True)

    Arm_constants.ARM.close_lite6_gripper()
    time.sleep(1)

    Arm_constants.ARM.set_position(curPos[0], curPos[1], Arm_constants.POS_Z_HIGHEST_PIECE, 180, 0, 0, None, 100, 50, wait=True)

def moveToSquare(square : str):
    # finish this





instantiateArm()

calibrate()


# if __name__ == "__main__":
    
# # Create an instance of the xArmAPI object
    # arm = XArmAPI('192.168.1.166')
    # arm.motion_enable(enable=True)
    # arm.set_mode(0)
    # # Connect to the robot
    # arm.connect()
    

#     current_pos = arm.get_position()[1]
#     print(current_pos)
#     arm.set_position(current_pos[0], current_pos[1], Arm_constants.POS_Z_BASE_BOARD, wait=True)
#     #print(current_pos)
#     # Move the arm to the target position
#     # arm.move_to(x, y, z)
#     #arm.reset(wait=True)
#     # Disconnect from the robot
#     arm.disconnect()