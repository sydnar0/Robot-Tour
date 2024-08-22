#region VEXcode Generated Robot Configuration
from vex import *
import urandom

# Brain should be defined by default
brain=Brain()

# Robot configuration code
dist = Distance(Ports.PORT10)
left_drive_smart = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
right_drive_smart = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)
drivetrain_inertial = Inertial(Ports.PORT11)
drivetrain = SmartDrive(left_drive_smart, right_drive_smart, drivetrain_inertial, 319.19, 320, 40, MM, 1)


# wait for rotation sensor to fully initialize
wait(30, MSEC)

def calibrate_drivetrain():
    # Calibrate the Drivetrain Inertial
    sleep(200, MSEC)
    brain.screen.print("Calibrating")
    brain.screen.next_row()
    brain.screen.print("Inertial")
    drivetrain_inertial.calibrate()
    while drivetrain_inertial.is_calibrating():
        sleep(25, MSEC)
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

#endregion VEXcode Generated Robot Configuration

# ------------------------------------------
# 
# 	Project:      VEXcode Project
#	Author:       Sydney Faranetta
#	Created:        1/21/24
#	Description:  VEXcode V5 Python Project
# 
# ------------------------------------------

# Library imports
from vex import *
import vex

# USE thingToPrint.c_str() to print numbers / status on the screen as a string

allTracks = [] # STORES ALL THE TRACK SPACES UPON INITIALIZATION
firstUp = True # Use this to skip over the obstacle detection when first moving up'

# Class for each track space on the competition floor
class Track:
    def __init__(self, left, right, top, bottom, Row, col):
        # these properties reference the OBSTACLES on each 'space'
        self.left = left
        self.right = right # CHOOSE: NONE, OBS, BOUND
        self.top = top #      No obstacle, obstacle, boundary
        self.bottom = bottom

        self.isGoal = False # Most are not the end goal, so set false
        self.isGate = False # There should be about 2-3 gates

        # for coordinates
        self.Row = Row 
        self.col = col

        # for string coordinates
        self.sRow = str(Row) 
        self.sCol = str(col)

        allTracks.append(self) # Add to allTracks when created

    def __str__(self): # What is printed on the screen by using brain.screen.print
        #return f"x: {str(self.row)} y: {str(self.col)}"
        return "("+self.sRow+", "+self.sCol+")"


# Class for the vex robot. Initialize with the current track, probably just reference the Track object variable
class Robot:
    def __init__(self, currTrack):
        self.currTrack = currTrack
        self.rotated = 0
        self.drivetrain = drivetrain # For some reason, this works. Don't know if i need a separate class but whatever
        # Keep track of which side of the currTrack we are facing.
        self.rotIndex = 0
        self.directions = ['top', 'right', 'bottom', 'left'] # directions so we can cycle through them when rotating
        self.currFacing = self.directions[self.rotIndex] # Start facing 'top'
        

        #
        #
        #
        # LIST OF EVERY SINGLE MOVE!!!!!!!!!!!
        self.allMoves = [self.enterTrack, self.l, self.up, self.r, self.up, self.r, self.up, self.l, self.up, self.r, self.up, self.l, self.up,
                            self.rev, self.l, self.up, self.up, self.r, self.up, self.r, self.up]
        #
        #
        #
        #
        #

        self.moveIndex = 0 # Start at the first move
        self.currMove = self.allMoves[self.moveIndex] # Get the actual text of what the move is
        self.nextMove = self.allMoves[self.moveIndex + 1] # Get the next planned move

    def clear(self):
        brain.screen.clear_screen()
        brain.screen.set_cursor(2, 1)

    # RIGHT = CLOCKWISE (POSITIVE), LEFT = COUNTERCLOCKWISE (NEGATIVE)
    def getRotation(self):
        floatRot = float(drivetrain.rotation(DEGREES))
        wholeNumRot = round(float(drivetrain.rotation(DEGREES)))
        return floatRot

    # Function to calculate the new orientation when rotating
    def calcIndex(self, amtChanges):
        ind = bot.rotIndex
        # Try to rotate normally
        try:
            ind += amtChanges
            return ind
        except IndexError: # Wrapping around the list
            ind = ind % len(self.directions)
            ind += (amtChanges - 1) # then change index normally but -1 because wrapping is one change
            return ind
        except Exception as e: 
            bot.clear()
            brain.screen.print("Turning error occurred")
            return ind

    def rotateTo(self, goal):
        # Using a small range here because the robot turns kinda crazy sometimes
        lower = goal-1 
        upper = goal+1
        doneRotating = False
        totalTurn = 0 # Track how much we've turned in increments, so we can check to adjust the facing

        # First check if there is an obstacle in front of us that we should avoid
        if self.getFacingPhysical() != None: # Check for obstacle, if there is one move back before turning
            backDist = 120 - self.getFacingPhysical() # Distance we should move back
            drivetrain.drive_for(REVERSE, backDist, MM)

        while not doneRotating:
            increment = goal-bot.getRotation() # Calculate the amount needed to turn
            bot.clear()
            brain.screen.print("Rotating from "+str(bot.getRotation())+ " adding "+str(increment))

            if bot.getRotation() < lower: # If we are outside the lower boundary
                drivetrain.turn_for(RIGHT, increment, DEGREES) 
                totalTurn += increment
            elif bot.getRotation() > upper: # If we are outside the upper boundary
                drivetrain.turn_for(RIGHT, increment, DEGREES) 
                totalTurn += increment
            else: # If you're in the range
                totalTurn += increment 

                bot.clear()
                brain.screen.print("Made it in the range "+str(lower)+" - "+str(upper))
                brain.screen.next_row()
                brain.screen.print(" with an exact value of "+str(bot.getRotation()))

                if totalTurn > 85 and totalTurn < 95: # Making a right turn (forwards through list)-- approx 90
                    bot.rotIndex = bot.calcIndex(1)
                elif totalTurn > -95 and totalTurn < -85: # Making a left turn (backwards through list) -- approx -90
                    bot.rotIndex = bot.calcIndex(-1)
                elif totalTurn > 175 and totalTurn < 185: # Turning around -- approx 180
                    bot.rotIndex = bot.calcIndex(2) # No matter which direction, a full turnaround will point to the same direction
                elif totalTurn > -185 and totalTurn < -175:
                    bot.rotIndex = bot.calcIndex(-2)

                doneRotating = True
        vex.sleep(500) # Wait AFTER finishing rotation
        bot.clear()
        # Print out what direciton we are currently facing
        brain.screen.print("rotIndex "+ str(bot.rotIndex)+", facing: "+ bot.directions[bot.rotIndex])
        vex.sleep(500)

    # Check if there is an object in front of the robot, and how far away is it.
    # Objects limit 120 MM to count
    def getFacingPhysical(self):
        if dist.is_object_detected() and dist.object_distance(MM) < 120: # Notice sensor name dist
            #self.facingPhysical = "Obstacle"
            objDist = dist.object_distance(MM)
            bot.clear() 
            brain.screen.print("Facing obstacle at "+str(objDist)+" MM")# String for printing purposes
            return objDist
        else:
            #self.facingPhysical = "Free"
            #bot.clear()
            #brain.screen.print("Not facing obstacle")
            return None

    # Return what is in the space which robot is facing, None, Obs, or Bound
    def getNextTrack(self):
        curr = bot.currTrack
        if curr.left == bot.currFacing:
            return curr.left
        elif curr.right == bot.currFacing:
            return curr.right
        elif curr.top == bot.currFacing:
            return curr.top
        elif curr.bottom == bot.currFacing:
            return curr.bottom

    # FUNCTIONS FOR MOVEMENT. SIMPLE SO THEY CAN BE CALLED FAST -- Up, L.eft, R.ight, D.own
    # From middle of track (10 inches) to next track -- 20 inches
    def up(self): # move forward One square
        global firstUp
        if self.getFacingPhysical() != None and not firstUp: # Check for obstacle
            self.clear()
            brain.screen.print("Cannot move forward")
            vex.sleep(500)
            return
        else:
            if firstUp:
                firstUp = False
            
            # Check if we are going to turn after this, if so we want to stop early
            if self.nextMove == self.l or self.nextMove == self.r:
                self.clear()
                brain.screen.print("Accounting for turn..")
                vex.sleep(500)
                drivetrain.drive_for(FORWARD, 17, INCHES)
            else: # If the next move is not a turn
                drivetrain.drive_for(FORWARD, 20, INCHES)

    def l(self): # Turn left, add on to the current degrees rather than turn to a set degree
        bot.rotateTo(drivetrain.rotation(DEGREES) - 90)
    
    def r(self): # Turn right
        bot.rotateTo(drivetrain.rotation(DEGREES) + 90)

    def ard(self): # Turn around
        bot.rotateTo(drivetrain.rotation(DEGREES) + 180)

    def rev(self): # Reverse out of a Track. Accounts for turns, if turning next it will need to drive 3 inches MORE than it would normally.
        if self.nextMove == self.l or self.nextMove == self.r:
            drivetrain.drive_for(REVERSE, 23, INCHES)
        else:
            drivetrain.drive_for(REVERSE, 20, INCHES)

    def enterTrack(self): # If starting outside the track, on the edge
        drivetrain.set_drive_velocity(15, PERCENT)
        drivetrain.drive_for(FORWARD, 13, INCHES)

##### FIGURE OUT: What is forward when we are rotating? pre define 90, -90, 180 degree turns only?
## how should we set the currTrack if robot's direction is changed?

# Building track at home manually starting from bottom right corner. 6 Tracks on the map
# IN ORDER: LEFT, RIGHT, TOP, BOTTOM
t1 = Track("None", "Bound", "None", "Bound", 2, 1)
t2 = Track("Bound", "None", "None", "Bound", 2, 0)
t3 = Track("Obs", "Bound", "Obs", "None", 1, 1)
t4 = Track("Bound", "Obs", "None", "None", 1, 0)
t5 = Track("None", "Bound", "Bound", "Obs", 0, 1)
t5.isGoal = True
t6 = Track("Bound", "None", "Bound", "None", 0, 0)

# Initialize robot object and get the starting position via inertial sensor
# Bot rotation fluctuates by ~0.05 degrees
# I WANT THE BOT AT THE 7 INCH LINE 
# ROBOT'S TURN DISPLACES IT BY 3 INCHES IN EITHER DIRECTION
# 70 MM IS THE ROTATION LIMIT for 90 degree or UNDER turns
# 200 MM is the LIMIT for 180 turns or OVER

bot = Robot(t1)
calibrate_drivetrain() # Always calibrate first
drivetrain.set_drive_velocity(15, PERCENT)

#allMoves = ['enter', 'l', 'up', 'r', 'up', 'r', 'up']

# Let's use a signal to start the robot
started = False
while not started:
    if dist.is_object_detected() and dist.object_distance(MM) < 100:
        # Velocity should be faster than 0 m/s (wiggle it fast)
        if dist.object_velocity() > 0:
            bot.clear()
            brain.screen.print("MOVE PENCIL")
            vex.sleep(1000) # Wait so i can move the pencil out of the way
            started = True
        else:
            vex.sleep(100) # Just keep waiting for the signal

# ROBOT'S MOVEMENT AFTER SIGNAL IS TRIGGERED
# Call each move in the list
for move in bot.allMoves:
    move()



