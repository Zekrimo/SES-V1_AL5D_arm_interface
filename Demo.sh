#!/bin/bash

# Start the server in the background
ros2 run lynxarm_srvcli server &
SERVER_PID=$!
echo "Server started with PID $SERVER_PID"

# Allow some time for the server to initialize
sleep 5

# Start the client in the background (if necessary)
#ros2 run lynxarm_srvcli client &
#CLIENT_PID=$!
#echo "Client started with PID $CLIENT_PID"

# Short pause
echo "Pausing before executing commands..."
sleep 5

# Ready command
echo "Setting arm to 'Ready' position..."
ros2 action send_goal /lynxcommand lynxarm_interface/action/PredefinedCommandAction "{command: 'ready'}"
sleep 10

# Straight up command
echo "Setting arm to 'Straight Up' position..."
ros2 action send_goal /lynxcommand lynxarm_interface/action/PredefinedCommandAction "{command: 'straight_up'}"
sleep 10

# Move to different positions
echo "Moving arm to different positions..."
# Example: Move servo 0 to position 90 with speed 2 and time 10000
ros2 action send_goal /lynxcommandservo lynxarm_interface/action/SingleServoCommand "{servo: 0, position: 80, time: 100}"
sleep 10

# Move to different positions
echo "Moving arm to different positions..."
# Example: Move servo 0 to position 90 with speed 2 and time 10000
ros2 action send_goal /lynxcommandservo lynxarm_interface/action/SingleServoCommand "{servo: 1, position: 15, time: 100}"
sleep 10

# Move to different positions
echo "Moving arm to different positions..."
# Example: Move servo 0 to position 90 with speed 2 and time 10000
ros2 action send_goal /lynxcommandservo lynxarm_interface/action/SingleServoCommand "{servo: 2, position: 45, time: 100}"
sleep 10

# Move to different positions
echo "Moving arm to different positions..."
# Example: Move servo 0 to position 90 with speed 2 and time 10000
ros2 action send_goal /lynxcommandservo lynxarm_interface/action/SingleServoCommand "{servo: 3, position: 30, time: 100}"
sleep 10

# Move to different positions
echo "Moving arm to different positions..."
# Example: Move servo 0 to position 90 with speed 2 and time 10000
ros2 action send_goal /lynxcommandservo lynxarm_interface/action/SingleServoCommand "{servo: 4, position: -90,  time: 100}"
sleep 10
# More movements can be added here using the move single servo command format
# Repeat the above command with different parameters as needed

# Move to different positions
echo "Moving arm to different positions..."
# Example: Move servo 0 to position 90 with speed 2 and time 10000
ros2 action send_goal /lynxcommandservo lynxarm_interface/action/SingleServoCommand "{servo: 5, position: -60,  time: 100}"
sleep 10
# More movements can be added here using the move single servo command format
# Repeat the above command with different parameters as needed

# Move to different positions
echo "Moving arm to different positions..."
# Example: Move servo 0 to position 90 with speed 2 and time 10000
ros2 action send_goal /lynxcommandservo lynxarm_interface/action/SingleServoCommand "{servo: 0, position: -90, time: 5000}"
sleep 2
# More movements can be added here using the move single servo command format
# Repeat the above command with different parameters as needed


# Emergency brake
# Assuming the emergency brake is triggered via a specific client command or similar
echo "Triggering emergency brake..."
ros2 run lynxarm_srvcli client
sleep 10

# Park command
echo "Parking arm..."
ros2 action send_goal /lynxcommand lynxarm_interface/action/PredefinedCommandAction "{command: 'park'}"
sleep 10

# Cleanup - stopping the server (and client if started)
echo "Stopping server..."
kill $SERVER_PID
#echo "Stopping client..."
#kill $CLIENT_PID

echo "Demo complete."
