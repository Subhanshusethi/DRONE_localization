<br>1.first you need to follow the instruction as shown in below link<br>
<br>https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy<br>
<br>2. after doing it you've to run the ddrone.py<br>
<p>You can either launch your launch file separately on another terminal or you can make changes in code to add a command for running <b>launch file</b> as shown below.</p>
<pre>
<code>
# Define your launch command
launch_command = "roslaunch realsense2_camera rs_camera.launch enable_gyro:=true enable_accel:=true unite_imu_method:='copy'"

# Execute the launch command in a subprocess
subprocess.Popen(launch_command, shell=True)
</code>
</pre>
<p>Below is the command you've to run if you are not getting <strong>IMU data</strong> after following the above link.</p>

<pre>
<code>
roslaunch realsense2_camera rs_camera.launch enable_gyro:=true enable_accel:=true unite_imu_method:='copy'
</code>
</pre>

