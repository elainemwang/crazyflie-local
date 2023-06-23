# ArenaXR
<table>
<tr>
<td>  File    </td> <td> Description </td>
</tr>
<tr>
<td> old_versions/crazyflie_connect_test.py </td>
<td>
Lighthouse position calculations from raw angles compared to logged xyz calculated by Crazyflie firmware. The orange line is ground truth from Crazyflie firmware and the blue line is our line generated from our position calculation algorithm. Uses Lighthouse 2 angles, which is not correct. Also only finds the position of one sensor.
</td>
</tr>
<td> old_versions/crazyflie_connect_testv2.py </td>
<td>
Same as crazyflie_connect_test.py except it uses Lighthouse 1 angles.
</td>
</tr>
<td> old_versions/crazyflie_connect_testv3.py </td>
<td>

Same as v2, but now uses all four sensors and takes the average position calculated from all four sensors. It also checks to see if a sensor is receiving valid data and only calculates position if all four sensors output valid data (All sensors are not blocked). This can be changed in 
```python
if not np.array_equal(pos,prev_pos[sensor_num]):
       valid_sensors[sensor_num] = 1
   else:
       valid_sensors[sensor_num] = 1 # set this to 0 if you want to ignore the sensor when invalid data is received
 ```
 The log blocks are organized by sensor (all four angles from one sensor come in on the same timestamp).
</td>
</tr>

<td> old_versions/crazyflie_connect_testv4.py </td>
<td>
Same as v3 except the log blocks are organized by angle (four same angles from all four sensors come in on the same timestamp Ex. The horizontal angle from Base Station 1). Does not take into account whether or not sensors are blocked.
</td>
</tr>

<td> old_versions/crazyflie_connect_testv5.py </td>
<td>
Same as v3 except there are only two log blocks: the first one is 12 angles from three sensors (0,1,2) in FP16, a two byte half-precision floating point (Each log block can send 27 bytes). So, the last sensor (3) is contained within another log block.
https://forum.bitcraze.io/viewtopic.php?p=21869&hilit=log+angles#p21869
</td>
</tr>

<td> crazyflie_connect_testv6.py </td>
<td>
Same as v5, now with yaw pitch roll from both the stabilizer (gyro) and our own lighthouse angles graphed and orientation matrix also graphed. Notes: The yaw and pitch measurements match up pretty well, but our roll measurements seem to have some trouble when rolling too far away from the lighthouse (facing the opposite direction), which is expected.
</td>
</tr>

<td> position_cal.py </td>
<td>
Helper functions for calculating the position and orientation from the raw sweep angles.
</tr>

<td> pos_grapher.py </td>
<td>
Helper functions for graphing lines in 3d for the position graphs and the 3d orientation matrix.
</td>
</tr>

<td> kalman_testing.py </td>
<td>
Untested, some basic incomplete starter code for future Extended Kalman Filter testing.
</tr>


<td> lighthouse_testew.py </td>
<td>
Test Lighthouse logger from https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/lighthouse/multi_bs_geometry_estimation.py, runs estimate geometry for the Lighthouses.
</td>
</tr>