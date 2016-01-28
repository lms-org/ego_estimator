# ego_estimator
Simple kalman ego-motion estimator
## Data channels
- sensor_utils::SensorContainer>("SENSORS")
## Config
```
<module>
<name>ego_estimator</name>
<config>
    <!-- Setup system model noise covariances -->
    <!-- Choose std-devs to be about the maximum error in a time-step -->
    <!-- See http://nbviewer.ipython.org/github/balzer82/Kalman/blob/master/Extended-Kalman-Filter-CTRV.ipynb?create=1 -->
    <sys_var_x><!--0.0025-->0.001</sys_var_x> <!-- estimate: max veloc 5m/s@100hz: (5*0.01)^2 -->
    <sys_var_y><!--0.0025-->0.001</sys_var_y> <!-- estimate: max veloc 5m/s@100hz: (5*0.01)^2 -->
    <sys_var_a><!--0.24-->0.0024</sys_var_a> <!-- estimate: max 0.5G/s acc -> (0.5*9.81*0.01)^2 -->
    <sys_var_v>0.0004</sys_var_v> <!-- estimate: max 0.3G acc @ 100hz -> (0.3*9.81*0.01)^2 -->
    <sys_var_theta>0.008</sys_var_theta> <!-- estimate: 500dps turn rate @ 100hz -> (500/180*pi*0.01)^2 -->
    <sys_var_omega>0.03</sys_var_omega> <!-- estimate: max 1000deg/s^2 turn rate acc @100hz -> (2*9.81*0.01)^2 -->

    <!-- Setup initial state covariances -->
    <filter_init_var_x>0.01</filter_init_var_x>
    <filter_init_var_y>0.01</filter_init_var_y>
    <filter_init_var_a>0.01</filter_init_var_a>
    <filter_init_var_v>0.01</filter_init_var_v>
    <filter_init_var_theta>0.01</filter_init_var_theta>
    <filter_init_var_omega>0.01</filter_init_var_omega>

    <!-- DEBUG -->
    <state_log>false</state_log>
</config>
<channelMapping priority="100" from="CAR" to="CAR" />
</module>
```


## Dependencies
