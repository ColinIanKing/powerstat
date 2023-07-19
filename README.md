![Powerstat Logo](mascot/powerstat-256x256.png)

# Powerstat

<a href="https://repology.org/project/powerstat/versions">
    <img src="https://repology.org/badge/vertical-allrepos/powerstat.svg" alt="Packaging status" align="right">
</a>

Powerstat measures the power consumption of a laptop using the ACPI battery information. The output is like vmstat but also shows power consumption statistics. At the end of a run, powerstat will calculate the average, standard deviation and min/max of the gathered data.

## Example

```
powerstat -cDHRf 2 
Running for 60.0 seconds (30 samples at 2.0 second intervals).
Power measurements will start in 0 seconds time.

  Time    User  Nice   Sys  Idle    IO  Run Ctxt/s  IRQ/s  Watts uncore   core  pkg-0  CPU Freq
10:00:09   3.4   0.0   2.0  94.6   0.0    2   6236   1144   5.28   0.21   2.86   5.28  2.27 GHz  
10:00:11   2.5   0.0   2.5  94.8   0.1    1   6171   1134   5.10   0.18   2.72   5.10  2.22 GHz  
10:00:13   3.5   0.0   2.2  94.1   0.1    1   5988   1079   5.17   0.24   2.73   5.17  2.60 GHz  
10:00:15   2.8   0.0   2.8  94.4   0.0    2   6028   1089   4.92   0.12   2.60   4.92  2.74 GHz  
10:00:17   2.8   0.0   2.0  95.1   0.1    1   6344   1156   5.66   0.38   3.05   5.66  2.98 GHz  
10:00:19   7.1   0.0   3.0  89.0   0.9    2   9686   1429   8.93   1.44   5.09   8.93  2.55 GHz  
10:00:21  10.1   0.0   3.0  86.4   0.5    2  10406   1515   9.48   1.11   6.03   9.48  2.84 GHz  
10:00:23   7.7   0.0   2.7  88.4   1.3    2   7892   1410  10.44   2.42   5.49  10.44  2.99 GHz  
10:00:25   8.1   0.0   2.7  86.4   2.8    2   7788   1488  12.12   3.46   5.90  12.12  2.54 GHz  
10:00:27   4.0   0.0   2.8  92.9   0.2    1   7206   1188   6.58   0.70   3.64   6.58  2.85 GHz  
10:00:29   4.3   0.0   1.8  92.9   1.1    1   7788   1286   7.25   0.85   4.14   7.25  2.41 GHz  
10:00:31   2.5   0.0   2.0  95.3   0.1    1   6296   1122   5.39   0.25   2.92   5.39  2.61 GHz  
10:00:33   3.6   0.0   2.6  93.6   0.1    1   6548   1140   5.61   0.36   3.02   5.61  2.63 GHz  
10:00:35   3.8   0.0   2.7  93.2   0.2    1   6896   1179   6.28   0.61   3.44   6.28  2.56 GHz  
10:00:37   4.5   0.0   2.7  92.7   0.1    1   7340   1220   6.97   0.73   3.99   6.97  2.97 GHz  
10:00:39   5.1   0.0   2.2  92.5   0.2    1   7337   1206   6.61   0.66   3.71   6.61  2.80 GHz  
10:00:41  66.3   0.0  10.5  23.3   0.0   11   5880   2087  15.47   1.35  11.51  15.47  2.90 GHz  
10:00:43  86.7   0.0  13.1   0.1   0.0    6   5542   2449  18.55   1.82  13.95  18.55  2.90 GHz  
10:00:45  87.6   0.0  12.3   0.1   0.0    8   5664   2484  18.33   1.58  14.00  18.33  2.90 GHz  
10:00:47  11.5   0.0   3.5  84.9   0.1    1   5984   1235   7.42   0.43   4.69   7.42  2.49 GHz  
10:00:49   3.0   0.0   3.2  93.6   0.1    2   6034   1096   5.13   0.19   2.72   5.13  2.49 GHz  
10:00:51   2.3   0.0   2.4  95.3   0.0    2   6020   1084   5.02   0.16   2.64   5.02  2.52 GHz  
10:00:53   3.0   0.0   3.0  94.0   0.0    1   6004   1082   4.88   0.12   2.55   4.88  2.39 GHz  
10:00:55   2.8   0.0   2.9  94.1   0.3    1   6052   1133   5.35   0.24   2.90   5.35  2.57 GHz  
10:00:57   3.6   0.0   2.4  93.9   0.1    3   6858   1216   5.71   0.27   3.19   5.71  2.51 GHz  
10:00:59   2.5   0.0   2.8  94.7   0.0    2   5910   1076   5.17   0.26   2.70   5.17  2.65 GHz  
10:01:01   2.5   0.0   2.4  94.9   0.1    1   6115   1103   5.37   0.28   2.87   5.37  2.47 GHz  
10:01:03   4.8   0.0   2.9  92.1   0.3    2   6410   1182   5.98   0.32   3.42   5.98  2.59 GHz  
10:01:05   2.0   0.0   2.3  95.7   0.0    1   6070   1070   5.27   0.31   2.74   5.27  2.58 GHz  
10:01:07   9.6   0.0   3.2  87.0   0.1    1   5922   1168   7.45   0.35   4.83   7.45  2.55 GHz  
-------- ----- ----- ----- ----- ----- ---- ------ ------ ------ ------ ------ ------ ---------
 Average  12.1   0.0   3.6  84.0   0.3  2.1 6680.4 1308.3   7.56   0.71   4.53   7.56  2.64 GHz  
  StdDev  23.0   0.0   2.9  25.8   0.6  2.2 1107.5  368.0   3.74   0.76   3.06   3.74  0.20 GHz  
-------- ----- ----- ----- ----- ----- ---- ------ ------ ------ ------ ------ ------ ---------
 Minimum   2.0   0.0   1.8   0.1   0.0  1.0 5542.0 1070.0   4.88   0.12   2.55   4.88  2.22 GHz  
 Maximum  87.6   0.0  13.1  95.7   2.8 11.0 10406.5 2484.0  18.55   3.46  14.00  18.55  2.99 GHz  
-------- ----- ----- ----- ----- ----- ---- ------ ------ ------ ------ ------ ------ ---------
Summary:
  7.56 Watts on average with standard deviation 3.74  
Note: power read from RAPL domains: uncore, core, package-0.
These readings do not cover all the hardware in this device.

C-State    Resident      Count Latency 
C7-IVB      80.109%     155253      87
C6-IVB       0.015%         63      80
C3-IVB       1.847%      10962      59
C1E-IVB      1.141%      11068      10
C1-IVB       0.511%       3339       1
POLL         0.000%         30       0
C0          16.376%

Histogram (of 30 power measurements)

 Range (Watts)  Count
 4.878 -  6.244    16 ########################################
 6.245 -  7.611     7 #################
 7.612 -  8.978     1 ##
 8.979 - 10.345     1 ##
10.346 - 11.712     1 ##
11.713 - 13.079     1 ##
13.080 - 14.446     0 
14.447 - 15.813     1 ##
15.814 - 17.180     0 
17.181 - 18.548     2 #####

Histogram (of 30 CPU utilization measurements)

  Range (%CPU)    Count
  4.309 -  13.865    26 ########################################
 13.866 -  23.421     1 #
 23.422 -  32.978     0 
 32.979 -  42.535     0 
 42.536 -  52.091     0 
 52.092 -  61.648     0 
 61.649 -  71.204     0 
 71.205 -  80.761     1 #
 80.762 -  90.317     0 
 90.318 -  99.874     2 ###

Histogram (of 30 CPU average frequencies)

Range (GHz)   Count
2.217 - 2.293     2 ##########
2.294 - 2.369     0 
2.370 - 2.446     2 ##########
2.447 - 2.523     5 #########################
2.524 - 2.600     8 ########################################
2.601 - 2.677     3 ###############
2.678 - 2.754     1 #####
2.755 - 2.831     1 #####
2.832 - 2.907     5 #########################
2.908 - 2.984     3 ###############
```
