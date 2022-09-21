# Landmark management for the application of Radar SLAM
Sample code for the conference paper "Landmark management for the application of Radar SLAM" ([ArXiv:2209.07199](https://arxiv.org/abs/2209.07199))  
Author: Shuai SUN, Chris GILLIAM, Beth JELFS  
Email: shuai.sun@dlmu.edu.cn

The sample code provides an example of landmark (vehicle) management for radar SLAM.  
Trajectory of the mobile platform is fixed, and a motion noise is added as pertubation

## Radar models ##
There are 2 different radar models which can be set using the parameter _sensor.model_ in the _loadParameters_ script:
1. __Ideal__ this model directly generates landmark detections by drawing randomly from within the landmark region  
  This is the model used in the paper  
2. __PhasedArray__ this models a phased array with 4 arrays each covering 90 degrees  
  Detections are obtained using a CFAR algorithm

## Simulation parameters ##
To reproduce the results obtained in the paper, the ideal model should be used with the following parameters (Table I in the paper)

| Symbol in Paper | Name in Code | Value |
|:---:|---|---|
| _R<sub>max</sub>_ | sensor.max_range | 20 |
| _R<sub>k</sub>_ | sensor.R_radar | [0.5^2 0; 0 (pi/180)^2] |
| _Q<sub>k</sub>_ | sensor.Q_process | [1.5e-3, 0 0; 0 1.5e-3 0; 0 0 5e-5] |
| _U<sub>k</sub>_ | sensor.U_odometer | [0.022^2, 0; 0 (0.008 * pi/180)^2] |
| _&alpha;_ | thresholds.alpha | 500 |
| _&beta;_ | thresholds.beta | 20 |
| _&gamma;<sub>c</sub>_ | thresholds.cluster_radius | 2.5 |
| _&gamma;<sub>s</sub>_ | thresholds.sifting_radius | 3 |
| _&gamma;<sub>a</sub>_ | thresholds.association_radius | 3.5 | 
| _&gamma;<sub>m</sub>_ | thresholds.merge_radius | 1.5 |
| _N<sub>c<sub>1</sub></sub>_ | thresholds.point_threshold | 6 |
| _N<sub>c<sub>2</sub></sub>_ | thresholds.min_cluster_points | 2 |
| _M_ (landmark initialization) | MN_logic.M_association | 5 |
| _N_ (landmark initialization) | MN_logic.N_association | 3 |
| _M_ (landmark removal) | MN_logic.M_deleting | 10 |
| _N_ (landmark removal) | MN_logic.N_deleting | 2 |
