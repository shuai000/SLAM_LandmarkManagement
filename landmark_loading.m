% Define a few vehicle groups
h1 = 30;
h2 = 5;
v1 = 33;
h3 = -14;
group1_h = [h1,0,4,2;
    h1,15,4,2;];
group2_h = [h2,5,4,2;
    h2,10,4,2;
    h2,15,4,2;];
group3_v = [h3,15,2,4;
    h3,4,2,4;
    h3,21,2,4;];
group4_v = [h3,15,2,4;
    h3,21,2,4;];
group4_h = [2,-8,5,2;
    11,-8,5,2;];
group5_h = [4,v1,5,2;
    14,v1,5,2];

landmark_groundtruth_1 = [group1_h; group2_h; group3_v;group4_h;group5_h];
landmark_groundtruth_2 = [group1_h; group2_h;group4_v;group4_h;group5_h];
