time = 10/1000;

KP = 0.5;
KI = 9*time;
KD = 0.003/time;

TARGET = 120;

distance = 120;
distance_pre = 119;
outputSum = 0;

error = TARGET - distance;
dInput = distance - distance_pre;

outputSum = outputSum + KI*error - KP*dInput;

output = KP*error + outputSum - KD*dInput;