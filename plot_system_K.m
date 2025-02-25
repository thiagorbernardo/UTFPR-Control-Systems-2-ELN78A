num = [0 0.1294];
den = [1 -1.185 0.1342 0.1803];

H_discrete = tf(num, den, 0.1);

C = pidtune(H_discrete, 'PID');

KP = C.Kp;
KI = C.Ki;
KD = C.Kd;

K = 3.2;
controller = K * C;

sys_cl = feedback(controller * H_discrete, 1);

step(sys_cl);