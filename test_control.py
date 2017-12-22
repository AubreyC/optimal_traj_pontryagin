import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as opt


lamba_1 = 1;
lamba_2 = 2;
lamba_3 = 3;
lamba_4 = 4;
M = 2;

time =  np.arange(0,10,0.1)
ux_array = np.array([]);
uy_array = np.array([]);

for t in time:
    den = np.sqrt(np.square(lamba_1*t - lamba_3) + np.square(lamba_2*t - lamba_4));
    ux = M* (lamba_1*t - lamba_3)/den;
    uy = M* (lamba_2*t - lamba_4)/den;
    ux_array = np.append(ux_array, ux);
    uy_array = np.append(uy_array, uy);
    #print t


if False:
    plt.figure(1)
    #Plotting Control ux
    plt.subplot(211)
    plt.plot(time, ux_array);
    plt.ylabel('ux')
    plt.xlabel('time (s)')

    # plotting control uy
    plt.subplot(212)
    plt.plot(time, uy_array);
    plt.ylabel('uy')
    plt.xlabel('time (s)')
    plt.show();

dt = 0.01;
A = np.matrix('0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0');

B = np.matrix('0 0; 0 0; 1 0; 0 1');

L = 10;
v_init = 2;
x_init = np.matrix([0,-L, 0, v_init]).transpose();
x_final = np.matrix([L,0, v_init, 0]).transpose();

def func(params):
    l1 = params[0];
    l2 = params[1];
    l3 = params[2];
    l4 = params[3];
    T = params[4];

    x_current = x_init;
    for t in np.arange(0,dt,T+dt):
        den = np.sqrt(np.square(l1*t - l3) + np.square(l2*t - l4));
        ux = M* (l1*t - l3)/den;
        uy = M* (l2*t - l4)/den;
        U_current = np.matrix([[ux],[uy]]);
        x_current_dot = A*x_current + B*U_current;
        x_current = x_current + x_current_dot*dt;

    return np.linalg.norm(x_current - x_final);

p_init = np.array([1,1,1,1,1]);
param = opt.root(func, p_init)





