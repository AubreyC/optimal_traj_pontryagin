import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as opt


if False:


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
M = 1;

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

    if T <= 0:
        print 'T <= 0:'
        print T

    if (np.sqrt(np.square(l1/l3 - l2/l4))) < 0.001:
        print 'daz'
        for t in np.arange(0,T+dt,dt):
            den = np.sqrt(np.square(l3) + np.square(l4));
            ux = M* (l3/den)*np.sign((l1/l3)*t-1);
            uy = M* (l4/den)*np.sign((l1/l3)*t-1);

            U_current = np.matrix([[ux],[uy]]);
            x_current_dot = A*x_current + B*U_current;
            x_current = x_current + x_current_dot*dt;

    else:
        for t in np.arange(0,T+dt,dt):
            den = np.sqrt(np.square(l1*t - l3) + np.square(l2*t - l4));
            ux = M* (l1*t - l3)/den;
            uy = M* (l2*t - l4)/den;

            U_current = np.matrix([[ux],[uy]]);
            x_current_dot = A*x_current + B*U_current;
            x_current = x_current + x_current_dot*dt;

    #print np.linalg.norm(x_current - x_final)
    return np.linalg.norm(x_current - x_final);

def func_cons(params):
    T = params[3];
    return T;

def draw_solution(params):
    l1 = params[0];
    l2 = params[1];
    l3 = params[2];
    l4 = params[3];
    T = params[4];

    ux_array = np.array([]);
    uy_array = np.array([]);

    vx_array = np.array([]);
    vy_array = np.array([]);

    x_array = np.array([]);
    y_array = np.array([]);

    x_current = x_init;
    for t in np.arange(0,T+dt, dt):
        den = np.sqrt(np.square(l1*t - l3) + np.square(l2*t - l4));
        ux = M* (l1*t - l3)/den;
        uy = M* (l2*t - l4)/den;
        # ux = M* (l1*t - l3);
        # uy = 0;

        U_current = np.matrix([[ux],[uy]]);
        x_current_dot = A*x_current + B*U_current;
        x_current = x_current + x_current_dot*dt;

        ux_array = np.append(ux_array, ux);
        uy_array = np.append(uy_array, uy);

        vx_array = np.append(vx_array, x_current[2]);
        vy_array = np.append(vy_array, x_current[3]);

        x_array = np.append(x_array, x_current[0]);
        y_array = np.append(y_array, x_current[1]);

    plt.figure()
    #Plotting Control ux
    plt.subplot(211)
    plt.plot(np.arange(0,T+dt, dt), ux_array);
    plt.ylabel('ux')
    plt.xlabel('time (s)')

    # plotting control uy
    plt.subplot(212)
    plt.plot(np.arange(0,T+dt, dt), uy_array);
    plt.ylabel('uy')
    plt.xlabel('time (s)')

    plt.figure()
    #Plotting Control ux
    plt.subplot(211)
    plt.plot(np.arange(0,T+dt, dt), vx_array);
    plt.ylabel('vx')
    plt.xlabel('time (s)')

    # plotting control uy
    plt.subplot(212)
    plt.plot(np.arange(0,T+dt, dt), vy_array);
    plt.ylabel('vy')
    plt.xlabel('time (s)')


    plt.figure()
    plt.subplot(111)
    #Plotting Control ux
    plt.plot(x_array, y_array);
    plt.ylabel('y')
    plt.xlabel('x')

    plt.show();


#Define Constraint
cons = ({'type': 'ineq', 'fun': lambda x : func_cons(x)});

for i in range(100):
    l_init = np.random.uniform(-10,10,4);
    T_init = np.random.uniform(0,20,1);

    p_init = np.append(l_init, T_init);
    param = opt.minimize(func, p_init, constraints=cons)
    
    print func(param.x);
    print p_init;
    print param.x;

    if func(param.x) < 0.1:
        break;

draw_solution(param.x)
