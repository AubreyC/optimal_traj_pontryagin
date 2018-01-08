import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as opt
import scipy.integrate as integrate


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


def func_accel(t, lam, ind):
    l1 = float(lam[0]);
    l2 = float(lam[1]);
    l3 = float(lam[2]);
    l4 = float(lam[3]);

    if (np.sqrt(np.square(l1/l3 - l2/l4))) < 0.001:
        print 'daz'
        den = np.sqrt(np.square(l3) + np.square(l4));
        ux = -M* (l3/den)*np.sign((l1/l3)*t-1);
        uy = -M* (l4/den)*np.sign((l1/l3)*t-1);
    else:
        den = np.sqrt(np.square(l1*t - l3) + np.square(l2*t - l4));
        ux = -M* (l1*t - l3)/den;
        uy = -M* (l2*t - l4)/den;

    u = np.array([ux, uy]);
    return u[ind];

def func_vel(t, lam, v0, ind):
    result = integrate.quad(func_accel, 0, t, args=(lam, ind));
    return  result[0] + v0[ind] 

def func_pos(t, lam, x0, v0, ind):
    result = integrate.quad(func_vel, 0, t, args=(lam, v0, ind));
    return result[0] + x0[ind] 


def func(params):
    l1 = float(params[0]);
    l2 = float(params[1]);
    l3 = float(params[2]);
    l4 = float(params[3]);
    T =  float(params[4]);

    lam = np.array([l1,l2,l3,l4])
    x0 = x_init[0:2];
    v0 = x_init[2:4];

    xf = np.array([0.,0.,0.,0.]);
    xf[0] = func_pos(T, lam,x0, v0,0)
    xf[1] = func_pos(T, lam,x0, v0,1)
    xf[2] = func_vel(T, lam, v0,0)
    xf[3] = func_vel(T, lam, v0,1)

    # x_current = x_init;

    # if (np.sqrt(np.square(l1/l3 - l2/l4))) < 0.001:
    #     for t in np.arange(0,T+dt,dt):
    #         den = np.sqrt(np.square(l3) + np.square(l4));
    #         ux = -M* (l3/den)*np.sign((l1/l3)*t-1);
    #         uy = -M* (l4/den)*np.sign((l1/l3)*t-1);

    #         U_current = np.matrix([[ux],[uy]]);
    #         x_current_dot = A*x_current + B*U_current;
    #         x_current = x_current + x_current_dot*dt;

    # else:
    #     for t in np.arange(0,T+dt,dt):
    #         den = np.sqrt(np.square(l1*t - l3) + np.square(l2*t - l4));
    #         ux = -M* (l1*t - l3)/den;
    #         uy = -M* (l2*t - l4)/den;

    #         U_current = np.matrix([[ux],[uy]]);
    #         x_current_dot = A*x_current + B*U_current;
    #         x_current = x_current + x_current_dot*dt;


    result = np.array([xf[0] - x_final[0,0],
                       xf[1] - x_final[1,0],
                       xf[2] - x_final[2,0],
                       xf[3] - x_final[3,0],
                       0]); 

    return result;

    #return np.linalg.norm(result);

def func_cons(params):
    T = params[4];
    return T;

def draw_solution(params):
    l1 = float(params[0]);
    l2 = float(params[1]);
    l3 = float(params[2]);
    l4 = float(params[3]);
    T =  float(params[4]);

    ux_array = np.array([]);
    uy_array = np.array([]);

    vx_array = np.array([]);
    vy_array = np.array([]);

    x_array = np.array([]);
    y_array = np.array([]);

    x_current = x_init;
    for t in np.arange(0,T+dt, dt):
        den = np.sqrt(np.square(l1*t - l3) + np.square(l2*t - l4));
        ux = -M* (l1*t - l3)/den;
        uy = -M* (l2*t - l4)/den;
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


# T = 10;
# x0 = x_init[0:2];
# v0 = x_init[2:4];
# lam = np.array([1.0,2.0,3.0,4.0])
# #lam = np.array([1,2,3,4])

# xf = np.array([0.,0.,0.,0.]);
# xf[0] = func_pos(10, lam,x0, v0,0)
# # xf[1] = func_pos(10, lam,x0, v0,1)
# # xf[2] = func_vel(10, lam, v0,0)
# # xf[3] = func_vel(10, lam, v0,1)
# print 'Int: '
# print xf
# print ''

# p_init = np.append(lam, T);
# print 'Iter: '
# print func(p_init);


#Define Constraint
cons = ({'type': 'ineq', 'fun': lambda x : func_cons(x)});

for i in range(100):
    l_init = np.random.uniform(0,5,4);
    T_init = np.random.uniform(0,30,1);

    p_init = np.append(l_init, T_init);
    p_init = p_init.flatten()
    #print p_init
    #param = opt.minimize(func, p_init, constraints=cons)
    param = opt.root(func, p_init, method='lm')

    print 'Solution:'
    print func(param.x);
    print p_init;
    print param.x;
    print ' '

    if np.linalg.norm(func(param.x)) < 0.1:
        print 'break'
        break;

draw_solution(param.x)
