import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as opt
import scipy.integrate as integrate

# Define the dynamic of the system: X_dot = A*X + B*U
A = np.matrix('0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0');
B = np.matrix('0 0; 0 0; 1 0; 0 1');

# The Control input according to the Pontryagin's Maximum Principle
def func_accel(t, lam, M, ind):
    l1 = float(lam[0]);
    l2 = float(lam[1]);
    l3 = float(lam[2]);
    l4 = float(lam[3]);

    if (np.sqrt(np.square(l1/l3 - l2/l4))) < 0.001:
        den = np.sqrt(np.square(l3) + np.square(l4));
        ux = -M* (l3/den)*np.sign((l1/l3)*t-1);
        uy = -M* (l4/den)*np.sign((l1/l3)*t-1);
    else:
        den = np.sqrt(np.square(l1*t - l3) + np.square(l2*t - l4));
        ux = -M* (l1*t - l3)/den;
        uy = -M* (l2*t - l4)/den;

    u = np.array([ux, uy]);
    return u[ind];

# The acceleration by integrating of the control input
def func_vel(t, lam, M, v0, ind):
    result = integrate.quad(func_accel, 0, t, args=(lam, M, ind));
    return  result[0] + v0[ind] 

# The position by integrating of the control velocity
def func_pos(t, lam, M, x0, v0, ind):
    result = integrate.quad(func_vel, 0, t, args=(lam, M, v0, ind));
    return result[0] + x0[ind] 

# Compute the trajectory based on the paramaters
def func(opti_params, x_cond_init, x_cond_final, M):

    l1 = float(opti_params[0]);
    l2 = float(opti_params[1]);
    l3 = float(opti_params[2]);
    l4 = float(opti_params[3]);
    T =  float(opti_params[4]);

    lam = np.array([l1,l2,l3,l4])
    x0 = x_cond_init[0:2];
    v0 = x_cond_init[2:4];

    xf = np.array([0.,0.,0.,0.]);
    xf[0] = func_pos(T, lam, M, x0, v0, 0)
    xf[1] = func_pos(T, lam, M, x0, v0, 1)
    xf[2] = func_vel(T, lam, M, v0, 0)
    xf[3] = func_vel(T, lam, M, v0, 1)

    result = np.array([xf[0] - x_cond_final[0],
                       xf[1] - x_cond_final[1],
                       xf[2] - x_cond_final[2],
                       xf[3] - x_cond_final[3]]); 

    return np.linalg.norm(result);

# Constraint function: Constraint the time to be positive
def func_cons(opti_params):
    T = opti_params[4];
    return T;

# Draw the solution
def draw_solution(opti_params, x_cond_init, x_cond_final, M):
    l1 = float(opti_params[0]);
    l2 = float(opti_params[1]);
    l3 = float(opti_params[2]);
    l4 = float(opti_params[3]);
    T =  float(opti_params[4]);

    lam = np.array([l1,l2,l3,l4])

    # Step used to draw solution
    dt = 0.01;

    # Create empty array to store the trajectory
    ux_array = np.array([]);
    uy_array = np.array([]);

    vx_array = np.array([]);
    vy_array = np.array([]);

    x_array = np.array([]);
    y_array = np.array([]);

    x_current = np.matrix(x_cond_init).transpose();


    for t in np.arange(0,T+dt,dt):
        ux = func_accel(t, lam, M, 0);
        uy = func_accel(t, lam, M, 1);

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
    plt.quiver(x_array[0::50], y_array[0::50], vx_array[0::50], vy_array[0::50], units='width')
    plt.plot(x_array, y_array);
    plt.ylabel('y')
    plt.xlabel('x')

    plt.figure()
    #Plotting Control
    plt.plot(ux_array, uy_array);
    plt.ylabel('ux')
    plt.xlabel('uy')
    plt.show();


def find_control(x_cond_init, x_cond_final, M):

    #Define Constraint: Only on time
    cons = ({'type': 'ineq', 'fun': lambda x : func_cons(x)});

    # Optimization problem with random initial condition to avoid local minimum
    for i in range(100):

        l_init = np.random.uniform(0,5,4);
        T_init = np.random.uniform(0,30,1);

        p_init = np.append(l_init, T_init);
        p_init = p_init.flatten()

        # Run the minimization
        param = opt.minimize(func, p_init, constraints=cons, args=(x_cond_init, x_cond_final, M))

        if np.linalg.norm(func(param.x, x_cond_init, x_cond_final, M)) < 0.1:
            print 'Converged: %s' %(np.linalg.norm(func(param.x, x_cond_init, x_cond_final, M)));
            print 'With param mu: %s, Time: %s' %(str(param.x[0:-2]), param.x[-1]);
            print 'Initial condition: %s, Final condition: %s ' %(str(x_cond_init), str(x_cond_final))
            print 'Maximum acceleration (L2 norm): %d' %(M);
            break;

    return param.x;

# Define initial and final conditions
L = 10;
M = 1;
v_init = 7;
x_init = np.array([0,-L, 0, v_init]);
x_final = np.array([L,0, v_init, 0]);

sol_param = find_control(x_init, x_final, M);
draw_solution(sol_param, x_init, x_final, M);
