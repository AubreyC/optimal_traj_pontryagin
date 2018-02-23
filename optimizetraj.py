import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as opt
import scipy.integrate as integrate
import os

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
def generate_traj(opti_params, x_cond_init, x_cond_final, M, plot_bool=False, save_csv = False, folder_name=''):
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
    t_array = np.arange(0,T+dt,dt);


    for t in t_array:
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


    ## Save the trajectory as a CSV to be plotted using pgfplot (Latex)
    if save_csv:
        # Create data (subsampled):
        sub_sampling = 50;
        data = np.array([t_array[0::sub_sampling],\
                         x_array[0::sub_sampling],\
                         y_array[0::sub_sampling],\
                         vx_array[0::sub_sampling],\
                         vy_array[0::sub_sampling],\
                         ux_array[0::sub_sampling],\
                         uy_array[0::sub_sampling]]).transpose();

        # Create commented first line with all the information
        param_comment = '# Param [mu_1, mu_2, mu_3, mu_4, time]: %s' %(str(opti_params));
        cond_comment = '# Initial condition [x_0, y_0, v_x_0, v_y_0]: %s, Final condition [x_f, y_f, v_x_f, v_y_f]: %s ' %(str(x_cond_init), str(x_cond_final));
        accel_comment = '# Maximum acceleration (L2 norm): %d' %(M);
        csv_comment = 't, x, y, vx, vy, ux, uy';
        header_comment = '%s\n%s\n%s\n%s' %(param_comment, cond_comment, accel_comment, csv_comment);
        
        # Create name for the CSV (pretty dirty naming...)
        name_csv = 'traj_';
        for i in range(len(x_cond_init)):
            if i == len(x_cond_init) - 1:
                name_csv = name_csv + str(x_cond_init[i]); 
            else:
                name_csv = name_csv + str(x_cond_init[i]) + ';';
        name_csv = name_csv + '_';
        for i in range(len(x_cond_final)):
            if i == len(x_cond_final) - 1:
                name_csv = name_csv + str(x_cond_final[i]); 
            else:
                name_csv = name_csv + str(x_cond_final[i]) + ';';
        name_csv = name_csv + '_' + str(M) + '.csv';

        #Add folder name if necessary
        if folder_name != '':

            # Create folder if necessary
            if not os.path.exists(folder_name):
                os.makedirs(folder_name)
            name_csv = folder_name +'/'+ name_csv;

        # Save as CSV
        np.savetxt(name_csv, data, header=header_comment, fmt='%4.6f',delimiter=',', comments='');

    # Plot in desired
    if plot_bool:

        # Plotting control:
        plt.figure()
        
        #Control ux
        plt.subplot(211)
        plt.plot(np.arange(0,T+dt, dt), ux_array);
        plt.ylabel('ux')
        plt.xlabel('time (s)')

        #Control uy
        plt.subplot(212)
        plt.plot(np.arange(0,T+dt, dt), uy_array);
        plt.ylabel('uy')
        plt.xlabel('time (s)')

        # Plotting position and velocity
        plt.figure()
        plt.subplot(111)
        plt.quiver(x_array[0::50], y_array[0::50], vx_array[0::50], vy_array[0::50], units='width')
        plt.plot(x_array, y_array);
        plt.ylabel('y (m)')
        plt.xlabel('x (m)')

        # Plot control [ux, uy] so it should like like a circle
        # plt.figure()
        # plt.plot(ux_array, uy_array);
        # plt.ylabel('ux')
        # plt.xlabel('uy')
        # plt.show();

        # Plot velocity
        # plt.figure()

        # Plot vx
        # plt.subplot(211)
        # plt.plot(np.arange(0,T+dt, dt), vx_array);
        # plt.ylabel('vx')
        # plt.xlabel('time (s)')

        # Plot vy
        # plt.subplot(212)
        # plt.plot(np.arange(0,T+dt, dt), vy_array);
        # plt.ylabel('vy')
        # plt.xlabel('time (s)')

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
            print 'With param [mu_1, mu_2, mu_3, mu_4, time]: %s' %(str(param.x));
            print 'Initial condition: %s, Final condition: %s ' %(str(x_cond_init), str(x_cond_final))
            print 'Maximum acceleration (L2 norm): %d' %(M);
            break;

    return param.x;