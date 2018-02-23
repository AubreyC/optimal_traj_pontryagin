from optimizetraj import *

if __name__ == "__main__":

    # Run code for several velocity:
    L = 10;
    M = 1;
    array = [1,2,3,4,5,6];
    for v_init in array:

        # Define init and final state
        x_init = np.array([0,-L, 0, v_init]);
        x_final = np.array([L,0, v_init, 0]);

        # Solve the system
        sol_param = find_control(x_init, x_final, M);
        
        # Draw or save the trajectories
        generate_traj(sol_param, x_init, x_final, M, False, True, 'results');