from optimizetraj import *
import sys
import argparse

def main(args_input):

    # ##########################################################
    # # Parse Arguments
    # ##########################################################
    argparser = argparse.ArgumentParser(
        description='Time Optimal Turn-Right Trajectory using Pontryagin\’s Maximum principle')
    argparser.add_argument(
        '-max_accel',
        type=float,
        default=1.0,
        help='Maximum Acceleration')
    argparser.add_argument(
        '-init_pos',
        type=float,
        default=10.0,
        help='Initial position along Y axis')
    argparser.add_argument(
        '-init_vel',
        type = float,
        nargs = '*',
        default = [1,2,3,4,5,6],
        help='Initial velocities');
    argparser.add_argument(
        '-show_figures',
        action ='store_true',
        help='Plot results on figures');
    argparser.add_argument(
        '-no_save_csv',
        action ='store_true',
        help='Do not save results');

    argparser.add_argument(
        '-output_dir',
        type=str,
        default='results',
        help='Path of the output');

    args = argparser.parse_args(args_input);

    # Run code with inputs:
    L = args.init_pos;
    M = args.max_accel;
    vel_init_list = args.init_vel;

    # Output dir:
    output_dir = args.output_dir;
    show_figures = args.show_figures;
    save_csv = not args.no_save_csv;

    for v_init in vel_init_list:

        # Define init and final state
        x_init = np.array([0,-L, 0, v_init]);
        x_final = np.array([L,0, v_init, 0]);

        # Solve the system
        sol_param = find_control(x_init, x_final, M);

        # Draw or save the trajectories
        generate_traj(sol_param, x_init, x_final, M, show_figures, save_csv, output_dir);


if __name__ == '__main__':

    try:
        main(sys.argv[1:])
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
