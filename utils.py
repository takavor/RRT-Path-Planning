import argparse
def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', type=str, default="shot.png")
    parser.add_argument('--step_size', type=int, default=50)
    parser.add_argument('--rrt_sampling_policy', type=str, default="gaussian")

    parser.add_argument('--start_pos_x', type=int, default=10)
    parser.add_argument('--start_pos_y', type=int, default=270)
    parser.add_argument('--start_pos_theta', type=float, default=0)
    parser.add_argument('--target_pos_x', type=int, default=900)
    parser.add_argument('--target_pos_y', type=int, default=30)
    parser.add_argument('--target_pos_theta', type=float, default=0)

    parser.add_argument('--robot_length', type=int, default=50)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--visualize', type=int, default=1)

    args = parser.parse_args()
    return args
