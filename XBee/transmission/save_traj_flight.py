import pickle
import obs_utils

while True:
	obs_utils.convert_traj_to_flight_log(pickle.load( open( "traj.p", "rb" ) ), "traj")