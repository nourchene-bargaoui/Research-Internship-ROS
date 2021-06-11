#!/usr/bin/env python3

#import M2wrTrainingEnv
import gym
import numpy
import time
import qlearn
from gym import wrappers
from gym.envs.registration import register
import rospy
import rospkg

from openai_ros.task_envs.robot import my_robot_training_env


if __name__ == '__main__':
	max_episode_steps_per_episode = 100 # Can be any Value

	register(
        id='M2wr-v0',
        entry_point='openai_ros.task_envs.robot.my_robot_training_env:M2wrTrainingEnv',
        max_episode_steps=max_episode_steps_per_episode
        )

	rospy.init_node('M2wrTrainingEnv_qlearn', anonymous=True, log_level=rospy.WARN)
    #task_and_robot_environment_name = rospy.get_param('/m2wr/task_and_robot_environment_name')
    #env = StartOpenAI_ROS_Environment(task_and_robot_environment_name)
    # Create the Gym environment


	env = gym.make('M2wr-v0')
	rospy.loginfo("Gym environment done")

    # Set the logging system
	rospack = rospkg.RosPack()
	pkg_path = rospack.get_path('robot_training')
	outdir = pkg_path + '/training_results'
	env = wrappers.Monitor(env, outdir, force=True)
	rospy.loginfo("Monitor Wrapper started")

	last_time_steps = numpy.ndarray(0)

    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
	Alpha = rospy.get_param("/m2wr/alpha")
	Epsilon = rospy.get_param("/m2wr/epsilon")
	Gamma = rospy.get_param("/m2wr/gamma")
	epsilon_discount = rospy.get_param("/m2wr/epsilon_discount")
	nepisodes = rospy.get_param("/m2wr/nepisodes")
	nsteps = rospy.get_param("/m2wr/nsteps")

	running_step = rospy.get_param("/m2wr/running_step")

	# Initialises the algorithm that we are going to use for learning
	qlearn = qlearn.QLearn(actions=range(env.action_space.n),
		                   alpha=Alpha, gamma=Gamma, epsilon=Epsilon)
	initial_epsilon = qlearn.epsilon

	start_time = time.time()
	highest_reward = 0

    # Starts the main training loop: the one about the episodes to do
	for x in range(nepisodes):
		rospy.logdebug("############### START EPISODE=>" + str(x))

		cumulated_reward = 0
		done = False
		if qlearn.epsilon > 0.05:
			qlearn.epsilon *= epsilon_discount

        # Initialize the environment and get first state of the robot
		observation = env.reset()
		state = ''.join(map(str, observation))

        # Show on screen the actual situation of the robot
        # env.render()
        # for each episode, we test the robot for nsteps
		for i in range(nsteps):
			rospy.logwarn("############### Start Step=>" + str(i))
			# Pick an action based on the current state
			action = qlearn.chooseAction(state)
			rospy.logwarn("Next action is:%d", action)
			# Execute the action in the environment and get feedback
			observation, reward, done, info = env.step(action)

			#rospy.logwarn(str(observation) + " " + str(reward))
			cumulated_reward += reward
			if highest_reward < cumulated_reward:
				highest_reward = cumulated_reward

			nextState = ''.join(map(str, observation))

            # Make the algorithm learn based on the results
			#rospy.logwarn("# state we were=>" + str(state))
			rospy.logwarn("# action that we took=>" + str(action))
			rospy.logwarn("# reward that action gave=>" + str(reward))
			rospy.logwarn("# episode cumulated_reward=>" + str(cumulated_reward))
			#rospy.logwarn("# State in which we will start next step=>" + str(nextState))
			qlearn.learn(state, action, reward, nextState)

			if not (done):
				rospy.logwarn("NOT DONE")
				state = nextState
			else:
				rospy.logwarn("DONE")
				last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
				break
			rospy.logwarn("############### END Step=>" + str(i))
            #raw_input("Next Step...PRESS KEY")
            # rospy.sleep(2.0)
		m, s = divmod(int(time.time() - start_time), 60)
		h, m = divmod(m, 60)
		rospy.logerr(("EP: " + str(x + 1) + " - [alpha: " + str(round(qlearn.alpha, 2)) + " - gamma: " + str(
			round(qlearn.gamma, 2)) + " - epsilon: " + str(round(qlearn.epsilon, 2)) + "] - Reward: " + str(
			cumulated_reward) + "     Time: %d:%02d:%02d" % (h, m, s)))

	rospy.loginfo(("\n|" + str(nepisodes) + "|" + str(qlearn.alpha) + "|" + str(qlearn.gamma) + "|" + str(
		initial_epsilon) + "*" + str(epsilon_discount) + "|" + str(highest_reward) + "| PICTURE |"))

	l = last_time_steps.tolist()
	l.sort()

	# print("Parameters: a="+str)
	rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
	rospy.loginfo("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))

	env.close()
