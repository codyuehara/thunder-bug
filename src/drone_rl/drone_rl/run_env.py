from drone_rl.drone_env import DroneRacingEnv
from stable_baselines3 import PPO

def main():
    env = DroneRacingEnv()
    env.start_ros()
#    model = PPO("MlpPolicy", env, verbose=1)
#    model.learn(total_timesteps=10000)
#    model.save('drone_racing_ppo')
    
    obs = env.reset()
    for _ in range(10):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        print(obs, reward, done)
