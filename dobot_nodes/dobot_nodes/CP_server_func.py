import sys
import os
#sys.path.insert(0, os.path.abspath('.'))
#sys.path.append("..")
from dobot_driver.dobot_handle import bot

import math

print('Bot status:', 'connected' if bot.connected() else 'not connected')


params = bot.get_continous_trajectory_params()
print('Params:', params)

[start_x, start_y, start_z, start_r] = bot.get_pose()[0:4]

print([start_x, start_y, start_z, start_r])

bot.set_continous_trajectory_real_time_params(20, 100, 10)

# Draw about half an arch as a single path
bot.stop_queue()
steps = 24
scale = 25
for i in range(steps + 2):
    x = math.cos((2*math.pi / steps) * i)
    y = math.sin((2*math.pi / steps) * i)

    # Absolute movement
    bot.set_continous_trajectory_command(1, start_x + x * scale, start_y + y * scale, start_z, 50, queue=True)

bot.start_queue()