"""Mission 1 — pick block, drop."""
from math import pi, hypot
from driftbot_task.task_node import BLOCK_XY, BIN_XY


def run(bot):
    bot.get_logger().info('── Mission 1 ──')

    bot.wait_for_odom()
    bot.wait_for_imu()
    bot.arm_carry()
    bot.halt(1.0)

    bot.get_logger().info('Pick block')
    bot.pick('/toy_block_1/attach', BLOCK_XY[0], BLOCK_XY[1])
    bot.halt()

    bot.get_logger().info('Turn 180°')
    bot.turn_by_imu(pi)
    bot.halt()

    bot.get_logger().info('Lock heading north')
    bot.settle_heading(pi / 2)
    bot.halt(0.3)

    pose = bot.world_pose()
    dist = hypot(BIN_XY[0] - pose[0], BIN_XY[1] - pose[1])
    bot.get_logger().info(f'Drive {dist:.2f}m north')
    bot.drive_straight(dist, hold_wyaw=pi / 2)
    bot.halt()

    bot.get_logger().info('Turn -90°')
    bot.turn_by_imu(-pi / 2)
    bot.halt()

    bot.get_logger().info('Lock heading east')
    bot.settle_heading(0.0)
    bot.halt(0.3)

    bot.get_logger().info('Drop block')
    bot.drop()
    bot.halt()

    bot.get_logger().info('── Mission 1 done ──')
