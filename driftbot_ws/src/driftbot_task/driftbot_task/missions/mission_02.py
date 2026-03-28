"""Mission 2 — pick can+ball, drop."""
from driftbot_task.task_node import CAN_XY, BALL_XY, BIN_XY


def run(bot):
    bot.get_logger().info('── Mission 2 ──')

    bot.wait_for_odom()
    bot.wait_for_imu()
    bot.arm_carry()
    bot.halt(1.0)

    # Can
    bot.get_logger().info('Bug2 to can')
    bot.bug2_drive_to(CAN_XY[0], CAN_XY[1])
    bot.halt()

    bot.get_logger().info('Pick can')
    bot.pick('/can/attach', CAN_XY[0], CAN_XY[1])
    bot.halt()

    bot.get_logger().info('Bug2 to bin')
    bot.bug2_drive_to(BIN_XY[0], BIN_XY[1])
    bot.halt()

    bot.get_logger().info('Drop can')
    bot.drop()
    bot.halt()

    # Ball
    bot.get_logger().info('Bug2 to ball')
    bot.bug2_drive_to(BALL_XY[0], BALL_XY[1])
    bot.halt()

    bot.get_logger().info('Pick ball')
    bot.pick('/ball/attach', BALL_XY[0], BALL_XY[1])
    bot.halt()

    bot.get_logger().info('Bug2 to bin')
    bot.bug2_drive_to(BIN_XY[0], BIN_XY[1])
    bot.halt()

    bot.get_logger().info('Drop ball')
    bot.drop()
    bot.halt()

    bot.get_logger().info('── Mission 2 done ──')
