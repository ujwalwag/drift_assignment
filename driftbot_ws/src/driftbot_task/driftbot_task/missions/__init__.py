"""Mission dispatcher."""
from . import mission_01
from . import mission_02


def run(bot, mission_id: int):
    """Run all phases."""
    bot.get_logger().info('=== Phase 1: pick block ===')
    mission_01.run(bot)

    bot.get_logger().info('=== Phase 2: pick can+ball ===')
    mission_02.run(bot)

    bot.get_logger().info('=== All phases complete ===')
