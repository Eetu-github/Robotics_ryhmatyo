
from controller import Robot

def line_follower(robot):
    timestep = 32
    max_speed = 6.28
    base_speed = 0.65 * max_speed

    # Motors
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')

    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    # IR sensors
    left_ir = robot.getDevice('ir0')
    right_ir = robot.getDevice('ir1')

    left_ir.enable(timestep)
    right_ir.enable(timestep)

    # Tune these values for your robot
    kp = 0.12
    last_error = 1.0
    lost_count = 0

    while robot.step(timestep) != -1:
        left = left_ir.getValue()
        right = right_ir.getValue()

        print("Left: {:.2f}, Right: {:.2f}".format(left, right))

        # Treat maximum readings as possible sensor saturation.
        # Verify this assumption against your sensor configuration.
        left_valid = left < 50
        right_valid = right < 50

        if not left_valid and not right_valid:
            # Both sensors lost: search in the last known direction.
            lost_count += 1
            search_speed = 0.25 * max_speed

            if last_error >= 0:
                left_speed = search_speed
                right_speed = -search_speed
            else:
                left_speed = -search_speed
                right_speed = search_speed

            # Stop searching if the line is lost for too long.
            if lost_count > 100:
                left_speed = 0.0
                right_speed = 0.0

        elif not left_valid or not right_valid:
            # One sensor is saturated: turn cautiously to search.
            lost_count += 1
            search_speed = 0.20 * max_speed

            if not left_valid:
                left_speed = -search_speed
                right_speed = search_speed
                last_error = -1.0
            else:
                left_speed = search_speed
                right_speed = -search_speed
                last_error = 1.0

        else:
            lost_count = 0

            # Proportional steering.
            error = left - right
            last_error = error if abs(error) > 0.05 else last_error

            correction = kp * error

            left_speed = base_speed - correction
            right_speed = base_speed + correction

            # Keep motor commands within limits.
            left_speed = max(-max_speed, min(max_speed, left_speed))
            right_speed = max(-max_speed, min(max_speed, right_speed))

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


if __name__ == "__main__":
    robot = Robot()
    line_follower(robot)
