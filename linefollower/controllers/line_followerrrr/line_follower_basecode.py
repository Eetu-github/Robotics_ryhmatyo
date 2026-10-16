from controller import Robot

def line_follower(my_robot):

    timestep = 32
    max_speed = 6.28
    
    # Moottorit
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    
    left_ir = robot.getDevice('ir0')
    left_ir.enable(timestep)
    
    right_ir = robot.getDevice('ir1')
    right_ir.enable(timestep)
    
    while robot.step(timestep) != -1:

        left_ir_value = left_ir.getValue()
        right_ir_value = right_ir.getValue()

        print("Left IR value: {}, Right IR value: {}".format(left_ir_value, right_ir_value))
        
        left_speed = max_speed
        right_speed = max_speed
        
        if (left_ir_value > right_ir_value) and (6 < left_ir_value < 15):
            left_speed = -max_speed * 0.25
        elif (right_ir_value > left_ir_value) and (6 < right_ir_value < 15):
            right_speed = -max_speed * 0.25
        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        
if __name__ == "__main__":
    robot = Robot()
    line_follower(robot)  