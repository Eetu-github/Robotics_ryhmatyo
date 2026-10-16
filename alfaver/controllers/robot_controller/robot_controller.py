
from controller import Robot

robot = Robot()
timestep = 32

# Moottorit
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Maasensorit
gs0 = robot.getDevice('gs0')
gs1 = robot.getDevice('gs1')
gs2 = robot.getDevice('gs2')

gs0.enable(timestep)
gs1.enable(timestep)
gs2.enable(timestep)

# Nopeudet
base_speed = 3.0
turn_speed = 1.5
threshold = 650

# Viimeisin kääntymissuunta:
# -1 = vasen, 1 = oikea
last_turn = 0

while robot.step(timestep) != -1:

    s0 = gs0.getValue()
    s1 = gs1.getValue()
    s2 = gs2.getValue()

    on0 = s0 > threshold
    on1 = s1 > threshold
    on2 = s2 > threshold

    print(
        "gs0:", round(s0),
        "gs1:", round(s1),
        "gs2:", round(s2)
    )

    # Kaikki sensorit näkevät viivan
    if on0 and on1 and on2:
        left_speed = 2.0
        right_speed = 2.0

    # Keskisensori näkee viivan
    elif on1 and not on0 and not on2:
        left_speed = base_speed
        right_speed = base_speed

    # Viiva gs0-sensorin kohdalla
    elif on0:
        last_turn = -1
        left_speed = 1.0
        right_speed = turn_speed

    # Viiva gs2-sensorin kohdalla
    elif on2:
        last_turn = 1
        left_speed = turn_speed
        right_speed = 1.0

    # Viivaa ei löydy: haetaan sitä varovasti
    else:
        if last_turn == -1:
            left_speed = 0.5
            right_speed = 2.0
        elif last_turn == 1:
            left_speed = 2.0
            right_speed = 0.5
        else:
            left_speed = 1.5
            right_speed = 1.5

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
