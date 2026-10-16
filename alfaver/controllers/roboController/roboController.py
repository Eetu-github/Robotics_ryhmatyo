
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

# Asetukset
speed = 1.2
threshold = 650

Kp = 0.15
Kd = 0.04

last_error = 0.0
last_turn = 0

print("PD-viivanseuranta kaynnistyi!")

while robot.step(timestep) != -1:
    # Luetaan sensorit
    s0 = gs0.getValue()
    s1 = gs1.getValue()
    s2 = gs2.getValue()

    # Tulostetaan lukemat tarvittaessa vianetsintaa varten
    # Poista kommenttimerkki alta, jos haluat tulostaa lukemat
    # print(f"Vasen: {s0:.0f}, Keski: {s1:.0f}, Oikea: {s2:.0f}")

    # Mustan voimakkuus: 0 = valkoinen, 1 = musta
    v0 = max(0.0, min(1.0, (threshold - s0) / 150.0))
    v1 = max(0.0, min(1.0, (threshold - s1) / 150.0))
    v2 = max(0.0, min(1.0, (threshold - s2) / 150.0))

    total = v0 + v1 + v2

    # Tallennetaan edellisen kierroksen virhe
    previous_error = last_error

    if total > 0.05:
        # Viivan sijainti anturien perusteella
        error = (v0 - v2) / total

        # Muistetaan viimeisin suunta
        if error < -0.1:
            last_turn = -1
        elif error > 0.1:
            last_turn = 1

        # Päivitetään muistissa oleva virhe
        last_error = error

    else:
        # Viiva katosi: käytetään viimeisintä virhettä
        error = last_error

    # PD-säätö
    derivative = error - previous_error
    correction = Kp * error + Kd * derivative

    # Moottorien ohjaus
    if total <= 0.05:
        # Etsitään viivaa viimeiseen suuntaan
        if last_turn == -1:
            left_speed = 1.0
            right_speed = -0.3
        elif last_turn == 1:
            left_speed = -0.3
            right_speed = 1.0
        else:
            left_speed = 1.0
            right_speed = 1.0
    else:
        left_speed = speed - correction
        right_speed = speed + correction

    # Rajataan nopeudet
    max_speed = 6.28
    left_speed = max(-max_speed, min(max_speed, left_speed))
    right_speed = max(-max_speed, min(max_speed, right_speed))

    # Asetetaan moottorien nopeudet
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
