from controller import Robot


def line_follower(robot):
    timestep = 32
    max_speed = 6.28
    base_speed = 0.65 * max_speed

    # Moottorit
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')

    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    # Alkuperäiset viiva-anturit
    left_ir = robot.getDevice('ir0')
    right_ir = robot.getDevice('ir1')
    left_ir.enable(timestep)
    right_ir.enable(timestep)

    # E-puckin kahdeksan läheisyysanturia
    ps_sensors = []
    for i in range(8):
        sensor = robot.getDevice('ps' + str(i))
        sensor.enable(timestep)
        ps_sensors.append(sensor)

    # Viivanseurannan asetukset
    kp = 0.12
    last_error = 1.0
    lost_count = 0

    # Esteväistön asetukset
    obstacle_threshold = 100.0
    turn_speed = 0.35 * max_speed
    pass_speed = 0.35 * max_speed

    turn_steps = 16
    min_pass_steps = 35
    max_pass_steps = 100
    clear_steps_required = 15
    max_search_steps = 200

    # Tilat: follow, turn, pass, search
    state = 'follow'
    state_count = 0
    clear_count = 0

    # +1 = väistä vasemmalle
    # -1 = väistä oikealle
    avoid_direction = 1

    while robot.step(timestep) != -1:

        # Lue kaikki esteanturit
        ps = [sensor.getValue() for sensor in ps_sensors]
        if state == 'pass':
                print(
                    "Ohitusanturit:",
                    [round(value, 1) for value in ps]
                )

        right_strength = max(ps[0], ps[1])
        left_strength = max(ps[5], ps[6], ps[7])

        obstacle_detected = max(ps) > obstacle_threshold

        # Viiva-anturit
        left = left_ir.getValue()
        right = right_ir.getValue()

        left_valid = left < 50
        right_valid = right < 50

        left_speed = 0.0
        right_speed = 0.0

        # 1. NORMAALI VIIVAN SEURAAMINEN
        if state == 'follow':

            if obstacle_detected:
                if right_strength > left_strength:
                    avoid_direction = 1
                elif left_strength > right_strength:
                    avoid_direction = -1
                else:
                    avoid_direction = 1 if last_error >= 0 else -1

                state = 'turn'
                state_count = 0
                clear_count = 0
                lost_count = 0

                print(
                    "Este havaittu:",
                    "oikea =", round(right_strength, 1),
                    "vasen =", round(left_strength, 1),
                    "väistösuunta =", avoid_direction
                )

                left_speed = -avoid_direction * turn_speed
                right_speed = avoid_direction * turn_speed

            else:
                # Alkuperäinen viivanseuranta
                if not left_valid and not right_valid:
                    lost_count += 1
                    search_speed = 0.25 * max_speed

                    if last_error >= 0:
                        left_speed = search_speed
                        right_speed = -search_speed
                    else:
                        left_speed = -search_speed
                        right_speed = search_speed

                    if lost_count > 100:
                        left_speed = 0.0
                        right_speed = 0.0

                elif not left_valid or not right_valid:
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
                    error = left - right

                    if abs(error) > 0.05:
                        last_error = error

                    correction = kp * error

                    left_speed = base_speed - correction
                    right_speed = base_speed + correction

                    left_speed = max(
                        -max_speed, min(max_speed, left_speed)
                    )
                    right_speed = max(
                        -max_speed, min(max_speed, right_speed)
                    )

        # 2. KÄÄNNY POIS ESTEESTÄ
        elif state == 'turn':

            left_speed = -avoid_direction * turn_speed
            right_speed = avoid_direction * turn_speed

            state_count += 1

            if state_count >= turn_steps:
                state = 'pass'
                state_count = 0
                clear_count = 0
                print("Käännös valmis, ajetaan esteen ohi")

        # 3. AJA ESTEEN OHI
        
        elif state == 'pass':

            state_count += 1

            # Anturiryhmät kuution ohitusta varten.
            # ps0 ja ps1 reagoivat voimakkaasti esteeseen.
            right_strength = max(ps[0], ps[1])
            left_strength = max(ps[5], ps[6], ps[7])

            # Käytetään myös ps2:ta, koska se reagoi
            # voimakkaasti kuution lähestyessä.
            front_strength = ps[2]

            # Perusnopeus ohituksen aikana
            pass_speed = 0.35 * max_speed

            # Voimakas havainto ps0/ps1-ryhmässä:
            # käännytään poispäin oikean puolen esteestä.
            # Voimakas havainto vasemmalla:
            # käännytään poispäin vasemman puolen esteestä.
            right_pressure = max(0.0, right_strength - 80.0)
            left_pressure = max(0.0, left_strength - 80.0)

            # Jos etuanturi reagoi voimakkaasti, käännytään
            # poispäin sen puolen estehavainnosta.
            front_pressure = max(0.0, front_strength - 100.0)

            # Valitaan ohjaussuunta mitattujen anturien perusteella.
            steering = right_pressure - left_pressure

            if front_pressure > 0:
                steering += avoid_direction * front_pressure

            # Rajataan käännöksen voimakkuus
            turn_amount = min(
                0.50 * max_speed,
                0.004 * abs(steering)
            )

            if steering > 0:
                # Este havaitaan voimakkaammin oikealla
                left_speed = pass_speed + turn_amount
                right_speed = pass_speed - turn_amount

            elif steering < 0:
                # Este havaitaan voimakkaammin vasemmalla
                left_speed = pass_speed - turn_amount
                right_speed = pass_speed + turn_amount

            else:
                left_speed = pass_speed
                right_speed = pass_speed

            # Rajataan moottorien nopeudet
            left_speed = max(
                -max_speed, min(max_speed, left_speed)
            )
            right_speed = max(
                -max_speed, min(max_speed, right_speed)
            )

            # Ohitus päättyy vasta, kun este ei enää
            # laukaise läheisyysantureita riittävän pitkään.
            if max(ps) > obstacle_threshold:
                clear_count = 0
            else:
                clear_count += 1

            if (
                state_count >= min_pass_steps
                and clear_count >= clear_steps_required
            ) or state_count >= max_pass_steps:

                state = 'search'
                state_count = 0
                clear_count = 0
                print("Ohitusvaihe valmis, etsitään viivaa")


        # 4. ETSI MUSTAA VIIVAA
        elif state == 'search':

            search_speed = 0.22 * max_speed
            turn_component = 0.28 * max_speed

            left_speed = (
                search_speed + avoid_direction * turn_component
            )
            right_speed = (
                search_speed - avoid_direction * turn_component
            )

            left_speed = max(
                -max_speed, min(max_speed, left_speed)
            )
            right_speed = max(
                -max_speed, min(max_speed, right_speed)
            )

            state_count += 1

            if left_valid or right_valid:
                state = 'follow'
                state_count = 0
                lost_count = 0
                print("Viiva löytyi, jatketaan seurantaa")

            elif state_count >= max_search_steps:
                left_speed = 0.0
                right_speed = 0.0

                state = 'follow'
                state_count = 0
                lost_count = 101

                print("Viivaa ei löytynyt, robotti pysähtyy")

        # Moottorien ohjaus suoritetaan joka kierroksella
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


if __name__ == "__main__":
    robot = Robot()
    line_follower(robot)
