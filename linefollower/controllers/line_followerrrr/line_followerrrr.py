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

    # Viiva-anturit
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

    # Anturien kynnykset (taustakohina on noin 60-75, ÄLÄ laske näitä sen lähelle)
    obstacle_threshold = 100.0   # esteen havaitseminen viivaa seuratessa
    front_threshold = 180.0      # este suoraan edessä kierron aikana
    side_lost = 90.0             # tämän alla anturi ei näe estettä

    # Viiva-anturit (ir0, ir1): lattia lukee ~3-4, viiva nostaa lukemaa
    # (lokissa 6-10). Viiva = lukema ylittää tämän.
    line_threshold = 5.5

    # Nopeudet
    turn_speed = 0.35 * max_speed
    pass_speed = 0.4 * max_speed

    # Käännökset ja viivalle palaaminen (askelta, 1 askel = 32 ms)
    turn_steps = 29            # noin 90 astetta paikallaan kääntyen
    cross_steps = 25           # aja viivan yli, kunnes robotti on viivan päällä
    max_align_steps = 80       # turvaraja viivaan kääntymiselle
    align_min_steps = 10
    line_confirm_steps = 2     # peräkkäisiä viivahavaintoja kierron aikana

    # Esteen kiertäminen seinänseurannalla
    wall_target = 120.0        # haluttu sivuanturin lukema (~2 cm esteestä)
    wall_k = 0.02              # kuinka voimakkaasti etäisyyttä korjataan
    max_wall_corr = 0.3 * max_speed
    corner_straight_steps = 12  # ajetaan suoraan ennen kulman kaartoa
    corner_arc = 0.25 * max_speed  # kaarron voimakkuus kulmassa
    max_pass_steps = 1000      # turvaraja koko kierrolle

    # Viivalle paluun jälkeen: kierretty este ei saa laukaista uutta väistöä
    post_avoid_clear_steps = 15

    # Tilat: follow, turn, pass, cross, align
    state = 'follow'
    state_count = 0
    line_found_count = 0
    off_line_count = 0
    armed = False              # tosi, kun robotti on poistunut viivalta
    wall_lost_count = 0
    post_avoid = False
    post_avoid_clear = 0

    # +1 = väistä vasemmalle (este oikealla)
    # -1 = väistä oikealle (este vasemmalla)
    avoid_direction = 1

    def give_up(msg):
        print(msg)
        return 'follow', 0, 0, 101

    while robot.step(timestep) != -1:

        # Lue kaikki esteanturit
        ps = [sensor.getValue() for sensor in ps_sensors]

        right_strength = max(ps[0], ps[1])
        left_strength = max(ps[5], ps[6], ps[7])

        # Takaanturit (ps3, ps4) eivät kerro edessä olevasta esteestä
        obstacle_detected = max(
            ps[0], ps[1], ps[2], ps[5], ps[6], ps[7]
        ) > obstacle_threshold

        if post_avoid:
            # Viivalle paluun jälkeen huomioidaan vain suoraan edessä oleva
            # este, kunnes kierretty este ei enää näy missään anturissa
            obstacle_detected = max(ps[0], ps[7]) > obstacle_threshold

            if max(ps) < side_lost:
                post_avoid_clear += 1
            else:
                post_avoid_clear = 0

            if post_avoid_clear >= post_avoid_clear_steps:
                post_avoid = False
                print("Kierretty este ei enää näy, esteet huomioidaan taas")

        # Viiva-anturit
        left = left_ir.getValue()
        right = right_ir.getValue()

        # Huom: left < 50 on aina tosi (lukemat ovat ~3-10), joten
        # 'valid' ei kerro viivasta. Viivan havaitsemiseen käytetään
        # left_on / right_on. left_valid/right_valid jäävät vain vanhaan
        # follow-koodiin muuttumattomina.
        left_valid = left < 50
        right_valid = right < 50
        left_on = left > line_threshold
        right_on = right > line_threshold

        left_speed = 0.0
        right_speed = 0.0

        d = avoid_direction

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
                lost_count = 0
                line_found_count = 0
                off_line_count = 0
                armed = False
                wall_lost_count = 0
                post_avoid = False
                post_avoid_clear = 0

                print(
                    "Este havaittu:",
                    "oikea =", round(right_strength, 1),
                    "vasen =", round(left_strength, 1),
                    "väistösuunta =", avoid_direction
                )

                left_speed = -avoid_direction * turn_speed
                right_speed = avoid_direction * turn_speed

            else:
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

                    left_speed = max(-max_speed, min(max_speed, left_speed))
                    right_speed = max(-max_speed, min(max_speed, right_speed))

        # 2. KÄÄNNY POIS ESTEESTÄ
        elif state == 'turn':

            left_speed = -d * turn_speed
            right_speed = d * turn_speed

            state_count += 1

            if state_count >= turn_steps:
                state = 'pass'
                state_count = 0
                wall_lost_count = 0
                print("Käännös valmis, kierretään este")

        # 3. KIERRÄ ESTE, KUNNES VIIVA LÖYTYY (viiva ohittaa esteen näkymisen)
        elif state == 'pass':

            state_count += 1

            # --- Viivan etsintä on ensisijainen: tarkistetaan ensin ---
            # Robotin on ensin poistuttava viivalta, ettei lähtöviiva
            # laukaise paluuta heti
            if not left_on and not right_on:
                off_line_count += 1
            else:
                off_line_count = 0

            if off_line_count >= 3 and not armed:
                armed = True
                print("DEBUG: viivalta poistuttu, viivan etsintä aktiivinen")

            if armed and (left_on or right_on):
                line_found_count += 1
            else:
                line_found_count = 0

            if line_found_count >= line_confirm_steps:
                # Viiva löytyi: lopeta esteen kierto heti, vaikka este näkyisi
                print("Viiva löytyi (ir0 =", round(left, 1),
                      "ir1 =", round(right, 1),
                      "), esteen kierto keskeytetty")
                state = 'cross'
                state_count = 0
                line_found_count = 0
                left_speed = pass_speed
                right_speed = pass_speed

            elif state_count >= max_pass_steps:
                state, state_count, line_found_count, lost_count = give_up(
                    "Viivaa ei löytynyt, robotti pysähtyy")

            else:
                # --- Este kierretään seinänseurannalla ---
                # Este on väistösuunnan vastaisella puolella
                if d == 1:
                    side = max(ps[1], ps[2])
                else:
                    side = max(ps[5], ps[6])
                front = max(ps[0], ps[7])

                if front > front_threshold:
                    # Este suoraan edessä: käänny pois paikallaan
                    left_speed = -d * turn_speed
                    right_speed = d * turn_speed

                elif side < side_lost:
                    # Este ei näy sivulla (kulma tai esteen alku):
                    # aja ensin suoraan, sitten kaarra esteen ympäri
                    wall_lost_count += 1
                    if wall_lost_count <= corner_straight_steps:
                        correction = 0.0
                    else:
                        correction = -corner_arc
                    left_speed = pass_speed - d * correction
                    right_speed = pass_speed + d * correction

                else:
                    # Pidä etäisyys: liian lähellä -> poispäin,
                    # liian kaukana -> kohti
                    wall_lost_count = 0
                    correction = wall_k * (side - wall_target)
                    correction = max(-max_wall_corr,
                                     min(max_wall_corr, correction))
                    left_speed = pass_speed - d * correction
                    right_speed = pass_speed + d * correction

                left_speed = max(-max_speed, min(max_speed, left_speed))
                right_speed = max(-max_speed, min(max_speed, right_speed))

                if state_count % 10 == 0:
                    print("pass side =", round(side, 1),
                          "front =", round(front, 1),
                          "armed =", armed,
                          "ir0 =", round(left, 1),
                          "ir1 =", round(right, 1))

        # 4. AJA VIIVAN YLI, KUNNES ROBOTTI ON SEN PÄÄLLÄ
        elif state == 'cross':

            left_speed = pass_speed
            right_speed = pass_speed
            state_count += 1

            if state_count >= cross_steps:
                state = 'align'
                state_count = 0
                line_found_count = 0
                print("Käännytään viivan suuntaan")

        # 5. KÄÄNNY VIIVAN SUUNTAISEKSI
        elif state == 'align':

            left_speed = -d * turn_speed
            right_speed = d * turn_speed
            state_count += 1

            if state_count >= align_min_steps and (left_on or right_on):
                line_found_count += 1
            else:
                line_found_count = 0

            if line_found_count >= 2:
                state = 'follow'
                state_count = 0
                line_found_count = 0
                lost_count = 0
                last_error = d
                post_avoid = True
                post_avoid_clear = 0
                print("Viiva löytyi, jatketaan seurantaa")

            elif state_count >= max_align_steps:
                state, state_count, line_found_count, lost_count = give_up(
                    "Viivaan ei saatu linjausta, robotti pysähtyy")

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


if __name__ == "__main__":
    robot = Robot()
    line_follower(robot)