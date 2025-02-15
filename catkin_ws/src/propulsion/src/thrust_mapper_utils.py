# forces produced by T200 thruster at 14V (N)
MAX_FWD_FORCE = 4.52 * 9.81
MAX_BKWD_FORCE = -3.52 * 9.81

thruster_mount_dirs = [1, -1, -1, 1, -1, 0.5, -0.5, 1]

def force_to_pwm(force):
    """
    Converts a force (N) to PWM (microseconds)
    """

    force = float(force)
    force = min(max(force, MAX_BKWD_FORCE), MAX_FWD_FORCE)

    # Different conversion for negative and positive forces
    if force > 0.0001:
        return int(positiveForceCurve(force / 9.81))

    elif force < 0.0001:
        return int(negativeForceCurve(force / 9.81))

    # Intersection is (0.0, 1500)
    else:
        return 1500


def negativeForceCurve(force):
    """
    Actual equations for converting a negative force (N) to a PWM (microseconds) for a T200 Thruster
    """
    return (
        1.4701043632380542 * (10**3)
        + force * 2.3999978362959104 * (10**2)
        + (force**2) * 2.5705773429064880 * (10**2)
        + (force**3) * 3.1133962497995367 * (10**2)
        + (force**4) * 2.1943237103469241 * (10**2)
        + (force**5) * 8.4596303821198617 * (10**1)
        + (force**6) * 1.6655229499580056 * (10**1)
        + (force**7) * 1.3116834437073399
    )


def positiveForceCurve(force):
    """
    Actual equations for converting a positive force (N) to a PWM (microseconds) for a T200 Thruster
    """
    return (
        1.5299083405100268 * (10**3)
        + force * 1.9317247519327023 * (10**2)
        + (force**2) * -1.6227874418158476 * (10**2)
        + (force**3) * 1.4980771349508325 * (10**2)
        + (force**4) * -8.0478019175136623 * (10**1)
        + (force**5) * 2.3661746039755371 * (10**1)
        + (force**6) * -3.5559291204780612
        + (force**7) * 2.1398707591286295 * (10**-1)
    )
