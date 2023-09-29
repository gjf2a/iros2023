def fuzzify_rising(value, start, end):
    if value > end:
        return 1.0
    elif value < start:
        return 0.0
    else:
        return (value - start) / (end - start)


def fuzzify_falling(value, start, end):
    return f_not(fuzzify_rising(value, start, end))


def f_not(value):
    return 1.0 - value


def defuzzify(value, zero, one):
    if zero > one:
        return defuzzify(f_not(value), one, zero)
    else:
        return zero + value * (one - zero)

