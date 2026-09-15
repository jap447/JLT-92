def is_prime_number(n):
    assert isinstance(n, int), "Did you cast to int in Q4.2?"
    """Return whether given integer is a prime_number.

    >>> is_prime_number(9)
    False
    >>> is_prime_number(2011)
    True

    """
    # BEGIN QUESTION 4.1
    if n <= 1:
        return False
    for i in range(2, int(n**0.5) + 1):
        if n % i == 0:
            return False
    return True
    # END QUESTION 4.1
