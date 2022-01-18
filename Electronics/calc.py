
# This file serves to provide calculation steps for solving a given circuit, as if by hand.


"""small-signal output resistance from """
ro = lambda l, ID:  1/(l * ID)


def solve_quadratic(A, B, C):
    """This equation arises from non-zero source resistance.
    x=A(B+Cx)^2
    x=A(B^2 + BCx + C^2x^2)
    0 = AB^2 + (ABC - 1)x + AC^2x^2"""

    a = A * C ** 2
    b = 2 * A * B * C - 1
    c = A * B ** 2

    return (-b + (b**2 - 4 * a * c) ** 0.5) / (2*a)


def solve_ID(kn_, Rs_, Vss_, Vod_):
    """Specific solution to the above equation for real parameters"""
    return solve_quadratic(2 * kn_, Vod_ - Vss_, -Rs_)


if __name__ == '__main__':
    kn = 0.001
    Rs = 500
    Vod = -3.25 - 0.8

    # solve ID
    Id = solve_ID(kn)


    print("ID is: {}".format(Id))