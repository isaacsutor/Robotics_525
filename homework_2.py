# Isaac Sutor
import numpy
import math
def q1():
    # 1
    # 1.a
    # v/r = w
    velocity = .2
    radius = 3
    body_rotation_rate = velocity/radius

    print("1)")
    print("a)")
    print(body_rotation_rate)

    # 1.b
    # wz(p - c) = R(Psi)L
    # wz p = vb
    # wz(p + c) = R(Psi)R
    # vb = (R(Psi)L - R(Psi)R)/2c
    p = 3
    c = .23*.5
    # print(c)
    # body_rotation_rate
    wheel_radius = .07/2
    speed_left = body_rotation_rate * (p - c)
    speed_left = speed_left/wheel_radius
    speed_right = body_rotation_rate * (p + c)
    speed_right = speed_right/wheel_radius

    print("b)")
    # print(body_rotation_rate*radius)
    # v_L_temp = (body_rotation_rate * 2) -
    print 'Speed Left:', speed_left
    print 'Speed Right:', speed_right


def q2():
    print("2)")
    print("a)")
    # p = L * tan(phi)
    # arctan(L/p) = phi
    # where phi is the angle of the steering wheel
    steering_angle = math.atan(.983/15)
    steering_angle_degrees = steering_angle * (180/numpy.pi)
    print(steering_angle_degrees)

    print("b)")
    # wrad = v/r
    angular_rotation_bike = 5.5 / 15
    print angular_rotation_bike


def q3():
    print "3)"
    print "a)"
    # vb = R(Psi)L/2 + R(Psi)R/2
    psi_l = 7.7
    psi_r = 8.3
    r_psi_l = psi_l * .035
    r_psi_r = psi_r * .035
    velocity_body = (r_psi_l/2) + (r_psi_r/2)
    print velocity_body

    print "b)"
    # body_rotation_rate = wz
    # vb = wz(r_psi_l/wz + c)
    # ISOLATE WZ: wz = (vb-r_psi_l)/c
    c = .230/2
    wz = (velocity_body - r_psi_l)/c
    print wz

    print "c)"
    # wzp = vb
    # p = vb/wz
    p = velocity_body/wz
    print p


def q6():
    print "b)"
    # curvature k = 2(delta_y)/(look_ahead_distance or LD)^2
    delta_y = .07/2
    LD = 1.5
    # k = (2*delta_y)/math.pow(LD, 2)
    k = 1 * .5
    print k
    # wz = k * vb
    # TRANSFORM: wz/k = vb

    # temp = (wz)/.278
    # print temp
    #wz = numpy.pi/4
    #vb = wz/k
    # print vb

    # try this
    radius = math.pow(2.75, 2) + math.pow(.75, 2)
    radius = math.sqrt(radius)
    print 1/radius
    #delta_y_try_2 = (math.pow(LD, 2))/(2*2)
    # print delta_y_try_2


if __name__ == '__main__':
    # q1()
    # q2()
    # q3()
    # q4 on sheet
    # q5 on sheet
    q6()
    print "hello world"





