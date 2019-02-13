# CPSC 472 / PCSE 572 Introduction to Robotics
# Simple control demo methods
#
# David Conner (david.conner@cnu.edu)
#

import numpy as np
import matplotlib.pyplot as plt



# calculate the state derivative given plant model, input, and piecewise constant disturbance
def plant(A,B,X, U,dist):

	dx = np.dot(A,X) + np.dot(B,U) + dist;

	return dx

# Do a integration at finer grained dt
def integrate(A,B,X,U,dist,dt,steps=10):
	dt2 = dt/steps
	for i in range(0,steps):
		dx = plant(A,B,X,U,dist) # State derivative
		X = X + dx*dt2
	return (X,dx)

# Time varying setpoint
def getSetpoint(t, bias, amplitude, period):
	return bias + amplitude*np.sin(t*period); # 2 second period


# Calculate the control output including maximum output voltage
# if using integral, we tie back the integral term to reduce overshoot
def control(X,P_desired,X_selection,err0,V_int,V_bias, k_p,k_i,k_d,dt,Vmax,cnt):

	# Calculate error based on selected parameter (includes any unit conversion)
	err =P_desired - np.dot(X_selection,X)[0]

	# Error derivative
	de = (err-err0)/dt
	#print "err=",err,"  de=",de

	# Integral
	if (k_i > 0.0):
		# calculate the integral based on two consecutive error measurements
		#   to reduce the impact of oscillations
		V_int = V_int + k_i*dt*(err+err0)/2.0;
	else:
		# If not using integral, just set this to 0
		V_int = 0.0;

	# Control output
	V = V_bias + k_p*err + k_d*de
	Vc = V + V_int

	# Check for control out of bounds
	if ((Vc > Vmax) or (Vc < -Vmax)):
		print "Vc[",cnt,"]=",Vc," exceeds Vmax=",Vmax, " - clamped!"
		print "    v_bias=",V_bias," kp_e=",k_p*err,"  kd*e=",k_d*de," => V=",V
		if (k_i > 0):
			V_int = np.sign(Vc)*Vmax - V
			if (V_int > 5*Vmax):
				V_int = 5*Vmax
			elif (V_int < -5*Vmax):
				V_int = -5*Vmax

		else:
			V_int = 0.0

		Vc = np.sign(Vc)*Vmax

	# Return the current control, integral term, and error
	return (Vc,V_int,err)

def mtr_simulation_plots(tv,SPv,Pv,Tv,wv,iv,vv,ev,intv,Vtitle,Vname,save=True):


	fig=plt.figure(1)
	fig.set_tight_layout(True)
	plt.subplot(131)
	plt.plot(tv, vv, 'r', linewidth=2.0)
	plt.ylabel('V')
	#plt.axis([0, tv[-1], -0.25*Vmax, Vmax])
	plt.grid()
	plt.xlabel('time')
	plt.subplot(132)
	plt.plot(tv, ev, 'g', linewidth=2.0)
	plt.ylabel('error')
	#plt.axis([0, tv[-1], -.5*P_desired, .5*P_desired])
	plt.grid()
	plt.xlabel('time')
	plt.subplot(133)
	plt.plot(tv, intv, 'b', linewidth=2.0)
	plt.ylabel('Integral')
	plt.xlabel('time')
	#plt.axis([0, tv[-1], -Vmax, Vmax])
	plt.grid()
	fig.canvas.set_window_title(Vtitle+" control")
	if (save):
		fig.savefig(Vname+"_control.png")

	fig=plt.figure(2)
	fig.set_tight_layout(True)
	plt.subplot(121)
	plt.plot(tv, Tv, 'r', linewidth=2.0)
	plt.ylabel('radians')
	#plt.axis([0, tv[-1], 0, np.pi])
	plt.grid()
	plt.xlabel('time')
	plt.subplot(122)
	plt.plot(tv, wv, 'g', linewidth=2.0)
	plt.ylabel('rad/s')
	#plt.axis([0, tv[-1], -5, 15])
	plt.grid()
	plt.xlabel('time')
	fig.canvas.set_window_title(Vtitle+" angles")
	if (save):
		fig.savefig(Vname+"_angles.png")

	fig=plt.figure(3)
	fig.set_tight_layout(True)
	plt.subplot(121)
	plt.plot(tv, vv, 'r', linewidth=2.0)
	plt.ylabel('V')
	#plt.axis([0, tv[-1], -0.25*Vmax, Vmax])
	plt.grid()
	plt.xlabel('time')
	plt.subplot(122)
	plt.grid()
	plt.plot(tv, iv, 'g', linewidth=2.0)
	plt.ylabel('Amps')
	plt.xlabel('time')
	#plt.axis([0, tv[-1], -10, 10])
	fig.canvas.set_window_title(Vtitle+" electrical")
	if (save):
		fig.savefig(Vname+"_elec.png")


	fig = plt.figure(4)
	fig.set_tight_layout(True)
	plt.subplot(121)
	plt.plot(tv,SPv,'b',tv, Pv, 'r',linewidth=2.0)
	plt.ylabel('process')
	#plt.axis([0, tv[-1], 0, 1.2*P_desired])
	plt.grid()
	plt.xlabel('time')
	plt.subplot(122)
	plt.plot(tv, ev, 'g', linewidth=2.0)
	plt.ylabel('error')
	#plt.axis([0, tv[-1], -.5*P_desired, .5*P_desired])
	plt.grid()
	plt.xlabel('time')
	fig.canvas.set_window_title(Vtitle+" process")
	if (save):
		fig.savefig(Vname+"_process.png")


	plt.show()
