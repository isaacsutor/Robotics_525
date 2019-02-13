# CPSC 472 / PCSE 572 Introduction to Robotics
# Simple control demo
#
# David Conner (david.conner@cnu.edu)
#

import numpy as np
from mtr_simulation import *

# edit this to add disturbance at given time (otherwise uses constant disturbance torque)
def getDisturbance(disturbance, t):
	dist2 = 1.0*disturbance;
	# Add in a disturbance at some point in time
	#if ((t > 5.0) and (t < 6.0)):
	#	#print "Disturbance in the force at t=",t
	#	dist2 += 5.0*disturbance;

	return dist2


# Model parameters
R=1.8; # omhs
L=0.00094; # henries
J=0.00000162*2000; # Jmtr in kg m^2  x 1000 for gearing and load (SWAG)
K=0.014;   #  V s/rad = Nm/A
G=45.6;	   #  1
f=0.00134; # Nm s/rad x100 for gearing

disturbance = np.array([[0],[-K*G/J],[0]]) # 1 Amp worth of disturbance torque
print "disturbance acceleration on output= ",disturbance


# vvvvvvvvvvvvvvvvv chose a state selection for the control process variable vvvvv
# setpoint is SP_bias + SP_amplitude*sin(t*SP_period)
#    set amplitude to 0 for constant setpoint at SP_bias
#
#
# motor angle in degrees
#X_selection  = np.array([180.0/np.pi,0.0,0.0]) # what are we attempting to control (non-zero)
#SP_bias      = 90   # setpoint for angle in degrees (with unit conversion)
#				# include any desired unit conversion here
#SP_amplitude = 0.0; # not time varying
#SP_period	 = np.pi; # pi means 1 cycle every 2 seconds

# motor speed in RPM
X_selection  = np.array([0.0, 60/(2*np.pi), 0.0]); # selection of angular rotation rate (with rpm conversion)
SP_bias      = 8*60/(2*np.pi) # setpoint for speed (with unit conversion)
SP_amplitude = 0.0*60/(2*np.pi); # not time varying
SP_period	  = np.pi; # pi means 1 cycle every 2 seconds
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

# Controller parameters
V_bias    = 0.0 	  # Used for constant controller output
k_p 	  = 0.0		  # Will depend on the variable we are controlling
T_u		  = 0.0		  # oscillation period for tuning
T_i		  = 0.0*T_u/2.0;  # allow tuning the time constants, but basically a function of T_u
T_d		  = 0.0*T_u/8.0;
           # ^ set K_i and K_d to zero for initial testing of K_p (has significant steady state error)

#controller sample time
dt        =  0.01;  # sample time for controller
Time      =  1.0;  # Final time for simulation (Adjust this to see detailed start or longer for tracking)
steps     = 50;     # integration steps for simulation between controller samples

# vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
# ------ Should not need to change below this line ------------

k_i = 0.0
if (T_i > 0.0):
	k_i=k_p/(2.0*T_i)

k_d = k_p*T_d;

# Motor parameters based on 15.2V rating, assume supply input of 18V
Vmax = 17.9
print "Vmax=",Vmax

V 	 = Vmax
print "V_bias=",V_bias
print "k_p=",k_p
print "k_i=",k_i,"    T_i=",T_i
print "k_d=",k_d,"    T_d=",T_d

# Set up the motor/load model

A=np.array([[0, 1, 0],[0, -f/J, K*G/J],[0, -K*G/L, -R/L]]);
print "A=",A

B=np.array([[0],[0],[1/L]]);
print "B=",B

X=np.array([[0],[0],[0]]);
print "Initial state=",X


# Text used in plot and file names
X_selection_str = str(int(X_selection[0]))+"_"+str(int(X_selection[1]))+"_"+str(int(X_selection[2]))
Vtitle="P_desired="+str(SP_bias)+"_X_select_"+X_selection_str+"  Kp="+str(k_p)+ " Ki="+str(k_i)+ "  Kd="+str(k_d)
Vname="P_desired_"+str(SP_bias)+"_X_select_"+X_selection_str+"_kp_"+str(int(10*k_p))+"_ki_"+str(int(10000*k_i))+"_kd_"+str(int(10000*k_d))+"_torq"

# Initialize prior controller data
err0 = 0.0
V_int = 0.0

# Create time vector for plots
# Initialize data vectors of same size (use 0.0* to generate new reference)
tv   = np.arange(0,Time,dt); # vector of time samples
SPv  = 0.0*tv; # setpoint variabl
Pv   = 0.0*tv; # process variable
Tv   = 0.0*tv; # Theta position
wv   = 0.0*tv; # rotational velocity
iv   = 0.0*tv; # currrent
vv   = 0.0*tv; # control voltage
ev   = 0.0*tv; # error term
intv = 0.0*tv; # integral term

for i,t in enumerate(tv):

	# Get the reference and disturbance
	SP = getSetpoint(t, SP_bias, SP_amplitude, SP_period)

	dist2 = getDisturbance(disturbance,t); # edit method above to modify disturbance

	# Calculate control input based on sampled state, prior error, gains, time step, and max voltage
	(V,V_int,err0) =control(X,SP,X_selection,err0,V_int,V_bias, k_p,k_i,k_d,dt,Vmax,i)

	# Store the sampled data used for controls so we can plot later
	Pv[i]   = np.dot(X_selection,X)[0] # Store the process variable
	SPv[i]  = SP;
	Tv[i]   = X[0][0]; # get scalar parts of the numpy array
	wv[i]   = X[1][0];
	iv[i]   = X[2][0];
	vv[i]   = V
	ev[i] 	= err0
	intv[i] = V_int; # control integral

	# Integrate the physical model of plant at smaller time step
	# assuming piecewise constant control input from the sampling
	(X,dx) = integrate(A,B,X,V,dist2,dt,50)


print "Steady State"
print "     X = ",X
print "     dx= ",dx
print "     V = ",V
print " V_int = ",V_int
print "     e = ",err0


Sm= G*X[1][0]
print "mtr speed=",Sm," rad/s = ",Sm*60/(2*np.pi)," rpm"
print "mtr torque=",K*X[2][0]
print "mtr back emf=",K*Sm
print "Vmax-Vb=",Vmax-K*Sm
print " imax=",(Vmax-K*Sm)/R
print "V_int=",V_int
print "V=",V
print "output torque=",K*G*X[2][0]

# Plot the simulation data
mtr_simulation_plots(tv,SPv,Pv,Tv,wv,iv,vv,ev,intv,Vtitle,Vname,False)
