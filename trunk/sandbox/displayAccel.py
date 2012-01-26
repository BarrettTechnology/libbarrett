from libbtpy import *
from pylab import *

CONTROL_LOOP_RATE = 500.0  # Hz
DURATION = 3000000  # microseconds
FILE_NAME = "accel.csv"

pm = ProductManager()
pm.getExecutionManager(1.0/CONTROL_LOOP_RATE)  # set loop rate
#pm.waitForWam()
pm.wakeAllPucks()
#if pm.foundWam4():
#	wam = pm.getWam4()
#elif pm.foundWam7():
#	wam = pm.getWam7()
#else:
#	print "Didn't find a WAM!"
#	exit()
#wam.gravityCompensate()


ion()
figure(1);
figure(2);
#figure(3);


while(1):
#while isWamActivated(pm):
	takeAccelSample(pm, DURATION, FILE_NAME)
	d = loadtxt(FILE_NAME, delimiter=',')

	figure(1);
	clf()
	plot(d[:,0], d[:,1:])

	#figure(2);
	#clf()
	#psd(d[:,1], Fs = CONTROL_LOOP_RATE, scale_by_freq = False)

	figure(2);
	clf()
	psd((d[:,1]**2 + d[:,2]**2 + d[:,3]**2)**0.5, Fs = CONTROL_LOOP_RATE, scale_by_freq = False)

	draw()

