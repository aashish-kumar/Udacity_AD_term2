# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Reflections

### P Controller 
This controller tries to minimize the error by applying a feedback proportional to the error. This will quickly achieve the target but will overshoot. This controller reaches the target fast but then oscillates around the target.

### D Controller
This controller reduces the oscillations around the target value. It is based on differential in the error. It reduces the oscillations and helps in convergence.

### I Controller
This controller reduces the long term errors or issues with the system like a mis-aligned steering wheel.

## PID Fine Tuning
I tried three different approaches for PID fine tuning.
### 1) Manual Tuning:
I tuned the three parameters in order of P -> D -> I. As I  tried increasing speed I realized that things which work for slow speed do not extend to higher speed. A key part is the steer. A low speed steer effect is much different high speed steer. So I modulated my controls for different speeds. This helped a lot in running it upto ~50mph.
### 2)  Zeigler Nicols Method:
This method was faster to get the parameters but my manual tuned parameters outperformed it.
<https://en.wikipedia.org/wiki/PID_controller#Ziegler%E2%80%93Nichols_method>
### 3) Stochastic Gradient Descent(SGD) on simulator:
Initially I achieved stability using manual tuning(so car doesn't steer away from road). Then I implemented a SGD for accumulated errors on simulator. I kept the simulator running and parameters were automatically tuned by accumulated errors on time. I also kept a reset code which brings car to a standstill in center of road before each gradient update step. Also I made sure each update step is atleast one round of the track. I achieved this by approximating the time required. This method was interesting to try but I was not able to get it working for higher speed. I think this is either due to the random behavior in the track or the presence of lot of local minima. 

PID controller is bad at system with delayed response as the feedback is based on current error. Also it needs to be modelled for different speed as the system response are not linear in nature.
