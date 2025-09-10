# -*- coding: utf-8 -*-
"""
Created on Mon Oct 30 21:16:02 2017

@author: Allen
"""

import numpy as np
from sympy import Symbol, nsolve
import matplotlib.pyplot as plt

coef_a_0_1 = Symbol('a_0_1')
coef_a_1_1 = Symbol('a_1_1')
coef_a_2_1 = Symbol('a_2_1')
coef_a_3_1 = Symbol('a_3_1')

coef_a_0_2 = Symbol('a_0_2')
coef_a_1_2 = Symbol('a_1_2')
coef_a_2_2 = Symbol('a_2_2')
coef_a_3_2 = Symbol('a_3_2')

coef_a_0_3 = Symbol('a_0_3')
coef_a_1_3 = Symbol('a_1_3')
coef_a_2_3 = Symbol('a_2_3')
coef_a_3_3 = Symbol('a_3_3')

coef_a_0_4 = Symbol('a_0_4')
coef_a_1_4 = Symbol('a_1_4')
coef_a_2_4 = Symbol('a_2_4')
coef_a_3_4 = Symbol('a_3_4')


lp = 0.3# longitud de paso 5.5 centimetros
amp = 0.7#0.05954#(*5.954 centimetros de distancia horizontal entre el COG del robot y el ZMP en la planta de los pies del robot*)
hp = 0.05#altura \del pie 3 cm *)

t0 = 100
y0 = 0

t1 = 150
y1 = (1.0/4.0)*lp

t2 = 200
y2 = (1.0/2.0)*lp

t3 = 250
y3 = (3.0/4.0)*lp

t4 = 300
y4 = lp

""" Equations for the splines in Z for both feet """

eq_1 = coef_a_0_1 + coef_a_1_1 * t0 + coef_a_2_1 * (t0**2) + coef_a_3_1 * (t0**3) - y0#f_k(x_k-1) = y_k-1
eq_2 = coef_a_0_2 + coef_a_1_2 * t1 + coef_a_2_2 * (t1**2) + coef_a_3_2 * (t1**3) - y1
eq_3 = coef_a_0_3 + coef_a_1_3 * t2 + coef_a_2_3 * (t2**2) + coef_a_3_3 * (t2**3) - y2
eq_4 = coef_a_0_4 + coef_a_1_4 * t3 + coef_a_2_4 * (t3**2) + coef_a_3_4 * (t3**3) - y3

eq_5 = coef_a_0_1 + coef_a_1_1 * t1 + coef_a_2_1 * (t1**2) + coef_a_3_1 * (t1**3) - y1#f_k(x_k) = y_k
eq_6 = coef_a_0_2 + coef_a_1_2 * t2 + coef_a_2_2 * (t2**2) + coef_a_3_2 * (t2**3) - y2
eq_7 = coef_a_0_3 + coef_a_1_3 * t3 + coef_a_2_3 * (t3**2) + coef_a_3_3 * (t3**3) - y3
eq_8 = coef_a_0_4 + coef_a_1_4 * t4 + coef_a_2_4 * (t4**2) + coef_a_3_4 * (t4**3) - y4

eq_9 = 3*coef_a_3_1*(t1**2) + 2*coef_a_2_1 * t1 + coef_a_1_1 - 3*coef_a_3_2*(t1**2) + 2*coef_a_2_2 * t1 + coef_a_1_2#f'_k(x_k) = f'_k+1(x_k)
eq_10 = 3*coef_a_3_2*(t2**2) + 2*coef_a_2_2 * t2 + coef_a_1_2 - 3*coef_a_3_3*(t2**2) + 2*coef_a_2_3 * t2 + coef_a_1_3
eq_11 = 3*coef_a_3_3*(t3**2) + 2*coef_a_2_3 * t3 + coef_a_1_3 - 3*coef_a_3_4*(t3**2) + 2*coef_a_2_4 * t3 + coef_a_1_4

eq_12 = 6*coef_a_3_1*t1 + 2*coef_a_2_1  - 6*coef_a_3_2*t1 + 2*coef_a_2_2#f''_k(x_k) = f''_k+1(x_k)
eq_13 = 6*coef_a_3_2*t2 + 2*coef_a_2_2  - 6*coef_a_3_3*t2 + 2*coef_a_2_3
eq_14 = 6*coef_a_3_3*t3 + 2*coef_a_2_3  - 6*coef_a_3_4*t3 + 2*coef_a_2_4


""" not a knot""" 
eq_15 = 6*coef_a_3_1 - 6*coef_a_3_2#f1'''(x_1) = f2'''(x_1)
eq_16 = 6*coef_a_3_3 - 6*coef_a_3_4



coef = nsolve((eq_1,eq_2,eq_3,eq_4,eq_5,eq_6,eq_7,eq_8,eq_9,eq_10,eq_11,eq_12,eq_13,eq_14,eq_15,eq_16),
              (coef_a_0_1,coef_a_1_1,coef_a_2_1,coef_a_3_1,
               coef_a_0_2,coef_a_1_2,coef_a_2_2,coef_a_3_2,
               coef_a_0_3,coef_a_1_3,coef_a_2_3,coef_a_3_3,
               coef_a_0_4,coef_a_1_4,coef_a_2_4,coef_a_3_4),
               (0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0))
print(coef)


a_0_1 = coef[0]
a_1_1 = coef[1]
a_2_1 = coef[2]
a_3_1 = coef[3]

a_0_2 = coef[4]
a_1_2 = coef[5]
a_2_2 = coef[6]
a_3_2 = coef[7]

a_0_3 = coef[8]
a_1_3 = coef[9]
a_2_3 = coef[10]
a_3_3 = coef[11]

a_0_4 = coef[12]
a_1_4 = coef[13]
a_2_4 = coef[14]
a_3_4 = coef[15]


""" FOOT RIGHT Z"""
foot_pos_x_right = np.zeros(1500)

for k in range(0,1500):
    #Trayectoria del ZMP en Y*)
    if(k >= 0 and k < 100):
        foot_pos_x_right[k]= 0
        
    if(k >= 100 and k < 150):#f1
        foot_pos_x_right[k]= a_0_1 + a_1_1 * k + a_2_1 * (k**2) + a_3_1 * (k**3)
#        if foot_pos_x_right[k] < 0:
#            foot_pos_x_right[k] = 0
        
    if(k >= 150 and k < 200):
        foot_pos_x_right[k]= a_0_2 + a_1_2 * k + a_2_2 * (k**2) + a_3_2 * (k**3)
        
    if(k >= 200 and k < 250):
        foot_pos_x_right[k]= a_0_3 + a_1_3 * k + a_2_3 * (k**2) + a_3_3 * (k**3)
        
    if(k >= 250 and k < 300):
        foot_pos_x_right[k]= a_0_4 + a_1_4 * k + a_2_4 * (k**2) + a_3_4 * (k**3)
        
    if(k >= 300 and k < 550):
        foot_pos_x_right[k]= lp
        
    if(k >= 550 and k < 750):
        foot_pos_x_right[k]= 2*foot_pos_x_right[k-450] + lp
        
#    if(k >= 600 and k < 650):
#        foot_pos_x_right[k]= a_0_2 + a_1_2 * (k-450) + a_2_2 * ((k-450)**2) + a_3_2 * ((k-450)**3)
#        
#    if(k >=650 and k < 700):
#        foot_pos_x_right[k]= a_0_3 + a_1_3 * (k-450) + a_2_3 * ((k-450)**2) + a_3_3 * ((k-450)**3)
#        
#    if(k >= 700 and k < 750):
#        foot_pos_x_right[k]= a_0_4 + a_1_4 * (k-450) + a_2_4 * ((k-450)**2) + a_3_4 * ((k-450)**3)
        
    if(k >= 750 and k < 1000):
        foot_pos_x_right[k]= 3*lp
        
    if(k >= 1000 and k < 1200):
        foot_pos_x_right[k]= foot_pos_x_right[k-900] + 3*lp
        
    if(k >= 1200 and k < 1500):
        foot_pos_x_right[k]= 4*lp
#        
#    if(k >=1100 and k < 1150):
#        foot_pos_x_right[k]= a_0_3 + a_1_3 * (k-900) + a_2_3 * ((k-900)**2) + a_3_3 * ((k-900)**3)
#        
#    if(k >= 1150 and k < 1200):
#        foot_pos_x_right[k]= a_0_4 + a_1_4 * (k-900) + a_2_4 * ((k-900)**2) + a_3_4 * ((k-900)**3)
        
""" FOOT LEFT Z"""
foot_pos_x_left = np.zeros(1500)
for k in range(0,1500):
    #Trayectoria del ZMP en Y*)
    if(k >= 0 and k < 300):
        foot_pos_x_left[k]= 0
        
    if(k >= 300 and k < 350):#f1
        foot_pos_x_left[k]= 2*(a_0_1 + a_1_1 * (k-200) + a_2_1 * ((k-200)**2) + a_3_1 * ((k-200)**3))
#        if foot_pos_x_left[k] < 0:
#            foot_pos_x_left[k] = 0
        
    if(k >= 350 and k < 400):
        foot_pos_x_left[k]= 2*(a_0_2 + a_1_2 * (k-200) + a_2_2 * ((k-200)**2) + a_3_2 * ((k-200)**3))
        
    if(k >= 400 and k < 450):
        foot_pos_x_left[k]= 2*(a_0_3 + a_1_3 * (k-200) + a_2_3 * ((k-200)**2) + a_3_3 * ((k-200)**3))
        
    if(k >= 450 and k < 500):
        foot_pos_x_left[k]= 2*(a_0_4 + a_1_4 * (k-200) + a_2_4 * ((k-200)**2) + a_3_4 * ((k-200)**3))
        
    if(k >= 500 and k < 775):
        foot_pos_x_left[k]= 2*lp
        
    if(k >= 775 and k < 975):
        foot_pos_x_left[k]= foot_pos_x_left[k-475] + 2*lp
        
#    if(k >= 825 and k < 875):
#        foot_pos_x_left[k]= a_0_2 + a_1_2 * (k-675) + a_2_2 * ((k-675)**2) + a_3_2 * ((k-675)**3)
#        
#    if(k >=875 and k < 925):
#        foot_pos_x_left[k]= a_0_3 + a_1_3 * (k-675) + a_2_3 * ((k-675)**2) + a_3_3 * ((k-675)**3)
#        
#    if(k >= 925 and k < 975):
#        foot_pos_x_left[k]= a_0_4 + a_1_4 * (k-675) + a_2_4 * ((k-675)**2) + a_3_4 * ((k-675)**3)
        
    if(k >= 975 and k < 1500):
        foot_pos_x_left[k]= 4*lp

kk = np.arange(0,1500)

fig1 = plt.figure()
ax = fig1.gca()
ax.set_xlabel('cuenta K')
ax.set_ylabel('spline z')
plt.title('Splines pie derecho y pie izquierdo')
plt.plot(kk,foot_pos_x_right)
plt.plot([100,150,200,250,300],[y0,y1,y2,y3,y4],'*')
plt.plot(kk,foot_pos_x_left)
plt.grid()

plt.show()


