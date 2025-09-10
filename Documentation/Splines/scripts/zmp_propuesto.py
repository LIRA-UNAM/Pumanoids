# -*- coding: utf-8 -*-
"""
Created on Sat Oct 14 11:48:40 2017

@author: Allen
"""

import numpy as np
import matplotlib.pyplot as plt
    

"""Periodo de soporte simple ss=140*dT=0.7, periodo de soporte 
doble=20*dT=0.1, periodo total=0.8s*)"""

    
"""off-set necesario en el ZMP en X para que el bipedo avance de \
acuerdo a la posición erecta: 0.0385
sí queremos que el COG se mantenga en el centro de la plana de los \
pies el off-set debe ser cero"""
off = 0.0385
esc = 1/10;
num_puntos = 320#(* número de puntos conocidos del ZMP para la ventana del control preventivo *)
no = 1080# número de puntos requeridos para la trayectoria de la cadera *)
g = 9.8# gravedad 
zh = 0.22#altura del centro de masa
dT = 0.005#tiempo de muestreo


lp = 0.3# longitud de paso 5.5 centimetros
amp = 0.05954#(*5.954 centimetros de distancia horizontal entre el COG del robot y el ZMP en la planta de los pies del robot*)
hp = 0.05#altura \del pie 3 cm *)

t0 = 160
z0 = 0
Y0 = 0
t1 = 195
Z1 = 0.6667*hp
Y1 = 0.25*lp
t2 = 230
Z2 = hp
Y2 = 0.5*lp
t3 = 265;  
z3 = 0.6667*hp
y3 = 0.75*lp
t4 = 300
z4 = 0
y4 = lp

zmp_y = np.zeros(1500)
zmp_x = np.zeros(1500)
#print(zmp_y)

for k in range(0,1500):
    #Trayectoria del ZMP en Y*)
    if(k >= 0 and k < 300):
        zmp_x[k]= 0 + off
        
    if(k >= 300 and k < 320):
        zmp_x[k]= k*lp/20 + (-300*lp/20) + off
        
    if(k >= 320 and k < 460):
        zmp_x[k]= lp + off
        
    if(k >= 460 and k < 480):
        zmp_x[k]= (k*lp/20) + (lp - (460*lp/20)) + off
        
    if(k >= 480 and k < 620):
        zmp_x[k]= 2*lp + off
        
    if(k >= 620 and k < 640):
        zmp_x[k]= (k*lp/20) + (2*lp - (620*lp/20)) + off
        
    if(k >= 640 and k < 780):
        zmp_x[k]= 3*lp + off
        
    if(k >= 780 and k < 800):
        zmp_x[k]= (k*lp/20) + (3*lp - (780*lp/20)) + off
        
    if(k >= 800 and k < 1500):
        zmp_x[k]= 4*lp + off
        
for k in range(0,1500):
    #Trayectoria del ZMP en Y*)
    if(k >= 0 and k < 140):
        zmp_y[k]= 0
        
    if(k >= 140 and k < 160):
        zmp_y[k]= -(k*(amp/20) + (-140*amp/20))
        
    if(k >= 160 and k < 300):
        zmp_y[k]= -amp
        
    if(k >= 300 and k < 320):
        zmp_y[k]= -(-k*(2*amp/20) + (amp + (300*2*amp/20)))
        
    if(k >= 320 and k < 460):
        zmp_y[k]= amp
        
    if(k >= 460 and k < 480):
        zmp_y[k]= -(k*(2*amp/20) + (-amp - (460*2*amp/20)))
        
    if(k >= 480 and k < 620):
        zmp_y[k]= -amp
        
    if(k >= 620 and k < 640):
        zmp_y[k]= -(-k*(2*amp/20) + (amp + (620*2*amp/20)))
        
    if(k >= 640 and k < 780):
        zmp_y[k]= amp
        
    if(k >= 780 and k < 800):
        zmp_y[k]= -(k*(2*amp/20) + (-amp - (780*2*amp/20)))
        
    if(k >= 800 and k < 940):
        zmp_y[k]= -amp
        
    if(k >= 940 and k < 1500):
        zmp_y[k]= 0
        
#print(zmp_y)
#print(zmp_x)

kk = np.arange(0,1500)

fig1 = plt.figure()
ax = fig1.gca()
ax.set_xlabel('cuenta K')
ax.set_ylabel('zmp_x')
plt.title('ZMP X')
plt.plot(kk,zmp_x)
plt.grid()

fig2 = plt.figure()
ax = fig2.gca()
ax.set_xlabel('cuenta K')
ax.set_ylabel('zmp_y')
plt.title('ZMP Y')
plt.plot(kk,zmp_y)
plt.grid()

fig3 = plt.figure()
ax = fig3.gca()
ax.set_xlabel('zmp_x')
ax.set_ylabel('zmp_y')
plt.title('ZMP')
plt.plot(zmp_x,zmp_y)
plt.grid()

plt.show()

 