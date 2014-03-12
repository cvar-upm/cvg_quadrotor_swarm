% clear all
% close all
% clc

s = tf('s');

% Yaw:
% ----

% LTI model
% Input:  dYaw/dt command
% Output: dYaw/dt

Kp = 0.92985;
Tw = 0.11762;
Zeta = 0.754;  
G_dY = Kp/(1+2*Zeta*Tw*s+(Tw*s)^2);


% Altitude:
% ---------

% LTI model

Kp = 0.97201;
Tw = 0.26173;
Zeta = 0.44037;
G_dZ = Kp/(1+2*Zeta*Tw*s+(Tw*s)^2);


% Pitch:
% ------

% LTI models
Kp = 2.3663;
Tp1 = 0.040639;
G_pr2p = Kp/(1+Tp1*s);
Tp1_p = Tp1;

Kp = 1.0644;
Tw = 0.65756;
Zeta = 0.99174;
G_p2vx = Kp/(1+2*Zeta*Tw*s+(Tw*s)^2);
% Gn_p2vx = 2.4617*s/((s+3.065)*(s-0.04859));

% Non-linear function that relates pitch with vx in steady-state
% vx = -2.625*sin(0.2703*theta), donde theta = pitch pertenece al intervalo +/-5º
% fuera del intervalo hay que saturar la salida a vx(theta = +5º ó -5º)

% Roll:
% -----

% LTI models
Kp = 2.5185;
Tp1 = 0.065288;
G_rr2r = Kp/(1+Tp1*s);
Tp1_r = Tp1;

Kp = 0.89738;
Tw = 3.0132;
Zeta = 1.3107;
Tz = 7.6102;
G_r2vy = Kp*(1+Tz*s)/(1+2*Zeta*Tw*s+(Tw*s)^2);                                            
% step(G_r2vy)

% Non-linear function that relates roll with vy in steady-state
% vy =  2.172*sin(0.2961*phi), donde phi = roll pertenece al intervalo +/-5º
% fuera del intervalo hay que saturar la salida a vy(phi = +5º ó -5º)

clear Zeta Tp1 Tw Tz Kp

% %%
% 
% My = G_r2vy;
% Gy = (1-My*s)/My
% figure
% pzplot(Gy)
% 
% Mx = G_p2vx;
% Gx = (1-Mx*s)/Mx
% figure
% pzplot(Gx)