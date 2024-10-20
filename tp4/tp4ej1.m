%% Trabajo Práctico N°4
clc; close all; clear;
s = tf('s');

%% Problema 1
% Planta
Ve = 250;
Kh = 350/250;
Gp = Kh / (400*s + 1);
% Sensor
Ks = 3.5/350;
% Controlador
Kpi = 0.4;
Zpi = 0.005399355;
Gc = Kpi * ((s + Zpi)/s);

Glanc = Ve * Gp * Ks;
Glcnc = feedback(Ve * Gp, Ks);
Glac = Ve * Gc * Gp * Ks;
Glcc  = feedback(Ve * Gc * Gp, Ks);
U = feedback(Ve * Gc, Gp * Ks);
step(3.2 * Glcnc, 3.2 * Glcc);
legend("No Controlada", "Controlada", "Location", "best")

%% Cálculo de la frecuencia de muestreo
[wn, zeta] = damp(Glcc);
wd = wn(1) * sqrt(1 - zeta(1));
Td = 2 * pi / wd;
Td1 = ceil(Td/10);
Td2 = ceil(Td/50);

%% Aproximaciones
% 10 Muestras
z1 = tf('z', Td1);
Gp_z1 = c2d(Gp, Td1, 'zoh');
Gtrdir_z1 = Ve * Gp_z1;
Glanc_z1 = Gtrdir_z1 * Ks;
step(Glanc, Glanc_z1);

Gc_z1_bw = c2d_euler(Gc, Td1, 'backward', 'zpk')
Glcc_z1_bw = feedback(Gc_z1_bw * Gtrdir_z1, Ks);

Gc_z1_fw = c2d_euler(Gc, Td1, 'forward', 'zpk')
Glcc_z1_fw = feedback(Gc_z1_fw * Gtrdir_z1, Ks);

Gc_z1_tu = zpk(c2d(Gc, Td1, 'tustin'))
Glcc_z1_tu = feedback(Gc_z1_tu * Gtrdir_z1, Ks);

Gc_z1_zo = zpk(c2d(Gc, Td1, 'zoh'))
Glcc_z1_zo = feedback(Gc_z1_zo * Gtrdir_z1, Ks);

% 50 Muestras
z2 = tf('z', Td2);
Gp_z2 = c2d(Gp, Td2, 'zoh');
Gtrdir_z2 = Ve * Gp_z2;
Glanc_z2 = Gtrdir_z2 * Ks;
figure; step(Glanc, Glanc_z2);

Gc_z2_bw = c2d_euler(Gc, Td2, 'backward', 'zpk')
Glcc_z2_bw = feedback(Gc_z2_bw * Gtrdir_z2, Ks);

Gc_z2_fw = c2d_euler(Gc, Td2, 'forward', 'zpk')
Glcc_z2_fw = feedback(Gc_z2_fw * Gtrdir_z2, Ks);

Gc_z2_tu = zpk(c2d(Gc, Td2, 'tustin'))
Glcc_z2_tu = feedback(Gc_z2_tu * Gtrdir_z2, Ks);

Gc_z2_zo = zpk(c2d(Gc, Td2, 'zoh'))
Glcc_z2_zo = feedback(Gc_z2_zo * Gtrdir_z2, Ks);

%% Respuestas al escalón: Temperatura
step(Glcc, Glcc_z1_bw, Glcc_z1_fw, Glcc_z1_tu, Glcc_z1_zo)
legend("Continuo", "Backward", "Forward", "Tustin", "ZOH")

figure;
step(Glcc, Glcc_z2_bw, Glcc_z2_fw, Glcc_z2_tu, Glcc_z2_zo)
legend("Continuo", "Backward", "Forward", "Tustin", "ZOH")

%% Respuestas al escalón: Acción de Control
U_z1_bw = feedback(Ve * Gc_z1_bw, Gp_z1 * Ks);
U_z1_fw = feedback(Ve * Gc_z1_fw, Gp_z1 * Ks);
U_z1_tu = feedback(Ve * Gc_z1_tu, Gp_z1 * Ks);
U_z1_zo = feedback(Ve * Gc_z1_zo, Gp_z1 * Ks);
step(U_z1_bw, U_z1_fw, U_z1_tu, U_z1_zo)
legend("Backward", "Forward", "Tustin", "ZOH")

U_z2_bw = feedback(Ve * Gc_z2_bw, Gp_z2 * Ks);
U_z2_fw = feedback(Ve * Gc_z2_fw, Gp_z2 * Ks);
U_z2_tu = feedback(Ve * Gc_z2_tu, Gp_z2 * Ks);
U_z2_zo = feedback(Ve * Gc_z2_zo, Gp_z2 * Ks);
figure; step(U_z2_bw, U_z2_fw, U_z2_tu, U_z2_zo)
legend("Backward", "Forward", "Tustin", "ZOH")

%% Respuestas en frecuencia
Glac_z1_bw = Ve * Gc_z1_bw * Gp_z1 * Ks;
Glac_z1_fw = Ve * Gc_z1_fw * Gp_z1 * Ks;
Glac_z1_tu = Ve * Gc_z1_tu * Gp_z1 * Ks;
Glac_z1_zo = Ve * Gc_z1_zo * Gp_z1 * Ks;
bode(Glac, Glac_z1_bw, Glac_z1_fw, Glac_z1_tu, Glac_z1_zo);
legend("Continuo", "Backward", "Forward", "Tustin", "ZOH")

Glac_z2_bw = Ve * Gc_z2_bw * Gp_z2 * Ks;
Glac_z2_fw = Ve * Gc_z2_fw * Gp_z2 * Ks;
Glac_z2_tu = Ve * Gc_z2_tu * Gp_z2 * Ks;
Glac_z2_zo = Ve * Gc_z2_zo * Gp_z2 * Ks;
figure;
bode(Glac, Glac_z2_bw, Glac_z2_fw, Glac_z2_tu, Glac_z2_zo);
legend("Continuo", "Backward", "Forward", "Tustin", "ZOH")

%% Mapa de Polos y Ceros
pzmap(Gc_z1_bw, Gc_z1_fw, Gc_z1_tu, Gc_z1_zo);
legend("Backward", "Forward", "Tustin", "ZOH")

figure; pzmap(Glcc_z1_bw, Glcc_z1_fw, Glcc_z1_tu, Glcc_z1_zo);
legend("Backward", "Forward", "Tustin", "ZOH")

figure; pzmap(Gc_z2_bw, Gc_z2_fw, Gc_z2_tu, Gc_z2_zo);
legend("Backward", "Forward", "Tustin", "ZOH")

figure; pzmap(Glcc_z2_bw, Glcc_z2_fw, Glcc_z2_tu, Glcc_z2_zo);
legend("Backward", "Forward", "Tustin", "ZOH")

%% Ecuación a diferencias
% Se eligió controlador Backward con un Nd=50

[num, den] = tfdata(Gc_z2_bw);
syms z;
Gc_sym = poly2sym(cell2mat(num),z)/poly2sym(cell2mat(den),z);
Gc_t = iztrans(Gc_sym)