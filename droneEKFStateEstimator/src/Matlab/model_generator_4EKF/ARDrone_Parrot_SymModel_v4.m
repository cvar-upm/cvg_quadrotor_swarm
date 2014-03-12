% Modelo simplificado del AR Drone en simbólico

clear all
close all
clc

%% Definición de variables de estado, entradas y salidas

% Name of State Space Variables
State_Names = {'Paux','Raux','dYaux','dYp','Y','dZaux','dZp','Z','x','y','vx','vy'};

varlist = ' ';
vectvarlist = '[';
for i=1:length(State_Names)
    State_Names{i} = strcat(State_Names{i},'_k');
    varlist     = horzcat(varlist,' ',State_Names{i});
    vectvarlist = horzcat(vectvarlist,' ',State_Names{i},',');
end
disp(' ');disp(' ')

naux = length(vectvarlist);
vectvarlist(naux) = ' ';
vectvarlist = strcat(vectvarlist,']');
clear naux i;
eval(horzcat('syms ',varlist,' real;'));
eval(horzcat('Xestate = ',vectvarlist,';'));
Xestate = Xestate';

varlist = ' ';
vectvarlist = '[';
State_Namesk1 = {};
for i=1:length(State_Names)
    State_Namesk1{i} = strcat(State_Names{i},'1');
    varlist     = horzcat(varlist,' ',State_Namesk1{i});
    vectvarlist = horzcat(vectvarlist,' ',State_Namesk1{i},',');
end
disp(' ');disp(' ')

naux = length(vectvarlist);
vectvarlist(naux) = ' ';
vectvarlist = strcat(vectvarlist,']');
clear naux i;
eval(horzcat('syms ',varlist,' real;'));
eval(horzcat('Xestatek1 = ',vectvarlist,';'));
Xestatek1 = Xestatek1';

Input_Names = {'P','R','dY','dZ'};

varlist = ' ';
vectvarlist = '[';
for i=1:length(Input_Names)
    Input_Names{i} = strcat(Input_Names{i},'_ref');
    varlist     = horzcat(varlist,' ',Input_Names{i});
    vectvarlist = horzcat(vectvarlist,' ',Input_Names{i},',');
end
disp(' ');
naux = length(vectvarlist);
vectvarlist(naux) = ' ';
vectvarlist = strcat(vectvarlist,']');
clear naux i;
eval(horzcat('syms ',varlist,' real;'));
eval(horzcat('Inputs = ',vectvarlist,';'));
Inputs = Inputs';

Output_Names = {'Po','Ro','dYo','Yo','dZo','Zo','xo','yo','vxo','vyo','vxmo','vymo'};

varlist = ' ';
vectvarlist = '[';
for i=1:length(Output_Names)
    Output_Names{i} = strcat(Output_Names{i},'_k');
    varlist     = horzcat(varlist,' ',Output_Names{i});
    vectvarlist = horzcat(vectvarlist,' ',Output_Names{i},',');
end
disp(' ');
naux = length(vectvarlist);
vectvarlist(naux) = ' ';
vectvarlist = strcat(vectvarlist,']');
clear naux i;
eval(horzcat('syms ',varlist,' real;'));
eval(horzcat('Outputs = ',vectvarlist,';'));
Outputs = Outputs';

%% Defining state space model

Parrot_Model_Parameters
% Pelican_Model_Parameters

% Linear part of the model:
% -------------------------

% modelo en altura, Z
[A_dZ,B_dZ,C_dZ,D_dZ] = tf2ss(-G_dZ.num{1},G_dZ.den{1});
A_Z  = horzcat(vertcat(A_dZ,C_dZ),[0;0;0]);
B_Z  = vertcat(B_dZ, 0);
C_Z  = vertcat(horzcat(C_dZ,0), horzcat(C_dZ*0,1));
D_Z  = vertcat(D_dZ, 0);

% modelo en yaw, Y
[A_dY,B_dY,C_dY,D_dY] = tf2ss(G_dY.num{1},G_dY.den{1});
A_Y = horzcat(vertcat(A_dY,C_dY),[0;0;0]);
B_Y = vertcat(B_dY, 0);
C_Y = vertcat(horzcat(C_dY,0), horzcat(C_dY*0,1));
D_Y = vertcat(D_dY, 0);

% modelo en pitch
% Opcional, poner ganancia a 1
% G_pr2p.num{1} = 1;
% G_pr2p.num{1} = G_pr2p.num{1} * 10*pi/180;
[A_P,B_P,C_P,D_P] = tf2ss(G_pr2p.num{1},G_pr2p.den{1});

% modelo en roll
% Opcional, poner ganancia a 1
% G_rr2r.num{1} = 1;
% G_rr2r.num{1} = G_rr2r.num{1} * 10*pi/180;
[A_R,B_R,C_R,D_R] = tf2ss(G_rr2r.num{1},G_rr2r.den{1});

% modelo completo - Parte lineal
Acomp = blkdiag(A_P,A_R,A_Y,A_Z);
Bcomp = blkdiag(B_P,B_R,B_Y,B_Z);
Ccomp = blkdiag(C_P,C_R,C_Y,C_Z);
Dcomp = blkdiag(D_P,D_R,D_Y,D_Z);

% Non-linear part of the model:
% -----------------------------

% Angulos de euler
syms g Y P R real;

R_Y = [cos(Y)  -sin(Y);
       sin(Y)   cos(Y)]; 
Axym = g*[-sin(P);sin(R)];
% % Alternativamente:
% axm =  -g*cos(roll)*sin(pitch);
% aym =   g*sin(roll);
Axy = R_Y*Axym;

% Rozamiento aerodinamico. Parametros ki, ci
syms ki ci vx vy vwx vwy real;
Axy_roz = ki*(sqrt(vx^2+vy^2)+ci)*[vx;vy];
% Alternativamente para añadir el viento
% Axy_roz = ki*(sqrt((vx - vwx)^2+(vy - vwy)^2)+ci)*[(vx - vwx);(vy - vwy)];

syms ktr real;
Axy_total = ktr*(Axy-Axy_roz);
% d(vx)dt = Axy_total(1)
% d(vy)dt = Axy_total(1)

syms x y real;
% modelo F(xk,uk)
F = [vx;...
     vy;...
     Axy_total(1);...
     Axy_total(2)];
 
 
% Joining both parts:
% -------------------

% State equation
% Input_gains = diag([10*pi/180, 10*pi/180, 100*pi/180, -1]);
syms gain_pitch gain_roll gain_dyaw gain_dz;
Input_gains = diag([gain_pitch, gain_roll, gain_dyaw, gain_dz]);
State_Model = Acomp*Xestate(1:8) + Bcomp*Input_gains*Inputs;

State_Model(9:12) = F;

syms deltaT real;
syms timeIntegration real;
deltaT = timeIntegration;
State_Model = Xestate + eye(12)*deltaT*State_Model;


% Observation equation
Cmeas = blkdiag(Ccomp,eye(4));
Output_model = Cmeas*Xestate;
syms vxm_k vym_k real;
V  = [vx; vy];
Vm = [vxm_k; vym_k];
Vm = R_Y'*V;
Output_model(11:12) = Vm;

% Some variables in the State Model have to be recalculated
Y  = Yo_k;
vx = vxo_k;
vy = vyo_k;
P  = Po_k;
R  = Ro_k;

for i=1:length(Outputs)
    eval(horzcat( char(Outputs(i)),' = ', char(Output_model(i)),';' ) );
end

% Show in window both parts of the model
disp(' ');disp(' ');
clc
digits(5);
disp('State Model'); disp(' ');
State_Model = eval( eval( State_Model ) );
for i = 1:length(State_Model)
    disp( horzcat( horzcat( State_Namesk1{i},' = ', char( vpa( eval( State_Model(i) ) ) ) ) ) );
end

disp('Observation Model'); disp(' ');
Output_model = eval( eval( Output_model ) );
for i = 1:length(Output_model)
    disp( horzcat( Output_Names{i},' = ',char( vpa( eval( Output_model(i) ) ) ) ) );
end
digits(40);

Output_model(13) = Output_model(4);

%% Write C++ for the Quadrotor model
clc

% // ************** State **************
% // Xestate order
% //  X1 : proportional to pitch
% //  X2 : proportional to roll
% //  X3 : internal variable related to d(yaw)/dt
% //  X4 : proportional to d(yaw)/dt
% //  X5 : yaw
% //  X6 : internal variable related to d(Z)/dt
% //  X7 : proportional to d(Z)/dt
% //  X8 : Z
% //  X9 : X
% //  X10: Y
% //  X11: VX
% //  X12: VY

disp('// Reading statek values')
disp(' ')

for i=1:length(State_Names)
    disp(horzcat('float ',State_Names{i},' = Statek->getValueData(',num2str(i),');'))
end
disp(' ');disp(' ')

disp('// Reading input values')
disp(' ')

for i=1:length(Input_Names)
    disp(horzcat('float ',Input_Names{i},' = Inputs->getValueData(',num2str(i),');'))
end
disp(' ');disp(' ')

disp('// Reference value gains are already included in the State Model')
disp(' ');disp(' ')

disp('// Definition of some model constants')
disp('float g   = 9.81;')
disp(' ')
disp('// Definition of aerodynamic friction model constants')
% disp('float ki  = 0.0578*2.45;')
% disp('float ci  = 3.0;')
% disp('float ktr = 3.5;')
disp('float ki  = 0.0496*2.45;') % el 2.45 es por la ganancia estatica de rollref2roll, y pitchref2pitch
disp('float ci  = 4.0;')
disp('float ktr = 2.0;')
disp(' ');disp(' ')

% digits(5);
% disp('// Parrot State Model')
% disp(' ');
% State_Model = eval( eval( State_Model ) );
% dStateNames = State_Names;
% for i = 1:length(State_Model)
%     dStateNames{i} = horzcat( 'd', State_Names{i} );
%     aux = horzcat( 'float ',dStateNames{i},' = ', char( vpa( eval( State_Model(i) ) ) ), ';' );
%     aux = regexprep(aux, 'vx_k\^2', 'pow(vx_k,2)');
%     aux = regexprep(aux, 'vy_k\^2', 'pow(vy_k,2)');
%     aux = regexprep(aux, '\(pow\(vx_k,2) \+ pow\(vy_k,2))\^\(1/2)', 'sqrt( pow(vx_k,2) + pow(vy_k,2) )');
%     disp( aux );
% end
% disp(' ');disp(' ')
% 
% disp('// Integracion de las variables de estado')
% disp(' ');
% 
% State_Names_k1 = State_Names;
% for i = 1:length(State_Names)
%     State_Names_k1{i} = horzcat(State_Names{i},'1');
%     disp( horzcat('float ',State_Names_k1{i},' = ',State_Names{i},' + ',dStateNames{i},';') )
% %     disp( horzcat('float ',State_Names_k1{i},' = ',State_Names{i},' + ',dStateNames{i},'*',char(timeIntegration),';') )
% end
% disp(' ');disp(' ')

digits(5);
disp('// Assignment to state in k+1')
disp(' ')

for i=1:length(State_Names)
    aux = horzcat( char( vpa( eval( State_Model(i) ) ) ) );
    aux = regexprep(aux, 'vx_k\^2', 'pow(vx_k,2)');
    aux = regexprep(aux, 'vy_k\^2', 'pow(vy_k,2)');
    aux = regexprep(aux, '\(pow\(vx_k,2) \+ pow\(vy_k,2))\^\(1/2)', 'sqrt( pow(vx_k,2) + pow(vy_k,2) )');
    disp(horzcat('Statek1->setValueData(', aux,',',num2str(i),');'));
end
disp(' ');disp(' ')

%% Write C++ for the Quadrotor Observation model
clc

% //  Medidas: 
% //  roll : C(1,1)*X1
% //  pitch: C(2,2)*X2
% //  dYaw : C(3,4)*X4
% //  Yaw  : X5
% //  dZ   : C(5,7)*X7
% //  Z    : X8
% //  X    : X9
% //  Y    : X10
% //  Vx   : X11
% //  Vy   : X12
% //  [Vxm;: [R_Y R_Y]' * [X11;
% //   Vym]: [R_Y R_Y]     X12]

% Aqui empieza la parte que muestra lineas en C++ por pantalla
disp('// Reading statek values')
disp(' ')

for i=1:length(State_Names)
    disp(horzcat('float ',State_Names{i},' = Statek->getValueData(',num2str(i),');'))
end
disp(' ');disp(' ');

disp('// Filling in Output/measurements vector')
disp(' ')

digits(5);
for i=1:length(Output_model)
    disp( horzcat( 'Output->setValueData(',char( vpa( eval(Output_model(i)) ) ),',',num2str(i),');' ) );
end
digits(40);
disp(' ');disp(' ');

%% Write C++ for the Quadrotor Jacobian State model

clc

disp('// Reading statek values')
disp(' ')

for i=1:length(State_Names)
    disp(horzcat('float ',State_Names{i},' = Statek->getValueData(',num2str(i),');'))
end
disp(' ');disp(' ');

disp('// Definition of some model constants')
disp('float g   = 9.81;')
disp(' ')
disp('// Definition of aerodynamic friction model constants')
% disp('float ki  = 0.0578*2.45;')
% disp('float ci  = 3.0;')
% disp('float ktr = 3.5;')
disp('float ki  = 0.0496*2.45;') % el 2.45 es por la ganancia estatica de rollref2roll, y pitchref2pitch
disp('float ci  = 4.0;')
disp('float ktr = 2.0;')
disp('double eps_jpp = 1e-6;')
disp(' ');disp(' ')

disp('// Filling in Process Jacobian, MatJacFx')
disp(' ')

digits(5);
for i=1:length(State_Model)
    for j=1:length(Xestate)
        comp_ij = diff(eval(eval(State_Model(i))), char( Xestate(j) ));
        if (comp_ij ~= 0)
            bad_expr = (vx_k^2 + vy_k^2)^(1/2);
            [coef term] = coeffs( expand(comp_ij), 1/bad_expr);
            ind_bad = find(term == 1/bad_expr);
            ind_good = find(term ~= 1/bad_expr);
            if not(isempty(ind_bad))
%                 disp( horzcat( 'float ', auxvar_names{i},';' ) );
                aux = char( vpa( eval( bad_expr ) ) );
                aux = regexprep(aux, 'vx_k\^2', 'pow(vx_k,2)');
                aux = regexprep(aux, 'vy_k\^2', 'pow(vy_k,2)');
                aux = regexprep(aux, '\(pow\(vx_k,2) \+ pow\(vy_k,2))\^\(1/2)', 'sqrt( pow(vx_k,2) + pow(vy_k,2) )');
                disp(['if (' aux ' < eps_jpp) {'])
                try
                    aux = char( vpa( eval( coef(ind_good)*term(ind_good)' ) ) );
                catch
                    aux = '0.0f';
                end
                aux = regexprep(aux, 'vx_k\^2', 'pow(vx_k,2)');
                aux = regexprep(aux, 'vy_k\^2', 'pow(vy_k,2)');
                aux = regexprep(aux, '\(pow\(vx_k,2) \+ pow\(vy_k,2))\^\(1/2)', 'sqrt( pow(vx_k,2) + pow(vy_k,2) )');
                disp( horzcat( 'MatJacFx->setValueData(',aux,',',num2str(i),',',num2str(j),');' ) );
                disp('} else {')
                aux = char( vpa( eval( coef*term' ) ) );
                aux = regexprep(aux, 'vx_k\^2', 'pow(vx_k,2)');
                aux = regexprep(aux, 'vy_k\^2', 'pow(vy_k,2)');
                aux = regexprep(aux, '\(pow\(vx_k,2) \+ pow\(vy_k,2))\^\(1/2)', 'sqrt( pow(vx_k,2) + pow(vy_k,2) )');
                disp( horzcat( 'MatJacFx->setValueData(',aux,',',num2str(i),',',num2str(j),');' ) );
                disp('}')
            else
                aux = char( vpa( eval( comp_ij ) ) );
                aux = regexprep(aux, 'vx_k\^2', 'pow(vx_k,2)');
                aux = regexprep(aux, 'vy_k\^2', 'pow(vy_k,2)');
                aux = regexprep(aux, '\(pow\(vx_k,2) \+ pow\(vy_k,2))\^\(1/2)', 'sqrt( pow(vx_k,2) + pow(vy_k,2) )');
                disp( horzcat( 'MatJacFx->setValueData(',aux,',',num2str(i),',',num2str(j),');' ) );
            end 
        end
    end
end
disp(' ');disp(' ');

disp('// Filling in Process Jacobian, MatJacFu')
disp(' ')

for i=1:length(State_Model)
    for j=1:length(Inputs)
        comp_ij = diff(eval(eval(State_Model(i))), char( Inputs(j) ));
        if (comp_ij ~= 0)
            disp( horzcat( 'MatJacFu->setValueData(',char( vpa( eval( comp_ij ) ) ),',',num2str(i),',',num2str(j),');' ) );
        end
    end
end
digits(40);
disp(' ');disp(' ');

%% Write C++ for the Quadrotor Jacobian Observation model

clc

disp('// Reading statek values')
disp(' ')

for i=1:length(State_Names)
    disp(horzcat('float ',State_Names{i},' = Statek->getValueData(',num2str(i),');'))
end
disp(' ');disp(' ');

disp('// Filling in Observation Jacobian')
disp(' ')

digits(5);
for i=1:length(Output_model)
    for j=1:length(Xestate)
        comp_ij = diff(eval(eval(Output_model(i))), char( Xestate(j) ));
        if (comp_ij ~= 0)
            disp( horzcat( 'MatJacHx->setValueData(',char( vpa( eval( comp_ij ) ) ),',',num2str(i),',',num2str(j),');' ) );
        end
    end
end
digits(40);
disp(' ');disp(' ');
