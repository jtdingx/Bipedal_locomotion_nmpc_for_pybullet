%% adaptive foot location + body inclination with time-varying height
function [comx,comy,comz,footx_real_next,footy_real_next,footz_real_next,ZMPx_real,ZMPy_real,comvx,comvy,comvz,thetax,thetay,footx_real,footy_real,footz_real,torquex_real,torquey_real,Nsum,t,Nh,tcpu]=NMPC_2018_3robust3
%%% the term of CoM position error has been replaced with ZMPy Position
%% reference foot location parameter
Tn = 15;

%%% modified the first steps
steplength = 0.3*ones(Tn,1);
steplength(1) = 0;
% %% change velocity: speed up ---> speed down
% steplength(5) = 0.7;
% steplength(6) = 0.7;
% steplength(7) = 0.7;


stepwidth = 0.4*ones(Tn,1);
stepwidth(1) = stepwidth(1)/2;

%%% stage switch
stepheight =  0*zeros(Tn,1);  %% no stage
%%% stage 
% stepheight =  0.1*ones(Tn,1);
% stepheight(4,:) =  0;
% stepheight(5,:) =  0;
% stepheight(6:9,:) =  -0.1*ones(4,1);
% stepheight(9:end,:) = zeros(Tn-8,1);

footx_ref = zeros(Tn,1);
footy_ref = zeros(Tn,1); 
footz_ref = zeros(Tn,1);
for i=2:Tn       %%% singular period ===> right support
    footx_ref(i) = footx_ref(i-1)+steplength(i-1);
    footy_ref(i) = footy_ref(i-1)+(-1)^(i)*stepwidth(i-1);                
    footz_ref(i) = footz_ref(i-1)+stepheight(i-1);
end



Ts = 0.8*ones(Tn,1); Td = 0.0*ones(Tn,1);
Tx = zeros(Tn,1);
for i=2:Tn
    Tx(i) = Tx(i-1)+Ts(i-1)+Td(i-1);
end
dt = 0.05;                     %%%%%time interval also determine the robustness
t = dt:dt:Tx(end);
Nsum = length(t);
nT = round((Ts(1)+Td(1))/dt);
nTs = floor(Ts(1)/dt);
nTd = nT-nTs;


ZMPx_real = zeros(1,Nsum);  ZMPy_real = zeros(1,Nsum); 
comx = zeros(1,Nsum);  comvx = zeros(1,Nsum);     comax = zeros(1,Nsum);
comy = zeros(1,Nsum);  comvy = zeros(1,Nsum);     comay = zeros(1,Nsum);
comz = zeros(1,Nsum);  comvz = zeros(1,Nsum);     comaz = zeros(1,Nsum);
thetax = zeros(1,Nsum);  thetavx = zeros(1,Nsum);     thetaax = zeros(1,Nsum);
thetay = zeros(1,Nsum);  thetavy = zeros(1,Nsum);     thetaay = zeros(1,Nsum);

torquex_real = zeros(Nsum,1);
torquey_real = zeros(Nsum,1);

%% CoM+angular state and control input
xk = zeros(3,Nsum);    x_vacc_k = zeros(1,Nsum);
yk = zeros(3,Nsum);    y_vacc_k = zeros(1,Nsum);
zk = zeros(3,Nsum);    z_vacc_k = zeros(1,Nsum);
thetaxk = zeros(3,Nsum);    thetax_vacc_k = zeros(1,Nsum);
thetayk = zeros(3,Nsum);    thetay_vacc_k = zeros(1,Nsum);


%% pamameters for MPC
hcom = 1.02;
g = 9.8;
Nh = floor(1.501/dt);
A = [1,dt,dt^2/2; 0, 1,dt;0,0,1];
B = [dt^3/6; dt^2/2;dt];
C = [1,0,-hcom/g];
Cp = [1,0,0];  
Cv = [0,1,0]; 
Ca = [0,0,1];


%%  vertical height range _initialize
%%%%fixed height 
% Z_max = 0.03*ones(Nsum,1);
%% time-varying constraints
% Z_max = 0.000*ones(Nsum,1);
% for iiii = 1:Nsum
%     Z_max(iiii,:) = Z_max(iiii,:)-0.015*cos(2*pi/Tx(2)*(iiii-1)*dt);
% end
%% time-varying constraints+stage
% for iiii = 4*nT:7*nT
%     Z_max(iiii,:) = Z_max(iiii,:)-abs(0.015*cos(2*pi/Tx(2)*(iiii-1)*dt));
% end

Z_max = 0.1*ones(Nsum,1);

Z_min = -0.2;


%% footz reference :just the height of footz location
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Zsc = zeros(Nsum,1);
for i = 1:Nsum
    TPX = find(i*dt>=Tx);
    tpx1 = TPX(end); 
    Zsc(i) = footz_ref(tpx1);
    
end

yk(1,:) = footy_ref(1)*ones(1,Nsum);
zk(1,:) = hcom*ones(1,Nsum);

%% predicitve model 
% %%% for CoP
% Pzs = matrix_ps(A,Nh,C);
% Pzu = matrix_pu(A,B,Nh,C);
%%% for comy
Pps = matrix_ps(A,Nh,Cp);
Ppu = matrix_pu(A,B,Nh,Cp);
%% for comvy
Pvs = matrix_ps(A,Nh,Cv);
Pvu = matrix_pu(A,B,Nh,Cv);
%% for comay
Pas = matrix_ps(A,Nh,Ca);
Pau = matrix_pu(A,B,Nh,Ca);

footx_real = zeros(Tn,1);
footx_real_next = zeros(Nsum,1);
footy_real = zeros(Tn,1);
footy_real_next = zeros(Nsum,1);
footz_real = zeros(Nsum,1);
footz_real_next = zeros(Nsum,1);

Footx_max = 0.6;
Footx_min = 0.0;
Footy_max = 0.6;
Footy_min = 0.2;


%% robot parameters
%%% physical
M_f = 100;
rad = 0.4;
J_ini = M_f*rad^2;

%% constraints-boundaries parameters initilize
%%% ZMP BOUNDARIES-UPPER BOUNDARIES AND LOWER BOUNDARIES

%%% based on current support leg
ZMPx_ub = 0.08*ones(Nsum,1);      ZMPx_lb = -0.05*ones(Nsum,1);
% for i =1:Nsum
%     bjx = find(i*dt>=Tx);
%     bjx = bjx(end); 
%     T_ru = i*dt-Tx(bjx);
%     if T_ru <= Td(1)
%         ZMPx_ub(i) = ZMPx_ub(i);
%         ZMPx_lb(i) = ZMPx_lb(i)-Footx_max;                              
%     end
% end           %%%foot_support(dsp) = foot_support(ssp)
ZMPy_ub = 0.055*ones(Nsum,1);      ZMPy_lb = -0.055*ones(Nsum,1);
% for i =1:Nsum
%     bjx = find(i*dt>=Tx);
%     bjx = bjx(end); 
%     T_ru = i*dt-Tx(bjx);
%     if T_ru <= Td(1)
%         if rem(bjx,2)==1
%             ZMPy_ub(i) = ZMPy_ub(i)+stepwidth(bjx);
%             ZMPy_lb(i) = ZMPy_lb(i);           
%         else
%             ZMPy_ub(i) = ZMPy_ub(i);
%             ZMPy_lb(i) = ZMPy_lb(i)-stepwidth(bjx);                 
%         end              
%     end
% end



%%% com-support range
comx_max = 0.06;
comx_min = -0.04;
comy_max = 0.6;
comy_min = 0.02;

%%% angle range
thetax_max = 15*pi/180;
thetax_min = -10*pi/180;

thetay_max = 10*pi/180;
thetay_min = -10*pi/180;


%%% torque range
torquex_max = 160;
torquex_min = -160;

torquey_max = 160;
torquey_min = -160;
torquex_max = torquex_max/J_ini;
torquex_min = torquex_min/J_ini;

torquey_max = torquey_max/J_ini;
torquey_min = torquey_min/J_ini;


%%% swing foot maximal velocity
footx_vmax = 3;
footx_vmin = -1;
footy_vmax = 1;
footy_vmin = -0.5;

%% solution preparation
%%%%%  optimization variables/state solution: be careful that the number of optimal variables is changing:
V_optimal = zeros(5*Nh+3*2,Nsum);
%%% n_vis flag
flag = zeros(1,Nsum);

% % Rx = 1; alphax = 100; beltax = 1000; gamax =10000000; %%%%% balance
% % Ry = 1; alphay = 100; beltay = 1000; gamay =10000000; %%%%% balance#
% Rx = 1; alphax = 10; beltax = 500000; gamax =50000000; %%%%% balance
% Ry = 1; alphay = 10; beltay = 100000; gamay =50000000; %%%%% balance
% Rz = 1; alphaz = 100; beltaz =10000000; gamaz =200; %%%%% balance
% Rthetax = 1; alphathetax = 10; beltathetax = 1000000;  %%%%% balance
% Rthetay = 1; alphathetay = 10;  beltathetay = 1000000;  %%%%% balance

% Rx = 1; alphax = 10; beltax = 500000; gamax =50000000; %%%%% balance
% Ry = 1; alphay = 10; beltay = 100000; gamay =50000000; %%%%% balance
% Rz = 1; alphaz = 10; beltaz = 100000000;gamaz =200; %%%%% balance
% Rthetax = 1; alphathetax = 1; beltathetax = 500000;  %%%%% balance
% Rthetay = 1; alphathetay = 1; beltathetay = 500000;  %%%%% balance

% Rx = 1; alphax = 10; beltax = 500000; gamax =10000000; %%%%% balance
% Ry = 1; alphay = 10; beltay = 100000; gamay =10000000; %%%%% balance
% Rz = 1; alphaz = 10; beltaz = 50000000;gamaz =200; %%%%% balance
% Rthetax = 1; alphathetax = 10; beltathetax = 500000;  %%%%% balance
% Rthetay = 1; alphathetay = 10; beltathetay = 100000;  %%%%% balance

% Rx = 1; alphax = 10; beltax = 500000; gamax =10000000; %%%%% balance
% Ry = 1; alphay = 10; beltay = 100000; gamay =10000000; %%%%% balance
% Rz = 1; alphaz = 10; beltaz = 50000000;gamaz =200; %%%%% balance
% Rthetax = 1; alphathetax = 10; beltathetax = 500000;  %%%%% balance
% Rthetay = 1; alphathetay = 10; beltathetay = 500000;  %%%%% balance

% Rx = 1; alphax = 10; beltax = 500000; gamax =10000000; %%%%% balance
% Ry = 1; alphay = 10; beltay = 100000; gamay =10000000; %%%%% balance
% Rz = 1; alphaz = 10;beltaz = 20000000;gamaz =200; %%%%% balance
% Rthetax = 1; alphathetax = 10; beltathetax = 500000;  %%%%% balance
% Rthetay = 1; alphathetay = 10; beltathetay = 250000;  %%%%% balance

Rx = 1; alphax = 10; beltax = 500000; gamax =10000000; %%%%% balance
Ry = 1; alphay = 10; beltay = 100000; gamay =10000000; %%%%% balance
Rz = 1; alphaz = 10; beltaz = 20000000;gamaz =200; %%%%% balance
Rthetax = 1; alphathetax = 10; beltathetax = 200000;  %%%%% balance
Rthetay = 1; alphathetay = 10; beltathetay = 200000;  %%%%% balance


%% test time consumption:
tcpu=zeros(1,Nsum-Nh-1);

%% predictive control_tracking with time-varying height
for i=1:Nsum-Nh-1
    t1=cputime;    
    %%%% current period:
    bj1x = find(i*dt>=Tx);
    bjxx = bj1x(end); 
       
    %% COM_center_ref = ZMP_center_ref = v_i*f + V_i*L_ref %%%% ref-foot
    %solve the following steps: 1.5s may couve 2 or three  steps, so  one/two following steps 
    t_f = (i+1):1:(i+Nh);
    bj1 = find(t_f(1)*dt>=Tx);
    bjx1 = bj1(end);
    bj2 = find(t_f(end)*dt>=Tx);
    bjx2 = bj2(end);
    mx = bjx2-bjx1+1;
    bjx = bjx1:1:bjx2;
    tnx = zeros(mx,1);
    
    for j =1:mx-1
        ccc = find(abs(Tx(bjx(j+1))-t_f*dt)<=0.001);
        tnx(j) = ccc(1);              
    end
    
    v_i = zeros(Nh,1);
    if tnx(1) == nT+1          %%%% mx ==2; two following steps
        V_i = zeros(Nh,mx);
        for jjjj = 1:mx
            if jjjj==1
                xxx =tnx(1)-1;
                V_i(1:xxx,jjjj)=ones(xxx,1); 
            else
                xxx =Nh-tnx(1)+1;
                V_i(tnx(1):end,jjjj)=ones(xxx,1); 
            end         
        end       
    else
        xxx =tnx(1)-1;
        v_i(1:tnx(1)-1,:) = ones(xxx,1); 
        V_i = zeros(Nh,mx-1);
        if mx ==2                              %%% one following steps
            V_i(tnx(1):Nh,1)=ones(Nh-xxx,1);                          
        else                                    %%% mx==3, two following steps
            xxx1 = Nh-tnx(2)+1;
            V_i(tnx(1):tnx(2)-1,1)=ones(nT,1);
            V_i(tnx(2):Nh,2)=ones(xxx1,1);  
        end 
    end
    [~,n_vis]=size(V_i);   %%% the number of following steps: one or two:    

    
    Lx_ref = zeros(n_vis,1);
    Ly_ref = zeros(n_vis,1);
    Lz_ref = zeros(n_vis,1);
    if n_vis == 1
            Lx_ref(1) = footx_ref(bjx2,:);
            Ly_ref(1) = footy_ref(bjx2,:);
            Lz_ref(1) = footz_ref(bjx2,:);            
    else
            Lx_ref(1) = footx_ref(bjx2-1,:);
            Lx_ref(n_vis) = footx_ref(bjx2,:); 
            Ly_ref(1) = footy_ref(bjx2-1,:);
            Ly_ref(n_vis) = footy_ref(bjx2,:);     
            Lz_ref(1) = footz_ref(bjx2-1,:);
            Lz_ref(n_vis) = footz_ref(bjx2,:);              
    end
    

    
    
         

    flag(i) = n_vis;
    
    Nt = 5*Nh+3*n_vis;
    
    %% hot start 
    V_ini = zeros(Nt,1);    
    if i==1
        V_ini(5*Nh+1,:) = footx_ref(2);
        V_ini(5*Nh+2,:) = footy_ref(2);
    else
        V_ini(1:3*Nh,:) = V_optimal(1:3*Nh,i-1);
        if n_vis>flag(i-1)              %%% 1-2
            V_ini(end-1,:)= V_optimal(end,i-1);
            V_ini(end-3,:)= V_optimal(end-2,i-1);
            V_ini(end-5,:)= V_optimal(end-4,i-1);
        else
            if n_vis < flag(i-1)       %%% 2-1
                V_ini(end,:) = V_optimal(end,i-1);
                V_ini(end-1,:) = V_optimal(end-2,i-1);
                V_ini(end-2,:) = V_optimal(end-4,i-1);
            else
                if n_vis ==1
                    V_ini(end,:) = V_optimal(end,i-1);
                    V_ini(end-1,:) = V_optimal(end-2,i-1);
                    V_ini(end-2,:) = V_optimal(end-4,i-1);                    
                else
                    V_ini(3*Nh+1:end,:) = V_optimal(3*Nh+1:end,i-1);                    
                end

            end
        end        
    end
    
    %%%% current foot location
%     f = footy_ref(bjxx);
%     fx = footx_ref(bjxx);
%     fy = footy_ref(bjxx);
    fx = footx_real(bjxx);
    fy = footy_real(bjxx);
    
    %%% COM_center_ref = ZMP_center_ref = v_i*f + V_i*Ly_ref %%%% ref-foot
    COMx_center_ref = v_i*fx + V_i*Lx_ref;             %%%%%%%
    COMy_center_ref = v_i*fy + V_i*Ly_ref;

    COMz_center_ref = Zsc(i+1:i+Nh,:)+hcom;   
    thetax_center_ref = zeros(Nh,1);             
    thetay_center_ref = zeros(Nh,1);    
    

        %% SEQUENCE QUADARTIC PROGRAMMING    
    for xxx = 1:2      %%% definition of Ns:   
       %% SQP MODELS
        % model coefficient matrix 
        %%%optimization number is = Nh*3+ n_vis*3
        %%%%% optimal programme formulation:
        %%%%% x0 = xk(:,i);y0 = yk(:,i);z0 = zk(:,i);
        WX = Rx/2*eye(Nh) + alphax/2*(Pvu'*Pvu) + beltax/2*(Ppu'*Ppu);  
        WY = Ry/2*eye(Nh) + alphay/2*(Pvu'*Pvu) + beltay/2*(Ppu'*Ppu);  
        WZ = Rz/2*eye(Nh) + alphaz/2*(Pvu'*Pvu) + beltaz/2*(Ppu'*Ppu);
        WthetaX = Rthetax/2*eye(Nh) + alphathetax/2*(Pvu'*Pvu) + beltathetax/2*(Ppu'*Ppu);  
        WthetaY = Rthetay/2*eye(Nh) + alphathetay/2*(Pvu'*Pvu) + beltathetay/2*(Ppu'*Ppu);         
        PHIX = gamax/2*eye(n_vis); 
        PHIY = gamay/2*eye(n_vis);  
        PHIZ = gamaz/2*eye(n_vis);
        Q_goal = blkdiag(WX,WY,WZ,WthetaX,WthetaY,PHIX,PHIY,PHIZ); 

        q_goal = [alphax*Pvu'*Pvs*xk(:,i)+beltax*Ppu'*Pps*xk(:,i)-beltax*Ppu'* COMx_center_ref;...
                  alphay*Pvu'*Pvs*yk(:,i)+beltay*Ppu'*Pps*yk(:,i)-beltay*Ppu'* COMy_center_ref;...
                  alphaz*Pvu'*Pvs*zk(:,i)+beltaz*Ppu'*Pps*zk(:,i)-beltaz*Ppu'* COMz_center_ref;...
                  alphathetax*Pvu'*Pvs*thetaxk(:,i)+beltathetax*Ppu'*Pps*thetaxk(:,i)-beltathetax*Ppu'* thetax_center_ref;...
                  alphathetay*Pvu'*Pvs*thetayk(:,i)+beltathetay*Ppu'*Pps*thetayk(:,i)-beltathetay*Ppu'* thetay_center_ref;...
                  -gamax*Lx_ref;...
                  -gamay*Ly_ref;...
                  -gamaz*Lz_ref];


        Q_goal1 = 2 * Q_goal;
        q_goal1 = 2 * Q_goal * V_ini + q_goal;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       %% boundary
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

        Sjx = zeros(Nh,Nt);  Sjy = zeros(Nh,Nt); Sjz = zeros(Nh,Nt);   Sjthetax = zeros(Nh,Nt);  Sjthetay = zeros(Nh,Nt);
        Sfx = zeros(n_vis,Nt);  Sfy = zeros(n_vis,Nt); Sfz = zeros(n_vis,Nt);
        Sjx(:,1:Nh) = eye(Nh);                 Sjy(:,Nh+1:2*Nh) = eye(Nh);                    Sjz(:,2*Nh+1:3*Nh) = eye(Nh);          Sjthetax(:,3*Nh+1:4*Nh) = eye(Nh);  Sjthetay(:,4*Nh+1:5*Nh) = eye(Nh);
        Sfx(:,5*Nh+1:5*Nh+n_vis) = eye(n_vis); Sfy(:,5*Nh+n_vis+1:5*Nh+2*n_vis) = eye(n_vis); Sfz(:,5*Nh+2*n_vis+1:5*Nh+3*n_vis) = eye(n_vis);


        % ZMP boundary preparation:
        H_q_upx = zeros(Nh,Nt);
        F_zmp_upx = zeros(Nh,1);
        H_q_lowx = zeros(Nh,Nt);
        F_zmp_lowx = zeros(Nh,1);  
        H_q_upy = zeros(Nh,Nt);
        F_zmp_upy = zeros(Nh,1);
        H_q_lowy = zeros(Nh,Nt);
        F_zmp_lowy = zeros(Nh,1);    

        % CoM height boundary perparation:
        H_h_upz = zeros(Nh,Nt);
        F_h_upz = zeros(Nh,1);
        H_h_lowz = zeros(Nh,Nt);
        F_h_lowz = zeros(Nh,1);  
          
        % CoM height accereation boundary preparation:
        H_hacc_lowz = zeros(Nh,Nt);
        F_hacc_lowz = zeros(Nh,1);   
        
        
        %angle boundary ppreparation
        q_upx = zeros(Nh,Nt);
        qq_upx = zeros(Nh,1);
        q_lowx = zeros(Nh,Nt);
        qq_lowx = zeros(Nh,1);  
        q_upy = zeros(Nh,Nt);
        qq_upy = zeros(Nh,1);
        q_lowy = zeros(Nh,Nt);
        qq_lowy = zeros(Nh,1);   
        
        
        % torque bondary preparation
        t_upx = zeros(Nh,Nt);
        tt_upx = zeros(Nh,1);
        t_lowx = zeros(Nh,Nt);
        tt_lowx = zeros(Nh,1);  
        t_upy = zeros(Nh,Nt);
        tt_upy = zeros(Nh,1);
        t_lowy = zeros(Nh,Nt);
        tt_lowy = zeros(Nh,1);         
        

        for j = 1:Nh
            Si = zeros(1,Nh); Si(j) = 1;
           %% ZMP constraints
           %% x-ZMP upper boundary
            phi_i_x_up =  Sjx'*Ppu'*Si'*Si*Pau*Sjz  ...
                + zeros(Nt,Nt) ...
                - Sjx'*Pau'*Si'*Si*Ppu*Sjz ...
                + zeros(Nt,Nt) ...
                - Sfx'*V_i'*Si'*Si*Pau*Sjz ...
                - zeros(Nt,Nt) ...
                - zeros(Nt,Nt);
            phi_i_x_up = M_f*(phi_i_x_up+phi_i_x_up')/2;

            p_i_x_t_up = (xk(:,i)'*Pps'*Si'*Si*Pau*Sjz  + zk(:,i)'*Pas'*Si'*Si*Ppu*Sjx  ...
                +  g*Si*Ppu*Sjx ...
                - (zk(:,i)'*Pps'*Si'*Si*Pau*Sjx  + xk(:,i)'*Pas'*Si'*Si*Ppu*Sjz) ...
                + Zsc(i+j,:)*Si*Pau*Sjx ...
                -(zk(:,i)'*Pas'*Si'*Si*V_i*Sfx + fx'*v_i'*Si'*Si*Pau*Sjz) ...
                - g*Si*V_i*Sfx ...
                - ZMPx_ub(i+j)*Si*Pau*Sjz)';
            p_i_x_t_up = M_f*p_i_x_t_up - (J_ini*Si*Pau*Sjthetay)';

            del_i_x_up = xk(:,i)'*Pps'*Si'*(zk(:,i)'*Pas'*Si')' ...
                + g*Si*Pps*xk(:,i) ...
                - xk(:,i)'*Pas'*Si'*(zk(:,i)'*Pps'*Si')' ...
                + xk(:,i)'*Pas'*Si'*  Zsc(i+j,:)...
                - fx'*v_i'*Si'*(zk(:,i)'*Pas'*Si')'...
                - g* Si*v_i*fx ...
                - ZMPx_ub(i+j)*Si*Pas*zk(:,i) ...
                - g*ZMPx_ub(i+j);
            del_i_x_up = M_f*del_i_x_up - J_ini*Si*Pas*thetayk(:,i);

            H_q_upx(j,:) = (2*phi_i_x_up*V_ini + p_i_x_t_up)';
            F_zmp_upx(j,:) = -(V_ini'*phi_i_x_up*V_ini + p_i_x_t_up'*V_ini+ del_i_x_up);
            
            

           %% x-ZMP low boundary
            phi_i_x_low =  Sjx'*Ppu'*Si'*Si*Pau*Sjz  ...
                + zeros(Nt,Nt) ...
                - Sjx'*Pau'*Si'*Si*Ppu*Sjz ...
                + zeros(Nt,Nt) ...
                - Sfx'*V_i'*Si'*Si*Pau*Sjz ...
                - zeros(Nt,Nt) ...
                - zeros(Nt,Nt);
            phi_i_x_low = M_f*(phi_i_x_low+phi_i_x_low')/2;

            p_i_x_t_low = (xk(:,i)'*Pps'*Si'*Si*Pau*Sjz  + zk(:,i)'*Pas'*Si'*Si*Ppu*Sjx  ...
                +  g*Si*Ppu*Sjx ...
                - (zk(:,i)'*Pps'*Si'*Si*Pau*Sjx  + xk(:,i)'*Pas'*Si'*Si*Ppu*Sjz) ...
                + Zsc(i+j,:)*Si*Pau*Sjx ...
                -(zk(:,i)'*Pas'*Si'*Si*V_i*Sfx + fx'*v_i'*Si'*Si*Pau*Sjz) ...
                - g*Si*V_i*Sfx ...
                - ZMPx_lb(i+j)*Si*Pau*Sjz)';
            p_i_x_t_low = M_f*p_i_x_t_low - (J_ini*Si*Pau*Sjthetay)';

            del_i_x_low = xk(:,i)'*Pps'*Si'*(zk(:,i)'*Pas'*Si')' ...
                + g*Si*Pps*xk(:,i) ...
                - xk(:,i)'*Pas'*Si'*(zk(:,i)'*Pps'*Si')' ...
                + xk(:,i)'*Pas'*Si'*  Zsc(i+j,:)...
                - fx'*v_i'*Si'*(zk(:,i)'*Pas'*Si')'...
                - g* Si*v_i*fx ...
                - ZMPx_lb(i+j)*Si*Pas*zk(:,i) ...
                - g*ZMPx_lb(i+j);
            del_i_x_low = M_f*del_i_x_low - J_ini*Si*Pas*thetayk(:,i);

            H_q_lowx(j,:) = -(2*phi_i_x_low*V_ini + p_i_x_t_low)';
            F_zmp_lowx(j,:) = (V_ini'*phi_i_x_low*V_ini + p_i_x_t_low'*V_ini+ del_i_x_low); 

           %% y-ZMP upper boundary
            phi_i_y_up =  Sjy'*Ppu'*Si'*Si*Pau*Sjz  ...
                + zeros(Nt,Nt) ...
                - Sjy'*Pau'*Si'*Si*Ppu*Sjz ...
                + zeros(Nt,Nt)  ...
                - Sfy'*V_i'*Si'*Si*Pau*Sjz ...
                - zeros(Nt,Nt) ...
                - zeros(Nt,Nt);
            phi_i_y_up =  M_f*(phi_i_y_up+phi_i_y_up')/2;

            p_i_y_t_up = (yk(:,i)'*Pps'*Si'*Si*Pau*Sjz  + zk(:,i)'*Pas'*Si'*Si*Ppu*Sjy  ...
                +  g*Si*Ppu*Sjy ...
                - (zk(:,i)'*Pps'*Si'*Si*Pau*Sjy  + yk(:,i)'*Pas'*Si'*Si*Ppu*Sjz) ...
                + Zsc(i+j,:)*Si*Pau*Sjy  ...
                -(zk(:,i)'*Pas'*Si'*Si*V_i*Sfy + fy'*v_i'*Si'*Si*Pau*Sjz) ...
                - g*Si*V_i*Sfy ...
                - ZMPy_ub(i+j)*Si*Pau*Sjz)';
            p_i_y_t_up = M_f*p_i_y_t_up + (J_ini*Si*Pau*Sjthetax)';            

            del_i_y_up = yk(:,i)'*Pps'*Si'*(zk(:,i)'*Pas'*Si')' ...
                + g*Si*Pps*yk(:,i) ...
                - yk(:,i)'*Pas'*Si'*(zk(:,i)'*Pps'*Si')' ...
                + yk(:,i)'*Pas'*Si'* Zsc(i+j,:) ...
                - fy'*v_i'*Si'*(zk(:,i)'*Pas'*Si')'...
                - g* Si*v_i*fy ...
                - ZMPy_ub(i+j)*Si*Pas*zk(:,i) ...
                - g*ZMPy_ub(i+j);
            del_i_y_up = M_f*del_i_y_up + (J_ini*Si*Pas*thetaxk(:,i))';
            
            H_q_upy(j,:) = (2*phi_i_y_up*V_ini + p_i_y_t_up)';
            F_zmp_upy(j,:) = -(V_ini'*phi_i_y_up*V_ini + p_i_y_t_up'*V_ini+ del_i_y_up); 

           %% y-ZMP lower boundary
            phi_i_y_low =  Sjy'*Ppu'*Si'*Si*Pau*Sjz  ...
                + zeros(Nt,Nt) ...
                - Sjy'*Pau'*Si'*Si*Ppu*Sjz ...
                + zeros(Nt,Nt)  ...
                - Sfy'*V_i'*Si'*Si*Pau*Sjz ...
                - zeros(Nt,Nt) ...
                - zeros(Nt,Nt);
            phi_i_y_low =  M_f*(phi_i_y_low+phi_i_y_low')/2;

            p_i_y_t_low = (yk(:,i)'*Pps'*Si'*Si*Pau*Sjz  + zk(:,i)'*Pas'*Si'*Si*Ppu*Sjy  ...
                +  g*Si*Ppu*Sjy ...
                - (zk(:,i)'*Pps'*Si'*Si*Pau*Sjy  + yk(:,i)'*Pas'*Si'*Si*Ppu*Sjz) ...
                + Zsc(i+j,:)*Si*Pau*Sjy  ...
                -(zk(:,i)'*Pas'*Si'*Si*V_i*Sfy + fy'*v_i'*Si'*Si*Pau*Sjz) ...
                - g*Si*V_i*Sfy ...
                - ZMPy_lb(i+j)*Si*Pau*Sjz)';
            p_i_y_t_low = M_f*p_i_y_t_low + (J_ini*Si*Pau*Sjthetax)';

            del_i_y_low = yk(:,i)'*Pps'*Si'*(zk(:,i)'*Pas'*Si')' ...
                + g*Si*Pps*yk(:,i) ...
                - yk(:,i)'*Pas'*Si'*(zk(:,i)'*Pps'*Si')' ...
                + yk(:,i)'*Pas'*Si'* Zsc(i+j,:) ...
                - fy'*v_i'*Si'*(zk(:,i)'*Pas'*Si')'...
                - g* Si*v_i*fy ...
                - ZMPy_lb(i+j)*Si*Pas*zk(:,i) ...
                - g*ZMPy_lb(i+j);
            del_i_y_low = M_f*del_i_y_low + J_ini*Si*Pas*thetaxk(:,i);
            
            H_q_lowy(j,:) = -(2*phi_i_y_low*V_ini + p_i_y_t_low)';
            F_zmp_lowy(j,:) = (V_ini'*phi_i_y_low*V_ini + p_i_y_t_low'*V_ini+ del_i_y_low); 

            
            
            
            %% angle range costraints             
            q1_upx = Si*Ppu*Sjthetax;
            qq1_upx = Si*Pps*thetaxk(:,i) - thetax_max; 
            q_upx(j,:) = q1_upx;
            qq_upx(j,:) = -(q1_upx*V_ini + qq1_upx);  
            
            q1_lowx = Si*Ppu*Sjthetax;
            qq1_lowx = Si*Pps*thetaxk(:,i) - thetax_min; 
            q_lowx(j,:) = -q1_lowx;
            qq_lowx(j,:) = (q1_lowx*V_ini + qq1_lowx);  
            
            q1_upy = Si*Ppu*Sjthetay;
            qq1_upy = Si*Pps*thetayk(:,i) - thetay_max; 
            q_upy(j,:) = q1_upy;
            qq_upy(j,:) = -(q1_upy*V_ini + qq1_upy);  
            
            q1_lowy = Si*Ppu*Sjthetay;
            qq1_lowy = Si*Pps*thetayk(:,i) - thetay_min; 
            q_lowy(j,:) = -q1_lowy;
            qq_lowy(j,:) = (q1_lowy*V_ini + qq1_lowy);      
            
            
            %% torque range constraints
            t1_upx = Si*Pau*Sjthetax;
            tt1_upx = Si*Pas*thetaxk(:,i) - torquex_max; 
            t_upx(j,:) = t1_upx;
            tt_upx(j,:) = -(t1_upx*V_ini + tt1_upx);  
            
            t1_lowx = Si*Pau*Sjthetax;
            tt1_lowx = Si*Pas*thetaxk(:,i) - torquex_min; 
            t_lowx(j,:) = -t1_lowx;
            tt_lowx(j,:) = (t1_lowx*V_ini + tt1_lowx);  
            
            t1_upy = Si*Pau*Sjthetay;
            tt1_upy = Si*Pas*thetayk(:,i) - torquey_max; 
            t_upy(j,:) = t1_upy;
            tt_upy(j,:) = -(t1_upy*V_ini + tt1_upy);  
            
            t1_lowy = Si*Pau*Sjthetay;
            tt1_lowy = Si*Pas*thetayk(:,i) - torquey_min; 
            t_lowy(j,:) = -t1_lowy;
            tt_lowy(j,:) = (t1_lowy*V_ini + tt1_lowy);             
            
            
            

           %% body height constraints
            P_footz_up = Si*Ppu*Sjz;
            delta_footz_up = Si*Pps*zk(:,i) - COMz_center_ref(j) - Z_max(i+j,:); 
            H_h_upz(j,:) = P_footz_up;
            F_h_upz(j,:) = -(P_footz_up*V_ini + delta_footz_up);

            P_footz_up = Si*Ppu*Sjz;
            delta_footz_lowp = Si*Pps*zk(:,i) - COMz_center_ref(j) - Z_min;         
            H_h_lowz(j,:)  = -P_footz_up;
            F_h_lowz(j,:)  = (P_footz_up*V_ini + delta_footz_lowp);     
            
            
            
           %% body height acceleration
            P_footzacc_up = Si*Pau*Sjz;
            delta_footzacc_up = Si*Pas*zk(:,i) -(-g);         
            H_hacc_lowz(j,:)  = -P_footzacc_up;
            F_hacc_lowz(j,:)  = (P_footzacc_up*V_ini + delta_footzacc_up);           

        end

        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       %% foot location constraints          
        % foot distance preparation;
        H_q_footx_up = zeros(n_vis,Nt);
        F_foot_upx = zeros(n_vis,1);
        H_q_footx_low = zeros(n_vis,Nt);
        F_foot_lowx = zeros(n_vis,1);    
        H_q_footy_up = zeros(n_vis,Nt);
        F_foot_upy = zeros(n_vis,1);
        H_q_footy_low = zeros(n_vis,Nt);
        F_foot_lowy = zeros(n_vis,1);        
      
       if n_vis ==1            %%%%only one next step
           % footx location constraints
           Sfoot = Sfx(1,:);
           P_footx_up = Sfoot;
           delta_footx_up = -fx - Footx_max; 
           H_q_footx_up(1,:) = P_footx_up;
           F_foot_upx(1,:) = -(Sfoot*V_ini + delta_footx_up);

           P_footx_up = Sfoot;
           delta_footx_up = -fx - Footx_min; 
           H_q_footx_low(1,:) = -P_footx_up;
           F_foot_lowx(1,:) = (Sfoot*V_ini + delta_footx_up);

           % footy location constraints
           if rem(bjxx,2) ==0                       
               Sfoot = Sfy(1,:);
               P_footy_up = Sfoot;
               delta_footy_up = -fy - (-Footy_min); 
               H_q_footy_up(1,:) = P_footy_up;
               F_foot_upy(1,:) = -(Sfoot*V_ini + delta_footy_up);

               P_footy_up = Sfoot;
               delta_footy_up = -fy - (- Footy_max); 
               H_q_footy_low(1,:) = -P_footy_up;
               F_foot_lowy(1,:) = (Sfoot*V_ini + delta_footy_up);             
           else
               Sfoot = Sfy(1,:);
               P_footy_up = Sfoot;
               delta_footy_up = -fy - Footy_max; 
               H_q_footy_up(1,:) = P_footy_up;
               F_foot_upy(1,:) = -(Sfoot*V_ini + delta_footy_up);

               P_footy_up = Sfoot(1,:);
               delta_footy_up = -fy - Footy_min; 
               H_q_footy_low(1,:) = -P_footy_up;
               F_foot_lowy(1,:) = (Sfoot*V_ini + delta_footy_up);           
           end      

       else                     %%%% two following steps
           % footx location constraints
           Sfoot = Sfx(1,:);
           P_footx_up = Sfoot;
           delta_footx_up = -fx - Footx_max; 
           H_q_footx_up(1,:) = P_footx_up;
           F_foot_upx(1,:) = -(Sfoot*V_ini + delta_footx_up);

           P_footx_up = Sfoot;
           delta_footx_up = -fx - Footx_min; 
           H_q_footx_low(1,:) = -P_footx_up;
           F_foot_lowx(1,:) = (Sfoot*V_ini + delta_footx_up);

           % footy location constraints
           if rem(bjxx,2) ==0                       
               Sfoot = Sfy(1,:);
               P_footy_up = Sfoot;
               delta_footy_up = -fy - (-Footy_min); 
               H_q_footy_up(1,:) = P_footy_up;
               F_foot_upy(1,:) = -(Sfoot*V_ini + delta_footy_up);

               P_footy_up = Sfoot;
               delta_footy_up = -fy - (- Footy_max); 
               H_q_footy_low(1,:) = -P_footy_up;
               F_foot_lowy(1,:) = (Sfoot*V_ini + delta_footy_up);             
           else
               Sfoot = Sfy(1,:);
               P_footy_up = Sfoot;
               delta_footy_up = -fy - Footy_max; 
               H_q_footy_up(1,:) = P_footy_up;
               F_foot_upy(1,:) = -(Sfoot*V_ini + delta_footy_up);

               P_footy_up = Sfoot(1,:);
               delta_footy_up = -fy - Footy_min; 
               H_q_footy_low(1,:) = -P_footy_up;
               F_foot_lowy(1,:) = (Sfoot*V_ini + delta_footy_up);           
           end  

           %%% next two steps
           % footx location constraints
           Sfoot = [-1,1];
           Sfoot = Sfoot*Sfx;
           P_footx_up = Sfoot;
           delta_footx_up = - Footx_max; 
           H_q_footx_up(2,:) = P_footx_up;
           F_foot_upx(2,:) = -(Sfoot*V_ini + delta_footx_up);

           P_footx_up = Sfoot;
           delta_footx_up = - Footx_min; 
           H_q_footx_low(2,:) = -P_footx_up;
           F_foot_lowx(2,:) = Sfoot*V_ini + delta_footx_up;

           % footy location constraints
           if rem(bjxx,2) ==0               
               Sfoot = [-1,1];
               Sfoot = Sfoot*Sfy;
               P_footy_up = Sfoot;
               delta_footy_up = -Footy_max; 
               H_q_footy_up(2,:) = P_footy_up;
               F_foot_upy(2,:) = -(Sfoot*V_ini + delta_footy_up);

               P_footy_up = Sfoot;
               delta_footy_up = - Footy_min; 
               H_q_footy_low(2,:) = -P_footy_up;
               F_foot_lowy(2,:) = (Sfoot*V_ini + delta_footy_up);             
           else                           %%% current is left support, next one is right support, next two is left
               Sfoot = [-1,1];
               Sfoot = Sfoot*Sfy;
               P_footy_up = Sfoot;
               delta_footy_up = -(-Footy_min); 
               H_q_footy_up(2,:) = P_footy_up;
               F_foot_upy(2,:) = -(Sfoot*V_ini + delta_footy_up);

               P_footy_up = Sfoot;
               delta_footy_up =- (-Footy_max); 
               H_q_footy_low(2,:) = -P_footy_up;
               F_foot_lowy(2,:) = (Sfoot*V_ini + delta_footy_up);           
           end          
       end

        %% swing foot velocity boundary
        if i==1
            footubxv = -(Sfx(1,:)*V_ini-Footx_max-footx_real_next(i+nT-1));
            footlbxv = Sfx(1,:)*V_ini-Footx_min-footx_real_next(i+nT-1);   
            footubyv = -(Sfy(1,:)*V_ini-Footy_max-footy_real_next(i+nT-1));
            footlbyv = Sfy(1,:)*V_ini-Footy_min-footy_real_next(i+nT-1);                 
        else
            if abs(i-round((Tx(bjxx))/dt))<=0.001
                if rem(bjxx,2)==1   %%%next one step
                    footubxv = -(Sfx(1,:)*V_ini-Footx_max-footx_real_next(i+nT-1));
                    footlbxv = Sfx(1,:)*V_ini-Footx_min-footx_real_next(i+nT-1);
                    footubyv = -(Sfy(1,:)*V_ini-Footy_max-footy_real_next(i+nT-1));
                    footlbyv = Sfy(1,:)*V_ini-Footy_min-footy_real_next(i+nT-1);                   
                else
                    footubxv = -(Sfx(1,:)*V_ini-Footx_max-footx_real_next(i+nT-1));
                    footlbxv = Sfx(1,:)*V_ini-Footx_min-footx_real_next(i+nT-1);
                    footubyv = -(Sfy(1,:)*V_ini-(-Footy_min)-footy_real_next(i+nT-1));
                    footlbyv = Sfy(1,:)*V_ini-(-Footy_max)-footy_real_next(i+nT-1);   
                end

            else
                footubxv = -(Sfx(1,:)*V_ini-dt*footx_vmax-footx_real_next(i+nT-1));
                footlbxv = Sfx(1,:)*V_ini-dt*footx_vmin-footx_real_next(i+nT-1);
                footubyv = -(Sfy(1,:)*V_ini-dt*footy_vmax-footy_real_next(i+nT-1));
                footlbyv = Sfy(1,:)*V_ini-dt*footy_vmin-footy_real_next(i+nT-1);                    
            end        
        end
        Footvx_max =Sfx(1,:);  Footvx_min = -Sfx(1,:); Footvy_max =Sfy(1,:);  Footvy_min = -Sfy(1,:);       
               

       %% CoM-supportl leg constraints
        %%% next time moment support leg: s1*(v_i*fx + V_i*Sfw*V_ini);
        S1 = zeros(1,Nh); S1(1)=1;
        if bjxx==1
                com_MAX1 = zeros(1,Nt); 
                com_max1 = zeros(1,1);

                com_MAX2 = zeros(1,Nt); 
                com_max2 = zeros(1,1)';            

                com_MAX3 = zeros(1,Nt); 
                com_max3 = zeros(1,1);

                com_MAX4 = zeros(1,Nt); 
                com_max4 = zeros(1,1);              
        else
            if rem(bjx1,2)==0              %%%left support

                com_MAX1 = S1*(3*(Ppu*Sjy -(V_i*Sfy)) -(Ppu*Sjx) +(V_i*Sfx)); 
                com_max1 = -S1*(3*(Ppu*Sjy*V_ini+Pps*yk(:,i) -(v_i*fy + V_i*Sfy*V_ini)) -(Ppu*Sjx*V_ini+Pps*xk(:,i)) +(v_i*fx + V_i*Sfx*V_ini+comx_min));

                com_MAX2 = S1*(3*(Ppu*Sjy -(V_i*Sfy)) +(Ppu*Sjx) -(V_i*Sfx)); 
                com_max2 = -S1*(3*(Ppu*Sjy*V_ini+Pps*yk(:,i) -(v_i*fy + V_i*Sfy*V_ini)) +(Ppu*Sjx*V_ini+Pps*xk(:,i)) -(v_i*fx + V_i*Sfx*V_ini+comx_max));            

                com_MAX3 = S1*((Ppu*Sjy -(V_i*Sfy)) ); 
                com_max3 = -S1*((Ppu*Sjy*V_ini+Pps*yk(:,i) -(v_i*fy + V_i*Sfy*V_ini)) +comy_min);

                com_MAX4 = -S1*((Ppu*Sjy -(V_i*Sfy))); 
                com_max4 = S1*((Ppu*Sjy*V_ini+Pps*yk(:,i) -(v_i*fy + V_i*Sfy*V_ini)) +comy_max);              
            else
                com_MAX1 = -S1*(3*(Ppu*Sjy -(V_i*Sfy)) +(Ppu*Sjx) -(V_i*Sfx)); 
                com_max1 = S1*(3*(Ppu*Sjy*V_ini+Pps*yk(:,i) -(v_i*fy + V_i*Sfy*V_ini)) +(Ppu*Sjx*V_ini+Pps*xk(:,i)) -(v_i*fx + V_i*Sfx*V_ini+comx_min));

                com_MAX2 = -S1*(3*(Ppu*Sjy -(V_i*Sfy)) -(Ppu*Sjx) +(V_i*Sfx)); 
                com_max2 = S1*(3*(Ppu*Sjy*V_ini+Pps*yk(:,i) -(v_i*fy + V_i*Sfy*V_ini)) -(Ppu*Sjx*V_ini+Pps*xk(:,i)) +(v_i*fx + V_i*Sfx*V_ini+comx_max));            

                com_MAX3 = -S1*((Ppu*Sjy -(V_i*Sfy)) ); 
                com_max3 = S1*((Ppu*Sjy*V_ini+Pps*yk(:,i) -(v_i*fy + V_i*Sfy*V_ini)) -comy_min );

                com_MAX4 = S1*((Ppu*Sjy -(V_i*Sfy))); 
                com_max4 = -S1*((Ppu*Sjy*V_ini+Pps*yk(:,i) -(v_i*fy + V_i*Sfy*V_ini)) -comy_max);                       
            end               
        end
        

     
       %% Foot vertical loction-equality constraints
        H_q_footz = Sfz;
        F_footz = -(Sfz*V_ini-Lz_ref);        
        
       %% fixed height
        h_h = Ppu*Sjz;
        hhhx = -(Ppu*Sjz*V_ini+Pps*zk(:,i)-hcom*ones(Nh,1));
        
        H_q_footz1 = [H_q_footz;h_h];
        F_footz1 = [F_footz;hhhx];
        
       %% fixed inclined angle
        a_hx = Ppu*Sjthetax;
        a_hxx = Ppu*Sjthetax*V_ini+Pps*thetaxk(:,1);
        a_hy = Ppu*Sjthetay;
        a_hyy = Ppu*Sjthetay*V_ini+Pps*thetayk(:,1);        

        H_q_footz2 = [H_q_footz1;a_hx;a_hy];
        F_footz2 = [F_footz1;a_hxx;a_hyy];        
        
        A_q1 = [H_q_upx;  H_q_lowx;   H_q_upy;   H_q_lowy;   H_h_upz; H_h_lowz; H_hacc_lowz; H_q_footx_up; H_q_footx_low; H_q_footy_up; H_q_footy_low; ...
            q_upx;  q_lowx;  q_upy;  q_lowy;  t_upx;  t_lowx;  t_upy;  t_lowy;...
            Footvx_max;Footvx_min;Footvy_max;Footvy_min;com_MAX1;com_MAX2;com_MAX3;com_MAX4]; 
        
        b_q1 = [F_zmp_upx;F_zmp_lowx; F_zmp_upy; F_zmp_lowy; F_h_upz; F_h_lowz; F_hacc_lowz; F_foot_upx;   F_foot_lowx;   F_foot_upy;   F_foot_lowy;   ...
            qq_upx; qq_lowx; qq_upy; qq_lowy; tt_upx; tt_lowx; tt_upy; tt_lowy;...
            footubxv;footlbxv;footubyv;footlbyv;com_max1;com_max2;com_max3;com_max4];                  

        X_vacc_k = quadprog(Q_goal1,q_goal1,A_q1,b_q1,H_q_footz,F_footz);                %%%%%(Nt)*1   

        [axxx,~]=size(X_vacc_k);
        
        if axxx ==Nt  
            V_ini = V_ini+X_vacc_k;
        end                 
    end
    

    %%% V_optimal+incremental    
    if n_vis ==1
        V_optimal(1:5*Nh,i)= V_ini(1:5*Nh,:);
        V_optimal(5*Nh+1:5*Nh+2,i)=V_ini(5*Nh+1,:)*ones(2,1);
        V_optimal(5*Nh+3:5*Nh+4,i)=V_ini(5*Nh+2,:)*ones(2,1);
        V_optimal(5*Nh+5:5*Nh+6,i)=V_ini(5*Nh+3,:)*ones(2,1);
    else
        V_optimal(:,i) = V_ini;
    end
    
    
    
    %% 
    %%% NEXT STEP LOCATION
    x_vacc_k(i) = V_ini(1);
    footx_real(bjxx+1) = V_ini(5*Nh+1);
    footx_real_next(i+nT) = V_ini(5*Nh+1);
    %%% actually, the xk is the actual state:here: it is the design_com_state
    comxx = Ppu*Sjx*V_ini+Pps*xk(:,i);
    comx(i+1) = comxx(1);
    comxxv = Pvu*Sjx*V_ini+Pvs*xk(:,i);
    comvx(i+1) = comxxv(1);
    comxxa = Pau*Sjx*V_ini+Pas*xk(:,i);
    comax(i+1) = comxxa(1);
    xk(:,i+1) = [comx(i+1);comvx(i+1);comax(i+1)];

    y_vacc_k(i) = V_ini(Nh+1);
    footy_real(bjxx+1) = V_ini(5*Nh+n_vis+1);
    footy_real_next(i+nT) = V_ini(5*Nh+n_vis+1);
    %%% actually, the xk is the actual state:here: it is the design_com_state
    yk(:,i+1) = A*yk(:,i)+B*y_vacc_k(i);
    comy(i+1) = yk(1,i+1);
    comvy(i+1) = yk(2,i+1);
    comay(i+1) = yk(3,i+1);  

    %%% external disturbance
%     if i == 40
%        comvx(i+1) = comvx(i+1)+0.81;
%        comvy(i+1) = comvy(i+1)+0.71;
%     end
%     if i == 40
%        comvx(i+1) = comvx(i+1)+0.83;
%        comvy(i+1) = comvy(i+1)+0.73;
%     end    
    
    if i == 40
       comvx(i+1) = comvx(i+1)+0.55;
       comvy(i+1) = comvy(i+1)+0.42;
    end
    xk(:,i+1) = [comx(i+1);comvx(i+1);comax(i+1)];
    yk(:,i+1) = [comy(i+1);comvy(i+1);comay(i+1)];
    
    z_vacc_k(i) = V_ini(2*Nh+1);
    footz_real(bjxx+1) = V_ini(5*Nh+2*n_vis+1);
    if i==1
        footz_real_next(i+nT) = V_ini(5*Nh+2*n_vis+1);
    else
        footz_real_next(i+nT) = V_ini(5*Nh+2*n_vis+1);
    end
    
    %%% actuallz, the xk is the actual state:here: it is the design_com_state
    zk(:,i+1) = A*zk(:,i)+B*z_vacc_k(i);
    comz(i+1) = zk(1,i+1);
    comvz(i+1) = zk(2,i+1);
    comaz(i+1) = zk(3,i+1);    

    %%% angle
    thetax_vacc_k(i) = V_ini(3*Nh+1);
    %%% actually, the xk is the actual state:here: it is the design_theta_state
    thetaxx = Ppu*Sjthetax*V_ini+Pps*thetaxk(:,i);
    thetax(i+1) = thetaxx(1);
    thetaxxv = Pvu*Sjthetax*V_ini+Pvs*thetaxk(:,i);
    thetavx(i+1) = thetaxxv(1);
    thetaxxa = Pau*Sjthetax*V_ini+Pas*thetaxk(:,i);
    thetaax(i+1) = thetaxxa(1);
    thetaxk(:,i+1) = [thetax(i+1);thetavx(i+1);thetaax(i+1)];

    thetay_vacc_k(i) = V_ini(4*Nh+1);
    %%% actually, the xk is the actual state:here: it is the design_theta_state
    thetayk(:,i+1) = A*thetayk(:,i)+B*thetay_vacc_k(i);
    thetay(i+1) = thetayk(1,i+1);
    thetavy(i+1) = thetayk(2,i+1);
    thetaay(i+1) = thetayk(3,i+1);  
    
    torquex_real(i+1) = J_ini*thetaax(i+1);
    torquey_real(i+1) = J_ini*thetaay(i+1);
    
    ZMPx_real(i+1) = comx(i+1) - (comz(i+1)-Zsc(i+1,:))/(comaz(i+1)+g)*comax(i+1) - J_ini*thetavy(i+1)/(M_f*(g+comaz(i+1)));
    ZMPy_real(i+1) = comy(i+1) - (comz(i+1)-Zsc(i+1,:))/(comaz(i+1)+g)*comay(i+1) + J_ini*thetavx(i+1)/(M_f*(g+comaz(i+1)));
    t2=cputime;
    tcpu(i)=t2-t1;    
end



%% RESULT DISPLAY
%% 3D direction
figure (1)
view(-20,15)
hold on;
plot3(comx(2:Nsum-Nh-1),comy(2:Nsum-Nh-1),comz(2:Nsum-Nh-1),'g','Linewidth',4);
plot3(ZMPx_real(2:Nsum-Nh-1),ZMPy_real(2:Nsum-Nh-1),footz_real_next(2:Nsum-Nh-1),'k','Linewidth',4);
plot3(footx_real_next(2:Nsum-Nh-1),footy_real_next(2:Nsum-Nh-1),footz_real_next(2:Nsum-Nh-1),'b','Linewidth',6);
legend('CoM','ZMP','next-foot-location');

%% forward direction
figure (2)
hold on;
xlabel('time(s)')
ylabel('x(m)')
plot(t(2:Nsum-Nh-1),comx(2:Nsum-Nh-1),'g','Linewidth',4);
plot(t(2:Nsum-Nh-1),ZMPx_real(2:Nsum-Nh-1),'--','Linewidth',4);
plot(t(2:Nsum-Nh-1),footx_real_next(2:Nsum-Nh-1),'b','Linewidth',6);
legend('CoMx','ZMPx','next-footx-location');

%% lateral direction
figure (3)
hold on;
xlabel('time(s)')
ylabel('y(m)')
plot(t(2:Nsum-Nh-1),comy(2:Nsum-Nh-1),'g','Linewidth',4);
plot(t(2:Nsum-Nh-1),ZMPy_real(2:Nsum-Nh-1),'--','Linewidth',4);
plot(t(2:Nsum-Nh-1),footy_real_next(2:Nsum-Nh-1),'b','Linewidth',6);
legend('CoMy','ZMPy','next-footy-location');

% vertical direction
figure (4)
hold on;
xlabel('time(s)')
ylabel('z(m)')
plot(t(2:Nsum-Nh-1),comz(2:Nsum-Nh-1),'g','Linewidth',4);
plot(t(2:Nsum-Nh-1),footz_real_next(2:Nsum-Nh-1),'b','Linewidth',6);
legend('CoMz','next-footz-location');

%% forward-lateral velocity
figure (5)
hold on;
xlabel('Forward velocity(m/s)')
ylabel('Lateral velocity(m/s)')
plot(comvx(2:Nsum-Nh-1),comvy(2:Nsum-Nh-1),'k','Linewidth',4);
% plot(comvx(2:5*nT),comvy(2:5*nT),'g','Linewidth',4);
% plot(comvx(5*nT:9*nT),comvy(5*nT:9*nT),'b','Linewidth',4);
% plot(comvx(9*nT:Nsum-Nh-1),comvy(9*nT:Nsum-Nh-1),'k','Linewidth',4);
% 
% lateral-vertical position
figure (6)
hold on;
xlabel('Y(m)')
ylabel('Z(m)')
plot(comy(2:Nsum-Nh-1),comz(2:Nsum-Nh-1),'k','Linewidth',4);
% plot(comy(2:5*nT),comz(2:5*nT),'g','Linewidth',4);
% plot(comy(5*nT:9*nT),comz(5*nT:9*nT),'b','Linewidth',4);
% plot(comy(9*nT:Nsum-Nh-1),comz(9*nT:Nsum-Nh-1),'k','Linewidth',4);

%% x angle
figure (7)
hold on;
xlabel('time(s)')
ylabel('theta(rad)')
plot(t(2:Nsum-Nh-1),thetax(2:Nsum-Nh-1),'g','Linewidth',4);
plot(t(2:Nsum-Nh-1),thetay(2:Nsum-Nh-1),'b','Linewidth',4);
legend('roll-angle','picth-angle');

%% y angle
figure (8)
hold on;
xlabel('time(s)')
ylabel('torque(n/m)')
plot(t(2:Nsum-Nh-1),torquex_real(2:Nsum-Nh-1),'g','Linewidth',4);
plot(t(2:Nsum-Nh-1),torquey_real(2:Nsum-Nh-1),'b','Linewidth',4);
legend('torque-x','torque-y');
end

%% matrix solution
function [y]=matrix_ps(A,N,C)
y = zeros(N,3);
for i =1:N
    y(i,:) = C*A^i;
end
 
end
function [y]=matrix_pu(A,B,N,C)
y = zeros(N,N);
for i = 1:N
    for j = 1:i
        y(i,j) = C*A^(i-j)*B;
    end
end
    

end