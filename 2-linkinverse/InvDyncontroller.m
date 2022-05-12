function [sys,x0,str,ts,simStateCompliance] = InvDyncontroller(t,x,u,flag)
%
% The following outlines the general structure of an S-function.
%
switch flag,

  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 3,
    sys=mdlOutputs(t,x,u);
    %Unhandled flags
  case {2, 4}
    sys = [];
  case 9,
    sys=mdlTerminate(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end


%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
sizes = simsizes;

sizes.NumOutputs     = 2;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);

% initialize the initial conditions
x0  = [];
str = [];
ts  = [0 0];
simStateCompliance = 'UnknownSimState';

% the paramater of the plant
global g m l
g = 9.8;
m = [1 1];
l = [1 1];


%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================

function sys=mdlOutputs(t,x,u)
global g m l

%q1_d = pi/4; 
%q2_d = pi/2; 
q1_d = 2.5*t^2 - 1.5*t^3; 
q2_d = 3.5*t^2 - 2.5*t^3; 
q1_d_dot = 5*t - 4.5*t^2;
q2_d_dot = 7*t - 7.5*t^2;
q1_d_ddot = 5 - 9*t;
q2_d_ddot = 7 - 15*t;

x(1) = u(1);
x(2) = u(2);
x(3) = u(3);
x(4) = u(4);

e1 = x(1) - q1_d;
e2 = x(3) - q2_d;
e = [e1; e2];

Kp = [100 0; 0 100];
Kd = [100 0; 0 100];
% dynamic modeling of 2-link manipulator
dq = [x(2); x(4)];

M = zeros(2, 2);
M(1, 1) = m(1)*l(1)^2 + m(2)*(l(1)^2 + l(2)^2 + 2*l(1)*l(2)*cos(x(3)));
M(1, 2) = m(2)*(l(2)^2 + l(1) * l(2) * cos(x(3)));
M(2, 1) = M(1, 2);
M(2, 2) = m(2) * l(2)^2;
C = zeros(2, 1);
C(1, 1) = -2*m(1)*l(1)*l(2)*sin(x(3))*dq(1)*dq(2) - m(2)*l(1)*l(2)*sin(x(3))*(dq(2)^2);
C(2, 1) = m(2)*l(1)*l(2)*sin(x(3))*(dq(1)^2);
G = zeros(2, 1);
G(1, 1) = (m(1) + m(2))*l(1)*g*cos(x(1)) + m(2)*g*l(2)*cos(x(1) + x(3));
G(2, 1) = m(2)*g*l(2)*cos(x(1) + x(3));

aq = [q1_d_ddot; q2_d_ddot] - Kp*([x(1);x(3)] - [q1_d; q2_d]) - Kd *([x(2);x(4)] - [q1_d_dot; q2_d_dot]);
torque = M*aq + C + G;

sys(1)=torque(1);
sys(2)=torque(2);

%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
function sys=mdlTerminate(t,x,u)

sys = [];