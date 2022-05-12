function [sys,x0,str,ts,simStateCompliance] = plant(t,x,u,flag)
%
% The following outlines the general structure of an S-function.
%
switch flag,

  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1,
    sys=mdlDerivatives(t,x,u);
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

sizes.NumContStates  = 4;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 0;   % at least one sample time is needed
sys = simsizes(sizes);

% initialize the initial conditions
% x0  = [0 0 0 0];
x0  = [0 0 pi/4 0];
str = [];
ts  = [];
simStateCompliance = 'UnknownSimState';

% the paramater of the plant
global g m l
g = 9.8;
m = [1 1];
l = [1 1];


%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
function sys=mdlDerivatives(t,x,u)
global g m l
% dynamic modeling of 2-link manipulator
dq = [x(2); x(4)];
torque = u(1:2);
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

S = inv(M)*(torque - C - G);

sys = [x(2); S(1); x(4); S(2)];


%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================

function sys=mdlOutputs(t,x,u)

sys = [x(1); x(2); x(3); x(4)];

%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
function sys=mdlTerminate(t,x,u)

sys = [];