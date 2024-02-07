%MPC Initialize
%function [uopt,problem] = mpcControl(obs,x0,goal,goalV)
yalmip('clear')

obStates = obNum*2; %Num obstacles
ns = 4; %Num states
nt = 2;
nx = ns + nt + obStates; %Total states
nu = 2; %Inputs (x/y acc)


%Initialize vars
u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x0 = sdpvar(nx,1);
x = x0;
slack = sdpvar(1,1);

constraints = [];
objective = 0;

%Initialize model
A = eye(nx);
B = [dt 0 0 0 zeros(1,obStates+nt); 0 dt 0 0 zeros(1,obStates+nt)]';
Q = zeros(nx); Q(1:2,1:2) = eye(2); Q(1:2,5:6) = -eye(2);
% R = zeros(nx); R(3,3) = 1; R(4,4) = 1; %R(3:4,7:8) = -eye(2);
%Obj/Cons
for k = 1:N
    x = A*x + B*u{k};
    objective = objective + norm(Q*x,1) + 1e3*slack^2;
    %Input and state constraints
    constraints = [constraints, -100<=x<=100];
    for o = ns+nt+1:2:nx-1
%         Dist from obstacle constraint
        constraints = [constraints, (x(1)-x(o))^2 + (x(2)-x(o+1))^2 - obDist^2 + slack>=0];
    end
%     constraints = [constraints, u{k}(1)^2 + u{k}(2)^2 - x(4)^2<=0];
end

%Compile
ops = sdpsettings('verbose',0,'warning',0);
controller = optimizer(constraints,objective,ops,x0,u{1});
