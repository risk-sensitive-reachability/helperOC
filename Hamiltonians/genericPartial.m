function alpha = genericPartial(t, data, derivMin, derivMax, schemeData, dim)
% alpha = genericPartial(t, data, derivMin, derivMax, schemeData, dim)

g = schemeData.grid;
dynSys = schemeData.dynSys;

if ismethod(dynSys, 'partialFunc')
%   disp('Using partial function from dynamical system')
  alpha = dynSys.partialFunc(t, data, derivMin, derivMax, schemeData, dim);
  return
end

if ~isfield(schemeData, 'uMode')
  schemeData.uMode = 'min';
end

if ~isfield(schemeData, 'dMode')
  schemeData.dMode = 'min';
end

% TIdim = [];
% dims = 1:dynSys.nx;
% if isfield(schemeData, 'MIEdims')
%   TIdim = schemeData.TIdim;
%   dims = schemeData.MIEdims;
% end

% x = cell(dynSys.nx, 1);
% x(dims) = g.xs;

%% Compute control
if isfield(schemeData, 'uIn')
  % Control
  uU = schemeData.uIn;
  uL = schemeData.uIn;
 
else
  % Optimal control assuming maximum deriv
  uU = dynSys.optCtrl(t, g.xs, derivMax, schemeData.uMode);
  
  % Optimal control assuming minimum deriv
  uL = dynSys.optCtrl(t, g.xs, derivMin, schemeData.uMode);
end

global control_update_available; 
global uU_previous; 
global uL_previous;
global config; 

if isempty(uU_previous) error(); end
if isempty(uL_previous) error(); end
if isempty(control_update_available) error(); end
%if (control_update_available == config.dt * config.N) control_update_available = 0; end

if t >= control_update_available 
   
    uU_previous = uU; 
    uL_previous = uL; 
    control_update_available = t + config.dt;

    
   % disp('dissipation controls updated'); 
   
else 
   % disp('dissipation controls from previous'); 
    
end
    
uL = uL_previous;
uU = uU_previous; 

%previous_control = uOpt;
%         control_timeout = control_timeout + 120; 
%         
%     else
%         
%         uOpt = previous_control;
%         
%     end



%% Compute disturbance
if isfield(schemeData, 'dIn')
  dU = schemeData.dIn;
  dL = schemeData.dIn;
  
else
  dU = dynSys.optDstb(t, g.xs, derivMax, schemeData.dMode);
  dL = dynSys.optDstb(t, g.xs, derivMin, schemeData.dMode);
end
  
%% Compute alpha
dxUU = dynSys.dynamics(t, schemeData.grid.xs, uU, dU);
dxUL = dynSys.dynamics(t, schemeData.grid.xs, uU, dL);
dxLL = dynSys.dynamics(t, schemeData.grid.xs, uL, dL);
dxLU = dynSys.dynamics(t, schemeData.grid.xs, uL, dU);
alpha = max(abs(dxUU{dim}), abs(dxUL{dim}));
alpha = max(alpha, abs(dxLL{dim}));
alpha = max(alpha, abs(dxLU{dim}));
end
